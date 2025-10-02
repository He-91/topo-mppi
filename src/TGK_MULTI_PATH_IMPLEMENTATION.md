# TGK多路径生成实现方案

## 问题分析

### 当前状态
- Corner Detection: 0% 成功率（Phase 4.5.1.7修复中）
- 多路径生成: **未实现**（只返回1条路径）
- 实际效果: 系统依赖Legacy TopoPRM

### 目标
- 生成3-7条拓扑不同的路径
- 给并行MPPI提供多样化选择

---

## 方案：Yen's K-Shortest Paths算法

### 算法原理

**输入**: 图G(V, E), 起点s, 终点t, 路径数k  
**输出**: k条最短路径（拓扑不同）

**核心思想**:
1. 找到最短路径P1
2. 对于每条已找到的路径Pi，依次"偏离"路径上的每个节点
3. 临时移除部分边，强制A*找到不同的路径
4. 重复直到找到k条路径

### 伪代码

```python
def yen_k_shortest_paths(graph, start, goal, k):
    paths = []
    
    # 1. 找到最短路径
    path = astar(graph, start, goal)
    paths.append(path)
    
    # 候选路径集合（优先队列，按长度排序）
    candidates = PriorityQueue()
    
    for i in range(1, k):
        # 2. 从第i-1条路径生成新的候选路径
        prev_path = paths[-1]
        
        # 对路径上的每个节点（除了终点）
        for j in range(len(prev_path) - 1):
            spur_node = prev_path[j]
            root_path = prev_path[0:j+1]
            
            # 3. 临时移除冲突的边
            removed_edges = []
            for p in paths:
                # 如果p的前缀与root_path相同
                if p[0:j+1] == root_path:
                    # 移除p的下一条边
                    edge = (p[j], p[j+1])
                    removed_edges.append(edge)
                    graph.remove_edge(edge)
            
            # 4. 从spur_node搜索到goal
            spur_path = astar(graph, spur_node, goal)
            
            if spur_path:
                # 5. 组合root_path + spur_path
                total_path = root_path[:-1] + spur_path
                candidates.put(total_path)
            
            # 6. 恢复移除的边
            for edge in removed_edges:
                graph.add_edge(edge)
        
        if candidates.empty():
            break
        
        # 7. 选择代价最小的候选路径
        paths.append(candidates.get())
    
    return paths
```

---

## C++实现（topo_graph_search.cpp）

### 修改extractMultiplePaths()

```cpp
void TopoGraphSearch::extractMultiplePaths(const Vector3d& start,
                                          const Vector3d& goal,
                                          vector<vector<Vector3d>>& paths) {
    paths.clear();
    
    // Step 1: 找到第一条最短路径
    vector<Vector3d> first_path;
    if (!astarSearch(start, goal, first_path)) {
        ROS_WARN("[TopoGraphSearch] Failed to find first path");
        return;
    }
    smoothPath(first_path);
    paths.push_back(first_path);
    
    ROS_INFO("[TopoGraphSearch] Found first path with %zu waypoints", first_path.size());
    
    // Step 2: 使用Yen's算法找到k条路径
    int target_paths = std::min(max_topo_paths_, 5);  // 最多5条
    
    // 候选路径集合（pair<cost, path>）
    typedef pair<double, vector<Vector3d>> CandidatePath;
    auto cmp = [](const CandidatePath& a, const CandidatePath& b) {
        return a.first > b.first;  // 最小堆
    };
    priority_queue<CandidatePath, vector<CandidatePath>, decltype(cmp)> candidates(cmp);
    
    for (int k = 1; k < target_paths; k++) {
        vector<Vector3d>& prev_path = paths[k-1];
        
        // 对prev_path上的每个节点尝试偏离
        for (size_t spur_idx = 0; spur_idx < prev_path.size() - 1; spur_idx++) {
            Vector3d spur_node = prev_path[spur_idx];
            vector<Vector3d> root_path(prev_path.begin(), 
                                      prev_path.begin() + spur_idx + 1);
            
            // 找到需要临时屏蔽的边
            vector<pair<int, int>> blocked_edges;
            for (const auto& p : paths) {
                // 检查p的前缀是否与root_path匹配
                if (p.size() > spur_idx + 1) {
                    bool prefix_match = true;
                    for (size_t i = 0; i <= spur_idx; i++) {
                        if ((p[i] - root_path[i]).norm() > 0.1) {
                            prefix_match = false;
                            break;
                        }
                    }
                    
                    if (prefix_match) {
                        // 找到对应的节点ID并屏蔽边
                        int node_id_1 = findNodeId(p[spur_idx]);
                        int node_id_2 = findNodeId(p[spur_idx + 1]);
                        if (node_id_1 >= 0 && node_id_2 >= 0) {
                            blocked_edges.push_back({node_id_1, node_id_2});
                        }
                    }
                }
            }
            
            // 从spur_node搜索到goal（屏蔽某些边）
            vector<Vector3d> spur_path;
            if (astarSearchWithBlockedEdges(spur_node, goal, spur_path, blocked_edges)) {
                // 组合路径
                vector<Vector3d> total_path = root_path;
                total_path.insert(total_path.end(), 
                                 spur_path.begin() + 1, spur_path.end());
                
                // 计算路径代价
                double path_cost = calculatePathCost(total_path);
                
                // 检查是否重复
                if (!isDuplicatePath(total_path, paths)) {
                    candidates.push({path_cost, total_path});
                }
            }
        }
        
        // 选择代价最小的候选路径
        if (candidates.empty()) {
            ROS_INFO("[TopoGraphSearch] No more alternative paths found");
            break;
        }
        
        auto best_candidate = candidates.top();
        candidates.pop();
        
        smoothPath(best_candidate.second);
        paths.push_back(best_candidate.second);
        
        ROS_INFO("[TopoGraphSearch] Found path %d with cost %.2f", 
                 k+1, best_candidate.first);
    }
    
    ROS_INFO("[TopoGraphSearch] Generated %zu topological paths", paths.size());
}
```

### 新增辅助函数

```cpp
// 1. 带边屏蔽的A*搜索
bool TopoGraphSearch::astarSearchWithBlockedEdges(
    const Vector3d& start,
    const Vector3d& goal,
    vector<Vector3d>& path,
    const vector<pair<int, int>>& blocked_edges) {
    
    // 与astarSearch()类似，但在canConnect()时检查blocked_edges
    // ...
    
    // 在扩展邻居时：
    for (size_t i = 0; i < node_pool_.size(); ++i) {
        // 检查这条边是否被屏蔽
        bool is_blocked = false;
        for (const auto& edge : blocked_edges) {
            if ((edge.first == current_id && edge.second == i) ||
                (edge.first == i && edge.second == current_id)) {
                is_blocked = true;
                break;
            }
        }
        
        if (is_blocked) continue;  // 跳过被屏蔽的边
        
        // 正常处理
        // ...
    }
}

// 2. 查找节点ID
int TopoGraphSearch::findNodeId(const Vector3d& pos) {
    for (size_t i = 0; i < node_pool_.size(); i++) {
        if ((node_pool_[i].pos - pos).norm() < 0.1) {
            return i;
        }
    }
    return -1;
}

// 3. 计算路径代价
double TopoGraphSearch::calculatePathCost(const vector<Vector3d>& path) {
    double cost = 0.0;
    for (size_t i = 0; i < path.size() - 1; i++) {
        cost += edgeCost(path[i], path[i+1]);
    }
    return cost;
}

// 4. 检查路径重复
bool TopoGraphSearch::isDuplicatePath(const vector<Vector3d>& new_path,
                                     const vector<vector<Vector3d>>& existing_paths) {
    for (const auto& p : existing_paths) {
        // 简化检查：如果waypoints非常相似，认为重复
        if (p.size() == new_path.size()) {
            double total_diff = 0.0;
            for (size_t i = 0; i < p.size(); i++) {
                total_diff += (p[i] - new_path[i]).norm();
            }
            
            if (total_diff / p.size() < 0.5) {  // 平均差异<0.5m
                return true;  // 重复
            }
        }
    }
    return false;
}
```

---

## 方案B：简化版（快速实现）

如果Yen's算法太复杂，可以先实现简化版：

```cpp
void TopoGraphSearch::extractMultiplePaths_Simplified(
    const Vector3d& start,
    const Vector3d& goal,
    vector<vector<Vector3d>>& paths) {
    
    paths.clear();
    
    // 1. 找到第一条路径
    vector<Vector3d> path1;
    if (!astarSearch(start, goal, path1)) return;
    paths.push_back(path1);
    
    // 2. 简化方法：依次屏蔽第一条路径上的节点，强制绕行
    for (int attempt = 0; attempt < max_topo_paths_ - 1; attempt++) {
        // 选择一个要屏蔽的节点（第一条路径的中间节点）
        if (path1.size() < 3) break;
        
        size_t block_idx = path1.size() / (attempt + 2);  // 均匀分布
        Vector3d blocked_node = path1[block_idx];
        
        // 临时标记这个节点为"已访问"（强制A*绕过它）
        // 在node_pool_中找到对应节点
        int blocked_node_id = findNodeId(blocked_node);
        if (blocked_node_id < 0) continue;
        
        // 备份并临时移除这个节点
        TopoNode backup = node_pool_[blocked_node_id];
        node_pool_.erase(node_pool_.begin() + blocked_node_id);
        
        // 重新搜索
        vector<Vector3d> alt_path;
        if (astarSearch(start, goal, alt_path)) {
            // 检查是否真的不同
            if (!isDuplicatePath(alt_path, paths)) {
                smoothPath(alt_path);
                paths.push_back(alt_path);
                ROS_INFO("[TopoGraphSearch] Found alternative path %zu", paths.size());
            }
        }
        
        // 恢复节点
        node_pool_.insert(node_pool_.begin() + blocked_node_id, backup);
    }
    
    ROS_INFO("[TopoGraphSearch] Generated %zu paths (simplified)", paths.size());
}
```

---

## 实施步骤

### Phase 1: 修复Corner Detection（进行中）
- 目标: key points从0变成10-50个
- 方案: Phase 4.5.1.7（移除ESDF依赖）
- 预期: TGK能生成1条路径

### Phase 2: 实现多路径生成（下一步）
- 目标: 从1条路径变成3-5条
- 方案: 先实现简化版（方案B），验证效果后再实现完整版（方案A）

### Phase 3: 拓扑过滤优化（后续）
- 使用拓扑签名（homotopy class）严格去重
- 确保路径真正拓扑不同

---

## 预期效果

### 修复前
```
TGK搜索:
  ├─> key points: 0个
  ├─> A*搜索: 失败
  └─> 输出: 0条路径

Fallback Legacy:
  └─> 输出: 4条路径

并行MPPI:
  └─> 优化4条legacy路径
```

### 修复后（Phase 1）
```
TGK搜索:
  ├─> key points: 15个 ✅
  ├─> A*搜索: 成功
  └─> 输出: 1条路径 ⚠️（多路径未实现）

并行MPPI:
  └─> 只优化1条路径（浪费）
```

### 修复后（Phase 1+2）
```
TGK搜索:
  ├─> key points: 15个 ✅
  ├─> A*搜索: 成功
  ├─> Yen's k-shortest: 成功
  └─> 输出: 5条拓扑不同路径 ✅✅✅

并行MPPI:
  └─> 优化5条路径，选最优 ✅
```

---

## 工作量估算

| 阶段 | 工作量 | 难度 | 优先级 |
|------|--------|------|--------|
| Phase 1（Corner Detection） | 已完成 | 中 | P0 |
| Phase 2-简化版 | 2小时 | 低 | P1 |
| Phase 2-完整版（Yen's） | 1天 | 中 | P2 |
| Phase 3（拓扑过滤） | 半天 | 中 | P3 |

---

## 总结

### 当前问题
1. ❌ Corner Detection失败 → 0 key points
2. ❌ 多路径生成未实现 → 只1条路径
3. ✅ Fallback Legacy工作 → 系统依赖旧方法

### 解决方案
1. ✅ Phase 4.5.1.7修复Corner Detection
2. ⏳ 实现Yen's k-shortest paths
3. ⏳ 添加拓扑过滤

### 预期结果
- TGK成功率: 0% → 30%+
- TGK输出: 0-1条 → 3-5条路径
- 系统性能: 完全依赖Legacy → TGK为主，Legacy为辅
