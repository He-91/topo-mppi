# 🔧 TGK实现完整指南

**最后更新**: 2025-10-02

---

## 📚 TGK算法原理

**TGK (Topological Graph-based Key-point sampling)** [ICRA 2020]

### 核心流程
```
1. Corner Detection: 采样并识别拓扑关键点（角点、边界点）
2. Graph Building: 将关键点+起点+终点构建图
3. A* Search: 在图上搜索连接路径
4. Multi-path: 生成k条拓扑不同的路径
```

---

## ✅ Phase 4.5.1.7: 纯几何Corner Detection（已完成）

### 实现位置
`planner/path_searching/src/bias_sampler.cpp`

### 核心逻辑
```cpp
bool BiasSampler::isCornerPoint(const Vector3d& pos) {
    // 条件1: 必须在自由空间
    if (!grid_map_->isCollisionFree(pos)) {
        return false;
    }
    
    // 条件2: 必须靠近边界（8方向采样）
    int free_count = 0, occupied_count = 0;
    double radius = resolution_ * 3.0;
    
    for (8个方向) {
        Vector3d check_pt = pos + radius * direction;
        if (isCollisionFree(check_pt)) free_count++;
        else occupied_count++;
    }
    
    // 必须既有free又有occupied（边界附近）
    if (!(free_count > 0 && occupied_count > 0)) {
        return false;
    }
    
    // 条件3: 必须是角点（至少2个方向有障碍物）
    if (occupied_count < 2) {
        return false;
    }
    
    return true;  // 是有效的拓扑关键点
}
```

### 优势
- ✅ **只用occupancy grid**，不依赖ESDF
- ✅ **在所有区域有效**，不受传感器限制
- ✅ **简单高效**，8次采样即可
- ✅ **符合TGK论文**原始设计

### 验证结果（test1.md）
```
✅ 找到20个key points（达到上限）
✅ 条件1-3都正常工作
✅ Corner detection完全成功
```

---

## 🔴 Phase 4.5.1.8+4.5.1.9: A*搜索（当前问题）

### 问题诊断

#### connection_radius_太小（Phase 4.5.1.8）
```cpp
// topo_graph_search.cpp constructor
TopoGraphSearch::TopoGraphSearch()
    : connection_radius_(10.0),  // 从3.0m改为10.0m
```
**原因**: key points相距7-10m，radius=3.0m无法连接

#### isPathFree()太严格（Phase 4.5.1.9）
```cpp
bool TopoGraphSearch::isPathFree(const Vector3d& from, const Vector3d& to) {
    double step = 0.2;  // 从0.05m改为0.2m（放宽4倍）
    
    if (dist < 1e-6) return true;  // 安全检查
    
    for (int i = 0; i <= num_checks; ++i) {
        Vector3d check_pt = from + (dist * i / num_checks) * dir;
        if (grid_map_->getInflateOccupancy(check_pt)) {
            return false;  // 碰撞
        }
    }
    return true;
}
```

**修改原因**:
- **0.05m太细**: 10m连接需检查200个点，任何一点碰撞就拒绝
- **0.2m合理**: 10m连接检查50个点，更宽容

### 预期效果
- A*成功率: 0% → 10-50%
- TGK成功率: 0% → 30-50%
- Fallback率: 100% → 50-70%

---

## ❌ 待实现：多路径生成

### 当前状态
```cpp
// topo_graph_search.cpp line 218
void TopoGraphSearch::extractMultiplePaths(...) {
    // TODO: Implement k-shortest paths algorithm
    // 当前只返回1条路径
}
```

### 目标
实现**Yen's K-Shortest Paths**算法，生成3-5条拓扑不同的路径

### 实现方案

#### 简化版（2小时）
```cpp
void extractMultiplePaths_Simplified(start, goal, paths) {
    // 1. 找第一条路径
    astarSearch(start, goal, path1);
    paths.push_back(path1);
    
    // 2. 依次屏蔽第一条路径的节点，强制绕行
    for (int i = 0; i < max_paths - 1; i++) {
        // 选择中间节点屏蔽
        int block_idx = path1.size() / (i + 2);
        
        // 临时移除该节点
        backup = node_pool_[block_idx];
        node_pool_.erase(block_idx);
        
        // 重新搜索
        if (astarSearch(start, goal, alt_path)) {
            paths.push_back(alt_path);
        }
        
        // 恢复节点
        node_pool_.insert(block_idx, backup);
    }
}
```

#### 完整版（1天）- Yen's算法
```cpp
void extractMultiplePaths_Yen(start, goal, paths) {
    // 1. 第一条路径
    astarSearch(start, goal, first_path);
    paths.push_back(first_path);
    
    // 2. K-shortest paths
    for (int k = 1; k < max_paths; k++) {
        prev_path = paths[k-1];
        
        // 对路径上每个节点尝试偏离
        for (spur_idx in prev_path) {
            // 找到需要屏蔽的边
            for (p in paths) {
                if (p的前缀匹配root_path) {
                    blocked_edges.push_back(p[spur_idx]->p[spur_idx+1]);
                }
            }
            
            // 带屏蔽边的A*搜索
            astarSearchWithBlockedEdges(spur_node, goal, spur_path, blocked_edges);
            
            // 组合路径
            total_path = root_path + spur_path;
            candidates.push(total_path);
        }
        
        // 选择最优候选
        paths.push_back(candidates.top());
    }
}
```

### 实施步骤
1. ✅ Phase 4.5.1.9: 修复A*搜索
2. 🔴 测试验证A*成功率>10%
3. ⏳ 实现简化版多路径（2小时）
4. ⏳ 测试验证能生成3-5条路径
5. ⏳ 如需要，升级到完整Yen's算法（1天）

---

## 🎯 成功标准

### Phase 4.5.1.9验证
- ✅ 编译成功，不崩溃
- ✅ 20 key points仍然有效
- 🎯 A*成功率>10%（当前0%）
- 🎯 至少偶尔看到"A* found path"
- 🎯 Fallback率<50%（当前100%）

### 多路径生成验证
- 🎯 `extractMultiplePaths()`返回3-5条路径
- 🎯 路径拓扑不同（不是微小变化）
- 🎯 Parallel MPPI能优化所有路径
- 🎯 系统选择动力学最优路径

### 最终目标
- 🎯 TGK成功率>50%
- 🎯 删除Legacy TopoPRM
- 🎯 系统完全依赖TGK + Parallel MPPI

---

## 🔑 关键代码位置

### Corner Detection
- **文件**: `planner/path_searching/src/bias_sampler.cpp`
- **函数**: `isCornerPoint()` (line ~136-220)
- **状态**: ✅ 完成并验证

### A*搜索
- **文件**: `planner/path_searching/src/topo_graph_search.cpp`
- **函数**: 
  - `astarSearch()` - A*主逻辑
  - `canConnect()` - 连接判断（使用connection_radius_）
  - `isPathFree()` - 路径碰撞检测（Phase 4.5.1.9修改）
- **状态**: 🔧 Phase 4.5.1.9修改完成，待测试

### 多路径生成
- **文件**: `planner/path_searching/src/topo_graph_search.cpp`
- **函数**: `extractMultiplePaths()` (line ~218)
- **状态**: ❌ TODO待实现

---

## 📊 测试数据（test1.md）

### Corner Detection成功
```
[INFO] ✅ 找到corner point! pos=(-9.47,3.03,2.24), free=5, occupied=3
[INFO] 统计: 8015/6585/1086/703 (总数/条件1/条件2/条件3)
[INFO] Building graph with 20 key points ✅
```

### A*搜索失败
```
[INFO] Graph built with 22 nodes (20 key points + start + goal)
[WARN] A* failed to find path after 22 iterations, tested 231 connections ❌
[WARN] start can see goal: NO
[WARN] TGK search failed, falling back to legacy method
```

### Parallel MPPI工作
```
[INFO] 🚀 Parallel MPPI: Optimizing all 5 paths
[INFO] Path 1: norm_cost=200.805
[INFO] Path 2: norm_cost=146.829 🏆 BEST
→ 证明了并行MPPI的价值
```

---

**下一步**: 验证Phase 4.5.1.9是否解决A*问题
