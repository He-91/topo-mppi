# 📋 详细行动计划

**最后更新**: 2025-10-02  
**当前阶段**: Phase 4.5.1.9验证

---

## 🔴 立即执行（现在）

### Step 1: 编译Phase 4.5.1.9
```bash
cd /home/he/ros_ws/test/ego-planner && catkin_make -j4
```

**预期**:
- ✅ 编译成功，0个错误
- ⚠️ 可能有IntelliSense警告（忽略）

---

### Step 2: 运行测试
```bash
source devel/setup.bash
roslaunch plan_manage run_in_sim.launch
```

**运行时间**: 1-2分钟，收集10-20个规划周期

---

### Step 3: 观察关键日志

#### ✅ 必须有（验证没退化）
```
[INFO] [TGK Corner] ✅ 找到corner point! pos=(...), free=X, occupied=Y
[INFO] [TopoGraphSearch] Building graph with 20 key points
```
→ 如果没有，说明Phase 4.5.1.9破坏了Corner Detection

#### 🎯 期望看到（A*改善）
```
[INFO] [TopoGraphSearch] A* search succeeded with X nodes
[INFO] [TopoPRM-TGK] Found 3 topological paths
```
→ 如果仍然"A* failed"，继续Step 4

#### ❌ 不应该看到（崩溃）
```
Segmentation fault (core dumped)
exit code -11
```
→ 如果崩溃，进入Step 5调试

---

### Step 4: 结果判断

#### 情况A: 成功（A*成功率>10%）✅
**观察到**:
- ✅ 20 key points仍然有效
- ✅ 至少偶尔看到"A* search succeeded"
- ✅ 偶尔看到"Found 3-5 topological paths"
- ✅ Fallback率<80%

**下一步**: 进入**Phase 4.5.1.10 - 多路径生成**

---

#### 情况B: 部分改善（A*成功率1-10%）⚠️
**观察到**:
- ✅ 20 key points仍然有效
- ⚠️ 偶尔看到1-2次"A* succeeded"（大部分仍失败）
- ⚠️ Fallback率80-95%

**分析**:
- step=0.2m有帮助，但还不够
- 需要进一步放宽

**下一步**: 进入**Phase 4.5.1.9-v3 - 进一步放宽**

---

#### 情况C: 无改善（A*仍0%）❌
**观察到**:
- ✅ 20 key points仍然有效
- ❌ 仍然100% "A* failed"
- ❌ 仍然100% fallback

**分析**:
- step=0.2m不够，或者问题不在step
- 可能是getInflateOccupancy膨胀半径太大

**下一步**: 进入**Phase 4.5.1.9-v3 - 诊断调试**

---

#### 情况D: 崩溃（exit code -11）💥
**观察到**:
- ❌ 运行一段时间后崩溃
- ❌ Segmentation fault

**分析**:
- 代码有内存问题或空指针
- 需要回滚或添加更多安全检查

**下一步**: 进入**Step 5 - 崩溃调试**

---

## 🔧 Phase 4.5.1.9-v3: 进一步优化（如需要）

### 方案A: 进一步放宽step
```cpp
// topo_graph_search.cpp isPathFree()
double step = 0.5;  // 从0.2m改为0.5m
```

### 方案B: 减少检查点数量
```cpp
int max_checks = 20;  // 无论距离多远，最多检查20个点
int num_checks = std::min(max_checks, static_cast<int>(dist / step));
```

### 方案C: 添加调试日志（安全版本）
```cpp
bool TopoGraphSearch::isPathFree(const Vector3d& from, const Vector3d& to) {
    static int success_count = 0;
    static int fail_count = 0;
    
    // ... 检查逻辑 ...
    
    if (result) {
        success_count++;
        if (success_count % 10 == 0) {
            ROS_INFO_THROTTLE(5.0, "[TopoGraphSearch] isPathFree stats: %d success, %d failed", 
                             success_count, fail_count);
        }
    } else {
        fail_count++;
    }
    
    return result;
}
```

---

## 🚀 Phase 4.5.1.10: 多路径生成（如果4.5.1.9成功）

### 前提条件
- ✅ Phase 4.5.1.9验证成功
- ✅ A*成功率>10%
- ✅ 至少能生成1条TGK路径

### 实施计划

#### Step 1: 实现简化版（2小时）
**文件**: `planner/path_searching/src/topo_graph_search.cpp`

```cpp
void TopoGraphSearch::extractMultiplePaths_Simplified(
    const Vector3d& start,
    const Vector3d& goal,
    vector<vector<Vector3d>>& paths) {
    
    paths.clear();
    
    // 1. 找第一条路径
    vector<Vector3d> path1;
    if (!astarSearch(start, goal, path1)) {
        ROS_WARN("[TopoGraphSearch] Failed to find first path");
        return;
    }
    smoothPath(path1);
    paths.push_back(path1);
    
    ROS_INFO("[TopoGraphSearch] Found first path with %zu waypoints", path1.size());
    
    // 2. 依次屏蔽节点生成备选路径
    for (int attempt = 0; attempt < max_topo_paths_ - 1; attempt++) {
        if (path1.size() < 3) break;
        
        // 选择要屏蔽的节点（均匀分布）
        size_t block_idx = path1.size() / (attempt + 2);
        Vector3d blocked_node = path1[block_idx];
        
        // 找到对应的node_pool索引
        int blocked_node_id = findNodeId(blocked_node);
        if (blocked_node_id < 0) continue;
        
        // 临时移除节点
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

#### Step 2: 添加辅助函数
```cpp
// 查找节点ID
int TopoGraphSearch::findNodeId(const Vector3d& pos) {
    for (size_t i = 0; i < node_pool_.size(); i++) {
        if ((node_pool_[i].pos - pos).norm() < 0.1) {
            return i;
        }
    }
    return -1;
}

// 检查路径重复
bool TopoGraphSearch::isDuplicatePath(
    const vector<Vector3d>& new_path,
    const vector<vector<Vector3d>>& existing_paths) {
    
    for (const auto& p : existing_paths) {
        if (p.size() == new_path.size()) {
            double total_diff = 0.0;
            for (size_t i = 0; i < p.size(); i++) {
                total_diff += (p[i] - new_path[i]).norm();
            }
            
            // 平均差异<0.5m认为重复
            if (total_diff / p.size() < 0.5) {
                return true;
            }
        }
    }
    return false;
}
```

#### Step 3: 修改调用
```cpp
// topo_graph_search.cpp searchTopoPaths()
// 从:
extractMultiplePaths(start, goal, paths);  // TODO版本

// 改为:
extractMultiplePaths_Simplified(start, goal, paths);  // 简化版
```

#### Step 4: 编译测试
```bash
catkin_make -j4
roslaunch plan_manage run_in_sim.launch
```

**观察**:
- 🎯 `Found X topological paths` (期望X=3-5)
- 🎯 Parallel MPPI能优化所有X条路径
- 🎯 系统选择最优路径

---

## 🟡 Phase 4.5.2: B-spline轻量级模式（后续）

### 前提
- ✅ TGK多路径生成完成
- ✅ Parallel MPPI正常工作

### 实施
参考 `BSPLINE_LIGHTWEIGHT_MODE_DESIGN.md`（如果该文件还在）

---

## 🎯 最终目标: 删除Legacy TopoPRM

### 前提条件
- ✅ TGK A*成功率>50%
- ✅ 多路径生成稳定
- ✅ 系统整体成功率>95%

### 操作
```cpp
// topo_prm.cpp searchTopoPaths()
// 删除整个legacy分支
if (use_tgk_algorithm_) {
    // TGK算法
    return searchWithTGK(start, goal, topo_paths);
} else {
    // ❌ 删除这个分支
    // return searchWithLegacy(start, goal, topo_paths);
}
```

---

## 📊 检查点

### Checkpoint 1: Phase 4.5.1.9
- [ ] 编译成功
- [ ] 不崩溃
- [ ] 20 key points仍有效
- [ ] A*成功率>0%

### Checkpoint 2: Phase 4.5.1.10
- [ ] 能生成3-5条路径
- [ ] 路径拓扑不同
- [ ] Parallel MPPI优化所有路径
- [ ] 系统选择最优

### Checkpoint 3: 最终验证
- [ ] TGK成功率>50%
- [ ] 整体成功率>95%
- [ ] 删除Legacy TopoPRM
- [ ] 系统稳定运行

---

**当前任务**: 执行Step 1-3，验证Phase 4.5.1.9
