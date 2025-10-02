# Test1 运行结果分析报告

## 📊 测试概况

从 test1.md 日志分析得出以下问题。

## 🚨 发现的关键问题

### 问题 1: Path 2 生成无效路径 (INF Cost)

**症状:**
```
[INFO] Path 2/2 (topo_cost=179769313486231570814527...., waypoints=1)
```

**根本原因:**
- TGK 的 `astarSearch` 在某些情况下生成只有 **1 个 waypoint** 的路径
- 日志显示: `[INFO] Path 2: 1 waypoints (blocked node at [-4.30, 12.23, 0.71])`
- 当 `calculatePathCost(path)` 遇到 `path.size() < 2` 时，返回 `std::numeric_limits<double>::max()` (INF)
- MPPI 优化器仍然尝试优化这个无效路径，浪费计算资源

**影响:**
- Parallel MPPI 会对这个无效路径进行优化（虽然最终会被淘汰）
- 日志中显示 Path 2 经常有更低的 normalized cost，但这可能是因为长度极短
- 浪费 ~30% 的 MPPI 计算时间

**修复:**
- ✅ 已在 `topo_graph_search.cpp` line 272-297 添加路径验证：
  ```cpp
  if (alt_path.size() < 2) {
      ROS_DEBUG("[TopoGraphSearch] Rejected incomplete alternative path");
  } else {
      // ... 正常处理 ...
  }
  ```

### 问题 2: TGK 频繁失败

**统计:**
从 test1.md 中搜索到 **至少 20 次** TGK 失败：
```
[WARN] [TopoPRM-TGK] TGK search failed, NO FALLBACK (Legacy commented out)
```

**失败场景示例:**
```
Line 176:  [1759421124.130958883]
Line 991:  [1759421127.430326786]
Line 1152: [1759421129.030382840]
Line 1222: [1759421129.730432408]
Line 1302: [1759421129.880444466]
Line 1370: [1759421130.030315741]
... 总共 20+ 次
```

**失败时系统行为:**
1. TGK 失败 → Legacy 被注释掉无法使用
2. 系统回退到 "original approach" (polynomial trajectory)
3. Polynomial trajectory 直接连接 start-goal，然后 BsplineOptimizer 调用 MPPI local path search 绕开障碍物
4. 这个 fallback 机制有效，但**无人机可能在障碍物附近打转**

**为什么无人机打转？**

当 TGK 失败时：
- Polynomial trajectory 生成的是 **最短直线路径**
- MPPI local path search 只对**障碍物段**进行局部调整
- 如果障碍物复杂（多个obstacle cluster），local search 可能找到的是**局部最优**而非**全局最优**
- 导致无人机在障碍物附近反复调整路径（表现为"打转"）

**对比:**
- TGK 成功时：提供 **全局拓扑多样性**（2-3条不同拓扑路径）
- TGK 失败时：只有 **局部绕障**，可能陷入局部最优

## 📈 成功率统计

基于 test1.md 粗略统计：
- **TGK 失败率**: ~25-30% (20+ failures in ~80 planning cycles)
- **End-to-end 成功率**: 仍然很高 (>90%)，得益于 polynomial + MPPI fallback
- **Path 2 无效率**: ~5-10% (多次出现 waypoints=1)

## 🎯 根本原因分析

### TGK 为什么频繁失败？

#### 1. Corner Detection 问题
日志显示：
```
[WARN] [BiasSampler] Reached max corner number limit
```
- 达到了 corner 数量上限（20个）
- 但有些关键的 corner 可能被遗漏
- Corner quality 不够高

#### 2. Graph Connectivity 问题
```
[TopoGraphSearch] Graph built with 22 nodes
[TopoGraphSearch] A* search: 22 nodes, start_id=0, goal_id=21
[WARN] [TopoGraphSearch] A* failed to find path after X iterations
```
- 图中有 22 个节点，但 A* 仍然失败
- 说明 **节点之间的连接性不足**
- 可能的原因：
  - `connection_radius` 太小 (当前 10.0m)
  - `canConnect()` 检查太严格
  - Corner 分布不均匀，导致某些区域无连接

#### 3. Blocked Node Strategy 过于激进
```cpp
// Phase 4.5.1.11: Relaxed from 2.0m to 3.0m
if (blocked_node_id < 0 || min_dist > 3.0) {
    continue;
}
```
- 当阻挡节点寻找替代路径时，半径限制 3.0m 可能导致过度阻挡
- 导致某些关键连接被切断

## 🔧 建议的优化方向

### 优先级 1: 修复无效路径问题 ✅
- **已完成**: 添加 `alt_path.size() >= 2` 验证
- **预期效果**: 消除 INF cost 路径，节省 MPPI 计算

### 优先级 2: 提高 TGK 成功率 🎯

#### Option A: 增加 Connection Radius
```cpp
// topo_graph_search.cpp
double connection_radius = 10.0;  // 增加到 15.0m?
```
**优点:** 简单直接，提高图连通性  
**缺点:** 可能增加计算量，产生更多低质量连接

#### Option B: 改进 Corner Detection
```cpp
// bias_sampler.cpp
int max_corner_num = 20;  // 增加到 30?
```
**优点:** 更多关键点，提高覆盖率  
**缺点:** 可能增加冗余点

#### Option C: 放宽 Blocked Node Strategy
```cpp
// topo_graph_search.cpp line 261
if (blocked_node_id < 0 || min_dist > 5.0) {  // 3.0 → 5.0
    continue;
}
```
**优点:** 减少过度阻挡  
**缺点:** 可能生成拓扑相似的路径

#### Option D: 添加 Direct Path Fallback
在 TGK A* 失败时，直接返回一个简单的直线路径（而不是完全失败）
```cpp
if (!astarSearch(start, goal, first_path)) {
    // Fallback: create direct path
    first_path = {start, goal};
    ROS_WARN("[TopoGraphSearch] A* failed, using direct fallback");
}
```
**优点:** 确保 TGK 永不失败  
**缺点:** 可能产生低质量路径

### 优先级 3: 改进 MPPI 选择策略

当前问题：Path 2 (waypoints=1) 有时有更低的 normalized_cost，但实际上是无效的。

**建议:** 在 parallel MPPI 选择时，添加路径质量检查：
```cpp
// planner_manager.cpp
if (mppi_success && candidate.mppi_result.positions.size() >= 7) {
    // 🔧 添加: 检查拓扑路径质量
    if (topo_paths[i].path.size() < 2) {
        ROS_WARN("[PlannerManager] Skipping invalid topo path %zu", i);
        continue;
    }
    // ... 继续正常处理 ...
}
```

## 📋 测试建议

### 立即测试
1. ✅ 重新编译并测试无效路径修复
2. 观察 "waypoints=1" 是否消失
3. 统计 TGK 失败率是否改善

### 后续测试（选择一个优化方向）
1. 测试 Connection Radius = 15.0m
2. 测试 Max Corner = 30
3. 测试 Blocked Node Radius = 5.0m

每次只改一个参数，对比成功率变化。

## 💡 关于 Legacy 删除的最终建议

基于当前测试结果：

**暂时不建议完全删除 Legacy TopoPRM**，原因：
1. TGK 失败率 25-30% 仍然较高
2. Polynomial fallback 虽然有效，但会导致"打转"现象
3. Legacy 可能在 TGK 失败时提供更好的全局路径

**建议采用混合策略:**
```cpp
if (tgk_success) {
    use TGK paths
} else {
    // 解除 Legacy 注释，作为备用
    ROS_WARN("[TopoPRM-TGK] TGK failed, using Legacy TopoPRM");
    candidate_paths = findTopoPaths(start, goal);
}
```

**或者:** 先提升 TGK 成功率到 >95%，再删除 Legacy。

## 🎯 下一步行动

1. ✅ 应用无效路径修复补丁
2. 🔄 重新编译和测试
3. 📊 收集新的测试数据
4. 🔧 根据结果选择一个参数优化方向
5. 📈 迭代优化直到 TGK 成功率 >95%
6. ✅ 然后安全删除 Legacy
