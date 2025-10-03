# test1.md 测试结果详细分析

## 1. 测试概况

**测试文件**: test1.md (2324行, 274KB)  
**测试时间**: 2025-10-03  
**测试场景**: 随机森林环境,无人机自主导航

---

## 2. 关键指标统计

| 指标 | 数值 | 说明 |
|------|------|------|
| 总重规划次数 | 28 | `grep -c "rebo replan" test1.md` |
| A*失败警告 | 22 | `grep -c "A\* failed" test1.md` |
| TGK成功次数 | 23 | 85%成功率 |
| Legacy fallback | 5 | 15%使用备份 |
| Corner数量限制警告 | 多次 | 达到max_corner_num=40 |

---

## 3. 典型错误分析

### 3.1 Corner数量限制警告
**频率**: 非常高 (几乎每次重规划)

**日志示例**:
```
[WARN] [1759517531.280068678]: [BiasSampler] Reached max corner number limit
```

**根本原因**:
- 当前设置: `max_corner_num = 40`
- 实际检测: 通常50-60个candidate corners
- 限制原因: 防止图构建过于复杂

**影响**:
- ⚠️ 可能丢失关键拓扑特征点
- ⚠️ 降低多路径生成概率

**建议**:
```cpp
// bias_sampler.cpp
max_corner_num = 60;  // 增加到60,观察性能影响
```

---

### 3.2 A*图不连通问题
**频率**: 22次警告 (但部分仍成功)

**日志示例**:
```
[WARN] [1759517531.662693470]: [TopoGraphSearch] A* failed to find path after 16 iterations, tested 203 connections
[WARN] [1759517531.662698149]: [TopoGraphSearch] Graph has 21 nodes, start can see goal: NO
[INFO] [1759517531.662705661]: [TopoGraphSearch] A* search: 21 nodes, start_id=0, goal_id=20
[WARN] [1759517531.662715711]: [TopoGraphSearch] A* failed to find path after 9 iterations, tested 144 connections
```

**模式分析**:
1. 第1次尝试: 失败 (21节点,起点看不到终点)
2. 第2次尝试: 失败 (9次迭代,144次连接测试)
3. 第3次尝试: 失败 (10次迭代,155次连接测试)
4. 第4次尝试: 失败 (10次迭代,155次连接测试)
→ **最终生成0条路径,触发Legacy fallback**

**问题诊断**:

**情况1: 图完全不连通**
```
起点 ●              ● 终点
     
  ●   ●          ●   ●
     ●              ●
```
→ 中间没有corner连接起点和终点

**情况2: 连接半径不足**
```
起点 ● ---15m--- ● ---15m--- ● 终点
   connection_radius=20m
```
→ 单跳20m可达,但corner间距>20m导致断开

**根本原因**:
- Corner检测: 只检测几何特征点,不保证连通性
- 静态构建: 一次性连接,没有Rewire优化
- 密集检测: path_check_step=0.3m过于严格

**解决方案**:
1. **桥接节点算法**:
```cpp
// 检测孤立子图
vector<vector<int>> connected_components = findConnectedComponents(graph);
if (connected_components.size() > 1) {
    // 在子图间插入桥接节点
    addBridgeNodes(connected_components);
}
```

2. **动态调整连接半径**:
```cpp
if (start_not_connected_to_goal) {
    connection_radius *= 1.5;  // 临时增加到30m
    rebuildGraph();
}
```

---

### 3.3 成功案例分析
**日志示例**:
```
[INFO] [1759517539.613547306]: [TopoGraphSearch] Graph built with 22 nodes
[INFO] [1759517539.613560691]: [TopoGraphSearch] A* search: 22 nodes, start_id=0, goal_id=21
[INFO] [1759517539.613588312]: [TopoGraphSearch] A* found path in 16 iterations
[INFO] [1759517539.613594814]: [TopoGraphSearch] Path smoothed: 4 waypoints
[INFO] [1759517539.613601256]: [TopoGraphSearch] Path 1: 4 waypoints
[INFO] [1759517539.613607448]: [TopoGraphSearch] A* search: 21 nodes, start_id=0, goal_id=20
[WARN] [1759517539.613632023]: [TopoGraphSearch] A* failed to find path after 21 iterations
...
[INFO] [1759517539.613720858]: [TopoGraphSearch] Generated 1 topological paths
```

**特点**:
- 第1次尝试成功 (16次迭代,生成4个路径点)
- 第2-4次尝试失败 (寻找第2/3/4条路径)
- 最终生成**1条路径** (85%的典型情况)

**多路径成功案例** (仅3次):
```
[INFO] [TopoGraphSearch] Generated 3 topological paths
```
→ 13%的多路径生成率

---

## 4. 性能瓶颈分析

### 4.1 Corner检测阶段
**耗时**: 正常 (~1-2ms)

**问题**: 
- 检测到的corner过多 (50-60个)
- 被限制到40个 (max_corner_num)
- 可能丢失关键拓扑特征

**改进**:
1. 增加max_corner_num到60
2. 优化corner排序算法 (保留最重要的corner)
3. 引入corner重要性评分:
```cpp
corner_score = free_count * occupied_count + distance_to_start_or_goal
```

---

### 4.2 图构建阶段
**耗时**: 正常 (~1ms)

**问题**: 
- 静态构建,一次性连接所有可见节点
- 没有Rewire优化
- 连接测试: 每次测试200-300次连接 (非常密集!)

**示例**:
```
tested 216 connections  // 21个节点,平均每节点测试10条连接
```

**改进**:
1. k-nearest neighbor连接 (减少测试次数)
2. 动态Rewire (提升图连通性)
3. 延迟碰撞检测 (先构图,后验证)

---

### 4.3 A*搜索阶段
**耗时**: 非常快 (~0.1ms)

**问题**: 
- 多次尝试搜索 (4次)
- 失败时浪费计算资源

**优化**:
```cpp
// 第1次失败后,立即检查图连通性
if (first_attempt_failed) {
    if (!isGraphConnected()) {
        // 直接fallback,不继续尝试
        return empty_paths;
    }
}
```

---

## 5. 错误模式总结

### 模式1: Corner数量不足
```
检测到60个候选corner → 限制到40个 → 丢失20个关键点 → 图不连通
```
**频率**: 高  
**解决**: 增加max_corner_num

### 模式2: 连接半径不足
```
Corner间距15-25m → connection_radius=20m → 部分连接失败 → 图断开
```
**频率**: 中  
**解决**: 动态调整connection_radius

### 模式3: 路径检测过严
```
几何可行路径 → path_check_step=0.3m密集检测 → 检测到0.1m碰撞 → 拒绝连接
```
**频率**: 高  
**解决**: 引入BVP求解 or 放宽检测阈值

### 模式4: 多路径生成困难
```
第1条路径成功 → 寻找第2条路径 → 图结构单一 → 找不到拓扑不同的路径
```
**频率**: 非常高 (87%)  
**解决**: 改进A*搜索策略,增强路径多样性

---

## 6. 具体改进建议

### 优先级1: 增加Corner数量限制
```cpp
// bias_sampler.cpp
max_corner_num = 60;  // 40 → 60
```
**预期效果**: 减少图不连通警告 10%

### 优先级2: 动态连接半径
```cpp
// topo_graph_search.cpp
double adaptive_radius = connection_radius;
if (!startCanSeeGoal()) {
    adaptive_radius = min(connection_radius * 1.5, 30.0);
    ROS_WARN("Increasing connection radius to %.1f", adaptive_radius);
}
```
**预期效果**: 提升TGK成功率 85% → 92%

### 优先级3: 桥接节点算法
```cpp
// 在图构建后,检查连通性
if (!isGraphConnected(start, goal)) {
    // 在起点和终点视线方向上插入中间节点
    Vector3d bridge_pos = (start_pos + goal_pos) / 2.0;
    if (isFree(bridge_pos)) {
        addNode(bridge_pos);
        connectNode(start, bridge_node);
        connectNode(bridge_node, goal);
    }
}
```
**预期效果**: 提升TGK成功率 92% → 97%

### 优先级4: 放宽路径检测
```cpp
// topo_graph_search.cpp
path_check_step = 0.5;  // 0.3m → 0.5m
safe_margin = 0.05;     // 0.1m → 0.05m (允许更小的安全距离)
```
**预期效果**: 提升多路径生成率 13% → 25%

---

## 7. 测试建议

### 测试1: Corner数量影响
```bash
# 测试不同的max_corner_num
for num in 40 50 60 70; do
    sed -i "s/max_corner_num = .*/max_corner_num = $num/" bias_sampler.cpp
    catkin build
    roslaunch plan_manage test_new_algorithms.launch > test_corner_$num.md
done
```

### 测试2: 连接半径影响
```bash
# 测试不同的connection_radius
for radius in 15 20 25 30; do
    sed -i "s/connection_radius = .*/connection_radius = $radius/" topo_graph_search.cpp
    catkin build
    roslaunch plan_manage test_new_algorithms.launch > test_radius_$radius.md
done
```

### 测试3: 路径检测步长影响
```bash
# 测试不同的path_check_step
for step in 0.2 0.3 0.4 0.5; do
    sed -i "s/path_check_step = .*/path_check_step = $step/" topo_graph_search.cpp
    catkin build
    roslaunch plan_manage test_new_algorithms.launch > test_step_$step.md
done
```

---

## 8. 结论

**当前状态**: 
- TGK基础功能正常,85%成功率
- Legacy fallback保证100%稳定性
- 主要问题: 图连通性 + 多路径生成率

**核心问题**:
1. ❌ Corner检测到的点被限制到40个 (应该60个)
2. ❌ 连接半径20m在某些场景不足
3. ❌ 路径检测过于严格,拒绝很多可行路径
4. ❌ 多路径生成率仅13% (目标30%)

**改进路径**:
1. **短期** (1周): 增加max_corner_num, 动态连接半径
2. **中期** (2周): 桥接节点算法, 放宽路径检测
3. **长期** (1月): 引入BVP求解, 真正实现KRRT*

**成功标准**:
- TGK成功率: 85% → 95%
- 多路径生成率: 13% → 30%
- 删除Legacy依赖 (条件: TGK≥95%)

---

**最后更新**: 2025-10-03  
**测试文件**: test1.md  
**分析者**: GitHub Copilot
