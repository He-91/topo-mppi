# 多路径生成失败深度分析

## 问题现状

**目标**: 多路径生成率 13% → 40%  
**当前**: 4次测试全部只生成1条路径 (0%)  
**核心问题**: K-shortest paths算法完全失效

---

## 根因分析

### 1. 系统架构问题

```
TGK Corner Detection (84k+ corners) 
    ↓
TopoGraph Building (只选62个节点!)  ← 问题1: 节点筛选过于激进
    ↓
A* Search (第1条路径: 2-17次迭代成功)
    ↓
K-Shortest Paths (阻塞节点后: A*完全失败)  ← 问题2: 阻塞策略错误
    ↓
结果: 只有1条路径
```

### 2. 数据流分析

**Corner检测阶段**:
- 检测到84k+个corner点
- 说明环境信息丰富

**TopoGraph构建阶段**:
- **只选了62个节点** (包括起点/终点)
- 即只有60个中间节点可用
- **问题**: 为什么从84k压缩到60个？筛选标准是什么？

**第1条路径**:
- A*搜索: 2-17次迭代成功
- 最终路径: 2-3个waypoints
- **说明**: 图的连通性是好的

**K-shortest paths失败**:
- 阻塞60个节点 (100%!)
- A*失败: 1次迭代, 1个连接
- **问题**: 阻塞了所有可用节点,图完全断开

---

## 三个层次的问题

### Level 1: 策略层问题 (当前尝试修复的)

**问题**: 阻塞策略过于激进
- 阻塞半径太大 (5-11m)
- 遍历整条路径的所有点
- 结果阻塞60/60个节点 (100%)

**已尝试的修复**:
1. ❌ 阻塞5个位置 → 仍然60个节点
2. ❌ 走廊阻塞8m → 仍然60个节点  
3. ❌ 渐进式阻塞 → 仍然60个节点
4. 🔄 精细化局部阻塞 (待测试)

**问题**: 为什么改了这么多次,还是阻塞60个节点？

### Level 2: 图结构问题 (更深层)

**核心问题**: 图的拓扑结构单一

```
62个节点的分布可能是这样:

起点 ----[一串节点]---- 目标
      ↑
      全部在一条狭窄的走廊里!
```

**证据**:
1. 阻塞任何半径 > 3m,就阻塞了所有60个节点
2. 说明60个节点在空间上**高度集中**
3. 没有形成多个分支,无法产生非同伦路径

**如何验证**:
- 统计节点之间的平均距离
- 检查节点的空间分布
- 分析图的连通性 (是否有多个分支)

### Level 3: 采样问题 (最根本)

**怀疑**: BiasedSampler采样策略有问题

从84k个corner → 60个节点,筛选标准是什么？

可能的问题:
1. **只保留距离路径最近的节点** → 导致节点集中在一条走廊
2. **没有探索侧向分支** → 无法产生非同伦路径
3. **缺少"远离第1条路径"的采样** → K-shortest无法生效

---

## 对比: Legacy四向 vs TGK

### Legacy四向 (多路径生成率 80%)

```python
# 简单直接的采样策略
for direction in [left, right, front, back]:
    sample_nodes_in_direction(direction, num=20)
# 结果: 4个方向共80个节点,空间分布均匀
```

**为什么能生成多路径**:
- 强制在4个方向采样
- 空间分布天然多样化
- 即使阻塞1个方向,还有3个方向可用

### TGK (多路径生成率 13% → 0%)

```cpp
// 基于corner的采样 (推测)
corners = detect_corners();  // 84k个
nodes = filter_corners_near_path(corners);  // 只保留60个?
```

**为什么失败**:
- 只保留靠近"某条路径"的节点
- 没有强制多样性
- 导致节点集中,无法绕行

---

## 真正的解决方案

### 方案A: 修复采样策略 (推荐)

**目标**: 从84k个corner中,**主动采样多样化的节点**

```cpp
// 1. 第1条路径采样 (当前方式)
nodes_path1 = sample_along_straight_line(start, goal, 30);

// 2. 侧向采样 (NEW!)
for offset in [-10m, -5m, +5m, +10m]:
    nodes_side = sample_parallel_line(start, goal, offset, 15);
    nodes.push_back(nodes_side);

// 3. 远离第1条路径的采样 (NEW!)
nodes_far = sample_farthest_corners(nodes_path1, 20);

// 总计: 30 + 60 + 20 = 110个节点
// 空间分布: 多样化,有侧向分支
```

**优势**:
- 从根源解决问题
- 不依赖K-shortest的阻塞技巧
- 图本身就包含多条非同伦路径

### 方案B: 改进K-shortest (辅助)

**前提**: 方案A已实施,图有多样性

**改进点**:
1. **拓扑感知的阻塞** - 不只看距离,看拓扑类别
2. **Homotopy class** - 使用h-signature区分路径
3. **智能阻塞半径** - 根据节点密度自适应

### 方案C: 混合策略 (折中)

**当TGK节点不足时,补充Legacy节点**:

```cpp
tgk_nodes = sample_from_corners(60);
if (num_paths_found < 3) {
    // 补充4向采样
    legacy_nodes = sample_4_directions(20);
    merge(tgk_nodes, legacy_nodes);
}
```

---

## 立即行动计划

### Step 1: 诊断当前图结构

**目标**: 验证"节点集中"的假设

```cpp
// 添加调试代码
ROS_INFO("Node distribution analysis:");
for (node : node_pool_) {
    // 统计每个节点到路径的最小距离
    double dist = min_distance_to_path(node, first_path);
    ROS_INFO("  Node %d: dist=%.2f", node.id, dist);
}

// 统计
avg_dist = mean(all_distances);
max_dist = max(all_distances);
ROS_INFO("Avg dist to path: %.2f, Max: %.2f", avg_dist, max_dist);
```

**预期结果**:
- 如果avg_dist < 3m → 说明节点确实集中
- 如果max_dist < 10m → 说明没有远离路径的节点

### Step 2: 检查采样逻辑

**查看`buildSearchGraph()`函数**:

```cpp
// src/planner/path_searching/src/topo_graph_search.cpp
bool TopoGraphSearch::buildSearchGraph(start, goal) {
    // 这里是如何选择60个节点的?
    // 查找关键逻辑:
    // 1. 从BiasedSampler获取corners
    // 2. 如何筛选/排序corners
    // 3. 为什么只保留60个
}
```

### Step 3: 实施修复

**如果确认是采样问题** → 方案A (修改buildSearchGraph)  
**如果图结构OK** → 方案B (优化K-shortest)

---

## 预期效果

### 修复前 (当前)
```
图: 60个节点,全在一条3m宽的走廊内
阻塞: 任何半径>3m都阻塞所有节点
结果: 0条替代路径
```

### 修复后 (目标)
```
图: 110个节点,分布在5条平行走廊内
  - 主走廊: 30个节点
  - 左侧-10m: 15个节点
  - 左侧-5m: 15个节点
  - 右侧+5m: 15个节点
  - 右侧+10m: 15个节点
  - 远离点: 20个节点

阻塞策略: 阻塞主走廊的8m半径 (覆盖主走廊+左右5m)
结果: 4条替代路径 (左10m, 右10m, 远离点组合, ...)
```

---

## 下一步

1. **立即**: 添加节点分布分析代码
2. **然后**: 查看buildSearchGraph的采样逻辑
3. **最后**: 根据诊断结果选择方案A/B/C

**关键**: 不要再盲目调参,先搞清楚问题本质!
