# 全局规划算法对比分析: 非同伦路径生成能力

> **核心目标**: 生成拓扑不同的路径,防止MPPI陷入局部最优

**分析日期**: 2025-10-03  
**分析维度**: 全局探索能力、非同伦路径多样性、防局部最优能力

---

## 1. 三种算法核心机制对比

### 1.1 Legacy 四向规避算法

**代码位置**: `topo_prm.cpp::generateAlternativePath()`

**核心机制**:
```cpp
// 固定4个规避方向
switch (direction) {
    case 0: avoidance_dir = Vector3d(0, 0, 1);   // 上
    case 1: avoidance_dir = Vector3d(0, 0, -1);  // 下
    case 2: avoidance_dir = forward.cross(z);    // 左
    case 3: avoidance_dir = -forward.cross(z);   // 右
}

// 简单规避: 障碍物 + 方向*半径
waypoint = obstacle_center + avoidance_dir * search_radius;
path = {start, waypoint, goal};  // 3点路径
```

**拓扑路径生成方式**:
1. 采样障碍物中心 (在起点-终点连线附近)
2. 对每个障碍物,固定4个方向生成绕行路径
3. 最多生成 `4 * N_obstacles` 条路径

**非同伦能力分析**:

| 维度 | 评估 | 说明 |
|------|------|------|
| **拓扑多样性** | ⭐⭐⭐ | 4个固定方向,多样性中等 |
| **全局探索** | ⭐⭐ | 只探索障碍物附近,探索范围受限 |
| **防局部最优** | ⭐⭐⭐ | 能绕过单个障碍物,但复杂环境不足 |
| **路径数量** | ⭐⭐⭐⭐ | 容易生成多条路径 (每障碍物4条) |

**典型场景表现**:

**✅ 擅长**:
```
起点 ●               ● 终点
         🟦
     (单个障碍物)
```
→ 生成上下左右4条路径,拓扑明确

**❌ 不擅长**:
```
起点 ●  🟦  🟦  🟦  ● 终点
        🟦      🟦
```
→ 多障碍物环境,4个方向可能都被堵,或者只能绕过最近障碍物

---

### 1.2 我们的TGK (简化版几何A*)

**代码位置**: `topo_graph_search.cpp::extractMultiplePaths()`

**核心机制**:
```cpp
// 步骤1: Corner检测 (几何特征点)
corners = detectObstacleCorners(start, goal);  // 检测所有边界角点

// 步骤2: A*图构建 (全连接)
for (corner_i in corners) {
    for (corner_j in corners) {
        if (distance < 20m && path_collision_free) {
            connect(i, j);  // 静态全连接
        }
    }
}

// 步骤3: K-shortest paths (阻塞节点)
path1 = A*(start, goal);                    // 第1条最优路径
block(path1[middle_node]);                  // 阻塞中间节点
path2 = A*(start, goal);                    // 第2条路径
block(path2[middle_node]);
path3 = A*(start, goal);                    // 第3条路径
...
```

**拓扑路径生成方式**:
1. 检测环境中所有拓扑关键点 (corner)
2. 构建全连接图 (20m半径内所有可见节点)
3. 通过**阻塞已有路径节点**强制生成新路径

**非同伦能力分析**:

| 维度 | 评估 | 说明 |
|------|------|------|
| **拓扑多样性** | ⭐⭐⭐⭐ | 基于环境几何特征,自然多样 |
| **全局探索** | ⭐⭐⭐⭐⭐ | 检测全局corner,探索范围广 |
| **防局部最优** | ⭐⭐⭐⭐ | 多路径并行MPPI,概率高 |
| **路径数量** | ⭐⭐ | **当前仅13%生成多路径** ❌ |

**问题诊断**:

**问题1: Corner检测不保证连通性**
```
起点 ●  corner1  corner2  corner3  ● 终点
         (孤岛)   (孤岛)   (孤岛)
```
→ 检测到很多corner,但彼此不连通 → A*失败15%

**问题2: 阻塞策略简单**
```cpp
// 当前实现: 阻塞第1条路径的中间点
block_idx = path1.size() / 2;  // 只阻塞1个点
blocked_pos = path1[block_idx];

// 问题: 如果图结构单一,阻塞1个点后无其他路径
if (only_one_corridor_exists) {
    return 0_alternative_paths;  // 第2-4次搜索全失败
}
```

**问题3: 静态图,无优化**
```cpp
// 一次性构建,好坏全看运气
buildGraph();  // 静态连接
// 没有Rewire → 图质量差
```

**典型场景表现**:

**✅ 擅长**:
```
起点 ●               ● 终点
    🟦    🟦
       🟦    🟦
```
→ 多个分散障碍物,能检测多个corner,生成多条路径

**❌ 不擅长**:
```
起点 ●  🟦🟦🟦  ● 终点
        🟦🟦🟦
     (单一走廊)
```
→ 只有1条走廊,第1次找到后,阻塞中间点导致第2次失败

---

### 1.3 原始TGK (KRRT*)

**理论机制** (基于论文):

```cpp
// 步骤1: 动态扩展图 (Forward/Backward Reachability)
for (new_sample) {
    // BVP求解 - 动力学可达性
    if (bvp_solver.canReach(from, new_sample, max_time)) {
        tau_optimal = bvp_solver.getOptimalTime();
        
        // 动态连接半径 (自适应15-20m)
        radius = getBackwardRadius(tau_optimal, cost);
        
        // 连接所有可达节点
        for (node in nearbyNodes(radius)) {
            if (bvp_solver.canReach(node, new_sample)) {
                connect(node, new_sample);
            }
        }
    }
}

// 步骤2: Rewire优化 (动态改进图结构)
for (node in affected_nodes) {
    if (new_path_cost < old_path_cost) {
        change_parent(node);  // 重新连接父节点
    }
}

// 步骤3: 多路径提取 (基于动态图)
paths = extractKShortestPaths(graph, k=5);
```

**拓扑路径生成方式**:
1. **动态扩展**: 不断采样,增量式构建图 (RRT*思想)
2. **BVP验证**: 用动力学可达性替代几何检查 (容错性强)
3. **Rewire优化**: 持续改进图结构,连通性↑
4. **K-shortest**: 从优化后的图提取多条路径

**非同伦能力分析**:

| 维度 | 评估 | 说明 |
|------|------|------|
| **拓扑多样性** | ⭐⭐⭐⭐⭐ | 动态采样,理论上无限多样 |
| **全局探索** | ⭐⭐⭐⭐⭐ | RRT*探索整个空间 |
| **防局部最优** | ⭐⭐⭐⭐⭐ | 多路径+Rewire,最强 |
| **路径数量** | ⭐⭐⭐⭐⭐ | K-shortest,可靠生成K条 |

**理论优势**:

**优势1: BVP比几何检查容错**
```cpp
// 几何检查 (我们的TGK)
for (t=0; t<1; t+=0.3/length) {
    if (collision) return false;  // ❌ 一点碰撞就拒绝
}

// BVP检查 (原始TGK)
if (exists_velocity_corridor(from, to)) {
    return true;  // ✅ 只要动力学可行即可
}
```
→ 很多被我们拒绝的路径,原始TGK能接受

**优势2: Rewire改进连通性**
```cpp
// 初始图 (可能不连通)
start ●  corner1      corner2  ● goal
         (孤岛)

// Rewire后 (新采样点桥接)
start ●  corner1 -- bridge -- corner2  ● goal
                     (新点)
```
→ 自动修复图不连通问题

**优势3: 动态半径适应环境**
```cpp
// 密集环境
if (many_corners_nearby) {
    radius = 15m;  // 减小半径,避免过度连接
}

// 稀疏环境  
if (few_corners) {
    radius = 20m;  // 增加半径,保证连通性
}
```
→ 我们的固定20m在某些场景不足

---

## 2. 实际测试数据对比

### 2.1 Legacy四向 (历史数据)

**测试场景**: 随机森林,复杂度中等

| 指标 | 数值 |
|------|------|
| 成功率 | 100% |
| 多路径生成率 | ~80% (估计) |
| 平均路径数 | 3-5条 |
| 计算耗时 | ~5ms |

**分析**: 虽然拓扑多样性一般,但路径数量稳定

---

### 2.2 我们的TGK (test1.md数据)

**测试场景**: 随机森林,27次重规划

| 指标 | 数值 |
|------|------|
| TGK成功率 | 85% (23/27) |
| 多路径生成率 | **13%** (3/27) ❌ |
| 平均路径数 | 1.13条 |
| Legacy fallback | 15% (4/27) |
| 系统成功率 | 100% |

**分析**: 
- ✅ 全局探索能力强 (corner检测全面)
- ❌ **多路径生成失败** (87%情况只有1条路径)
- ❌ 图连通性问题 (15%完全失败)

---

### 2.3 原始TGK (理论预期)

**基于论文和算法设计**:

| 指标 | 预期 |
|------|------|
| 成功率 | >95% |
| 多路径生成率 | >60% |
| 平均路径数 | 3-5条 |
| 计算耗时 | ~10-20ms |

**分析**: 理论上最优,但实现复杂度高

---

## 3. 非同伦路径生成能力总结

### 3.1 排名 (按防局部最优能力)

**1. 原始TGK (KRRT*)** ⭐⭐⭐⭐⭐
- **原因**: 动态扩展+Rewire+BVP,最强全局探索
- **路径质量**: 多样且可靠 (理论>60%多路径)
- **防局部最优**: 最强 (多路径+动态优化)

**2. 我们的TGK (几何A*)** ⭐⭐⭐⭐
- **原因**: 全局corner检测,探索范围广
- **路径质量**: 高质量但数量不足 (仅13%多路径)
- **防局部最优**: 中等 (依赖Legacy兜底)

**3. Legacy四向** ⭐⭐⭐
- **原因**: 固定4方向,多样性一般
- **路径质量**: 数量稳定 (80%多路径)
- **防局部最优**: 基础 (能绕过简单障碍)

---

### 3.2 关键发现

**发现1: 我们的TGK有全局探索能力,但多路径生成失败**
```
全局探索 ✅ → Corner检测覆盖全场景
   ↓
图构建 ✅ → 静态全连接
   ↓
第1条路径 ✅ → A*成功 (85%)
   ↓
第2-4条路径 ❌ → 阻塞策略失败 (87%)
   ↓
最终: 只有1条路径 → MPPI无法并行对比
```

**发现2: Legacy四向路径数量稳定,但缺乏全局视野**
```
障碍物采样 ⚠️ → 只采样起点-终点连线附近
   ↓
4方向规避 ✅ → 每障碍物4条路径
   ↓
路径数量 ✅ → 稳定3-5条
   ↓
问题: 复杂环境探索不足 (可能错过远处绕行)
```

**发现3: 原始TGK理论最优,但实现难度大**
```
BVP求解 😰 → 需要实现动力学边值问题求解器
Rewire 😰 → 需要维护动态图结构
动态半径 😰 → 需要自适应逻辑
K-shortest 😰 → 需要保证拓扑多样性

预计开发时间: 2-3周
```

---

## 4. 明确方向: 改进建议

### 方案A: 增强我们的TGK (推荐★★★★★)

**目标**: 保留全局探索优势,修复多路径生成

**改进点**:

**1. 改进K-shortest paths算法** (优先级P0)
```cpp
// ❌ 当前: 简单阻塞中间点
block_idx = path1.size() / 2;
blocked_pos = path1[block_idx];

// ✅ 改进: 阻塞整条路径走廊
for (waypoint in path1) {
    block_waypoint(waypoint, radius=2m);  // 阻塞半径2m范围
}

// ✅ 改进: 路径相似度检查
if (similarity(path_new, path_existing) < 0.3) {  // 当前0.5→0.3
    accept(path_new);
}
```

**预期效果**: 多路径生成率 13% → 40%

**2. 桥接节点算法** (优先级P0)
```cpp
// 检测图不连通
if (!startCanSeeGoal(graph)) {
    // 在起点-终点视线上插入中间节点
    Vector3d bridge = (start + goal) / 2.0;
    if (isFree(bridge)) {
        addNode(bridge);
        connect(start, bridge);
        connect(bridge, goal);
    }
}
```

**预期效果**: TGK成功率 85% → 95%

**3. 动态调整连接半径** (优先级P1)
```cpp
// 自适应半径
if (corner_count < 10) {
    connection_radius = 25m;  // 稀疏环境增大
} else if (corner_count > 30) {
    connection_radius = 15m;  // 密集环境减小
} else {
    connection_radius = 20m;  // 默认
}
```

**预期效果**: 适应性提升,减少图断开

**开发时间**: 1-2周  
**风险**: 低  
**收益**: 高 (多路径40% + 成功率95%)

---

### 方案B: 混合Legacy+TGK (备选★★★)

**思路**: TGK负责全局探索,Legacy负责局部多样性

```cpp
// TGK全局规划
tgk_paths = TGK_search(start, goal);  // 1-2条高质量路径

// Legacy补充路径
if (tgk_paths.size() < 3) {
    legacy_paths = Legacy_search(start, goal);  // 补充到5条
    merge(tgk_paths, legacy_paths);
}

// 最终: 5条路径 (TGK 1-2条 + Legacy 3-4条)
```

**优势**:
- ✅ 路径数量稳定 (≥5条)
- ✅ 兼顾全局+局部

**劣势**:
- ⚠️ 代码复杂度高 (维护两套系统)
- ⚠️ Legacy路径质量一般

**开发时间**: 1周  
**风险**: 中  
**收益**: 中

---

### 方案C: 完整实现原始TGK (长期★★★★★)

**内容**:
1. 实现BVP求解器
2. 实现Rewire机制
3. 实现动态半径
4. 改进K-shortest

**优势**: 理论最优,长期收益大

**劣势**: 开发周期长 (2-3周)

**建议**: 作为长期目标,先执行方案A

---

## 5. 最终推荐方案

### 🎯 短期 (1-2周): 方案A - 增强我们的TGK

**具体任务**:
1. ✅ 改进K-shortest paths (阻塞走廊+相似度检查)
2. ✅ 桥接节点算法 (修复图不连通)
3. ✅ 增加max_corner_num到60
4. ✅ 动态连接半径 (自适应15-25m)

**预期结果**:
- TGK成功率: 85% → 95%
- 多路径生成率: 13% → 40%
- 平均路径数: 1.1 → 2.5条
- 删除Legacy依赖 ✅

**收益**: 
- ✅ 防局部最优能力大幅提升 (1条→2.5条路径给MPPI)
- ✅ 保留全局探索优势
- ✅ 开发成本低

---

### 🚀 中期 (1个月): 方案C前期准备

1. 调研BVP求解器实现
2. 设计Rewire算法
3. 测试动态图结构

---

### 📌 关键结论

**Q: 三种算法哪个最符合"用topo路径更广探索最优路径防止陷入局部最优"的目的？**

**A: 排名**
1. **原始TGK** (理论最优,但未实现)
2. **增强版TGK** (推荐方案A,1-2周可实现)
3. **当前TGK** (全局探索✅,多路径❌)
4. **Legacy四向** (路径稳定✅,全局探索❌)

**Q: 应该怎么改？**

**A: 立即行动**
1. 改进K-shortest paths算法 (核心!)
2. 桥接节点修复连通性
3. 动态连接半径
4. 增加corner数量限制

**目标**: 让我们的TGK从"能探索但只生成1条路径"变成"能探索且稳定生成3-5条路径"

---

**最后更新**: 2025-10-03  
**结论**: 方案A (增强TGK) 是当前最优选择! 🚀
