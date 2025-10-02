# TGK 失败率分析与改进方案 🚨

## 📊 实际测试数据 (test1.md)

### 严峻的现实
- **总规划周期**: 27次
- **TGK 成功**: 14次 (51.9%)
- **TGK 失败**: 13次 (48.1%)
- **多路径生成**: 仅1次生成2条路径，其余全是1条

### 问题严重性评估
**用户的质疑完全正确**: 如果 TGK 失败率接近 50%，那写 TGK 的意义确实不大！

**现状**:
```
TGK (52%成功，且多数只有1条路径) ────┐
                                    ├─→ Legacy 接管 48%
Legacy (100%成功) ───────────────────┘
```

**问题**: Legacy 承担了近一半的工作，TGK 没有发挥应有的作用！

---

## 🔍 根本原因分析

### 问题 1: A* Search 成功率太低

从日志中可以看到大量：
```
[TopoGraphSearch] A* search: 22 nodes, start_id=0, goal_id=21
[WARN] [TopoGraphSearch] A* failed to find path
```

**原因诊断**:

#### 1.1 Graph Connectivity 不足 ⭐⭐⭐⭐⭐ (最可能)
**当前设置**:
```cpp
double connection_radius_ = 10.0;  // topo_graph_search.cpp
```

**问题**: 
- Corner 之间距离通常 7-15m
- 10.0m 的连接半径刚好在边界，很多节点连不上
- 导致图不连通，A* 失败

**证据**: 日志显示 22 nodes，但大量连接失败

#### 1.2 Corner Detection 质量不够 ⭐⭐⭐⭐
**当前设置**:
```cpp
int max_corner_num_ = 20;  // bias_sampler.cpp
```

**问题**:
- 20 个 corner 在复杂环境中可能不够
- Corner 分布不均匀，某些关键区域缺失
- 导致图的覆盖率不足

#### 1.3 `canConnect()` 检查过于严格 ⭐⭐⭐
**当前实现**:
```cpp
bool canConnect(pos1, pos2) {
    return isPathFree(pos1, pos2);  // 碰撞检测
}
```

**问题**:
- `isPathFree` 使用 step=0.2m，在狭窄空间仍可能太严格
- 障碍物膨胀半径如果过大，会导致很多连接被拒绝

### 问题 2: 多路径生成率极低 (7%)

**统计**: 14 次成功中，只有 1 次生成 2 条路径

**原因**:
- Blocked node strategy 太保守
- 路径相似度判断太严格
- K-Shortest Paths 简化版算法效果不佳

---

## 🎯 改进方案（按优先级）

### 方案 A: 快速修复 - 放宽连接条件 ⭐⭐⭐⭐⭐ (强烈推荐)

**目标**: 将 TGK 成功率从 52% 提升到 >80%

#### Step 1: 增加 Connection Radius (最重要！)
```cpp
// topo_graph_search.cpp
// 当前: double connection_radius_ = 10.0;
double connection_radius_ = 15.0;  // 增加 50%
```

**预期效果**: 
- 更多节点能够连接
- 图连通性大幅提升
- A* 成功率应该能提升到 70-80%

**风险**: 可能会产生一些低质量连接（可接受）

#### Step 2: 增加 Corner 数量上限
```cpp
// bias_sampler.cpp
// 当前: int max_corner_num_ = 20;
int max_corner_num_ = 30;  // 增加 50%
```

**预期效果**:
- 更密集的 corner 覆盖
- 更好的图连通性
- 更多路径选择

**风险**: 略微增加计算量（可接受）

#### Step 3: 放宽路径检查步长
```cpp
// topo_graph_search.cpp - isPathFree()
// 当前: step = 0.2;
step = 0.3;  // 或者 0.25
```

**预期效果**: 
- 更宽松的连接判断
- 更多有效连接

**风险**: 可能有更多擦边球路径（MPPI会优化掉）

---

### 方案 B: 改进多路径生成 ⭐⭐⭐⭐

**当前问题**: K-Shortest Paths 简化版效果差

#### 改进策略 1: 调整 Blocked Node 参数
```cpp
// topo_graph_search.cpp line 261
// 当前: if (min_dist > 3.0) continue;
if (min_dist > 5.0) continue;  // 放宽搜索范围
```

#### 改进策略 2: 降低路径相似度阈值
```cpp
// topo_graph_search.cpp - arePathsSimilar()
// 当前可能太严格，导致很多路径被判定为"相似"而被丢弃
```

#### 改进策略 3: 增加尝试次数
```cpp
// topo_graph_search.cpp line 244
// 当前: int max_paths = max_topo_paths_;
int max_attempts = max_topo_paths_ * 2;  // 增加尝试次数
```

---

### 方案 C: 完整重构 ⭐⭐ (不推荐，代价太高)

**如果方案A+B都无效**，考虑：
1. 实现完整的 Yen's K-Shortest Paths 算法
2. 改用 Lazy Theta* 或 JPS 替代 A*
3. 改进 Corner Detection 使用 ESDF gradient

**工作量**: 10-20 小时
**风险**: 高，可能引入新bug

---

## 📋 立即执行方案（推荐）

### 快速实验：Connection Radius 15.0m

**理由**:
- 改动最小（1行代码）
- 预期效果最明显
- 零风险

**步骤**:

1. **修改参数**:
```cpp
// planner/path_searching/src/topo_graph_search.cpp
// Line 16 (init函数)
connection_radius_ = 15.0;  // 从 10.0 改为 15.0
```

2. **编译测试**:
```bash
cd /home/he/ros_ws/test/ego-planner
catkin_make -j4
source devel/setup.bash
roslaunch plan_manage run_in_sim.launch
```

3. **观察指标**:
- TGK 成功率应该从 52% → >75%
- 多路径生成率应该有所提升
- A* "failed" 警告应该明显减少

4. **如果仍不够**:
- 继续增加到 20.0m
- 或者叠加 Step 2 (max_corner_num = 30)

---

## 💡 保守策略：如果快速修复仍不够

### 选项 1: 接受现状，继续用 Legacy
**理由**:
- Legacy 100% 可靠
- 双层架构稳定
- TGK 作为"优化尝试"，失败了就用 Legacy

**问题**: 
- ❌ 违背项目初衷（想删除 Legacy）
- ❌ TGK 投入产出比低

### 选项 2: 放弃 TGK，专注优化 Legacy
**理由**:
- Legacy 已经很好用
- 不如把精力用在优化 Legacy 上

**问题**: 
- ❌ 前期 TGK 开发全部浪费
- ❌ 失去拓扑多样性的优势

### 选项 3: 坚持优化 TGK 到成功率 >90%
**理由**:
- TGK 理论上更优（全局视角、多路径）
- 当前失败率是实现问题，不是算法问题

**工作量**: 
- 快速修复: 1-2 小时
- 深度优化: 4-8 小时

---

## 🎯 我的建议

### 立即尝试方案 A - Step 1 (connection_radius = 15.0)

**执行**:
1. 修改 1 行代码
2. 测试 10 分钟
3. 看效果

**预期**:
- 如果成功率提升到 >75%，继续优化
- 如果仍然 <60%，执行 Step 2+3
- 如果全部失败，重新评估 TGK 架构

**时间成本**: 30 分钟 - 1 小时

---

## ⚠️ 严峻的现实

**用户的质疑是对的**: 
- 当前 TGK 成功率 52%，确实太低
- Legacy 承担了 48% 的工作，TGK 没有体现价值
- 如果无法提升到 >80%，确实应该考虑放弃 TGK

**项目价值重估**:
- ✅ Parallel MPPI: 有价值（证明了拓扑≠动力学最优）
- ✅ 系统稳定性: 有价值（100%成功率）
- ⚠️ TGK: **当前价值存疑**（成功率不足，多路径生成差）

**建议**: 
1. **先尝试快速修复**（connection_radius = 15.0）
2. **如果成功率仍 <70%，考虑放弃 TGK**
3. **专注于优化 Legacy + Parallel MPPI 组合**

---

要不要现在就试试 connection_radius = 15.0？只需要改 1 行代码。
