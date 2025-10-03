# EGO-Planner with ESDF-MPPI Upgrade

## 核心目标

**提升多拓扑路径生成率**：从13%提升到40%+，为MPPI提供拓扑多样化的初始路径，防止陷入局部最优


## 关键发现（基于test1分析）

### ✅ 已成功：多路径生成率提升
- 成功生成2-3条路径（相似度0.467-0.615）
- 走廊感知阻塞策略生效
- 中间waypoint相似度比较准确

### ❌ 核心问题：Topo路径偏离target太远

**现象**：
```
Path 1: topo_cost=47.9,  waypoints=55   (短路径)
Path 2: topo_cost=80.6,  waypoints=133  (绕行路径)
```

**根本原因**：过度插值
- A*找到的稀疏路径：3-5个waypoint
- planner_manager.cpp插值后：55-133个waypoint
- 插值逻辑：`num = segment_len / (0.4 * 0.5)` → 5米段插25个点

**影响**：
- Topo路径本应是稀疏引导（3-5个转折点）
- 变成密集轨迹后，MPPI优化时偏离原始拓扑意图
- 绕行路径过长，成本过高


## 优化方案

### A. 减少Topo路径插值密度 ⭐
**问题代码** (`planner_manager.cpp:377`):
```cpp
int num_intermediate = std::max(1, (int)(segment_len / (pp_.ctrl_pt_dist * 0.5)));
// ctrl_pt_dist = 0.4米
// 5米路径段 → 25个插值点
```

**优化选项**:
```cpp
// 选项1：稀疏插值（推荐）
int num_intermediate = std::max(0, (int)(segment_len / (pp_.ctrl_pt_dist * 4.0)));
// 5米路径段 → 3个插值点

// 选项2：固定稀疏
int num_intermediate = (segment_len > 3.0) ? 1 : 0;
// 每段最多1个中间点
```

**预期效果**:
- Waypoints: 55 → 10以内
- Topo路径保持稀疏引导特性
- MPPI可以更好地优化局部细节

### B. 改用走廊序列判断路径差异 ⭐⭐
**当前问题**：
- 使用Hausdorff距离比较密集waypoint
- 阈值难以调优（0.65）

**新策略**（用户建议）：
> "只要绕行方式不同应该就算一条新路径"

```cpp
bool isPathDifferent(const vector<int>& corridors1, 
                     const vector<int>& corridors2) {
    // 走廊序列不同 = 拓扑不同
    return corridors1 != corridors2;
}
```

**示例**:
```
Path 1: [0]           → 直线
Path 2: [-5, 0]       → 左绕
Path 3: [+5, 0]       → 右绕  
Path 4: [-10, -5, 0]  → 大幅左绕
→ 全部保留！
```

### C. 移除2节点路径拒绝（FIX 10回退）
**原因**：
- 起点→终点直线是最短路径baseline
- 应保留用于对比绕行路径
- 走廊序列比较会自然区分 `[0]` vs `[-5,0]`

### D. 调整相似度阈值（如果保留Hausdorff）
```cpp
// 当前: 0.65
// 建议: 0.5（允许更多路径）
```

