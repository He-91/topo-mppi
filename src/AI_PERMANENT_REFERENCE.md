# AI永久参考信息

本文件用于长期记录项目重要信息、决策、约定、特殊参数、历史问题与解决方案等，供AI和开发者随时查阅。

---

## 1. 项目目标
- 实现高鲁棒性、多样性、可实时的无人机三层路径规划系统（TopoPRM+MPPI+B-spline）。

## 2. 重要约定
- 只保留README.md和test1.md，其余文档可随时删除。
- test1.md专用于存放测试日志和数据。
- 本文件内容应长期保留，供AI和开发者参考。

## 3. 关键参数与经验
- TopoPRM切线点安全距离建议：动态估算障碍物半径+1.0m。
- 碰撞检测分辨率建议：0.1~0.15m。
- MPPI采样数建议：300~500。
- B-spline rebound次数理想值：<5。

## 4. 历史问题与解决方案

### 问题1：TopoPRM多路径生成率0% (2025-10-04)
**症状**：test1.md显示56次规划，全部只生成1条路径，0%触发MPPI
**根本原因**：
- 8方向代码存在，但每个障碍物只生成0-2个切线点（应该8个）
- 100%切线点因`start→tangent=COLLISION`被拒绝
- `estimateObstacleSize() + 1.0m`安全余量不足
- `path_length_limit = 2.0x`过于严格

**解决方案（激进优化v2.0）**：
```cpp
// 1. 安全余量：1.0m → 3.0m
double safety_margin = 3.0;

// 2. 最小绕行距离：7.5m (search_radius * 1.5)
double min_avoidance = search_radius_ * 1.5;
avoidance_radius = max(obstacle_radius + 3.0, 7.5);

// 3. 路径长度限制：2.0x → 3.5x
if (path_length > direct_dist * 3.5) reject;

// 4. 碰撞检测精度：0.15m → 0.2m
collision_check_resolution_ = 0.2;
```

**预期效果**：多路径生成率从0%提升到30-60%

### 问题2：MPPI未触发（历史）
- 已通过动态安全距离、8方向切线、放宽碰撞检测等优化。
- MPPI未触发：需保证topo_paths.size()>1。

## 5. 重要决策记录
- 任何影响全局行为的参数/策略变更，建议在此处补充说明。

---

> 本文件为AI与开发者的长期知识库，内容可随时补充、修订。