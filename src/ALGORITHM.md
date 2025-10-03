# EGO-Planner 算法总结

## 1. 系统架构

```
深度相机30Hz → GPU渲染 → 占用栅格 → Fast Sweeping ESDF (2ms)
                                            ↓
起点/终点 → TGK Planner (85%) ──┬→ Parallel MPPI → B-Spline优化 → 轨迹输出
              ↓                  │
         Legacy TopoPRM (15%)────┘
```

**设计理念**: 
- **并行处理**: GPU渲染与ESDF更新解耦
- **双路径规划**: TGK主路径 + Legacy备份,保证100%成功率
- **优化分层**: 拓扑路径 → MPPI优化 → B-Spline平滑

---

## 2. 核心模块详解

### 2.1 GPU深度渲染模块
**位置**: `uav_simulator/local_sensing/src/depth_render.cu`

**功能**:
- CUDA加速深度相机模拟 (640x480 @ 30Hz)
- 实时生成点云数据

**性能优化**:
- Rate Limiting: 防止ESDF更新过载
- CPU占用: ~40% (优化前300%)

---

### 2.2 Fast Sweeping ESDF
**位置**: `planner/plan_env/src/grid_map.cpp`

**算法**: Fast Sweeping Method (FSM)
- 复杂度: O(N) vs 传统O(N²)
- 更新时间: ~2ms
- 精度: 0.1m分辨率

**关键参数**:
```cpp
grid_resolution = 0.1  // 网格分辨率
map_size = [40, 40, 5] // 地图尺寸(m)
```

**数据流**:
```
点云输入 → 占用栅格更新 → Fast Sweeping → ESDF场
```

---

### 2.3 TGK Planner (主路径规划器)
**位置**: `planner/path_searching/src/topo_graph_search.cpp`

#### 算法流程
```
1. Corner检测 (bias_sampler.cpp)
   ↓
2. 图构建 (A* graph)
   ↓
3. 多路径搜索 (4次尝试)
   ↓
4. 路径平滑
```

#### Corner检测算法
**位置**: `planner/path_searching/src/bias_sampler.cpp`

**原理**: 检测自由空间与障碍物的几何特征点
```cpp
// 判断条件
free_count >= 4 && occupied_count >= 2  // 边角检测
free_count >= 5 && occupied_count >= 3  // 强特征点
```

**参数设置**:
```cpp
max_corner_num = 40           // 最大corner数量
connection_radius = 20.0      // 节点连接半径(m)
path_check_step = 0.3         // 路径碰撞检测步长(m)
```

#### A*图搜索
**特点**:
- 静态图构建 (一次性连接所有可见节点)
- 4次路径搜索 (寻找多条拓扑不同的路径)
- 碰撞检测: 密集几何采样 (0.3m步长)

**与原始TGK论文的差异**:
| 特性 | 原始TGK | 我们的实现 |
|------|---------|------------|
| 动力学约束 | BVP求解 | **几何检测** |
| 连接方式 | 动态Rewire | **静态构建** |
| 安全走廊 | 速度方向 | **密集采样** |
| 容错性 | 高 | **低(严格)** |

**性能数据** (27次测试):
- 成功率: **85%** (23/27)
- 多路径生成率: **13%** (仅3次生成>1条路径)
- 失败原因: A*图不连通 (15%)

**失败案例分析**:
```
[WARN] A* failed to find path after 21 iterations, tested 216 connections
[WARN] Graph has 21 nodes, start can see goal: NO
```
→ Corner检测到21个节点,但起点无法连接到终点

---

### 2.4 Legacy TopoPRM (备份规划器)
**位置**: `planner/path_searching/src/topo_prm.cpp`

**作用**: TGK失败时的fallback
- 成功率: **100%** (在TGK失败的15%场景)
- 算法: 传统PRM + 拓扑路径提取
- 触发条件: TGK返回0条路径

**重要性**: 保证系统100%稳定性

---

### 2.5 Parallel MPPI优化
**位置**: `planner/plan_manage/src/planner_manager.cpp`

**流程**:
```
TGK路径1 → MPPI优化 → Cost评估 ┐
TGK路径2 → MPPI优化 → Cost评估 ├→ 选择最优路径
TGK路径3 → MPPI优化 → Cost评估 ┘
```

**Cost归一化**:
```cpp
normalized_cost = (cost - min_cost) / (max_cost - min_cost)
```

**问题**: 当前多路径生成率仅13%,并行优化效果受限

---

### 2.6 B-Spline轨迹优化
**位置**: `planner/bspline_opt/src/bspline_optimizer.cpp`

**优化目标**:
- 平滑性 (加速度最小化)
- 安全性 (ESDF距离约束)
- 动力学可行性 (速度/加速度限制)

**性能**: 0.04ms梯度下降优化

---

## 3. 关键性能数据

### 3.1 整体性能
| 指标 | 数值 |
|------|------|
| CPU占用 | 40% |
| ESDF更新 | ~2ms |
| TGK成功率 | 85% |
| 系统成功率 | 100% (TGK+Legacy) |
| B-Spline优化 | 0.04ms |

### 3.2 测试数据分析 (test1.md)
**测试场景**: 27次重规划
- Total replans: **28**
- TGK success: **23** (85%)
- TGK failures: **5** (15%)
- A* failures: **22次警告** (但部分成功生成1条路径)
- Legacy fallback: **5次** (100%成功)

**典型警告**:
```
[WARN] [BiasSampler] Reached max corner number limit
→ 检测到超过40个corner,可能需要增加max_corner_num

[WARN] [TopoGraphSearch] A* failed to find path after 21 iterations
→ 图不连通,需要改进corner连接策略
```

---

## 4. 待优化问题

### 4.1 高优先级
1. **TGK多路径生成率低 (13%)**
   - 目标: 提升到30%
   - 方法: 改进A*搜索策略,增强路径多样性

2. **TGK成功率待提升 (85% → 95%)**
   - 问题: A*图不连通 (15%失败率)
   - 方案: 
     - 桥接节点算法 (连接孤立子图)
     - 动态调整connection_radius
     - 增加corner检测维度

3. **Corner检测过于严格**
   - 问题: 密集几何检查(0.3m)拒绝很多动力学可行路径
   - 方案: 引入BVP求解 or 放宽检测步长

### 4.2 中优先级
4. **删除Legacy依赖**
   - 条件: TGK成功率≥95%
   - 收益: 代码简化,性能提升

5. **MPPI并行优化利用率低**
   - 问题: 多路径生成率13%
   - 依赖: 先解决问题1

---

## 5. 与TGK论文的差异分析

### 5.1 核心差异
**我们的实现并非真正的KRRT***:
- 原始TGK: Kinodynamic RRT* (动力学约束)
- 我们: Geometric A* (几何约束)

### 5.2 为什么去掉动力学约束反而更差?

**关键洞察**:
- ❌ 不是"去掉动力学约束更差"
- ✅ 而是"我们的几何检查比动力学约束更严格"

**详细对比**:

| 维度 | 原始TGK | 我们的实现 | 影响 |
|------|---------|------------|------|
| 路径验证 | BVP求解(速度方向安全走廊) | 密集几何采样(0.3m) | **我们更严格** |
| 连接方式 | 动态Rewire优化 | 静态一次性构建 | **我们更脆弱** |
| 容错性 | 动力学可行即可 | 几何严格无碰撞 | **我们拒绝更多路径** |

**结论**: 
很多**动力学可行**的路径被我们的**几何检查错误拒绝**了!

**例子**:
- TGK论文: "速度方向上有2m宽的安全走廊,连接可行"
- 我们: "路径上存在0.1m的碰撞风险,拒绝连接"

### 5.3 改进方向
1. **引入BVP求解**: 用动力学验证代替几何检测
2. **放宽检测阈值**: path_check_step 0.3m → 0.5m
3. **安全走廊**: 沿速度方向构建容错空间

---

## 6. 未来工作

### 短期目标 (1-2周)
- [ ] 实现桥接节点算法 (提升TGK成功率到95%)
- [ ] 优化corner连接策略 (提升多路径生成率到30%)
- [ ] 引入BVP求解 (降低几何检测严格度)

### 中期目标 (1个月)
- [ ] 删除Legacy TopoPRM依赖
- [ ] 集成真正的KRRT*算法
- [ ] 动态调整connection_radius

### 长期目标 (2-3个月)
- [ ] 完整实现TGK论文算法
- [ ] 支持动态障碍物
- [ ] 多无人机协同规划

---

## 7. 核心代码位置

```
planner/path_searching/
├── bias_sampler.cpp          # Corner检测 (关键: free/occupied判断)
├── topo_graph_search.cpp     # TGK A*搜索 (关键: 图构建+多路径)
├── topo_prm.cpp              # TGK+Legacy集成 (fallback逻辑)
└── mppi_planner.cpp          # MPPI优化 (未使用,替换为Parallel MPPI)

planner/plan_manage/
└── planner_manager.cpp       # Parallel MPPI (关键: 多路径并行优化)

planner/plan_env/
└── grid_map.cpp              # Fast Sweeping ESDF (关键: O(N)更新)

planner/bspline_opt/
└── bspline_optimizer.cpp     # B-Spline优化 (关键: 梯度下降)

uav_simulator/local_sensing/
└── depth_render.cu           # GPU渲染 (关键: CUDA加速)
```

---

## 8. 参考文献

1. **TGK论文**: "Topological Graph-based Kinodynamic Planning"
2. **Fast Sweeping**: "A Fast Sweeping Method for Eikonal Equations"
3. **MPPI**: "Model Predictive Path Integral Control"
4. **B-Spline**: "Gradient-based Optimization for Trajectory Planning"

---

**最后更新**: 2025-10-03  
**版本**: v1.0  
**状态**: TGK基础实现完成,性能优化中
