# Ego-Planner 代码框架全面分析

**分析日期**: 2025年10月1日  
**基线版本**: d257634 (最后稳定版本)  
**目标**: 实现 TGK全局拓扑 + MPPI局部优化 + B样条平滑 的完整规划框架

---

## 📊 当前代码框架总览

### 核心模块关系图

```
EGOReplanFSM (状态机)
    ↓
EGOPlannerManager (规划管理器)
    ↓
    ├── TopoPRM (拓扑路径生成)
    ├── MPPIPlanner (局部轨迹优化) 
    ├── BsplineOptimizer (B样条优化器)
    │       ↓
    │   MPPIPlanner (内部集成)
    └── GridMap (环境地图)
```

---

## 🔍 详细模块分析

### 1. EGOReplanFSM - 有限状态机

**文件**: `planner/plan_manage/src/ego_replan_fsm.cpp`

**状态定义**:
```cpp
enum FSM_EXEC_STATE {
    INIT,            // 初始化
    WAIT_TARGET,     // 等待目标点
    GEN_NEW_TRAJ,    // 生成新轨迹
    REPLAN_TRAJ,     // 重新规划
    EXEC_TRAJ,       // 执行轨迹
    EMERGENCY_STOP   // 紧急停止
}
```

**职责**:
- 管理规划流程状态转换
- 接收航点和里程计数据
- 调用`PlannerManager`执行规划
- 发布轨迹到控制器

**关键函数**:
- `execFSMCallback()`: 主状态机循环
- `planGlobalTrajbyGivenWps()`: 按航点全局规划
- `checkCollision()`: 碰撞检测触发重规划

---

### 2. EGOPlannerManager - 规划管理器 ⭐核心⭐

**文件**: `planner/plan_manage/src/planner_manager.cpp`

**核心成员变量**:
```cpp
class EGOPlannerManager {
    GridMap::Ptr grid_map_;                    // 环境地图
    BsplineOptimizer::Ptr bspline_optimizer_rebound_;  // B样条优化器
    TopoPRM::Ptr topo_planner_;                // 拓扑路径规划器
    MPPIPlanner::Ptr mppi_planner_;            // MPPI轨迹优化器
    PlanningVisualization::Ptr visualization_; // 可视化
};
```

**主要规划流程** (`reboundReplan`):

```
STEP 1: 初始化路径
    ├─ 使用MinSnap生成初始轨迹
    └─ 参数化为B样条控制点

STEP 1.5: 拓扑规划 (已集成但未启用TGK)
    ├─ TopoPRM::searchTopoPaths()
    ├─ 生成多条候选拓扑路径
    └─ 选择最优路径更新控制点

STEP 2: B样条优化
    ├─ BsplineOptimizer::BsplineOptimizeTrajRebound()
    │   ├─ initControlPoints() 🔥BUG: 覆盖MPPI结果
    │   └─ LBFGS优化平滑度和避障
    └─ 返回优化后的控制点

STEP 2.5: MPPI局部优化 (已集成但效果不佳)
    └─ MPPIPlanner::planTrajectory()

STEP 3: 时间重分配
    └─ refineTrajAlgo() 确保动力学可行性
```

**已有接口**:
```cpp
// 主规划接口
bool reboundReplan(start_pt, start_vel, start_acc, 
                   end_pt, end_vel, ...);

// 拓扑规划接口
bool planWithTopo(start_pos, goal_pos, topo_paths);

// MPPI规划接口  
bool planWithMPPI(start_pos, start_vel, goal_pos, goal_vel, 
                  optimal_traj);
```

---

### 3. TopoPRM - 拓扑路径规划器

**文件**: `planner/path_searching/src/topo_prm.cpp`

**当前实现策略** (Fast-Planner风格):
```cpp
// 1. 直线路径
direct_path = {start, goal}

// 2. 检测障碍物
obstacles = detectObstaclesAlongLine(start, goal)

// 3. 生成绕障路径
for each obstacle:
    - generateCircularPath(left/right)   // 左右绕行
    - generateVerticalPath(over/under)   // 上下绕行
    - generateTangentPoints()            // 切线路径
```

**核心函数**:
```cpp
// 主接口
bool searchTopoPaths(start, goal, topo_paths)

// 路径生成
vector<TopoPath> findTopoPaths(start, goal)
    ├─ generateCircularPath()
    ├─ generateVerticalPath()
    └─ generateTangentPoints()

// 路径验证
bool isPathValid(path)
bool isLineCollisionFree(start, end)

// 成本计算
double calculatePathCost(path)
    ├─ calculateSmoothnessCost()
    └─ calculateObstacleCost()

// 路径选择
TopoPath selectBestPath(topo_paths)
```

**当前问题**:
- ❌ 硬编码策略（4个方向 + 切线），不智能
- ❌ 无角点检测，路径质量一般
- ❌ 成本函数简单，不考虑拓扑差异性

**TGK升级目标**:
- ✅ 使用BiasSampler进行角点检测
- ✅ TopoGraphSearch几何A*搜索
- ✅ 生成拓扑不同质的多条路径

---

### 4. MPPIPlanner - 局部轨迹优化器

**文件**: `planner/path_searching/src/mppi_planner.cpp`

**核心参数**:
```cpp
int num_samples_ = 1000;        // 采样轨迹数量
int horizon_steps_ = 20;        // 规划步数
double dt_ = 0.1;               // 时间步长
double lambda_ = 1.0;           // 温度参数

// 噪声参数
double sigma_pos_, sigma_vel_, sigma_acc_;

// 成本权重
double w_obstacle_;    // 障碍物权重 = 100.0
double w_smoothness_;  // 平滑度权重 = 10.0
double w_goal_;       // 目标权重 = 50.0
double w_velocity_;   // 速度权重 = 20.0
```

**规划流程**:
```cpp
bool planTrajectory(start_pos, start_vel, goal_pos, goal_vel, 
                    optimal_trajectory) {
    // 1. 生成N条随机轨迹
    for i = 0 to num_samples_:
        rolloutTrajectory(trajectory[i])
        cost[i] = calculateTrajectoryCost(trajectory[i])
    
    // 2. 计算重要性权重
    for each trajectory:
        weight = exp(-(cost - min_cost) / lambda_)
    
    // 3. 加权平均
    optimal_trajectory = weightedAverage(trajectories)
    
    return true
}
```

**rolloutTrajectory** (动力学模拟):
```cpp
void rolloutTrajectory(...) {
    for t = 1 to horizon_steps_:
        // 名义控制 (PD控制)
        nominal_acc = 2.0 * pos_error + 1.0 * vel_error
        
        // 添加噪声
        acc = nominal_acc + sigma_acc * noise()
        
        // 动力学约束
        constrainDynamics(vel, acc)
        
        // 前向积分
        vel[t] = vel[t-1] + acc * dt
        pos[t] = pos[t-1] + vel[t-1] * dt + 0.5 * acc * dt^2
}
```

**当前成本函数**:
```cpp
double calculateTrajectoryCost(trajectory) {
    cost = 0
    
    // 1. 障碍物成本 (O(n³) 暴力搜索) 🔥性能瓶颈
    for each position:
        cost += w_obstacle * obstacleCost(position)
    
    // 2. 平滑度成本
    cost += w_smoothness * smoothnessCost(trajectory)
    
    // 3. 目标成本
    cost += w_goal * goalCost(trajectory, goal)
    
    // 4. 速度成本
    cost += w_velocity * velocityCost(trajectory, desired_vel)
    
    return cost
}
```

**obstacleCost** (当前实现):
```cpp
double obstacleCost(position) {
    // 🔥 O(n³) 暴力搜索 - 需要优化为ESDF
    double min_dist = inf
    for dx in [-1.0, 1.0]:
        for dy in [-1.0, 1.0]:
            for dz in [-1.0, 1.0]:
                sample = position + (dx, dy, dz)
                if grid_map_->getInflateOccupancy(sample):
                    dist = norm(dx, dy, dz)
                    min_dist = min(min_dist, dist)
    
    return 1.0 / (min_dist + 0.1)  // 反距离成本
}
```

**ESDF升级后的obstacleCost**:
```cpp
double obstacleCost(position) {
    // ✅ O(1) ESDF查询
    double dist = grid_map_->evaluateEDT(position)
    
    if (dist < 0.2):
        return 1000.0  // 碰撞区域
    elif dist < 0.5:
        return 50.0 * ((0.5 - dist) / 0.3)^2  // 危险区域
    elif dist < 1.0:
        return 5.0 * ((1.0 - dist) / 0.5)  // 接近区域
    else:
        return 0.0  // 安全区域
}
```

---

### 5. BsplineOptimizer - B样条优化器

**文件**: `planner/bspline_opt/src/bspline_optimizer.cpp`

**核心功能**:
```cpp
class BsplineOptimizer {
    MPPIPlanner::Ptr mppi_planner_;  // 内部集成MPPI
    
    // 主优化接口
    bool BsplineOptimizeTrajRebound(ctrl_pts, ts);
    
    // 🔥BUG: 初始化控制点
    vector<vector<Vector3d>> initControlPoints(init_points, flag_first_init);
    
    // 成本函数 (LBFGS优化)
    void combineCost(x, grad, cost_function);
    
    // 成本组件
    void calcSmoothnessCost(...);
    void calcDistanceCost(...);
    void calcFeasibilityCost(...);
};
```

**优化流程**:
```cpp
bool BsplineOptimizeTrajRebound(ctrl_pts, ts) {
    // 1. 初始化控制点 🔥BUG: 覆盖MPPI结果
    a_star_pathes = initControlPoints(ctrl_pts, true)
    
    // 2. LBFGS优化
    lbfgs::lbfgs_optimize(
        x,                    // 控制点
        final_cost,
        combineCost,          // 成本函数
        nullptr,              // 监视器
        nullptr,              // 进度回调
        lbfgs_params
    )
    
    // 3. 回弹检测 (碰撞检测 + 重新优化)
    if (collisionDetected):
        rebound()
    
    return success
}
```

**initControlPoints的BUG** 🔥:
```cpp
vector<vector<Vector3d>> initControlPoints(init_points, flag_first_init) {
    // 1. 扫描控制点检测碰撞
    for i in range(init_points):
        if (collision detected at i):
            collision_points.push_back(i)
    
    // 2. 对碰撞段进行A*搜索
    for each collision_segment:
        a_star_path = searchPath(start, end)
    
    // 3. 🔥问题: 用线性插值重新生成控制点
    //    这会覆盖MPPI精心优化的动力学轨迹！
    for each waypoint in a_star_path:
        linear_interpolate(waypoints)
    
    // 4. 更新控制点 (丢失MPPI结果)
    cps_.points = linear_interpolated_points
    
    return a_star_pathes
}
```

**修复方案**:
```cpp
// 方案1: 直接使用MPPI结果
if (optimal_points.size() > 0):
    setControlPoints(optimal_points)  // 不调用initControlPoints

// 方案2: 条件初始化
if (cps_.points.empty()):
    initControlPoints(optimal_points)  // 仅首次初始化
else:
    setControlPoints(optimal_points)   // 后续直接使用
```

---

### 6. GridMap - 环境地图

**文件**: `planner/plan_env/src/grid_map.cpp`

**当前功能**:
```cpp
class GridMap {
    // 占据查询
    int getOccupancy(pos);
    int getInflateOccupancy(pos);  // 膨胀后的占据
    
    // 地图管理
    void initMap(nh);
    void resetBuffer();
    
    // 数据结构
    vector<double> occupancy_buffer_;        // 占据概率
    vector<char> occupancy_buffer_inflate_;  // 膨胀占据
    
    // 参数
    double resolution_;           // 分辨率
    double obstacles_inflation_;  // 膨胀半径
};
```

**当前限制**:
- ❌ 无ESDF (Euclidean Signed Distance Field)
- ❌ 障碍物距离查询需要O(n³)暴力搜索
- ❌ 无梯度信息，无法计算斥力

**ESDF升级需求**:
```cpp
class GridMap {
    // 新增: ESDF查询接口
    double evaluateEDT(const Vector3d& pos);
    
    void evaluateEDTWithGrad(const Vector3d& pos,
                            double& dist,
                            Vector3d& grad);
    
private:
    // 可选: ESDF缓存 (如果需要预计算)
    vector<double> edt_buffer_;
};
```

---

## 🎯 升级目标架构

### 理想的规划流程

```
┌─────────────────────────────────────────────────────┐
│ STEP 1: TGK 全局拓扑路径生成                          │
├─────────────────────────────────────────────────────┤
│ BiasSampler                                          │
│   ├─ 检测环境中的角点/关键点                           │
│   └─ 生成拓扑偏置采样点                                │
│                                                      │
│ TopoGraphSearch                                      │
│   ├─ 在关键点之间进行几何A*搜索                         │
│   ├─ 生成3-5条拓扑不同质的全局路径                      │
│   └─ 按成本排序，选择最优路径                          │
│                                                      │
│ 输出: topo_paths[] = [path1, path2, path3, ...]     │
└─────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────┐
│ STEP 2: MPPI 局部轨迹优化 (for each topo_path)       │
├─────────────────────────────────────────────────────┤
│ 对每条拓扑路径:                                        │
│                                                      │
│ MPPIPlanner::planTrajectory()                        │
│   ├─ 沿拓扑路径生成N=1000条随机轨迹                    │
│   ├─ 使用ESDF计算障碍物成本 (O(1))                    │
│   ├─ 添加ESDF梯度引导的斥力                           │
│   ├─ 考虑动力学约束 (速度/加速度限制)                   │
│   └─ 加权平均得到最优轨迹                             │
│                                                      │
│ 输出: mppi_trajectories[] = [traj1, traj2, ...]     │
└─────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────┐
│ STEP 3: 选择最优MPPI轨迹                              │
├─────────────────────────────────────────────────────┤
│ best_traj = selectBestTrajectory(mppi_trajectories) │
│                                                      │
│ 选择标准:                                             │
│   ├─ 总成本最低                                       │
│   ├─ 无碰撞                                          │
│   └─ 动力学可行                                       │
└─────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────┐
│ STEP 4: B样条最终平滑                                 │
├─────────────────────────────────────────────────────┤
│ BsplineOptimizer::BsplineOptimizeTrajRebound()       │
│   ├─ 使用best_traj的航点作为控制点                     │
│   ├─ ✅ 修复: 不调用initControlPoints (避免覆盖)      │
│   ├─ LBFGS优化平滑度                                  │
│   ├─ ESDF碰撞检测                                    │
│   └─ 时间重分配确保动力学可行性                         │
│                                                      │
│ 输出: final_bspline_trajectory                       │
└─────────────────────────────────────────────────────┘
```

---

## 📊 可视化需求

### RViz可视化主题

```
/topo_paths              # 拓扑路径 (多条候选路径)
    ├─ path_0 (blue)     # 最优路径
    ├─ path_1 (green)    # 次优路径
    └─ path_2 (yellow)   # 第三路径

/mppi_trajectories       # MPPI采样轨迹 (灰色半透明)
    └─ 1000条轨迹线

/mppi_optimal_trajectory # MPPI最优轨迹 (红色粗线)

/final_bspline          # 最终B样条轨迹 (青色实线)

/topo_key_points        # TGK角点 (红色球体)

/esdf_gradient_field    # ESDF梯度场 (箭头, 可选)
```

---

## 🔍 关键代码位置总结

| 模块 | 头文件 | 源文件 | 行数 | 说明 |
|------|--------|--------|------|------|
| **EGOReplanFSM** | `plan_manage/ego_replan_fsm.h` | `plan_manage/ego_replan_fsm.cpp` | ~600 | 状态机 |
| **EGOPlannerManager** | `plan_manage/planner_manager.h` | `plan_manage/planner_manager.cpp` | ~639 | 规划管理器 |
| **TopoPRM** | `path_searching/topo_prm.h` | `path_searching/topo_prm.cpp` | ~520 | 拓扑规划 |
| **MPPIPlanner** | `path_searching/mppi_planner.h` | `path_searching/mppi_planner.cpp` | ~472 | MPPI优化 |
| **BsplineOptimizer** | `bspline_opt/bspline_optimizer.h` | `bspline_opt/bspline_optimizer.cpp` | ~1100 | B样条优化 |
| **GridMap** | `plan_env/grid_map.h` | `plan_env/grid_map.cpp` | ~800 | 环境地图 |

### 规划调用链

```
ego_replan_fsm.cpp::execFSMCallback()
    ↓ (line ~280)
planner_manager.cpp::reboundReplan()
    ↓ (line ~245)
topo_prm.cpp::searchTopoPaths()
    ↓ (line ~320)
planner_manager.cpp::planWithMPPI()
    ↓ (line ~617)
mppi_planner.cpp::planTrajectory()
    ↓ (line ~350)
planner_manager.cpp::BsplineOptimizeTrajRebound()
    ↓ (line ~757)
bspline_optimizer.cpp::initControlPoints() 🔥BUG
```

---

## 🚨 已识别问题清单

### 高优先级 🔥

1. **BsplineOptimizer::initControlPoints覆盖MPPI结果**
   - 位置: `bspline_optimizer.cpp` line ~757
   - 影响: 飞行质量严重下降
   - 修复: 改用`setControlPoints()`直接使用MPPI结果

2. **MPPI障碍物成本O(n³)性能瓶颈**
   - 位置: `mppi_planner.cpp::obstacleCost()` line ~178
   - 影响: 规划速度慢
   - 修复: 添加ESDF查询，O(1)时间

3. **TopoPRM硬编码策略不智能**
   - 位置: `topo_prm.cpp::findTopoPaths()` line ~63
   - 影响: 路径质量一般，无法处理复杂环境
   - 修复: 集成TGK算法 (BiasSampler + TopoGraphSearch)

### 中优先级

4. **GridMap缺少ESDF功能**
   - 需要添加: `evaluateEDT()` 和 `evaluateEDTWithGrad()`
   
5. **MPPI缺少梯度引导**
   - 需要添加: ESDF梯度引导的斥力

6. **可视化不完整**
   - 拓扑路径未显示
   - MPPI采样轨迹未显示

---

## 📝 数据流分析

### 关键数据结构

```cpp
// 拓扑路径
struct TopoPath {
    vector<Vector3d> path;    // 航点序列
    double cost;              // 路径成本
    int path_id;              // 路径ID
};

// MPPI轨迹
struct MPPITrajectory {
    vector<Vector3d> positions;      // 位置序列
    vector<Vector3d> velocities;     // 速度序列
    vector<Vector3d> accelerations;  // 加速度序列
    double cost;                     // 轨迹成本
    double weight;                   // 重要性权重
};

// B样条控制点
Eigen::MatrixXd ctrl_pts;  // 3 x N 矩阵
```

### 数据流转

```
Waypoints (航点)
    ↓
TopoPath[] (拓扑路径)
    ↓
MPPITrajectory[] (MPPI轨迹)
    ↓
Eigen::MatrixXd (B样条控制点)
    ↓
UniformBspline (最终轨迹)
```

---

## 🎯 总结

### 当前架构优点
- ✅ 模块化设计清晰
- ✅ 已有TopoPRM和MPPI集成
- ✅ B样条优化器成熟
- ✅ 可视化框架完整

### 当前架构缺陷
- ❌ BsplineOptimizer的initControlPoints覆盖MPPI结果
- ❌ MPPI障碍物成本O(n³)太慢
- ❌ TopoPRM策略硬编码不智能
- ❌ 缺少ESDF距离场支持
- ❌ 缺少梯度引导的避障

### 升级路线图
1. **阶段1**: 修复BsplineOptimizer bug
2. **阶段2**: 添加ESDF到GridMap
3. **阶段3**: 升级MPPI使用ESDF
4. **阶段4**: 集成TGK拓扑算法
5. **阶段5**: 完善可视化系统

---

**下一步**: 参考 `DETAILED_IMPLEMENTATION_PLAN.md` 查看详细实施步骤
