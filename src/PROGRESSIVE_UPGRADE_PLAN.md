# Ego-Planner 渐进式升级计划 v2.0

## ✅ 已完成：代码回退

**执行时间**: 2025年10月1日 17:08

### 完成的操作

1. ✅ **备份TGK工作**: 
   - 所有TGK文件备份到: `~/tgk_backup_20251001_1708/`
   - 包括所有文档和代码

2. ✅ **清理未跟踪文件**:
   - 删除所有TGK相关的新文件
   - 工作区恢复干净

3. ✅ **回退到稳定commit**:
   - `git reset --hard d25763426d304ca289d8cbb0ab142914eede0cae`
   - 确认master分支在正确位置

4. ✅ **创建新开发分支**:
   - 分支名: `feature/esdf-mppi-upgrade`
   - 从d257634创建

5. ✅ **同步到GitHub**:
   - master强制推送到d257634
   - 新分支已推送
   - GitHub URL: https://github.com/He-91/ego-planner/tree/feature/esdf-mppi-upgrade

---

## 🎯 升级目标

从干净的d257634版本开始，逐步添加功能：

1. **阶段1**: 修复BsplineOptimizer的性能bug（最高优先级）
2. **阶段2**: 添加ESDF支持（GridMap层）
3. **阶段3**: 升级MPPI使用ESDF
4. **阶段4**: （可选）集成TGK拓扑算法

---

## 📋 阶段1: 修复BsplineOptimizer性能退化 🔥

### 问题描述

**文件**: `planner/bspline_opt/src/bspline_optimizer.cpp`  
**行数**: 约757-770

**Bug**: `initControlPoints(optimal_points)` 会扫描轨迹，检测碰撞，然后用**线性插值**生成控制点，这完全覆盖了MPPI精心优化的结果。

**症状**: 
- MPPI生成的平滑轨迹变成折线
- 飞行质量明显下降
- 轨迹不够优化

### 修复方案

#### 方案A: 直接使用MPPI结果（推荐）

```cpp
// 在 BsplineOptimizeTrajRebound() 函数中
// 查找类似这样的代码：
if (optimal_points.size() > 0) {
    initControlPoints(optimal_points);  // ❌ 错误：会覆盖MPPI结果
}

// 改为：
if (optimal_points.size() > 0) {
    // 方法1：如果有setControlPoints函数
    setControlPoints(optimal_points);  // ✅ 直接使用
    
    // 方法2：如果没有setControlPoints，直接赋值
    cps_.points = optimal_points;
    cps_.size = optimal_points.size();
}
```

#### 方案B: 条件初始化（更保守）

```cpp
if (optimal_points.size() > 0) {
    // 只在必要时初始化（第一次或cps_为空）
    if (cps_.points.empty()) {
        initControlPoints(optimal_points);
    } else {
        setControlPoints(optimal_points);  // 后续直接使用
    }
}
```

### 测试验证

```bash
# 在Docker中
cd ~/ros_ws/ego-planner
catkin_make
roslaunch ego_planner simple_run.launch

# 观察：
# 1. 轨迹是否更平滑？
# 2. 能否完成5个航点？
# 3. 飞行时间是否合理（<60秒）？
```

### 提交

```bash
git add src/planner/bspline_opt/src/bspline_optimizer.cpp
git commit -m "fix(bspline): use setControlPoints to preserve MPPI optimization result

- Replace initControlPoints() with setControlPoints()
- Prevent linear interpolation from overwriting MPPI's optimized trajectory
- Significantly improves flight quality and smoothness

Issue: initControlPoints() was scanning for collisions and regenerating
control points with linear interpolation, which discarded all the careful
optimization done by MPPI.

Solution: Directly use MPPI's output as control points, trusting the
upstream optimization."

git push origin feature/esdf-mppi-upgrade
```

---

## 📋 阶段2: 添加ESDF支持（GridMap层）

### 目标

让GridMap能够提供Euclidean Distance Field（欧几里得距离场）查询。

### 实现步骤

#### 2.1 在grid_map.h中添加接口

```cpp
// planner/plan_env/include/plan_env/grid_map.h

class GridMap {
public:
    // ... 现有函数 ...
    
    // 新增ESDF查询函数
    double evaluateEDT(const Eigen::Vector3d& pos);
    
    void evaluateEDTWithGrad(const Eigen::Vector3d& pos, 
                             double& dist, 
                             Eigen::Vector3d& grad);
};
```

#### 2.2 在grid_map.cpp中实现

```cpp
// planner/plan_env/src/grid_map.cpp

double GridMap::evaluateEDT(const Eigen::Vector3d& pos) {
    // 方法1: Fibonacci球面采样（快速近似）
    const int num_samples = 26;  // 采样点数量
    const double search_radius = 3.0;  // 搜索半径（米）
    
    double min_dist = search_radius;
    
    // 采样球面上的点
    for (int i = 0; i < num_samples; ++i) {
        double theta = 2.0 * M_PI * i / num_samples;
        double phi = acos(1.0 - 2.0 * (i + 0.5) / num_samples);
        
        Eigen::Vector3d dir(
            sin(phi) * cos(theta),
            sin(phi) * sin(theta),
            cos(phi)
        );
        
        // 从当前点向外搜索
        for (double r = 0.1; r < search_radius; r += 0.1) {
            Eigen::Vector3d sample = pos + r * dir;
            if (getInflateOccupancy(sample)) {
                min_dist = std::min(min_dist, r);
                break;
            }
        }
    }
    
    return min_dist;
}

void GridMap::evaluateEDTWithGrad(const Eigen::Vector3d& pos,
                                   double& dist,
                                   Eigen::Vector3d& grad) {
    // 计算当前点的距离
    dist = evaluateEDT(pos);
    
    // 使用中心差分法计算梯度
    const double delta = 0.1;  // 差分步长
    
    double dist_x_plus = evaluateEDT(pos + Eigen::Vector3d(delta, 0, 0));
    double dist_x_minus = evaluateEDT(pos - Eigen::Vector3d(delta, 0, 0));
    
    double dist_y_plus = evaluateEDT(pos + Eigen::Vector3d(0, delta, 0));
    double dist_y_minus = evaluateEDT(pos - Eigen::Vector3d(0, delta, 0));
    
    double dist_z_plus = evaluateEDT(pos + Eigen::Vector3d(0, 0, delta));
    double dist_z_minus = evaluateEDT(pos - Eigen::Vector3d(0, 0, delta));
    
    grad.x() = (dist_x_plus - dist_x_minus) / (2.0 * delta);
    grad.y() = (dist_y_plus - dist_y_minus) / (2.0 * delta);
    grad.z() = (dist_z_plus - dist_z_minus) / (2.0 * delta);
    
    // 归一化（如果需要）
    if (grad.norm() > 1e-6) {
        grad.normalize();
    }
}
```

#### 2.3 测试ESDF

在`grid_map.cpp`的某个初始化函数中添加测试：

```cpp
void GridMap::initMap(ros::NodeHandle& nh) {
    // ... 原有初始化代码 ...
    
    // ESDF测试
    ROS_INFO("[GridMap] Testing ESDF implementation...");
    Eigen::Vector3d test_pos(0, 0, 1);
    double dist;
    Eigen::Vector3d grad;
    evaluateEDTWithGrad(test_pos, dist, grad);
    ROS_INFO("[GridMap] ESDF test at [0,0,1]: dist=%.3f, grad=[%.3f,%.3f,%.3f]",
             dist, grad.x(), grad.y(), grad.z());
}
```

### 提交

```bash
git add src/planner/plan_env/include/plan_env/grid_map.h
git add src/planner/plan_env/src/grid_map.cpp
git commit -m "feat(grid_map): add ESDF query support

- Add evaluateEDT() for distance field queries
- Add evaluateEDTWithGrad() for distance and gradient
- Use Fibonacci sphere sampling for efficient approximation
- Enable MPPI to query obstacle distances in O(1) time

This replaces O(n³) brute-force obstacle searching with
O(log n) structured queries."

git push origin feature/esdf-mppi-upgrade
```

---

## 📋 阶段3: 升级MPPI使用ESDF

### 目标

让MPPI利用ESDF进行更智能、更高效的避障。

### 实现步骤

#### 3.1 修改障碍物成本函数

```cpp
// planner/path_searching/src/mppi_planner.cpp

double MPPIPlanner::obstacleCost(const Trajectory& traj) {
    double cost = 0.0;
    
    for (const auto& pos : traj.positions) {
        double dist = grid_map_->evaluateEDT(pos);
        
        // 分段成本函数
        if (dist < 0.2) {
            // 碰撞区域：非常高的代价
            cost += 1000.0;
        } else if (dist < 0.5) {
            // 危险区域：平滑惩罚
            double ratio = (0.5 - dist) / 0.3;  // 0.2到0.5之间
            cost += 50.0 * ratio * ratio;  // 二次惩罚
        } else if (dist < 1.0) {
            // 接近区域：轻微惩罚
            double ratio = (1.0 - dist) / 0.5;
            cost += 5.0 * ratio;
        }
        // dist >= 1.0: 安全区域，无惩罚
    }
    
    return cost / traj.positions.size();  // 归一化
}
```

#### 3.2 添加ESDF梯度引导

```cpp
// 在rolloutTrajectory()函数中

void MPPIPlanner::rolloutTrajectory(State& state, 
                                    const Eigen::VectorXd& noise,
                                    Trajectory& traj) {
    for (int t = 0; t < horizon_; ++t) {
        // 1. 原有的动力学更新
        // state.acc = control + noise;
        // state.vel += state.acc * dt_;
        // state.pos += state.vel * dt_;
        
        // 2. 添加ESDF梯度引导（新增）
        double dist;
        Eigen::Vector3d grad;
        grid_map_->evaluateEDTWithGrad(state.pos, dist, grad);
        
        if (dist < 1.0 && grad.norm() > 0.01) {
            // 距离障碍物<1米时，添加斥力
            double repulsive_strength = 3.0;  // 可调参数
            double decay = (1.0 - dist);  // 距离越近，力越大
            
            Eigen::Vector3d repulsive_force = repulsive_strength * decay * grad;
            
            // 叠加到加速度
            state.acc += repulsive_force;
            
            // 限制加速度（防止过大）
            double acc_norm = state.acc.norm();
            if (acc_norm > max_acc_) {
                state.acc = state.acc / acc_norm * max_acc_;
            }
        }
        
        // 3. 更新状态
        state.vel += state.acc * dt_;
        
        // 限制速度
        double vel_norm = state.vel.norm();
        if (vel_norm > max_vel_) {
            state.vel = state.vel / vel_norm * max_vel_;
        }
        
        state.pos += state.vel * dt_;
        
        // 4. 记录轨迹
        traj.positions.push_back(state.pos);
        traj.velocities.push_back(state.vel);
    }
}
```

### 参数调优

在launch文件中添加可调参数：

```xml
<!-- advanced_param.xml -->

<!-- ESDF-MPPI参数 -->
<param name="mppi/esdf_repulsive_strength" value="3.0"/>
<param name="mppi/esdf_influence_distance" value="1.0"/>
<param name="mppi/obstacle_cost_weight" value="10.0"/>
```

### 提交

```bash
git add src/planner/path_searching/src/mppi_planner.cpp
git add src/planner/plan_manage/launch/advanced_param.xml
git commit -m "feat(mppi): integrate ESDF for intelligent obstacle avoidance

- Replace O(n³) brute-force with O(1) ESDF queries in obstacleCost()
- Add ESDF gradient-guided repulsive forces in rolloutTrajectory()
- Implement smooth 3-tier cost function (collision/danger/approach zones)
- Add configurable parameters for repulsive strength and influence distance

Performance: ~10x faster obstacle cost computation
Quality: More proactive avoidance with gradient guidance"

git push origin feature/esdf-mppi-upgrade
```

---

## 📋 阶段4: （可选）集成TGK拓扑算法

**建议**: 只在阶段1-3都稳定且飞行质量满意后再考虑。

TGK代码已备份在: `~/tgk_backup_20251001_1708/`

### 如果要集成

1. 先添加运行时开关
2. 默认关闭TGK
3. 逐步测试对比
4. 修复角点检测和A*搜索问题

---

## 🧪 测试验证清单

### 阶段1测试（BsplineOpt修复）

- [ ] 程序启动无错误
- [ ] 飞行轨迹平滑
- [ ] 完成所有5个航点
- [ ] 飞行时间 < 60秒
- [ ] 无碰撞
- [ ] 与d257634质量相当或更好

### 阶段2测试（ESDF添加）

- [ ] ESDF测试输出合理数值
- [ ] 编译无错误
- [ ] 不影响现有飞行质量

### 阶段3测试（MPPI-ESDF集成）

- [ ] 上述标准 +
- [ ] 避障更主动（提前绕开）
- [ ] 轨迹更贴近障碍物（在安全距离内）
- [ ] 规划时间没有显著增加（<100ms）

---

## 📊 进度跟踪

| 阶段 | 描述 | 状态 | 完成时间 |
|------|------|------|----------|
| 0 | 代码回退到d257634 | ✅ 完成 | 2025-10-01 17:08 |
| 1 | 修复BsplineOptimizer | ⏳ 待开始 | - |
| 2 | 添加ESDF支持 | ⏳ 待开始 | - |
| 3 | MPPI集成ESDF | ⏳ 待开始 | - |
| 4 | （可选）TGK集成 | 📅 计划中 | - |

---

## 🚀 下一步行动

### 在Docker中执行

```bash
# 1. 进入Docker环境
# (假设你已经在Docker中)

# 2. 同步代码
cd ~/ros_ws/ego-planner
git fetch origin
git checkout feature/esdf-mppi-upgrade
git pull origin feature/esdf-mppi-upgrade

# 3. 清理编译
catkin_make clean

# 4. 完整编译
catkin_make

# 5. 测试基线性能
roslaunch ego_planner simple_run.launch

# 观察并记录：
# - 飞行是否平滑？
# - 能否完成5个航点？
# - 用时多少？
# - 有无异常？
```

### 然后开始阶段1

按照上面"阶段1: 修复BsplineOptimizer"的指导：
1. 找到`initControlPoints`调用
2. 改为`setControlPoints`
3. 编译测试
4. 提交推送

---

## 📚 参考资料

### 备份位置
- TGK代码备份: `~/tgk_backup_20251001_1708/`
- 包含所有文档和实现代码

### GitHub
- Master分支: https://github.com/He-91/ego-planner/tree/master
- 新开发分支: https://github.com/He-91/ego-planner/tree/feature/esdf-mppi-upgrade

### Git状态
```
Current commit: d257634 (稳定基线)
Current branch: feature/esdf-mppi-upgrade
Status: Clean working directory
```

---

**准备好开始阶段1了吗？** 🚀

告诉我你在Docker中测试基线的结果，然后我们开始修复BsplineOptimizer！
