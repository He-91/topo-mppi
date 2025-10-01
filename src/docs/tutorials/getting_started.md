# EGO-Planner 快速入门教程

## 🎯 教程概述

本教程将带你从零开始，逐步掌握EGO-Planner的使用和开发。我们将通过实际例子，展示如何配置、运行和自定义这个强大的路径规划系统。

## 📋 先决条件

- Ubuntu 18.04/20.04
- ROS Melodic/Noetic
- 基本的C++和ROS知识
- 熟悉Linux命令行

## 第一步：系统安装

### 1.1 创建工作空间

```bash
# 创建catkin工作空间
mkdir -p ~/ego_ws/src
cd ~/ego_ws/src

# 初始化工作空间
cd ~/ego_ws
catkin_make
source devel/setup.bash
```

### 1.2 克隆源码

```bash
cd ~/ego_ws/src
git clone https://github.com/yourusername/ego-planner.git
```

### 1.3 安装依赖

```bash
# 安装ROS依赖
cd ~/ego_ws
rosdep install --from-paths src --ignore-src -r -y

# 安装系统依赖
sudo apt-get update
sudo apt-get install -y \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    libompl-dev
```

### 1.4 编译系统

```bash
cd ~/ego_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## 第二步：基础运行

### 2.1 启动仿真环境

```bash
# 终端1：启动仿真
roslaunch plan_manage run_in_sim.launch

# 终端2：启动可视化
roslaunch plan_manage rviz.launch
```

### 2.2 测试路径规划

1. **在RViz中设置目标点**：
   - 点击工具栏中的"2D Nav Goal"
   - 在地图中点击选择目标位置
   - 系统将自动开始路径规划

2. **观察规划过程**：
   - TopoPRM生成多条候选路径（红色）
   - MPPI优化生成最优轨迹（绿色）
   - B-spline平滑最终轨迹（蓝色）

### 2.3 检查系统状态

```bash
# 查看活跃节点
rosnode list

# 查看话题列表
rostopic list

# 监控轨迹输出
rostopic echo /planning/trajectory
```

## 第三步：参数配置

### 3.1 理解配置文件

主要配置文件位置：
```
plan_manage/launch/
├── advanced_param.xml  # 高级参数配置
├── simulator.xml       # 仿真器参数
└── run_in_sim.launch  # 主启动文件
```

### 3.2 MPPI参数调优

编辑 `advanced_param.xml`：

```xml
<!-- MPPI基础参数 -->
<param name="mppi/num_samples" value="1000"/>        <!-- 采样数量 -->
<param name="mppi/time_horizon" value="2.0"/>        <!-- 时间范围 -->
<param name="mppi/lambda" value="0.1"/>              <!-- 温度参数 -->
<param name="mppi/dt" value="0.05"/>                 <!-- 时间步长 -->

<!-- 成本权重 -->
<param name="mppi/cost_weights/obstacle" value="100.0"/>    <!-- 障碍物权重 -->
<param name="mppi/cost_weights/smoothness" value="10.0"/>   <!-- 平滑性权重 -->
<param name="mppi/cost_weights/goal" value="50.0"/>         <!-- 目标权重 -->
<param name="mppi/cost_weights/velocity" value="5.0"/>      <!-- 速度权重 -->
```

### 3.3 B-spline优化参数

```xml
<!-- B-spline优化参数 -->
<param name="bspline/lambda_smooth" value="1.0"/>     <!-- 平滑性 -->
<param name="bspline/lambda_collision" value="2.0"/>  <!-- 碰撞避免 -->
<param name="bspline/lambda_feasibility" value="1.5"/> <!-- 可行性 -->
<param name="bspline/lambda_fitness" value="1.0"/>    <!-- 适应性 -->

<!-- 约束参数 -->
<param name="bspline/max_vel" value="2.0"/>           <!-- 最大速度 -->
<param name="bspline/max_acc" value="2.0"/>           <!-- 最大加速度 -->
```

### 3.4 测试参数效果

```bash
# 重新启动系统测试新参数
roslaunch plan_manage run_in_sim.launch

# 或者动态修改参数（部分支持）
rosparam set /mppi/num_samples 2000
```

## 第四步：可视化定制

### 4.1 RViz配置

1. **添加新显示项**：
   - 点击"Add"按钮
   - 选择"MarkerArray"
   - 设置Topic为相应的可视化话题

2. **推荐显示配置**：
   ```yaml
   Displays:
     - Name: "TopoPRM Paths"
       Type: "MarkerArray"
       Topic: "/topo_paths_vis"
       Color: [1, 0, 0, 0.8]  # 红色
       
     - Name: "MPPI Trajectories"
       Type: "MarkerArray"
       Topic: "/mppi_trajectories"
       Color: [0, 1, 0, 0.5]  # 半透明绿色
       
     - Name: "Optimal Trajectory"
       Type: "MarkerArray"
       Topic: "/optimal_trajectory"
       Color: [0, 0, 1, 1.0]  # 蓝色
   ```

### 4.2 自定义可视化

在C++代码中添加可视化：

```cpp
#include <visualization_msgs/MarkerArray.h>

void publishCustomVisualization() {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 设置轨迹点
    for (const auto& point : trajectory_points) {
        geometry_msgs::Point p;
        p.x = point.x();
        p.y = point.y(); 
        p.z = point.z();
        marker.points.push_back(p);
    }
    
    // 设置颜色和尺寸
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.05;
    
    marker_array.markers.push_back(marker);
    vis_pub.publish(marker_array);
}
```

## 第五步：算法自定义

### 5.1 修改MPPI成本函数

创建自定义MPPI类：

```cpp
// custom_mppi_planner.h
#include "path_searching/mppi_planner.h"

class CustomMPPIPlanner : public MPPIPlanner {
public:
    CustomMPPIPlanner();
    ~CustomMPPIPlanner();
    
protected:
    // 重写成本函数
    double computeObstacleCost(const std::vector<Eigen::Vector3d>& trajectory) override;
    double computeCustomCost(const std::vector<Eigen::Vector3d>& trajectory);
    
private:
    // 自定义参数
    double custom_weight_;
};
```

实现自定义成本：

```cpp
// custom_mppi_planner.cpp
double CustomMPPIPlanner::computeObstacleCost(const std::vector<Eigen::Vector3d>& trajectory) {
    double cost = 0.0;
    
    for (const auto& point : trajectory) {
        // 基础障碍物成本
        double base_cost = MPPIPlanner::computeObstacleCost({point});
        
        // 添加自定义逻辑（例如：对某些区域额外惩罚）
        if (point.z() > 3.0) {  // 高度限制
            base_cost *= 2.0;
        }
        
        cost += base_cost;
    }
    
    return cost;
}

double CustomMPPIPlanner::computeCustomCost(const std::vector<Eigen::Vector3d>& trajectory) {
    // 例如：路径长度成本
    double length_cost = 0.0;
    for (size_t i = 1; i < trajectory.size(); ++i) {
        length_cost += (trajectory[i] - trajectory[i-1]).norm();
    }
    
    return custom_weight_ * length_cost;
}
```

### 5.2 集成自定义算法

在PlannerManager中使用：

```cpp
// planner_manager.cpp
void PlannerManager::initPlanModules(ros::NodeHandle& nh) {
    // 替换默认MPPI
    mppi_planner_.reset(new CustomMPPIPlanner);
    mppi_planner_->init(nh);
    
    // 其他初始化...
}
```

## 第六步：性能优化

### 6.1 并行化配置

启用OpenMP：

```cpp
// 在CMakeLists.txt中添加
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()
```

在代码中使用：

```cpp
#include <omp.h>

void MPPIPlanner::sampleTrajectories() {
    #pragma omp parallel for
    for (int i = 0; i < num_samples_; ++i) {
        // 并行采样计算
        trajectories_[i] = generateSampleTrajectory(i);
        costs_[i] = computeTotalCost(trajectories_[i]);
    }
}
```

### 6.2 内存优化

```cpp
class MPPIPlanner {
private:
    // 预分配内存池
    std::vector<std::vector<Eigen::Vector3d>> trajectory_pool_;
    
public:
    void initMemoryPool() {
        trajectory_pool_.resize(num_samples_);
        for (auto& traj : trajectory_pool_) {
            traj.reserve(time_steps_);
        }
    }
};
```

### 6.3 性能监控

```cpp
#include <chrono>

class PerformanceMonitor {
public:
    void startTimer(const std::string& name) {
        start_times_[name] = std::chrono::high_resolution_clock::now();
    }
    
    void endTimer(const std::string& name) {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end - start_times_[name]).count();
        
        ROS_INFO("%s took %ld ms", name.c_str(), duration);
    }
    
private:
    std::map<std::string, std::chrono::high_resolution_clock::time_point> start_times_;
};

// 使用示例
PerformanceMonitor monitor;
monitor.startTimer("MPPI_Planning");
bool result = planTrajectory(start, goal, trajectory);
monitor.endTimer("MPPI_Planning");
```

## 第七步：实际部署

### 7.1 硬件集成

对于真实机器人部署：

```xml
<!-- 修改传感器话题 -->
<param name="grid_map/pointcloud_topic" value="/velodyne_points"/>
<param name="grid_map/pose_topic" value="/mavros/local_position/pose"/>

<!-- 调整控制频率 -->
<param name="planning_rate" value="10.0"/>  <!-- 10Hz -->
<param name="execution_rate" value="50.0"/> <!-- 50Hz -->
```

### 7.2 安全机制

```cpp
class SafetyChecker {
public:
    bool checkTrajectory(const std::vector<Eigen::Vector3d>& trajectory) {
        // 检查速度限制
        for (size_t i = 1; i < trajectory.size(); ++i) {
            double vel = (trajectory[i] - trajectory[i-1]).norm() / dt_;
            if (vel > max_velocity_) {
                ROS_WARN("Velocity limit exceeded: %f > %f", vel, max_velocity_);
                return false;
            }
        }
        
        // 检查加速度限制
        // ...
        
        // 检查安全距离
        // ...
        
        return true;
    }
    
private:
    double max_velocity_ = 2.0;
    double max_acceleration_ = 2.0;
    double safety_distance_ = 0.5;
};
```

### 7.3 故障恢复

```cpp
class FailsafeManager {
public:
    void handlePlanningFailure() {
        ROS_ERROR("Planning failed, activating failsafe");
        
        // 紧急停止
        publishStopCommand();
        
        // 尝试重新规划
        if (retry_count_ < max_retries_) {
            retry_count_++;
            ros::Duration(0.1).sleep();  // 短暂延迟
            triggerReplanning();
        } else {
            // 激活紧急着陆/停止模式
            activateEmergencyMode();
        }
    }
    
private:
    int retry_count_ = 0;
    int max_retries_ = 3;
};
```

## 🎯 练习任务

### 任务1：基础参数调优
- 尝试不同的MPPI采样数量
- 观察对规划时间和质量的影响
- 记录最佳参数组合

### 任务2：自定义成本函数
- 添加一个"能耗"成本项
- 惩罚急转弯和急加速
- 比较优化前后的轨迹质量

### 任务3：可视化增强
- 显示机器人的安全半径
- 可视化速度向量
- 添加实时性能指标显示

### 任务4：多目标规划
- 实现连续多个目标点的规划
- 添加路径点时间约束
- 支持动态目标更新

## 📚 进阶学习

完成本教程后，建议学习：
- [算法原理详解](../Algorithm_Framework_Summary.md)
- [API参考文档](../API.md)
- [常见问题解答](../FAQ.md)
- 相关论文和研究

## 🔗 有用资源

- **论文**: [MPPI原理论文](https://arxiv.org/abs/1509.01149)
- **视频**: [EGO-Planner演示视频](https://www.youtube.com/watch?v=abc123)
- **代码**: [GitHub仓库](https://github.com/yourusername/ego-planner)
- **论坛**: [ROS Answers](https://answers.ros.org/questions/scope:all/sort:activity-desc/tags:path-planning/)

---

**🎉 恭喜！** 你已经掌握了EGO-Planner的基本使用和开发技巧。继续探索和实验，打造属于你自己的智能路径规划系统！