# FAQ - 常见问题解答

## 🚀 安装与配置

### Q1: 编译时出现"Eigen3未找到"错误
**A**: 安装Eigen3开发包：
```bash
sudo apt-get install libeigen3-dev
# 或者从源码编译
git clone https://gitlab.com/libeigen/eigen.git
cd eigen && mkdir build && cd build
cmake .. && make install
```

### Q2: ROS依赖问题，rosdep install失败
**A**: 更新rosdep数据库：
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Q3: 编译时出现C++17相关错误
**A**: 确保使用支持C++17的编译器：
```bash
# 检查gcc版本
gcc --version  # 需要7.0+

# 在CMakeLists.txt中添加
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

## 🧠 算法相关

### Q4: MPPI采样数量如何选择？
**A**: 根据计算资源和精度需求：
- **实时性要求高**: 500-1000个样本
- **精度要求高**: 2000-5000个样本  
- **复杂环境**: 增加到5000+样本

```xml
<!-- 在launch文件中调整 -->
<param name="mppi/num_samples" value="1000"/>
```

### Q5: TopoPRM找不到路径怎么办？
**A**: 调整以下参数：
```xml
<!-- 增加采样数量 -->
<param name="topo_prm/max_sample_num" value="20000"/>
<!-- 增大连接半径 -->
<param name="topo_prm/connection_radius" value="2.0"/>
<!-- 减小膨胀半径 -->
<param name="topo_prm/sample_inflate_r" value="0.05"/>
```

### Q6: B-spline优化结果不平滑
**A**: 调整优化权重：
```xml
<!-- 增加平滑性权重 -->
<param name="bspline/lambda_smooth" value="2.0"/>
<!-- 增加迭代次数 -->
<param name="optimization/max_iteration_num" value="100"/>
```

## 🎯 性能优化

### Q7: 规划速度太慢怎么优化？
**A**: 多方面优化：
1. **减少MPPI采样数**
2. **启用并行计算**
   ```cpp
   export OMP_NUM_THREADS=4
   ```
3. **调整时间范围**
   ```xml
   <param name="mppi/time_horizon" value="1.5"/>
   ```
4. **降低地图分辨率**
   ```xml
   <param name="sdf_map/resolution" value="0.2"/>
   ```

### Q8: 内存占用过高
**A**: 
- 减少采样数量和时间步数
- 定期清理轨迹历史
- 使用内存池管理

```cpp
// 清理旧轨迹
if (trajectory_history.size() > MAX_HISTORY) {
    trajectory_history.pop_front();
}
```

### Q9: CPU使用率100%
**A**: 
- 检查循环频率设置
- 使用ROS的定时器而不是while循环
- 合理设置规划频率

```cpp
ros::Timer planning_timer = nh.createTimer(
    ros::Duration(0.1), &PlannerManager::planCallback, this);
```

## 🎮 使用相关

### Q10: 如何设置起点和终点？
**A**: 
1. **通过RViz**: 使用"2D Nav Goal"工具
2. **通过代码**:
   ```cpp
   geometry_msgs::PoseStamped goal;
   goal.pose.position.x = 10.0;
   goal.pose.position.y = 5.0; 
   goal.pose.position.z = 2.0;
   goal_pub.publish(goal);
   ```
3. **通过话题**:
   ```bash
   rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "..."
   ```

### Q11: 轨迹不连续或有跳跃
**A**: 
- 检查时间步长设置
- 增加B-spline控制点数量
- 调整速度和加速度约束

```xml
<param name="bspline/max_vel" value="2.0"/>
<param name="bspline/max_acc" value="2.0"/>
```

### Q12: 机器人不跟随规划轨迹
**A**: 
- 确认控制器正确订阅轨迹话题
- 检查坐标系是否一致
- 验证轨迹时间戳

```cpp
// 检查话题连接
rostopic echo /planning/trajectory
rostopic info /cmd_vel
```

## 🐛 调试相关

### Q13: 如何调试MPPI算法？
**A**: 
1. **启用可视化**:
   ```cpp
   mppi_planner_->enableVisualization(true);
   ```
2. **查看采样轨迹**:
   ```bash
   rostopic echo /mppi_trajectories
   ```
3. **调试成本函数**:
   ```cpp
   ROS_INFO("Obstacle cost: %f", obstacle_cost);
   ROS_INFO("Smoothness cost: %f", smoothness_cost);
   ```

### Q14: RViz中看不到轨迹
**A**: 
- 检查话题名称是否正确
- 确认MarkerArray消息正常发布
- 检查坐标系设置

```bash
# 检查消息发布
rostopic list | grep trajectory
rostopic hz /planning/trajectory_vis

# 检查坐标系
rosrun tf view_frames
```

### Q15: 如何记录和回放测试数据？
**A**: 
```bash
# 记录数据包
rosbag record -a -O test_data.bag

# 回放数据包  
rosbag play test_data.bag

# 分析特定话题
rosbag info test_data.bag
rostopic echo -b test_data.bag /planning/trajectory
```

## 🔧 高级配置

### Q16: 如何添加自定义成本函数？
**A**: 
1. **继承MPPIPlanner类**:
   ```cpp
   class CustomMPPI : public MPPIPlanner {
   public:
       double computeCustomCost(const std::vector<Eigen::Vector3d>& traj) override;
   };
   ```

2. **重写成本计算函数**:
   ```cpp
   double CustomMPPI::computeCustomCost(const std::vector<Eigen::Vector3d>& traj) {
       // 自定义成本逻辑
       return custom_cost;
   }
   ```

### Q17: 如何支持不同类型的机器人？
**A**: 
1. **修改动力学模型**:
   ```cpp
   // 在MPPI中定义机器人特定的动力学
   struct RobotDynamics {
       double max_vel, max_acc, max_turn_rate;
   };
   ```

2. **调整约束参数**:
   ```xml
   <!-- 地面机器人 -->
   <param name="robot/max_vel_xy" value="1.0"/>
   <param name="robot/max_vel_z" value="0.0"/>
   
   <!-- 飞行器 -->  
   <param name="robot/max_vel_xy" value="3.0"/>
   <param name="robot/max_vel_z" value="2.0"/>
   ```

### Q18: 如何处理动态障碍物？
**A**: 
1. **启用重规划**:
   ```cpp
   if (detectDynamicObstacle()) {
       planner_manager_->replan();
   }
   ```

2. **调整规划频率**:
   ```xml
   <param name="replanning_rate" value="5.0"/>  <!-- 5Hz重规划 -->
   ```

3. **预测障碍物运动**:
   ```cpp
   // 在成本函数中考虑障碍物未来位置
   Eigen::Vector3d predicted_obs_pos = current_pos + velocity * dt;
   ```

## 📊 性能基准

### Q19: 如何评估规划性能？
**A**: 
1. **时间指标**:
   ```cpp
   auto start = std::chrono::high_resolution_clock::now();
   planner_->plan();
   auto end = std::chrono::high_resolution_clock::now();
   auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
   ```

2. **质量指标**:
   ```cpp
   double path_length = computePathLength(trajectory);
   double smoothness = computeSmoothness(trajectory);
   double safety_margin = computeSafetyMargin(trajectory);
   ```

3. **成功率统计**:
   ```cpp
   int success_count = 0, total_attempts = 100;
   double success_rate = (double)success_count / total_attempts;
   ```

### Q20: 性能对比工具
**A**: 
```bash
# 使用基准测试节点
rosrun plan_manage benchmark_node

# 或者使用Python脚本
python scripts/performance_test.py

# 性能分析工具
sudo apt-get install linux-tools-common
perf record rosrun plan_manage ego_planner_node
perf report
```

## 🔗 扩展功能

### Q21: 如何集成到MoveBase？
**A**: 
1. **创建插件**:
   ```cpp
   #include <nav_core/base_global_planner.h>
   
   class EgoGlobalPlanner : public nav_core::BaseGlobalPlanner {
   public:
       bool makePlan(const geometry_msgs::PoseStamped& start,
                     const geometry_msgs::PoseStamped& goal,
                     std::vector<geometry_msgs::PoseStamped>& plan) override;
   };
   ```

2. **配置MoveBase**:
   ```xml
   <param name="base_global_planner" value="ego_planner/EgoGlobalPlanner"/>
   ```

### Q22: 支持多机器人规划？
**A**: 
- 为每个机器人创建独立的规划器实例
- 在MPPI成本函数中考虑其他机器人的轨迹
- 使用分布式协调机制

```cpp
// 多机器人冲突检测
bool checkInterRobotCollision(const std::vector<Trajectory>& robot_trajs);
```

### Q23: 如何保存和加载地图？
**A**: 
```bash
# 保存地图
rosrun map_server map_saver -f my_map

# 加载地图
rosrun map_server map_server my_map.yaml

# 在launch文件中
<node name="map_server" pkg="map_server" type="map_server" args="$(find my_package)/maps/my_map.yaml"/>
```

## 📞 获取帮助

### Q24: 在哪里报告bug？
**A**: 
- GitHub Issues: https://github.com/yourusername/ego-planner/issues
- 邮件联系: your.email@example.com
- 论坛讨论: [ROS Answers](https://answers.ros.org/)

### Q25: 如何贡献代码？
**A**: 
1. Fork项目仓库
2. 创建feature分支
3. 提交Pull Request
4. 参与代码审查

详见：[贡献指南](../README.md#贡献指南)

---

**💡 提示**: 如果这里没有找到你的问题，请查看：
- [API文档](API.md) - 详细的接口说明
- [算法详解](../Algorithm_Framework_Summary.md) - 算法原理
- [Issues页面](https://github.com/yourusername/ego-planner/issues) - 已知问题和讨论