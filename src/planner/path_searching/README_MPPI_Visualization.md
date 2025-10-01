# MPPI可视化功能说明

## 功能概述

为MPPI算法添加了完整的可视化功能，可以在RViz中实时观察：
1. **采样轨迹** - 显示MPPI算法生成的多条随机采样轨迹
2. **最优轨迹** - 显示加权平均后的最优轨迹
3. **速度向量** - 显示最优轨迹上的速度箭头

## 可视化话题

### `/mppi_trajectories`
- **类型**: `visualization_msgs/MarkerArray`
- **内容**: 显示MPPI采样的多条轨迹
- **颜色编码**: 基于轨迹权重，绿色表示低成本（高权重），红色表示高成本（低权重）
- **透明度**: 0.3（半透明，避免过于拥挤）
- **数量控制**: 最多显示50条轨迹以避免性能问题

### `/mppi_optimal_trajectory`
- **类型**: `visualization_msgs/MarkerArray`
- **内容**: 显示最优轨迹和速度箭头
- **轨迹颜色**: 橙色（亮橙色，醒目）
- **线宽**: 0.15m（比采样轨迹更粗）
- **速度箭头**: 蓝色箭头显示速度方向和大小

## 使用方法

### 1. 编译项目
```bash
cd /home/he/ros_ws/test/ego-planner
catkin_make
```

### 2. 运行测试
```bash
# 启动测试节点和RViz
roslaunch plan_manage test_mppi_visualization.launch
```

### 3. 在RViz中观察
- 打开RViz后会自动加载配置文件
- 可以看到以下显示项：
  - **Grid**: 参考网格
  - **Topological Paths**: 拓扑路径（红绿蓝等不同颜色）
  - **MPPI Sample Trajectories**: MPPI采样轨迹（半透明彩色线条）
  - **MPPI Optimal Trajectory**: MPPI最优轨迹（橙色粗线+蓝色速度箭头）

## 代码结构

### 头文件修改 (`mppi_planner.h`)
- 添加了可视化发布器
- 添加了frame_id参数
- 声明了可视化函数

### 实现文件修改 (`mppi_planner.cpp`)
- `init()`: 初始化ROS发布器
- `visualizeTrajectories()`: 可视化采样轨迹
- `visualizeOptimalTrajectory()`: 可视化最优轨迹和速度向量
- `planTrajectory()`: 集成可视化调用

### 管理器集成 (`planner_manager.cpp`)
- 更新MPPI初始化调用，传递NodeHandle参数

## 性能优化

1. **采样轨迹数量限制**: 最多显示50条轨迹
2. **速度箭头稀疏化**: 每个轨迹最多显示5个箭头
3. **条件渲染**: 只有速度大于0.1m/s才显示箭头
4. **半透明渲染**: 采样轨迹使用半透明避免遮挡

## 参数配置

### 可调参数
- `num_samples_`: 采样轨迹数量（默认500）
- `horizon_steps_`: 时域步数（默认20）
- `frame_id_`: 坐标系名称（默认"world"）

### 可视化参数（硬编码，可根据需要调整）
- 采样轨迹线宽: 0.05m
- 最优轨迹线宽: 0.15m
- 速度箭头缩放因子: 0.5
- 最大显示轨迹数: 50条

## 调试信息

启用调试日志可以看到详细的可视化过程：
```bash
rosrun rqt_logger_level rqt_logger_level
# 将ego_planner节点的日志级别设置为DEBUG
```

## 故障排除

### 问题1: RViz中看不到轨迹
- 检查话题是否发布: `rostopic list | grep mppi`
- 检查坐标系: 确保frame_id正确
- 检查RViz中对应的Display是否启用

### 问题2: 轨迹显示不完整
- 可能是轨迹点太少，检查horizon_steps参数
- 可能是轨迹计算失败，查看控制台错误信息

### 问题3: 性能问题
- 减少num_samples_参数
- 调整可视化频率
- 在代码中增加visualization_step参数

## 扩展建议

1. **成本可视化**: 可以添加成本热图显示
2. **实时参数调整**: 添加dynamic_reconfigure支持
3. **3D加速度可视化**: 显示加速度向量
4. **轨迹历史**: 保留前几次规划的轨迹对比