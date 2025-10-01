# API文档 - EGO-Planner

## 核心API接口

### PlannerManager 类

规划管理器是整个系统的核心调度器，负责协调各个算法模块。

#### 构造与初始化

```cpp
class PlannerManager {
public:
    PlannerManager();
    ~PlannerManager();
    
    // 初始化所有规划模块
    void initPlanModules(ros::NodeHandle& nh);
    
    // 设置环境地图
    void setEnvironment(const std::shared_ptr<GridMap>& env);
};
```

#### 规划接口

```cpp
// 主要规划接口
enum PlannerResult {
    SUCCEED = 1,
    FAIL = -1,
    RETRY = 0
};

// 全局路径规划
PlannerResult planGlobalTraj(const Eigen::Vector3d& start_pos,
                            const Eigen::Vector3d& start_vel,
                            const Eigen::Vector3d& start_acc,
                            const Eigen::Vector3d& end_pos,
                            const Eigen::Vector3d& end_vel,
                            const Eigen::Vector3d& end_acc);

// 局部重规划  
PlannerResult planLocalTraj(const Eigen::Vector3d& start_pos,
                           const Eigen::Vector3d& start_vel,
                           const Eigen::Vector3d& start_acc,
                           const Eigen::Vector3d& end_pos,
                           const Eigen::Vector3d& end_vel);
```

#### 轨迹查询

```cpp
// 获取规划的轨迹
std::shared_ptr<UniformBspline> getLocalTraj() const;
std::shared_ptr<UniformBspline> getGlobalTraj() const;

// 获取轨迹在指定时间的状态
Eigen::Vector3d getPosition(double time) const;
Eigen::Vector3d getVelocity(double time) const;
Eigen::Vector3d getAcceleration(double time) const;
```

### TopoPRM 类

拓扑路径规划算法，生成多条不同拓扑的候选路径。

#### 核心接口

```cpp
class TopoPRM {
public:
    TopoPRM();
    ~TopoPRM();
    
    // 初始化参数
    void init(ros::NodeHandle& nh);
    
    // 设置环境
    void setEnvironment(const std::shared_ptr<GridMap>& env);
    
    // 主要规划接口
    bool searchTopoPaths(const Eigen::Vector3d& start,
                        const Eigen::Vector3d& goal,
                        std::vector<std::vector<Eigen::Vector3d>>& topo_paths,
                        std::vector<double>& costs);
};
```

#### 路径生成策略

```cpp
// 路径类型枚举
enum PathType {
    DIRECT_PATH = 0,    // 直接路径
    TANGENT_PATH = 1,   // 切线路径  
    VERTICAL_PATH = 2,  // 垂直路径
    AROUND_PATH = 3     // 环绕路径
};

// 获取指定类型的路径
bool getSpecificPath(const Eigen::Vector3d& start,
                    const Eigen::Vector3d& goal,
                    PathType type,
                    std::vector<Eigen::Vector3d>& path);
```

#### 参数配置

```cpp
struct TopoParams {
    double sample_inflate_r;     // 采样膨胀半径
    int max_sample_num;         // 最大采样数
    double connection_radius;    // 连接半径
    double lambda_heu;          // 启发式权重
    
    // 路径优化参数
    double weight_length;       // 长度权重
    double weight_angle;        // 角度权重
    double weight_obstacle;     // 障碍物权重
};

void setParameters(const TopoParams& params);
```

### MPPIPlanner 类

模型预测路径积分规划器，统一处理全局和局部规划。

#### 核心接口

```cpp
class MPPIPlanner {
public:
    MPPIPlanner();
    ~MPPIPlanner();
    
    // 初始化
    void init(ros::NodeHandle& nh);
    void setGridMap(std::shared_ptr<GridMap>& map);
    
    // 全局轨迹规划
    bool planTrajectory(const Eigen::Vector3d& start_pos,
                       const Eigen::Vector3d& start_vel,
                       const Eigen::Vector3d& goal_pos,
                       const Eigen::Vector3d& goal_vel,
                       std::vector<Eigen::Vector3d>& trajectory);
    
    // 局部路径规划(替代A*算法)
    bool planLocalPath(const Eigen::Vector3d& start_pos,
                      const Eigen::Vector3d& goal_pos,
                      std::vector<Eigen::Vector3d>& path_points);
};
```

#### MPPI算法参数

```cpp
struct MPPIParams {
    // 基础参数
    int num_samples;           // 采样数量
    double time_horizon;       // 时间范围
    double lambda;            // 温度参数
    double dt;               // 时间步长
    
    // 成本权重
    double weight_obstacle;    // 障碍物成本权重
    double weight_smoothness; // 平滑性成本权重
    double weight_goal;       // 目标成本权重
    double weight_velocity;   // 速度成本权重
    
    // 控制约束
    double max_vel;           // 最大速度
    double max_acc;           // 最大加速度
    double max_jerk;          // 最大急动度
};

void setMPPIParams(const MPPIParams& params);
```

#### 成本函数接口

```cpp
// 障碍物成本计算
double computeObstacleCost(const std::vector<Eigen::Vector3d>& trajectory);

// 平滑性成本计算  
double computeSmoothnessCost(const std::vector<Eigen::Vector3d>& trajectory);

// 目标跟踪成本
double computeGoalCost(const std::vector<Eigen::Vector3d>& trajectory,
                      const Eigen::Vector3d& goal);

// 总成本计算
double computeTotalCost(const std::vector<Eigen::Vector3d>& trajectory);
```

#### 可视化接口

```cpp
// 启用/禁用可视化
void enableVisualization(bool enable);

// 可视化采样轨迹
void visualizeTrajectories(const std::vector<std::vector<Eigen::Vector3d>>& trajectories);

// 可视化最优轨迹
void visualizeOptimalTrajectory(const std::vector<Eigen::Vector3d>& trajectory);
```

### BsplineOptimizer 类

B样条轨迹优化器，对轨迹进行平滑和约束满足优化。

#### 核心接口

```cpp
class BsplineOptimizer {
public:
    BsplineOptimizer();
    ~BsplineOptimizer();
    
    // 初始化
    void setParam(ros::NodeHandle& nh);
    void setEnvironment(const std::shared_ptr<GridMap>& env);
    void setMPPIPlanner(std::shared_ptr<MPPIPlanner>& mppi_planner);
    
    // 轨迹优化
    Eigen::MatrixXd BsplineOptimTraj(const Eigen::MatrixXd& points,
                                    const double& ts);
};
```

#### 优化参数

```cpp
struct OptimizeParams {
    // 优化权重
    double lambda_smooth;      // 平滑性权重
    double lambda_collision;   // 碰撞避免权重
    double lambda_feasibility; // 可行性权重
    double lambda_fitness;     // 适应性权重
    
    // 约束参数
    double max_vel;           // 最大速度约束
    double max_acc;           // 最大加速度约束
    double max_jerk;          // 最大急动度约束
    
    // 优化参数
    int max_iteration_num;    // 最大迭代次数
    double min_cost_diff;     // 最小成本差异
    double step_size;         // 步长
};

void setOptimizeParams(const OptimizeParams& params);
```

#### 约束处理

```cpp
// 碰撞约束检查和处理
bool checkCollisionAndRebound(Eigen::MatrixXd& control_points);

// 动力学可行性检查
bool checkFeasibility(const Eigen::MatrixXd& control_points);

// 边界约束处理
void applyBoundaryConstraints(Eigen::MatrixXd& control_points);
```

### GridMap 类

环境地图管理器，提供障碍物检测和碰撞查询服务。

#### 核心接口

```cpp
class GridMap {
public:
    GridMap();
    ~GridMap();
    
    // 地图初始化
    void initMap(ros::NodeHandle& nh);
    
    // 碰撞检测
    bool isOccupied(const Eigen::Vector3d& pos) const;
    bool isOccupied(const int& idx_x, const int& idx_y, const int& idx_z) const;
    
    // 距离查询
    double getDistance(const Eigen::Vector3d& pos) const;
    Eigen::Vector3d getGradient(const Eigen::Vector3d& pos) const;
    
    // 坐标转换
    Eigen::Vector3d indexToPos(const Eigen::Vector3i& id) const;
    Eigen::Vector3i posToIndex(const Eigen::Vector3d& pos) const;
};
```

#### 地图参数

```cpp
struct MapParams {
    double resolution;         // 地图分辨率
    Eigen::Vector3d map_origin; // 地图原点
    Eigen::Vector3d map_size;   // 地图尺寸
    double inflate_r;          // 膨胀半径
    
    // SDF参数
    double truncate_dist;      // 截断距离  
    double update_rate;        // 更新频率
};

void setMapParams(const MapParams& params);
```

## ROS接口

### 主要话题

#### 订阅话题

```cpp
// 目标点订阅
"/move_base_simple/goal" (geometry_msgs/PoseStamped)

// 里程计信息
"/odom" (nav_msgs/Odometry)

// 点云数据
"/local_pointcloud" (sensor_msgs/PointCloud2)
```

#### 发布话题

```cpp
// 规划轨迹
"/planning/trajectory" (plan_manage/Bspline)

// 可视化轨迹
"/planning/trajectory_vis" (visualization_msgs/MarkerArray)

// TopoPRM路径
"/topo_paths_vis" (visualization_msgs/MarkerArray)

// MPPI轨迹
"/mppi_trajectories" (visualization_msgs/MarkerArray)
"/optimal_trajectory" (visualization_msgs/MarkerArray)
```

### 服务接口

```cpp
// 规划服务
"/plan_global_traj" (std_srvs/Empty)
"/plan_local_traj" (std_srvs/Empty)

// 参数服务
"/set_planner_params" (plan_manage/SetPlannerParams)
```

### 消息类型

#### Bspline.msg

```cpp
# B样条轨迹消息
geometry_msgs/Point[] pos_pts      # 位置控制点
geometry_msgs/Vector3[] vel_pts    # 速度控制点  
geometry_msgs/Vector3[] acc_pts    # 加速度控制点
int32[] knots                      # 节点向量
float64 start_time                 # 起始时间
```

#### DataDisp.msg

```cpp
# 数据显示消息
string info_type                   # 信息类型
float64[] values                   # 数值数组
geometry_msgs/Point[] positions    # 位置数组
```

## 错误码和异常处理

### 错误码定义

```cpp
enum ErrorCode {
    SUCCESS = 0,                    // 成功
    INVALID_PARAM = -1,            // 无效参数
    INITIALIZATION_FAILED = -2,     // 初始化失败
    PLANNING_FAILED = -3,          // 规划失败
    COLLISION_DETECTED = -4,       // 检测到碰撞
    TIMEOUT = -5,                  // 超时
    MEMORY_ERROR = -6              // 内存错误
};
```

### 异常处理

```cpp
try {
    planner_manager.planGlobalTraj(start, vel, acc, goal, goal_vel, goal_acc);
} catch (const std::exception& e) {
    ROS_ERROR("Planning failed: %s", e.what());
    return false;
}
```

## 性能优化建议

### 参数调优

1. **MPPI采样数量**: 根据计算资源调整，一般1000-5000
2. **时间步长**: 较小步长提高精度但增加计算量，推荐0.02-0.1s
3. **优化迭代次数**: B样条优化迭代次数，推荐50-200次

### 并行化

```cpp
// 启用OpenMP并行化
#pragma omp parallel for
for (int i = 0; i < num_samples; ++i) {
    // MPPI采样计算
}
```

### 内存管理

```cpp
// 使用智能指针避免内存泄漏
std::shared_ptr<GridMap> grid_map = std::make_shared<GridMap>();

// 预分配向量大小
trajectory.reserve(time_steps);
```