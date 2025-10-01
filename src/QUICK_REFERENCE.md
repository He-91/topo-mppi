# 快速参考指南 - Ego-Planner升级项目

**当前版本**: d257634 (稳定基线)  
**开发分支**: feature/esdf-mppi-upgrade  
**最后更新**: 2025年10月1日

---

## 📋 项目状态总览

### ✅ 已完成
- [x] 代码回退到稳定基线d257634
- [x] 创建新开发分支 feature/esdf-mppi-upgrade
- [x] 完成代码框架全面分析
- [x] 制定详细分阶段实施计划

### ⏳ 进行中
- [ ] 阶段1: 修复BsplineOptimizer bug (最高优先级🔥)

### 📅 待开始
- [ ] 阶段2: 添加ESDF到GridMap
- [ ] 阶段3: 升级MPPI使用ESDF
- [ ] 阶段4: 集成TGK拓扑算法 (可选)
- [ ] 阶段5: 完善可视化系统

---

## 🎯 核心目标

实现规划架构：
```
TGK全局拓扑路径 → MPPI局部优化(ESDF) → B样条平滑 → RViz可视化
```

**关键改进点**:
1. 修复BsplineOptimizer覆盖MPPI结果的bug
2. 添加ESDF距离场查询（O(n³)→O(1)）
3. MPPI使用ESDF梯度引导避障
4. （可选）TGK生成多样化拓扑路径

---

## 🔍 关键问题定位

### 问题1: BsplineOptimizer性能退化 🔥
- **文件**: `planner/bspline_opt/src/bspline_optimizer.cpp`
- **位置**: line ~757, 函数 `BsplineOptimizeTrajRebound()`
- **调用**: `planner/plan_manage/src/planner_manager.cpp` line ~233
- **原因**: `initControlPoints()`用线性插值覆盖MPPI优化结果
- **修复**: 改用`setControlPoints()`直接使用MPPI结果

### 问题2: MPPI障碍物成本O(n³)慢
- **文件**: `planner/path_searching/src/mppi_planner.cpp`
- **函数**: `obstacleCost()` line ~178
- **原因**: 三重循环暴力搜索障碍物距离
- **修复**: 使用ESDF进行O(1)查询

### 问题3: TopoPRM硬编码策略
- **文件**: `planner/path_searching/src/topo_prm.cpp`
- **函数**: `findTopoPaths()` line ~63
- **原因**: 仅4个方向+切线，无角点检测
- **修复**: 集成TGK算法 (BiasSampler + TopoGraphSearch)

---

## 📂 核心文件速查

### 规划管理器
```
planner/plan_manage/
├── include/plan_manage/
│   ├── planner_manager.h          # 规划管理器头文件
│   └── ego_replan_fsm.h           # 状态机头文件
├── src/
│   ├── planner_manager.cpp        # 主规划流程 (~639行)
│   │   ├─ reboundReplan()         # line 80-400, 核心规划函数
│   │   ├─ planWithTopo()          # line 599, 拓扑规划接口
│   │   └─ planWithMPPI()          # line 617, MPPI规划接口
│   └── ego_replan_fsm.cpp         # 状态机实现
└── launch/
    └── advanced_param.xml         # 参数配置文件
```

### 路径搜索
```
planner/path_searching/
├── include/path_searching/
│   ├── topo_prm.h                 # 拓扑规划器头文件
│   └── mppi_planner.h             # MPPI规划器头文件
└── src/
    ├── topo_prm.cpp               # 拓扑路径生成 (~520行)
    │   ├─ searchTopoPaths()       # line 29, 主接口
    │   ├─ findTopoPaths()         # line 63, 路径生成
    │   └─ visualizeTopoPaths()    # line 480, 可视化
    └── mppi_planner.cpp           # MPPI优化 (~472行)
        ├─ planTrajectory()        # line 40, 主接口
        ├─ rolloutTrajectory()     # line 100, 动力学模拟
        ├─ obstacleCost()          # line 178, 🔥需要优化
        └── calculateTrajectoryCost() # line 150, 成本计算
```

### B样条优化
```
planner/bspline_opt/
├── include/bspline_opt/
│   └── bspline_optimizer.h        # B样条优化器头文件
└── src/
    └── bspline_optimizer.cpp      # B样条优化 (~1100行)
        ├─ BsplineOptimizeTrajRebound() # line 757, 主优化
        └─ initControlPoints()     # 🔥BUG位置
```

### 环境地图
```
planner/plan_env/
├── include/plan_env/
│   └── grid_map.h                 # 地图头文件
└── src/
    └── grid_map.cpp               # 地图实现 (~800行)
        ├─ getInflateOccupancy()   # 占据查询
        └─ 🆕 evaluateEDT()         # 需要添加
```

---

## 🛠️ 常用命令

### Git操作
```bash
# 查看当前状态
git status
git log --oneline -5

# 切换分支
git checkout feature/esdf-mppi-upgrade

# 提交代码
git add <files>
git commit -m "feat(module): description"
git push origin feature/esdf-mppi-upgrade

# 回滚（如果出错）
git reset --hard HEAD~1
```

### 编译测试
```bash
# 清理编译
cd ~/ros_ws/ego-planner
catkin_make clean

# 完整编译
catkin_make -j4

# 只编译某个包
catkin_make --pkg ego_planner

# 查看编译错误（查看最后15行）
catkin_make 2>&1 | tail -15
```

### 运行测试
```bash
# 启动仿真
roslaunch ego_planner simple_run.launch

# 查看topic列表
rostopic list | grep -E "topo|mppi|bspline"

# 查看日志
rostopic echo /planning/bspline
rqt_console  # 图形化日志查看器
```

### 性能分析
```bash
# 查看节点CPU使用
top -p $(pgrep -f ego_planner)

# 查看topic频率
rostopic hz /planning/bspline

# 查看消息延迟
rostopic delay /planning/bspline
```

---

## 📊 关键参数速查

### MPPI参数 (mppi_planner.cpp)
```cpp
num_samples_ = 1000;          // 采样轨迹数量
horizon_steps_ = 20;          // 规划步数
dt_ = 0.1;                    // 时间步长 (秒)
lambda_ = 1.0;                // 温度参数

// 成本权重
w_obstacle_ = 100.0;          // 障碍物权重
w_smoothness_ = 10.0;         // 平滑度权重
w_goal_ = 50.0;               // 目标权重
w_velocity_ = 20.0;           // 速度权重
```

### ESDF参数 (需要添加到advanced_param.xml)
```xml
<param name="mppi/esdf_repulsive_strength" value="3.0"/>
<param name="mppi/esdf_influence_distance" value="1.0"/>
```

### 动力学限制 (advanced_param.xml)
```xml
<param name="manager/max_vel" value="2.0"/>
<param name="manager/max_acc" value="3.0"/>
<param name="manager/max_jerk" value="4.0"/>
```

---

## 🐛 调试技巧

### 添加调试输出
```cpp
// 在关键位置添加
ROS_INFO("[Module] Message with value: %.3f", value);
ROS_WARN("[Module] Warning message");
ROS_ERROR("[Module] Error message");

// 条件输出
ROS_INFO_THROTTLE(1.0, "[Module] Message every 1 second");
```

### 性能计时
```cpp
ros::Time start = ros::Time::now();
// ... 要测量的代码 ...
ros::Duration elapsed = ros::Time::now() - start;
ROS_INFO("Operation took %.3f ms", elapsed.toSec() * 1000.0);
```

### 可视化调试
```cpp
// 在visualization_中添加调试marker
visualization_->displaySphereList(points, 0.1, Eigen::Vector4d(1,0,0,1), id);
visualization_->displayLineList(path, 0.05, Eigen::Vector4d(0,1,0,1), id);
```

---

## 📈 预期性能指标

| 指标 | 基线 | 目标 | 说明 |
|------|------|------|------|
| 规划时间 | 150ms | <100ms | 主要靠ESDF优化MPPI |
| 飞行时间 | 55s | <50s | 更优的轨迹 |
| 成功率 | 95% | >98% | 更稳定的避障 |
| 最小安全距离 | 0.3m | >0.4m | 更主动的避障 |

---

## 🔗 相关文档

### 项目文档
- **CODE_ARCHITECTURE_ANALYSIS.md** - 代码框架全面分析 (必读📖)
- **DETAILED_IMPLEMENTATION_PLAN.md** - 分阶段详细实施计划 (必读📖)
- **PROGRESSIVE_UPGRADE_PLAN.md** - 渐进式升级总览

### TGK备份 (阶段4需要)
位置: `~/tgk_backup_20251001_1708/`
- TGK_INTEGRATION_SUMMARY.md - TGK完整集成说明
- TGK_QUICK_REFERENCE.md - TGK参数调优指南
- TGK_DEBUG_GUIDE.md - TGK调试流程
- bias_sampler.h/cpp - 角点检测代码
- topo_graph_search.h/cpp - 几何A*搜索代码

---

## ✅ 每日检查清单

### 开始工作前
- [ ] 确认在正确的分支 (feature/esdf-mppi-upgrade)
- [ ] git pull 同步最新代码
- [ ] 查看TODO列表了解当前任务

### 代码修改后
- [ ] 编译通过 (catkin_make)
- [ ] 运行测试 (roslaunch)
- [ ] 验证功能正常
- [ ] 添加必要注释
- [ ] 提交到Git

### 完成阶段后
- [ ] 更新进度跟踪表
- [ ] 记录性能指标
- [ ] 推送到GitHub
- [ ] 更新文档（如有需要）

---

## 🚨 紧急问题处理

### 编译失败
1. 检查错误信息 `catkin_make 2>&1 | tail -30`
2. 确认文件路径正确
3. 检查头文件包含
4. 清理重新编译 `catkin_make clean && catkin_make`

### 运行崩溃
1. 查看日志 `rqt_console` 或终端输出
2. 检查是否有段错误 (Segmentation fault)
3. 使用gdb调试 `gdb --args rosrun ...`
4. 回滚到上一个工作版本

### 飞行质量下降
1. 对比之前的参数设置
2. 检查是否引入新的bug
3. 验证MPPI结果是否被覆盖
4. 调整成本函数权重

---

## 📞 下一步行动

### 立即执行 (阶段1)
```bash
# 1. 进入Docker环境
# 2. 编译测试基线
cd ~/ros_ws/ego-planner
catkin_make clean && catkin_make -j4
roslaunch ego_planner simple_run.launch

# 3. 记录基线性能
# 4. 开始修复BsplineOptimizer bug
```

参考: **DETAILED_IMPLEMENTATION_PLAN.md** 阶段1详细步骤

---

**祝升级顺利！** 🚀

有问题随时参考详细文档或查看代码注释。
