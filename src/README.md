# EGO-Planner with TGK & Parallel MPPI

> **基于TGK拓扑规划和并行MPPI优化的自主无人机导航系统**

**最后更新**: 2025-10-03  
**状态**: TGK基础实现完成,性能优化中

---

## 快速导航

- 📚 **[ALGORITHM.md](./ALGORITHM.md)** - 完整算法说明、技术细节、改进方向
- 📊 **[TEST1_DETAILED_ANALYSIS.md](./TEST1_DETAILED_ANALYSIS.md)** - 测试数据分析、错误诊断
- 📝 **[test1.md](./test1.md)** - 原始测试日志 (274KB)

---

## 系统架构

```
深度相机30Hz → GPU渲染 → 占用栅格 → Fast Sweeping ESDF (2ms)
                                            ↓
起点/终点 → TGK Planner (85%) ──┬→ Parallel MPPI → B-Spline → 轨迹
              ↓                  │
         Legacy TopoPRM (15%)────┘
```

---

## 关键性能

| 指标 | 当前 | 目标 |
|------|------|------|
| TGK成功率 | **85%** | 95% |
| 多路径生成 | **13%** | 30% |
| 系统成功率 | **100%** | 100% |
| CPU占用 | **40%** | <50% |
| ESDF更新 | **2ms** | <5ms |

---

## **关键问题：为什么照搬TGK但去掉动力学约束反而更差？**

### 你的直觉是对的！
理论上，**没有动力学约束 = 更多路径选择**。但实际上我们的实现根本不是"照搬TGK"。

### 原始TGK-Planner的核心
```cpp
// 原始TGK = Kinodynamic RRT*
## 核心算法

### 1. Fast Sweeping ESDF
- **复杂度**: O(N) vs 传统O(N²)
- **更新时间**: ~2ms
- **分辨率**: 0.1m

### 2. TGK Planner
**流程**:
```
Corner检测 → A*图构建 → 多路径搜索 → 路径平滑
```

**参数**:
- `max_corner_num = 40` (检测到的关键拓扑点)
- `connection_radius = 20m` (节点连接半径)
- `path_check_step = 0.3m` (碰撞检测步长)

**与原始TGK论文的差异**: 详见 [ALGORITHM.md](./ALGORITHM.md) 第5节

### 3. Parallel MPPI
对每条拓扑路径并行优化,选择最优:
```cpp
normalized_cost = mppi_cost / path_length
```

---
```

### 我们的"简化版"TGK
```cpp
// 我们的实现 = 静态几何A*
bool TopoGraphSearch::searchTopoPaths(...) {
    // 1. 固定连接半径 (死的)
    connection_radius_ = 20.0;  // 硬编码
    
    // 2. 简单欧氏距离
    double dist = (to - from).norm();
    
    // 3. 密集碰撞检查
    for (double t = 0; t <= 1.0; t += 0.3/length) {
        if (grid_map_->getInflateOccupancy(pt)) return false;
    }
}
```

### **核心差异对比表**

| 特性 | 原始TGK (KRRT*) | 我们的实现 | 影响 |
|------|----------------|-----------|------|
## 主要问题

### 1. TGK多路径生成率低 (13%)
**现象**: 27次测试仅3次生成多条路径  
**原因**: A*搜索策略单一,路径相似度高  
**目标**: 提升到30%

### 2. TGK成功率待提升 (85%)
**现象**: 15%失败,触发Legacy fallback  
**原因**: A*图不连通 - "start can see goal: NO"  
**目标**: 提升到95%

### 3. 与TGK论文的差异
我们的实现是**简化版几何A***,而非真正的**KRRT***:

| 特性 | 原始TGK | 我们 |
|------|---------|------|
| 路径验证 | BVP动力学 | 密集几何检查(0.3m) |
| 连接策略 | 动态半径(15-20m) | 固定20m |
| 图优化 | Rewire | 静态构建 |

**结论**: 很多动力学可行的路径被我们过严的几何检查拒绝了!

详见 [ALGORITHM.md 第5节](./ALGORITHM.md#5-与tgk论文的差异分析)

---

## 改进计划

### 短期 (1-2周)
- [ ] 增加max_corner_num到60
- [ ] 动态调整connection_radius
- [ ] 桥接节点算法

### 中期 (1个月)
- [ ] 引入BVP求解
- [ ] 删除Legacy依赖
- [ ] 完整K-shortest paths

---

## 核心代码位置

```
planner/path_searching/
├── bias_sampler.cpp          # Corner检测
├── topo_graph_search.cpp     # TGK A*搜索
└── topo_prm.cpp              # TGK+Legacy集成

planner/plan_manage/
└── planner_manager.cpp       # Parallel MPPI

planner/plan_env/
└── grid_map.cpp              # Fast Sweeping ESDF
```

---

## 测试结果

**测试场景**: 随机森林,27次重规划
```
✅ TGK成功:  23次 (85%)
⚠️  TGK失败:   4次 (15%) → Legacy兜底
✅ 多路径:    3次 (13%)
✅ 系统成功: 27次 (100%)
```

详见 [TEST1_DETAILED_ANALYSIS.md](./TEST1_DETAILED_ANALYSIS.md)

---

## 启动

```bash
# 编译
cd ~/ros_ws/test/ego-planner
catkin build

# 运行仿真
roslaunch plan_manage test_new_algorithms.launch

# 查看拓扑路径
rviz  # 订阅 /topo_paths
```

---

**项目状态**: ✅ 基础功能完成 | ⚠️ 性能优化中

4. 测试达到95%后删除Legacy

