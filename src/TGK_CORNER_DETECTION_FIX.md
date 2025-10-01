# 🐛 TGK角点检测失败诊断报告

**问题日期**: 2025-10-01  
**严重程度**: 🟡 中等（有降级方案）

---

## 🔍 问题症状

### 运行日志
```
[WARN] [TopoGraphSearch] Building graph with 0 key points  ← 核心问题
[WARN] [TopoGraphSearch] A* search failed
[WARN] [TopoPRM-TGK] TGK search failed, falling back to legacy method
```

**频率**: 几乎每次规划（27/28次）

**影响**:
- ✅ 系统仍可工作（降级到legacy方法）
- ⚠️ TGK优势未发挥（无法生成多样拓扑路径）
- ⚠️ 并行MPPI效果受限（只有1条路径时无法并行）

---

## 🔬 根因分析

### 问题1: 角点检测条件过严

**当前实现** (bias_sampler.cpp:148-152):
```cpp
// Check 2: Must be close to obstacles (near boundary)
Vector3d grad;
double dist = grid_map_->getDistanceWithGrad(pos, grad);

if (dist > sampling_radius_ * 0.5) {  // ← 问题！
    return false;  // Too far from obstacles
}
```

**分析**:
```
sampling_radius_ = 2.0m（默认）
sampling_radius_ * 0.5 = 1.0m

条件：dist > 1.0m → 不是角点
→ 只接受距离障碍物 < 1.0m 的点

问题：
1. 太严格：很多潜在角点被过滤
2. 不适配环境：稀疏障碍物环境找不到角点
3. 导致：0 key points → TGK失败
```

### 问题2: 采样密度可能不足

**当前实现** (bias_sampler.cpp:72-95):
```cpp
double search_radius = corner_detection_radius_;  // 3.0m
double step = resolution_ * 5.0;  // ← 采样步长

// 在start-goal连线附近采样
for (double t = 0; t <= 1.0; t += 0.05) {  // 20个采样点
    // ...
}
```

**分析**:
```
采样密度 = 20点 / 路径长度

日志显示路径长度 ~6-8米
实际采样点 ≈ 20点 / 7m = 2.86点/米

可能太稀疏，错过角点
```

---

## 🔧 修复方案

### 方案A: 放宽角点检测条件（推荐）⭐

**修改1**: 增加距离阈值
```cpp
// bias_sampler.cpp:152

// Before:
if (dist > sampling_radius_ * 0.5) {  // 1.0m
    return false;
}

// After:
if (dist > sampling_radius_ * 1.5) {  // 3.0m ← 放宽3倍
    return false;
}
```

**修改2**: 添加参数配置
```cpp
// bias_sampler.h - 添加成员变量
double corner_distance_threshold_;  // 角点距离阈值倍数

// bias_sampler.cpp - 构造函数
BiasSampler::BiasSampler()
    : corner_detection_radius_(3.0),
      sampling_radius_(2.0),
      corner_distance_threshold_(1.5),  // ← NEW
      // ...

// bias_sampler.cpp - init()
void BiasSampler::init(...) {
    // ...
    nh.param("bias_sampler/corner_distance_threshold", 
             corner_distance_threshold_, 1.5);
    
    ROS_INFO("  - Corner distance threshold: %.2f × sampling_radius", 
             corner_distance_threshold_);
}

// bias_sampler.cpp - isCornerPoint()
if (dist > sampling_radius_ * corner_distance_threshold_) {
    return false;
}
```

**修改3**: Launch文件配置
```xml
<!-- advanced_param.xml -->

<!-- TGK角点检测参数 -->
<param name="bias_sampler/corner_detection_radius" value="5.0" type="double"/>
<param name="bias_sampler/sampling_radius" value="3.0" type="double"/>
<param name="bias_sampler/corner_distance_threshold" value="2.0" type="double"/>
<!-- 实际阈值 = 3.0 × 2.0 = 6.0m -->
```

### 方案B: 增加采样密度

**修改**: 更密集的采样
```cpp
// bias_sampler.cpp:getTopoKeyPoints()

// Before:
for (double t = 0; t <= 1.0; t += 0.05) {  // 20点

// After:
for (double t = 0; t <= 1.0; t += 0.02) {  // 50点 ← 2.5倍
```

### 方案C: 添加诊断日志

**目的**: 了解为什么找不到角点

```cpp
// bias_sampler.cpp:getTopoKeyPoints()

vector<Vector3d> BiasSampler::getTopoKeyPoints(...) {
    vector<Vector3d> key_points;
    
    // 统计
    int total_samples = 0;
    int collision_free_samples = 0;
    int distance_rejected = 0;
    int transition_rejected = 0;
    
    // ... 采样循环 ...
    for (...) {
        total_samples++;
        
        if (!isCollisionFree(sample)) {
            continue;
        }
        collision_free_samples++;
        
        // Check 2: Distance
        if (dist > sampling_radius_ * corner_distance_threshold_) {
            distance_rejected++;
            continue;
        }
        
        // Check 3: Transitions
        if (transitions < 2) {
            transition_rejected++;
            continue;
        }
        
        // 是角点
        key_points.push_back(sample);
    }
    
    // 诊断日志
    ROS_INFO("[BiasSampler] Sampling statistics:");
    ROS_INFO("  - Total samples: %d", total_samples);
    ROS_INFO("  - Collision-free: %d (%.1f%%)", 
             collision_free_samples, 
             100.0 * collision_free_samples / total_samples);
    ROS_INFO("  - Distance rejected: %d", distance_rejected);
    ROS_INFO("  - Transition rejected: %d", transition_rejected);
    ROS_INFO("  - Final key points: %zu", key_points.size());
    
    return key_points;
}
```

---

## 📊 推荐配置

### 配置1: 宽松模式（推荐开始）
```xml
<!-- advanced_param.xml -->

<!-- 角点检测半径（搜索范围） -->
<param name="bias_sampler/corner_detection_radius" value="5.0" type="double"/>

<!-- 采样半径（角点周围采样范围） -->
<param name="bias_sampler/sampling_radius" value="3.0" type="double"/>

<!-- 距离阈值倍数（接受的最大距离 = sampling_radius × threshold） -->
<param name="bias_sampler/corner_distance_threshold" value="2.0" type="double"/>
<!-- 实际阈值 = 3.0 × 2.0 = 6.0m -->

<!-- 最大角点数量 -->
<param name="bias_sampler/max_corner_num" value="30" type="int"/>
```

**预期效果**:
- 角点检测成功率：0% → 60-80%
- 找到角点数量：0 → 5-15个
- TGK成功率：5% → 70%+

### 配置2: 标准模式（调优后）
```xml
<param name="bias_sampler/corner_detection_radius" value="4.0" type="double"/>
<param name="bias_sampler/sampling_radius" value="2.5" type="double"/>
<param name="bias_sampler/corner_distance_threshold" value="1.5" type="double"/>
<!-- 实际阈值 = 2.5 × 1.5 = 3.75m -->
```

---

## 🎯 实施优先级

### 🔴 高优先级（立即实施）

**1. 放宽距离阈值**
```cpp
// 快速修复：直接改代码
if (dist > sampling_radius_ * 1.5) {  // 改为1.5或2.0
    return false;
}
```

**时间**: 1分钟  
**效果**: 立竿见影，角点数量应该从0增加到5+

### 🟡 中优先级（今天完成）

**2. 添加参数配置**
- 添加`corner_distance_threshold_`参数
- 在launch文件中配置
- 重新编译测试

**时间**: 10分钟  
**效果**: 可调节，适配不同环境

### 🟢 低优先级（可选）

**3. 添加诊断日志**
- 了解采样统计
- 分析哪个条件过滤太多

**时间**: 15分钟  
**效果**: 帮助调优

**4. 增加采样密度**
- 从0.05改为0.02
- 2.5倍采样点

**时间**: 1分钟  
**效果**: 可能找到更多角点

---

## 🧪 验证方法

### 测试1: 快速验证
```bash
# 1. 修改代码（方案A修改1）
# 2. 编译
cd /home/he/ros_ws/test/ego-planner
catkin_make

# 3. 运行
roslaunch plan_manage run_in_sim.launch

# 4. 观察日志
# 应该看到：
# [TopoGraphSearch] Building graph with X key points  ← X > 0
# [TopoPRM] 🚀 Using TGK algorithm  ← 成功
```

### 测试2: 统计成功率
```bash
# 运行100次规划，统计：
# - TGK成功次数
# - 平均角点数量
# - 降级到legacy次数

grep "Building graph with" ~/.ros/log/latest/*.log | \
    awk '{print $NF}' | \
    awk '{sum+=$1; count++} END {print "Avg key points:", sum/count}'
```

---

## 📈 预期改进

| 指标 | 修复前 | 修复后（预期） |
|------|--------|---------------|
| **角点检测成功率** | ~5% | 70-90% |
| **平均角点数量** | 0 | 5-15个 |
| **TGK成功率** | ~5% | 70-90% |
| **多路径MPPI触发率** | 低（只有1条路径） | 高（2-5条路径） |
| **并行MPPI效果** | 受限 | 显著 |

---

## 🎉 总结

### 当前状态
- ✅ 系统功能正常（有降级方案）
- ⚠️ TGK未发挥作用（0 key points）
- ⚠️ 并行MPPI受限（缺少多样路径）

### 根本原因
- 🔴 角点检测条件过严（dist < 1.0m）
- 🟡 采样密度可能不足

### 推荐行动
1. **立即**: 放宽距离阈值（1.0m → 3.0m）
2. **今天**: 添加参数配置
3. **可选**: 添加诊断日志

### 预期效果
- TGK成功率：5% → 70%+
- 并行MPPI充分发挥
- 路径质量提升

---

**诊断完成时间**: 2025-10-01  
**建议执行人**: 开发者  
**预计修复时间**: 15分钟
