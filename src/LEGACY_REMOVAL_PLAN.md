# Legacy TopoPRM删除计划

**日期**: 2025-10-02  
**目标**: 删除Legacy TopoPRM算法，完全切换到TGK

---

## 📊 当前状态评估

### TGK性能（来自test1.md和PROJECT_STATUS.md）
- ✅ Corner Detection: 100%成功率（20 key points）
- ✅ A*搜索: **87%成功率**
- ✅ 多路径生成: 1-3条路径（平均1.56条）
- ✅ Parallel MPPI: 100%成功
- ✅ **端到端成功率: ~95%**

### 判定
**TGK已足够稳定，可以删除Legacy作为主要算法** ✅

---

## 🗂️ Legacy代码位置

### 主要文件
**文件**: `planner/path_searching/src/topo_prm.cpp`

### 需要删除的函数（大约400行）

#### 1. 主入口函数
- `findTopoPaths()` (line 105-231) - Legacy主算法

#### 2. 路径生成函数
- `generateAlternativePath()` (line 237-284) - 4方向路径
- `generateCircularPath()` (line 454-492) - 圆形绕行
- `generateVerticalPath()` (line 494-522) - 垂直绕行  
- `generateTangentPoints()` (line 524-563) - 切线路径

#### 3. 辅助函数（保留）
- `isPathValid()` - TGK也在用 ✅
- `isLineCollisionFree()` - TGK也在用 ✅
- `calculatePathCost()` - TGK也在用 ✅
- `calculateSmoothnessCost()` - TGK也在用 ✅
- `calculateObstacleCost()` - TGK也在用 ✅
- `selectBestPath()` - TGK也在用 ✅
- `visualizeTopoPaths()` - TGK也在用 ✅

---

## ✂️ 删除方案

### 方案A: 完全删除（推荐）✅
**操作**:
1. 删除fallback逻辑
2. 删除use_tgk_algorithm_参数（永远true）
3. 删除findTopoPaths()及其4个路径生成函数
4. 保留所有辅助函数（TGK复用）

**代码量**: 删除约400行

**优点**:
- ✅ 代码简洁
- ✅ 维护成本低
- ✅ 强制使用TGK（符合项目目标）

**风险**: 
- ⚠️ 如果TGK失败（13%概率），直接无路径
- ✅ 但测试显示95%端到端成功率可接受

### 方案B: 保留fallback（保守）
**操作**:
1. 保留use_tgk_algorithm_=false选项
2. 保留所有Legacy代码
3. 只在配置文件中默认启用TGK

**优点**:
- ✅ 可随时切回Legacy调试

**缺点**:
- ❌ 维护两套代码
- ❌ 代码冗余
- ❌ 违背项目目标（删除Legacy）

---

## 🎯 推荐方案：方案A（完全删除）

### 理由
1. ✅ TGK成功率87%，端到端95%，已满足生产要求
2. ✅ Parallel MPPI能弥补路径质量问题
3. ✅ 有完整备份（topo_prm.cpp.backup_before_legacy_removal）
4. ✅ 符合项目最终目标

---

## 📝 删除操作清单

### Step 1: 备份文件 ✅ 已完成
```bash
cp topo_prm.cpp topo_prm.cpp.backup_before_legacy_removal
```

### Step 2: 修改searchTopoPaths()
**删除**:
```cpp
// 删除整个if-else分支
if (use_tgk_algorithm_) {
    // TGK...
} else {
    candidate_paths = findTopoPaths(start, goal);  // ❌ 删除这个分支
}
```

**简化为**:
```cpp
// 只保留TGK
vector<vector<Vector3d>> raw_paths;
bool tgk_success = topo_graph_search_->searchTopoPaths(start, goal, raw_paths);

if (!tgk_success || raw_paths.empty()) {
    ROS_WARN("[TopoPRM-TGK] TGK search failed, no paths available");
    return false;  // 直接失败，不fallback
}

// 转换为TopoPath...
```

### Step 3: 删除fallback日志
删除:
```cpp
ROS_WARN("[TopoPRM-TGK] TGK search failed, falling back to legacy method");
candidate_paths = findTopoPaths(start, goal);  // ❌ 删除
```

### Step 4: 删除use_tgk_algorithm_变量
**头文件** (topo_prm.h):
```cpp
bool use_tgk_algorithm_;  // ❌ 删除这个成员变量
```

**构造函数**:
```cpp
use_tgk_algorithm_(true)  // ❌ 删除初始化
```

**init()函数**:
```cpp
nh.param("topo_prm/use_tgk_algorithm", use_tgk_algorithm_, true);  // ❌ 删除
ROS_INFO("[TopoPRM] 🚀 TGK algorithm: %s", ...);  // ❌ 删除日志
```

### Step 5: 删除Legacy函数（400行）
删除以下函数定义：
```cpp
❌ vector<TopoPath> findTopoPaths(...)
❌ vector<Vector3d> generateAlternativePath(...)
❌ vector<Vector3d> generateCircularPath(...)
❌ vector<Vector3d> generateVerticalPath(...)
❌ vector<Vector3d> generateTangentPoints(...)
```

### Step 6: 删除头文件声明
**文件**: `planner/path_searching/include/path_searching/topo_prm.h`

删除函数声明：
```cpp
❌ bool use_tgk_algorithm_;
❌ vector<TopoPath> findTopoPaths(...);
❌ vector<Vector3d> generateAlternativePath(...);
❌ vector<Vector3d> generateCircularPath(...);
❌ vector<Vector3d> generateVerticalPath(...);
❌ vector<Vector3d> generateTangentPoints(...);
```

### Step 7: 更新日志信息
**修改**:
```cpp
// 从:
ROS_INFO("[TopoPRM] Using legacy TopoPRM algorithm");

// 改为:
// (直接删除，因为只有TGK了)
```

---

## ✅ 验证步骤

### 编译测试
```bash
cd /home/he/ros_ws/test/ego-planner
catkin_make -j4
```
**预期**: 0错误，0警告

### 运行测试
```bash
source devel/setup.bash
roslaunch plan_manage run_in_sim.launch
```

**预期结果**:
- ✅ 只看到TGK相关日志
- ✅ 不再有"fallback to legacy"警告
- ✅ 成功率维持95%左右
- ❌ 不应有崩溃

### 失败处理
如果出现问题：
```bash
# 恢复备份
cp topo_prm.cpp.backup_before_legacy_removal topo_prm.cpp
catkin_make -j4
```

---

## 📊 预期效果

### 代码规模
- 删除: ~400行Legacy代码
- 保留: ~160行辅助函数（TGK复用）
- 净减少: **70%代码量**

### 系统行为
- TGK成功(87%): 正常规划 ✅
- TGK失败(13%): 直接报错，不fallback ⚠️
- 端到端成功率: 维持95% ✅

### 维护成本
- Legacy维护成本: 0（已删除）
- TGK改进空间: 多路径生成优化

---

## 🚀 执行时机

**建议**: **立即执行**

**理由**:
1. ✅ TGK已验证稳定（test1.md证明）
2. ✅ 有完整备份保障
3. ✅ 是项目最终目标
4. ✅ GPU问题已解决，系统稳定

---

**准备好了吗？请确认后我立即执行删除！** 🎯
