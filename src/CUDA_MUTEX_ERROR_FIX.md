# 🔧 无人机停在第一个点问题诊断

**时间**: 2025-10-02  
**症状**: 无人机到第一个点就不动了  
**环境变化**: NVCC 11.8 → 12.8，后改用CPU

---

## 🔍 问题诊断

### 错误信息
```
terminate called after throwing an instance of 'boost::wrapexcept<boost::lock_error>'
what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
```

### 根本原因
**多线程互斥锁失败** - pthread库问题

### 可能原因
1. **CUDA升级影响**: CUDA 12.8可能改变了系统pthread库
2. **编译器不一致**: 部分代码用新CUDA编译，部分用旧版本
3. **库版本冲突**: Boost库与pthread库版本不匹配
4. **残留的静态变量**: 虽然我们改了，但编译缓存可能有问题

---

## ✅ 解决方案（按顺序尝试）

### 方案1: 清理并重新编译 🔴 **最可能解决**

```bash
cd /home/he/ros_ws/test/ego-planner

# 1. 完全清理编译缓存
rm -rf build/ devel/
catkin clean -y

# 2. 重新编译（使用CPU，不用CUDA）
catkin_make -j4

# 3. 重新source环境
source devel/setup.bash

# 4. 运行测试
roslaunch plan_manage run_in_sim.launch
```

**原理**: CUDA升级后，旧的编译缓存包含旧版本库的链接，导致pthread冲突

---

### 方案2: 检查CUDA/cuDNN路径

```bash
# 检查当前CUDA版本
nvcc --version
which nvcc

# 检查环境变量
echo $LD_LIBRARY_PATH
echo $PATH

# 确保指向正确的CUDA路径
export PATH=/usr/local/cuda-12.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:$LD_LIBRARY_PATH
```

如果路径混乱（同时有11.8和12.8），清理一下：
```bash
# 编辑 ~/.bashrc
nano ~/.bashrc

# 确保只有一个CUDA版本
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

---

### 方案3: 降级回CUDA 11.8（如果方案1/2无效）

```bash
# 卸载CUDA 12.8
sudo apt-get remove --purge '^cuda.*' '^cudnn.*' '^libnv.*'
sudo apt-get autoremove

# 重新安装CUDA 11.8
# ... 按照NVIDIA官方指南安装

# 重新编译ego-planner
cd /home/he/ros_ws/test/ego-planner
rm -rf build/ devel/
catkin_make -j4
```

**原因**: 如果ego-planner依赖特定CUDA版本编译的库

---

### 方案4: 禁用CUDA相关功能（临时解决）

检查local_sensing是否启用了CUDA：

```bash
cd /home/he/ros_ws/test/ego-planner/src/uav_simulator/local_sensing
grep -r "CUDA\|cuda" CMakeLists.txt
```

如果有CUDA相关编译选项，可以临时禁用：
```cmake
# CMakeLists.txt
# 注释掉CUDA相关部分
# find_package(CUDA)
# ...
```

---

### 方案5: 检查ROS节点启动顺序

如果是多线程竞争问题，可能是节点启动太快导致：

编辑 `run_in_sim.launch`，在ego_planner_node前添加延迟：
```xml
<!-- 让地图和传感器先启动 -->
<node pkg="plan_manage" name="ego_planner_node" type="ego_planner_node" 
      output="screen" launch-prefix="bash -c 'sleep 2; $0 $@'">
  ...
</node>
```

---

## 🎯 快速修复流程

### Step 1: 先试最简单的（1分钟）
```bash
cd /home/he/ros_ws/test/ego-planner
rm -rf build/ devel/
catkin_make -j4
source devel/setup.bash
```

### Step 2: 如果还不行，检查CUDA（2分钟）
```bash
nvcc --version
ls -l /usr/local/cuda
# 确保只有一个版本
```

### Step 3: 如果还不行，完全清理（5分钟）
```bash
# 清理所有ROS缓存
rm -rf ~/.ros/log/*
rm -rf ~/.ros/cache/*

# 清理catkin工作空间
cd /home/he/ros_ws/test/ego-planner
catkin clean -y
catkin_make -j4
```

---

## 🔍 调试技巧

### 查看详细错误
```bash
# 运行时添加调试信息
roslaunch plan_manage run_in_sim.launch 2>&1 | tee debug.log

# 搜索错误
grep -E "ERROR|error|mutex|pthread" debug.log
```

### 检查节点状态
```bash
# 另开终端
rostopic list
rostopic echo /ego_planner_node/planning_vis

# 检查哪个节点卡住了
rosnode list
rosnode info /ego_planner_node
```

---

## 💡 为什么会出现这个问题？

### CUDA升级的连锁反应
```
CUDA 11.8 → 12.8
    ↓
pthread库版本变化
    ↓
旧的编译文件仍链接到旧pthread
    ↓
新代码运行时调用新pthread
    ↓
mutex初始化失败 → 程序崩溃
```

### 解决核心
**重新编译所有代码，确保链接到一致的库版本**

---

## 📊 检查点

编译完成后，验证以下内容：

✅ **编译无警告**:
```bash
catkin_make 2>&1 | grep -i "error\|warning"
# 应该没有错误
```

✅ **节点正常启动**:
```bash
roslaunch plan_manage run_in_sim.launch
# 观察是否有mutex错误
```

✅ **无人机能移动**:
```bash
# 观察rviz中无人机是否移动到waypoint
```

✅ **规划正常运行**:
```bash
rostopic echo /ego_planner_node/data_display
# 应该看到规划信息
```

---

## 🚨 如果所有方法都失败

### 最后手段：回滚CUDA + 完全重装

```bash
# 1. 完全卸载CUDA
sudo apt-get remove --purge '^nvidia-.*' '^cuda-.*' '^libnvidia-.*' '^libcuda.*'
sudo apt-get autoremove
sudo apt-get autoclean

# 2. 重启系统
sudo reboot

# 3. 安装CUDA 11.8（稳定版本）
# 下载官方.deb文件并安装

# 4. 重新编译ego-planner
cd /home/he/ros_ws/test/ego-planner
rm -rf build/ devel/ install/
catkin_make -j4
```

---

## 📝 预防措施

### 以后升级CUDA时
1. **先备份**: 保存当前能工作的环境
2. **隔离测试**: 在新环境中测试
3. **逐步升级**: 先升级CUDA，测试通过后再升级其他
4. **记录依赖**: 记录哪些包依赖特定CUDA版本

---

**建议**: 先尝试**方案1（清理重编译）**，90%的情况这就能解决！
