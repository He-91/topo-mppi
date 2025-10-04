# TopoPRM 大改造路线图

## 📋 目标
- **多路径生成率**: 18.75% → >60%
- **拓扑路径数量**: 平均2条 → 平均5-8条
- **计算时间**: 保持 <10ms

---

## 🎯 方案选择

### ✅ **推荐:渐进式改造 (基于Fast-Planner思想)**
- **风险**: 低
- **周期**: 4周
- **兼容性**: 与现有MPPI/B-spline完全兼容
- **成功率**: 85%

### ⚠️ **激进:完全移植Fast-Planner**
- **风险**: 高
- **周期**: 6-8周
- **需要重写**: 50%代码
- **成功率**: 60%

---

## 📅 渐进式改造时间表

### **Week 1: 改进采样**
**目标**: 从"障碍物切线点"改为"自由空间随机采样"

**实施**:
1. 修改 `topo_prm.cpp::generateTangentPoints()`
2. 新增函数:
   ```cpp
   vector<Vec3d> sampleFreeSpaceInEllipsoid(Vec3d start, Vec3d goal, int num_samples);
   bool isPointFree(Vec3d pt, double min_clearance);
   ```
3. **参数调优**:
   - `max_samples`: 50 → 100 → 150 (逐步测试)
   - `sample_inflate_radius`: 2m → 3m → 5m
   - `min_clearance`: 0.5m → 0.8m → 1.0m

**验证指标**:
- 采样点数量: ≥30个有效点
- 采样耗时: <5ms

---

### **Week 2: 构建可见性图**
**目标**: 替换"障碍物邻接"为"全局可见性图"

**实施**:
1. 新增函数:
   ```cpp
   void buildVisibilityGraph(vector<GraphNode::Ptr>& nodes);
   bool lineCollisionFree(Vec3d p1, Vec3d p2, double safety_margin);
   ```
2. **优化**:
   - 使用KD-Tree加速最近邻查询
   - 限制邻居搜索半径 (5-10m)
   - 并行碰撞检测

**参数**:
- `max_neighbors_per_node`: 5-10
- `neighbor_search_radius`: 8m
- `line_check_resolution`: 0.2m

**验证指标**:
- 平均每节点邻居数: 3-6个
- 图构建耗时: <3ms

---

### **Week 3: 深度优先搜索**
**目标**: 从"贪心单路径"改为"DFS多路径"

**实施**:
1. 重写 `searchPaths()`:
   ```cpp
   void depthFirstSearch(vector<GraphNode::Ptr>& visited);
   vector<Path> searchMultiplePaths(int max_paths);
   ```
2. **剪枝策略**:
   - 路径长度 > 直线距离 * 4.0 时剪枝
   - DFS深度 > 20层时剪枝
   - 已找到50条路径时停止

**参数**:
- `max_raw_paths`: 50
- `max_path_length_ratio`: 4.0
- `max_dfs_depth`: 20

**验证指标**:
- 原始路径数: 30-50条
- DFS耗时: <5ms

---

### **Week 4: 拓扑去重**
**目标**: 过滤相同拓扑类的路径

**实施**:
1. 实现Fast-Planner的等价性判断:
   ```cpp
   bool sameTopoPath(Path& p1, Path& p2);
   vector<Path> pruneEquivalentPaths(vector<Path>& raw_paths);
   ```
2. **优化**:
   - 路径离散化: 统一采样30个点
   - 并行比较: 使用OpenMP

**参数**:
- `discretize_points_num`: 30
- `topo_threshold`: 0.0 (完全等价)

**验证指标**:
- 唯一拓扑路径: 5-10条
- 去重耗时: <2ms

---

## 📊 预期效果对比

| 指标 | 当前 | 改造后 |
|------|------|--------|
| 多路径生成率 | 18.75% | **>60%** |
| 平均路径数 | 1.2条 | **6-8条** |
| MPPI触发率 | 18.75% | **>60%** |
| 总耗时 | 3-5ms | **8-10ms** |
| rebound次数 | 38次 | **<15次** |

---

## 🔧 关键代码示例

### 1. 椭球采样
```cpp
Vec3d sampleInEllipsoid(Vec3d start, Vec3d goal, double inflate) {
    Vec3d center = 0.5 * (start + goal);
    double semi_major = 0.5 * (goal - start).norm() + inflate;
    
    // 在椭球坐标系下均匀采样
    double theta = 2 * M_PI * rand_uniform();
    double phi = acos(2 * rand_uniform() - 1);
    double r = pow(rand_uniform(), 1.0/3.0) * semi_major;
    
    Vec3d pt_local(r * sin(phi) * cos(theta),
                   r * sin(phi) * sin(theta),
                   r * cos(phi));
    
    // 变换到世界坐标系
    return rotation * pt_local + center;
}
```

### 2. DFS多路径
```cpp
void depthFirstSearch(vector<GraphNode::Ptr>& visited) {
    GraphNode::Ptr cur = visited.back();
    
    // 到达目标
    if (cur->id == goal_id) {
        savePath(visited);
        return;
    }
    
    // 递归搜索
    for (auto& neighbor : cur->neighbors) {
        if (!isVisited(neighbor, visited)) {
            visited.push_back(neighbor);
            depthFirstSearch(visited);
            if (raw_paths.size() >= max_paths) return;
            visited.pop_back();
        }
    }
}
```

### 3. 拓扑去重
```cpp
bool sameTopoPath(Path& p1, Path& p2) {
    vector<Vec3d> pts1 = discretize(p1, 30);
    vector<Vec3d> pts2 = discretize(p2, 30);
    
    for (int i = 0; i < 30; i++) {
        if (!lineVisible(pts1[i], pts2[i])) {
            return false;  // 不同拓扑类
        }
    }
    return true;
}
```

---

## 🚨 风险控制

### **关键风险点**:
1. **采样点太少** → 图不连通 → 无路径
   - **缓解**: 动态增加采样数 (50→100→150)
   
2. **DFS爆栈** → 路径过长
   - **缓解**: 限制DFS深度 (max_depth=20)
   
3. **去重太严** → 有效路径被误删
   - **缓解**: 放宽topo_threshold (0.0→0.5)

### **回退方案**:
每周保留Git分支,测试失败可立即回滚

---

## 📈 测试方案

### **Week 1-4 每周测试**:
```bash
# 运行测试
roslaunch plan_manage run_in_sim.launch

# 记录数据到test_weekX.md
# 关键指标:
# 1. 采样点数量
# 2. 图节点/边数量
# 3. 原始路径数
# 4. 过滤后路径数
# 5. MPPI触发次数
# 6. rebound次数
```

### **对比基线**:
- Week 0: test1.md (当前18.75%多路径率)
- Week 4: 目标 >60%多路径率

---

## 💡 快速实施建议

### **如果你只有1周时间**:
集中实施 **Week 1 + Week 3**:
1. 改进采样 (椭球随机采样)
2. DFS多路径搜索

**预期效果**: 多路径率 18.75% → 40%

### **如果你有1个月时间**:
完整实施 Week 1-4

**预期效果**: 多路径率 18.75% → 65%

---

## 📚 参考资料

### **Fast-Planner核心论文**:
1. [RA-L 2019] Robust and Efficient Quadrotor Trajectory Generation
2. [ICRA 2020] Robust Real-time UAV Replanning Using Guided Gradient-based Optimization and Topological Paths

### **关键源文件**:
- `fast_planner/path_searching/src/topo_prm.cpp`
- `fast_planner/path_searching/include/path_searching/topo_prm.h`

### **GitHub地址**:
- https://github.com/HKUST-Aerial-Robotics/Fast-Planner

---

## ✅ 下一步行动

1. **立即**: 阅读 Fast-Planner 的 topo_prm.cpp (300行核心代码)
2. **本周**: 实施 Week 1 采样改进
3. **本月**: 完成 Week 1-4 全部改造
4. **验证**: 在test1.md相同场景下测试,对比效果

---

**记住**: 你不是从零开始,而是在一个已经部分工作的系统上改进。渐进式改造,每周验证,保证稳定性!
