# Fast-Planner PRM v3.0 升级总结

## 📊 测试结果对比

### 性能提升（test1.md 31次规划）

| 版本 | 多路径率 | 平均路径数 | MPPI触发率 | 主要特点 |
|------|----------|-----------|-----------|----------|
| **v2.0 (切线点法)** | 18.75% | 1.19 | 18.75% | 障碍物切线点，局部贪心搜索 |
| **v3.0 (PRM)** | **58.06%** | **1.87** | **58.06%** | 椭球采样，全局可见性图，DFS多路径 |
| **提升** | **+39.31%** | **+0.68** | **+39.31%** | 3倍性能提升 ✅ |

### 路径分布详情

```
v3.0路径分布 (31次规划):
  1条: 13次 (41.9%) ████████████████
  2条: 12次 (38.7%) ███████████████  ← 最常见
  3条:  3次 ( 9.7%) ███
  4条:  3次 ( 9.7%) ███
```

### 核心改进点

✅ **多路径生成率**: 从18.75% → 58.06% (+39.31%)  
✅ **MPPI并行优化**: 从几乎不触发 → 58%的规划运行MPPI  
✅ **路径多样性**: 最多生成4条拓扑不同的路径  
✅ **稳定性**: 椭球采样100%成功（100个有效点）  
✅ **DFS覆盖**: 稳定搜索50条原始路径（满载）

---

## 🔧 技术实现

### v2.0 → v3.0 算法对比

#### v2.0: 障碍物切线点法
```
问题: 
1. 切线点距离障碍物太近 → start→tangent碰撞率100%
2. 局部贪心 → 仅生成1条路径
3. 依赖障碍物检测 → 开阔区域失效
```

#### v3.0: Fast-Planner PRM
```cpp
STEP 1: 椭球自由空间采样
  - 在起点-终点椭球内随机采样100个点
  - 检查clearance_=0.8m安全距离
  - 成功率: 100% (test1.md验证)

STEP 2: 可见性图构建
  - 创建start/goal节点
  - 采样点转GraphNode (总计102节点)
  - 可见性连接 (半径限制10m)

STEP 3: DFS多路径搜索
  - 深度优先搜索 (递归)
  - 限制50条原始路径
  - 深度限制20层

STEP 4: 拓扑等价性去重
  - 离散化30点
  - 检查可见性不同 → 不同拓扑类
  - 去重: 50条 → 5-13条唯一路径

STEP 5: 最优路径选择
  - 过滤超长路径 (>2.5倍最短路径)
  - 保留8条最短路径
  - 实际输出: 1-4条路径
```

---

## 🎯 关键参数配置

### TopoPRM PRM参数

```cpp
// topo_prm.cpp 构造函数
max_raw_paths_ = 50;        // DFS最大搜索路径数
reserve_num_ = 8;           // 保留路径数量
clearance_ = 0.8;           // 采样点安全距离 (m)
sample_inflate_ = 3.0;      // 椭球膨胀因子
ratio_to_short_ = 2.5;      // 最长路径=最短路径*2.5
discretize_points_num_ = 30; // 拓扑比对离散化点数
```

### 参数调优建议

| 参数 | 当前值 | 建议调整 | 效果 |
|------|--------|---------|------|
| `sample_inflate_` | 3.0 | 3.5-4.0 | 扩大采样区域，增加路径多样性 |
| `ratio_to_short_` | 2.5 | 3.0-3.5 | 保留更多较长路径 |
| `discretize_points_num_` | 30 | 20-25 | 减少去重严格度，保留更多路径 |
| `max_raw_paths_` | 50 | 100 | 增加DFS搜索深度 |

**优化目标**: 将多路径率从58% → 75%+

---

## 📁 修改文件清单

### 核心代码修改

1. **topo_prm.h** (60行新增)
   - 添加GraphNode结构体
   - 6个PRM参数成员变量
   - PRM数据结构: graph_nodes_, raw_paths_
   - 8个新方法声明

2. **topo_prm.cpp** (400行新增)
   - searchTopoPaths() 完全重写
   - 实现5步PRM流程
   - 添加椭球采样、可见性图、DFS、拓扑去重等功能
   - findTopoPaths() 重命名为 findTopoPathsLegacy() (备用)

3. **编译修复**
   - 删除头文件重复声明
   - 添加size_t类型转换 (3处)

### 文档更新

- **TOPO_UPGRADE_ROADMAP.md**: 详细改造计划（已完成Week 1-4）
- **PRM_V3_UPGRADE_SUMMARY.md**: 本总结文档（新建）

---

## 🐛 已知问题与改进方向

### 问题1: 多路径率不稳定

**现象**: 
- 部分规划生成4条路径 ✅
- 部分规划仅1条路径 ⚠️

**分析**:
```
STEP 3: DFS稳定生成50条原始路径
STEP 4: 去重后5-13条唯一路径 (损失大)
STEP 5: 最终选择1-4条路径 (不稳定)
```

**根本原因**: 拓扑去重过严（30点离散化）

### 改进方向1: 放宽拓扑等价性判断 ⭐

```cpp
// 当前: 30点离散化，任意一对点不可见→不同拓扑
discretize_points_num_ = 30;

// 建议: 20点离散化，减少误判
discretize_points_num_ = 20;
```

**预期效果**: 去重后保留更多路径 (5-13条 → 8-18条)

### 改进方向2: 增加采样区域

```cpp
// 当前: 3倍椭球膨胀
sample_inflate_ = 3.0;

// 建议: 4倍膨胀，覆盖更多绕行路径
sample_inflate_ = 4.0;
```

**预期效果**: 生成更多样化的路径

### 改进方向3: 保留更多较长路径

```cpp
// 当前: 最长路径=最短路径*2.5
ratio_to_short_ = 2.5;

// 建议: 放宽到3.5
ratio_to_short_ = 3.5;
```

**预期效果**: STEP 5过滤更少路径

---

## 🚀 下一步优化计划

### Phase 1: 参数调优（优先级高）

1. **放宽拓扑去重**: discretize_points_num_ = 20
2. **扩大椭球采样**: sample_inflate_ = 4.0
3. **保留更多路径**: ratio_to_short_ = 3.5
4. **运行测试**: 观察多路径率是否 >70%

### Phase 2: 算法优化（中期）

1. **自适应采样**: 根据环境复杂度动态调整sample_num
2. **局部重采样**: 在关键障碍物区域加密采样
3. **路径质量评估**: 添加路径平滑度、曲率等指标

### Phase 3: 系统集成（长期）

1. **MPPI参数自适应**: 根据路径数量动态调整采样数
2. **B-spline平滑优化**: 减少rebound次数
3. **实时性能优化**: 减少DFS搜索时间 (<5ms)

---

## 📝 Git提交信息

```bash
git add planner/path_searching/include/path_searching/topo_prm.h
git add planner/path_searching/src/topo_prm.cpp
git add TOPO_UPGRADE_ROADMAP.md
git add PRM_V3_UPGRADE_SUMMARY.md
git add test1.md

git commit -m "feat: Implement Fast-Planner PRM v3.0 for multi-topology path generation

Major improvements:
- Replace obstacle tangent point method with PRM-based approach
- Multi-path generation rate: 18.75% → 58.06% (+39.31%)
- Average paths per planning: 1.19 → 1.87 (+57%)
- MPPI parallel optimization trigger rate: 18.75% → 58.06%

Technical changes:
- STEP 1: Ellipsoid free space sampling (100 points, 100% success)
- STEP 2: Visibility graph construction (102 nodes)
- STEP 3: DFS multi-path search (up to 50 raw paths)
- STEP 4: Topology equivalence pruning (5-13 unique paths)
- STEP 5: Short path selection (1-4 final paths)

Parameters:
- sample_inflate_ = 3.0 (椭球膨胀因子)
- clearance_ = 0.8m (采样安全距离)
- discretize_points_num_ = 30 (拓扑比对精度)
- ratio_to_short_ = 2.5 (路径长度过滤)

Performance verified by test1.md (31 planning cycles):
- 1 path: 13 times (41.9%)
- 2 paths: 12 times (38.7%)
- 3 paths: 3 times (9.7%)
- 4 paths: 3 times (9.7%)

Next optimizations:
- Relax topology pruning (discretize_points_num_ = 20)
- Expand sampling area (sample_inflate_ = 4.0)
- Preserve more long paths (ratio_to_short_ = 3.5)

References:
- Based on Fast-Planner PRM implementation
- Week 1-4 roadmap completed (TOPO_UPGRADE_ROADMAP.md)"
```

---

## 📚 参考资料

- **Fast-Planner**: https://github.com/HKUST-Aerial-Robotics/Fast-Planner
- **TGK-Planner**: https://github.com/ZJU-FAST-Lab/TGK-Planner
- **TOPO_UPGRADE_ROADMAP.md**: 详细4周改造计划
- **test1.md**: 完整测试日志（31次规划，1725行）

---

**总结**: PRM v3.0成功将多路径率从18.75%提升到58.06%，实现3倍性能提升。下一步通过参数调优（放宽拓扑去重、扩大采样）预期可达到70%+多路径率。✅
