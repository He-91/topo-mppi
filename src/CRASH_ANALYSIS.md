# 🔴 崩溃问题分析报告

**时间**: 2025-10-03  
**错误**: `std::bad_alloc` (内存耗尽)  
**位置**: K-shortest paths第2次A*搜索

---

## 🔍 崩溃现场

```
[INFO] [TopoGraphSearch] Path 1: 3 waypoints  ✅ 第1条路径成功
[INFO] [TopoGraphSearch] A* search: 60 nodes, start_id=0, goal_id=59  
                          ⬆️ 开始第2次搜索
terminate called after throwing an instance of 'std::bad_alloc'
  what():  std::bad_alloc  ❌ 内存分配失败!
```

---

## 🐛 根本原因

### 问题: 阻塞节点后的索引错乱

**我们的代码** (`extractMultiplePaths()`):
```cpp
// 删除被阻塞的节点 (从后向前删除)
for (int id : blocked_node_ids) {
    node_pool_.erase(node_pool_.begin() + id);  // ❌ 删除节点
}

// 第2次A*搜索
astarSearch(start, goal, alt_path);  // ❌ start/goal索引失效!
```

**问题分析**:
```
原始node_pool_:
  [0]=start, [1]=corner1, [2]=corner2, ..., [61]=goal

阻塞节点2后:
  [0]=start, [1]=corner1, [3]=corner3, ..., [60]=goal
                          ⬆️ 索引跳过2

astarSearch(start, goal):
  int start_id = 0;  ✅ 正确
  int goal_id = node_pool_.size() - 1;  ❌ 现在是60,但应该是61!
                                        ❌ goal_id指向了corner,不是goal!
```

**连锁反应**:
1. goal_id指向错误节点 (corner而非goal)
2. A*搜索错误的目标 → 无限扩展
3. open_set无限增长 → 内存耗尽 → `bad_alloc`

---

## 🔧 修复方案

### 方案1: 不删除节点,改用标记 (推荐⭐⭐⭐⭐⭐)

```cpp
// ❌ 删除节点 (导致索引错乱)
node_pool_.erase(node_pool_.begin() + id);

// ✅ 标记节点为被阻塞 (保持索引稳定)
node_pool_[id].is_blocked = true;

// A*搜索时跳过被阻塞节点
for (size_t i = 0; i < node_pool_.size(); ++i) {
    if (node_pool_[i].is_blocked) continue;  // 跳过
    // ... 正常扩展
}
```

**优势**:
- ✅ 索引始终稳定 (start=0, goal=最后)
- ✅ 不需要备份/恢复节点
- ✅ 性能更好 (无内存分配)

---

### 方案2: 修复索引查找 (备选⭐⭐⭐)

```cpp
// 在astarSearch()中,不用固定索引
int start_id = findNodeByPosition(start);  // 查找start节点
int goal_id = findNodeByPosition(goal);    // 查找goal节点
```

**劣势**:
- ⚠️ 每次搜索都要查找,性能差
- ⚠️ 代码复杂

---

## ✅ 立即修复 (方案1)

### 步骤1: 为TopoNode添加blocked标志

**文件**: `topo_graph_search.h`
```cpp
struct TopoNode {
    Eigen::Vector3d pos;
    int node_id;
    int topo_class;
    double g_cost;
    double h_cost;
    int parent_id;
    bool is_blocked;  // 🚀 NEW: 标记节点是否被阻塞
    
    TopoNode() : node_id(-1), topo_class(0), 
                 g_cost(0), h_cost(0), parent_id(-1),
                 is_blocked(false) {}  // 默认不阻塞
};
```

---

### 步骤2: 修改extractMultiplePaths()

**文件**: `topo_graph_search.cpp`
```cpp
// ❌ 旧代码: 删除节点
vector<TopoNode> backup_nodes;
for (int id : blocked_node_ids) {
    backup_nodes.push_back(node_pool_[id]);
    node_pool_.erase(node_pool_.begin() + id);  // 删除
}

// ✅ 新代码: 标记阻塞
for (int id : blocked_node_ids) {
    node_pool_[id].is_blocked = true;  // 标记
}

// 第2次搜索
astarSearch(start, goal, alt_path);

// ✅ 新代码: 清除标记
for (int id : blocked_node_ids) {
    node_pool_[id].is_blocked = false;  // 恢复
}
```

---

### 步骤3: 修改astarSearch()跳过被阻塞节点

**文件**: `topo_graph_search.cpp`
```cpp
// 在扩展邻居时,跳过被阻塞节点
for (size_t i = 0; i < node_pool_.size(); ++i) {
    if (closed_set[i]) continue;
    if (i == static_cast<size_t>(current_id)) continue;
    
    // 🚀 NEW: 跳过被阻塞节点
    if (node_pool_[i].is_blocked) continue;
    
    // ... 正常连接检查
}
```

---

## 📊 预期效果

修复前:
```
第1次搜索 ✅
删除节点 → 索引错乱
第2次搜索 → goal_id错误 → 无限扩展 → OOM崩溃 ❌
```

修复后:
```
第1次搜索 ✅
标记节点 → 索引稳定
第2次搜索 → goal_id正确 → 正常完成 ✅
第3/4次搜索 ✅
```

---

## 🚨 紧急程度

**严重性**: ⭐⭐⭐⭐⭐ (系统崩溃)  
**紧急性**: ⭐⭐⭐⭐⭐ (立即修复)  
**影响**: 100% K-shortest paths失败 → 无多路径 → 目标无法达成

---

## 📝 其他发现

### 好消息 ✅
1. **Corner检测正常**: 检测到60个corner (达到max限制)
2. **第1条路径成功**: A*正常工作,3个路径点
3. **桥接节点未触发**: 图已经连通 (62节点=60corner+start+goal)

### 坏消息 ❌
1. **K-shortest完全失败**: 第2次搜索就崩溃
2. **内存管理错误**: 删除节点导致索引错乱

---

**立即行动**: 修复is_blocked标志! 🔧
