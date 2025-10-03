# K-Shortest Paths 深度研究报告

## 问题诊断

### 当前状态
- ✅ 多走廊采样成功: 45-48个节点来自5个走廊
- ✅ 阻塞策略合理: 20-24个节点 (44-53%)
- ✅ A*找到替代路径: 每次都成功
- ❌ **所有替代路径相似度0.68-0.89 > 0.5阈值** ← 核心问题

### 为什么相似度这么高？

即使有5个走廊的节点分布，A*仍然找到高度相似的路径。原因：

1. **图的连通性问题** - 5个走廊的节点之间可能高度互联
2. **A*的贪心特性** - 总是选最短路径，即使阻塞了一些节点
3. **阻塞策略不够激进** - 只阻塞路径附近的点，没有"强制"A*绕行

---

## 原始TGK-Planner的解决方案

### 论文: "Topological Path Planning..."

**核心方法**: **Homotopy Class** (同伦类)

```
不是简单的K-shortest paths!
而是基于拓扑类的多路径生成!
```

#### 关键算法

**1. 拓扑签名 (Topological Signature)**

每条路径有一个"签名" - 记录它绕过哪些障碍物的哪一侧

```cpp
struct TopoSignature {
    vector<int> obstacle_ids;    // 经过的障碍物ID
    vector<int> passing_sides;   // 左侧(-1) 还是 右侧(+1)
};

// 例子:
// Path A: [obs1:left, obs2:right] → signature = [-1, +1]
// Path B: [obs1:right, obs2:right] → signature = [+1, +1]
// → 两条路径拓扑不同!
```

**2. RRT* + Rewiring (KRRT*)**

原始TGK不是用A* + 阻塞，而是用增强的RRT*:

```python
def KRRT_star(start, goal, K):
    trees = [RRT_star(start, goal)]  # 第1棵树
    
    for k in range(2, K+1):
        # 创建第k棵树，但避免已有的拓扑类
        tree_k = RRT_star_with_constraints(
            start, goal,
            forbidden_signatures = [t.signature for t in trees]
        )
        trees.append(tree_k)
    
    return [tree.path for tree in trees]
```

**关键**: 不是阻塞节点，而是**禁止某些拓扑类**

---

## 通用K-Shortest Paths方法

### 方法1: Yen's Algorithm (经典)

**核心思想**: 系统地排除已找到的路径

```python
def Yens_K_Shortest(graph, start, goal, K):
    A = [shortest_path(graph, start, goal)]  # 第1条
    B = []  # 候选路径池
    
    for k in range(1, K):
        # 对于第k-1条路径的每个节点
        for i in range(len(A[k-1]) - 1):
            spur_node = A[k-1][i]
            root_path = A[k-1][:i+1]
            
            # 移除与root_path冲突的边
            for path in A:
                if path[:i+1] == root_path:
                    remove_edge(graph, path[i], path[i+1])
            
            # 从spur_node搜索到goal
            spur_path = shortest_path(graph, spur_node, goal)
            
            if spur_path:
                total_path = root_path + spur_path[1:]
                B.add(total_path)
            
            # 恢复边
            restore_edges(graph)
        
        if B.empty():
            break
        
        # 选择B中最短的加入A
        A.append(min(B, key=lambda p: len(p)))
        B.remove(A[-1])
    
    return A
```

**优势**: 保证找到真正的K条最短路径  
**缺点**: 计算复杂度高 O(K·N·(M + N·logN))

### 方法2: Lazy Deletion (我们当前用的)

```python
def Lazy_K_Shortest(graph, start, goal, K):
    path1 = A_star(graph, start, goal)
    paths = [path1]
    
    for k in range(1, K):
        # 阻塞第1条路径上的一些节点
        blocked = select_nodes_to_block(path1)
        mark_blocked(graph, blocked)
        
        path_k = A_star(graph, start, goal)
        
        if is_different(path_k, paths):
            paths.append(path_k)
        
        unmark_blocked(graph, blocked)
    
    return paths
```

**问题**: 
- 阻塞策略很难设计
- 可能找不到真正不同的路径
- 依赖相似度阈值(主观)

### 方法3: Plateau Method (新方法)

**思想**: 不是阻塞节点，而是**惩罚已使用的边**

```python
def Plateau_K_Shortest(graph, start, goal, K):
    paths = []
    edge_usage_count = {}  # 记录每条边被使用的次数
    
    for k in range(K):
        # 修改边权重: 使用次数越多,权重越大
        for edge in graph.edges:
            edge.weight = edge.original_weight * (1 + penalty * edge_usage_count[edge])
        
        # 用修改后的权重搜索
        path_k = A_star(graph, start, goal)
        
        if path_k:
            paths.append(path_k)
            # 更新边使用统计
            for edge in path_k.edges:
                edge_usage_count[edge] += 1
        else:
            break
    
    return paths
```

**优势**: 
- 自然地引导搜索避开已用路径
- 不需要相似度阈值
- 计算效率高

---

## 我们的问题根因

### 深度诊断

让我看看实际的图结构和路径:

```
假设场景:
起点 A ───────────────────── 目标 B
      │                    │
      ├─── 障碍物1 ────┤
      │                    │
      └─── 障碍物2 ────┘

我们的5个走廊:
- 走廊0 (主): A直接到B (最短)
- 走廊-5m: A→左侧5m→B
- 走廊-10m: A→左侧10m→B  
- 走廊+5m: A→右侧5m→B
- 走廊+10m: A→右侧10m→B

问题: 这5个走廊如果都是**开阔空间**,没有障碍物强制分离:
→ A*总会选择"稍微偏一点点"的路径
→ 相似度仍然很高 (0.68-0.89)
```

**验证方法**: 检查5个走廊是否有真正的障碍物分隔

---

## 三级解决方案

### 方案A: 改进阻塞策略 (短期, 2小时)

**当前问题**: 阻塞了20-24个节点,但都是"周围"的节点,没有强制绕行

**改进**: **阻塞整个走廊区域** (不是点,是区域)

```cpp
// 不是阻塞path1附近的点,而是:
// 1. 识别path1所在的走廊ID (0, -5, -10, +5, +10)
// 2. 阻塞该走廊的所有节点
// 3. 强制A*使用其他走廊

int corridor_id_of_path1 = identify_corridor(path1);  // 比如 0 (主走廊)
for (node : node_pool_) {
    if (node.corridor_id == corridor_id_of_path1) {
        node.is_blocked = true;  // 阻塞整个主走廊!
    }
}
```

### 方案B: 实施Homotopy Class (中期, 1天)

**核心**: 给每个节点标记所属的拓扑类

```cpp
struct TopoNode {
    Vector3d pos;
    int corridor_id;     // 新增: 所属走廊ID (-10, -5, 0, +5, +10)
    int obstacle_side;   // 新增: 相对主障碍物的位置 (left/right)
};

// 路径签名
struct PathSignature {
    vector<int> corridor_sequence;  // 经过的走廊序列
    
    bool operator==(const PathSignature& other) {
        return corridor_sequence == other.corridor_sequence;
    }
};

// K-shortest with signature
for (int k = 0; k < K; k++) {
    PathSignature forbidden = paths[k].signature;
    
    // 搜索时避免生成相同签名的路径
    path_new = A_star_with_signature_constraint(start, goal, forbidden);
}
```

### 方案C: 实施Plateau Method (中期, 1天)

**最简单且有效**:

```cpp
// 第1次搜索: 正常权重
path1 = A_star(graph);
paths.push_back(path1);

// 第2-K次搜索: 惩罚已使用的边
map<pair<int,int>, int> edge_usage;
for (edge in path1) {
    edge_usage[edge] += 1;
}

// 修改边权重
for (edge in graph.edges) {
    edge.weight *= (1.0 + 2.0 * edge_usage[edge]);  // 2倍惩罚
}

path2 = A_star(graph);  // 会自动避开path1的边!
```

---

## 立即行动计划

### Step 1: 诊断 - 查看实际路径分布 (10分钟)

添加可视化代码:

```cpp
ROS_INFO("=== PATH ANALYSIS ===");
ROS_INFO("Path 1 waypoints:");
for (size_t i = 0; i < first_path.size(); i++) {
    Vector3d p = first_path[i];
    ROS_INFO("  [%zu] (%.2f, %.2f, %.2f)", i, p.x(), p.y(), p.z());
}

if (alt_path.size() > 0) {
    ROS_INFO("Alternative path waypoints:");
    for (size_t i = 0; i < alt_path.size(); i++) {
        Vector3d p = alt_path[i];
        ROS_INFO("  [%zu] (%.2f, %.2f, %.2f)", i, p.x(), p.y(), p.z());
    }
    
    // 计算路径中点偏移
    Vector3d mid1 = first_path[first_path.size()/2];
    Vector3d mid2 = alt_path[alt_path.size()/2];
    double lateral_offset = (mid2 - mid1).norm();
    ROS_INFO("Lateral offset at midpoint: %.2f m", lateral_offset);
}
```

### Step 2: 快速修复 - 方案A (1小时)

**给节点标记走廊ID**:

```cpp
// 在buildSearchGraph中,给每个节点标记走廊ID
Vector3d lateral_dir = Vector3d(-(goal-start).y(), (goal-start).x(), 0).normalized();
Vector3d midline = (start + goal) / 2.0;

for (auto& node : node_pool_) {
    Vector3d to_node = node.pos - midline;
    double lateral_offset = to_node.dot(lateral_dir);
    
    // 根据偏移量分类走廊
    if (lateral_offset < -7.5) node.corridor_id = -10;
    else if (lateral_offset < -2.5) node.corridor_id = -5;
    else if (lateral_offset < 2.5) node.corridor_id = 0;
    else if (lateral_offset < 7.5) node.corridor_id = 5;
    else node.corridor_id = 10;
}
```

**阻塞整个走廊**:

```cpp
// 识别path1的主走廊
int main_corridor = identify_main_corridor(first_path);

// 阻塞该走廊的所有节点
for (auto& node : node_pool_) {
    if (node.corridor_id == main_corridor) {
        node.is_blocked = true;
    }
}
```

### Step 3: 中期优化 - 方案C Plateau (2小时)

实施边权重惩罚:

```cpp
struct Edge {
    int from, to;
    double base_cost;
    int usage_count;
    
    double get_cost() {
        return base_cost * (1.0 + 2.0 * usage_count);
    }
};

// 在extractMultiplePaths中:
map<pair<int,int>, int> edge_usage;

for (int k = 0; k < K; k++) {
    // 更新A*代价函数使用edge_usage
    path_k = A_star_with_edge_costs(start, goal, edge_usage);
    
    // 更新使用统计
    for (size_t i = 0; i < path_k.size()-1; i++) {
        edge_usage[{path_k[i], path_k[i+1]}]++;
    }
}
```

---

## 预期效果

### 当前 (方案A前)
```
5个走廊节点分布 → A*搜索 → 相似度0.68-0.89 → 只有1条路径
```

### 方案A实施后
```
识别path1走廊 = 0 (主走廊)
阻塞走廊0的所有节点
→ A*被迫使用走廊-5或+5
→ 相似度预计0.3-0.4
→ 成功生成2-3条路径
```

### 方案C实施后
```
第1次: 走廊0 (权重1.0)
第2次: 走廊0权重3.0, 走廊±5权重1.0 → 选走廊+5
第3次: 走廊0权重5.0, 走廊+5权重3.0, 走廊-5权重1.0 → 选走廊-5
第4次: 走廊±10...
→ 4-5条不同路径!
```

---

## 总结

**根本问题**: 
1. ✅ 节点分布已解决 (5个走廊)
2. ❌ **阻塞策略错误** - 阻塞"附近的点"而不是"整个走廊"
3. ❌ **相似度计算不准** - 已改用Hausdorff距离
4. ❌ **缺乏拓扑感知** - 没有走廊ID标记

**下一步**:
1. 立即实施方案A (给节点标记走廊ID + 阻塞整个走廊)
2. 测试效果
3. 如果还不够,实施方案C (Plateau Method)

**预期多路径生成率**: 0% → 60-80%
