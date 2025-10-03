#include "path_searching/topo_graph_search.h"
#include <algorithm>
#include <cmath>
#include <map>

using namespace std;
using namespace Eigen;

namespace ego_planner {

TopoGraphSearch::TopoGraphSearch()
    : max_search_nodes_(1000),
      connection_radius_(20.0),  // � CRITICAL FIX: 增加到 20m 匹配 TGK 动态半径
      path_pruning_threshold_(0.5),
      max_topo_paths_(5),
      node_counter_(0) {
}

TopoGraphSearch::~TopoGraphSearch() {
}

void TopoGraphSearch::init(GridMap::Ptr grid_map, BiasSampler::Ptr bias_sampler) {
    grid_map_ = grid_map;
    bias_sampler_ = bias_sampler;
    
    ROS_INFO("[TopoGraphSearch] Initialized");
}

bool TopoGraphSearch::searchTopoPaths(const Vector3d& start,
                                     const Vector3d& goal,
                                     vector<vector<Vector3d>>& paths) {
    paths.clear();
    
    ROS_INFO("[TopoGraphSearch] Searching multiple topological paths");
    
    // Build search graph
    if (!buildSearchGraph(start, goal)) {
        ROS_WARN("[TopoGraphSearch] Failed to build search graph");
        return false;
    }
    
    // Extract multiple topologically distinct paths
    extractMultiplePaths(start, goal, paths);
    
    if (paths.empty()) {
        ROS_WARN("[TopoGraphSearch] No valid paths found");
        return false;
    }
    
    ROS_INFO("[TopoGraphSearch] Found %zu topological paths", paths.size());
    return true;
}

bool TopoGraphSearch::searchSinglePath(const Vector3d& start,
                                      const Vector3d& goal,
                                      vector<Vector3d>& path) {
    path.clear();
    
    // Build search graph
    if (!buildSearchGraph(start, goal)) {
        ROS_WARN("[TopoGraphSearch] Failed to build search graph");
        return false;
    }
    
    // Run A* search
    if (!astarSearch(start, goal, path)) {
        ROS_WARN("[TopoGraphSearch] A* search failed");
        return false;
    }
    
    // Smooth path
    smoothPath(path);
    
    ROS_INFO("[TopoGraphSearch] Found path with %zu waypoints", path.size());
    return true;
}

bool TopoGraphSearch::buildSearchGraph(const Vector3d& start, const Vector3d& goal) {
    node_pool_.clear();
    node_counter_ = 0;
    
    // Get topological key points from bias sampler
    vector<Vector3d> key_points = bias_sampler_->getTopoKeyPoints(start, goal);
    
    ROS_INFO("[TopoGraphSearch] Building graph with %zu key points", key_points.size());
    
    // If no key points found and direct path is free, just connect start-goal
    if (key_points.empty() && isPathFree(start, goal)) {
        ROS_INFO("[TopoGraphSearch] Direct path is free, using simple connection");
        // Still build graph with just start and goal for consistency
    }
    
    // Add start node
    TopoNode start_node;
    start_node.pos = start;
    start_node.node_id = node_counter_++;
    start_node.topo_class = 0;
    node_pool_.push_back(start_node);
    
    // Add key point nodes
    for (const auto& pt : key_points) {
        TopoNode node;
        node.pos = pt;
        node.node_id = node_counter_++;
        node.topo_class = -1;  // Will be set by sampler if needed
        node_pool_.push_back(node);
    }
    
    // Add goal node
    TopoNode goal_node;
    goal_node.pos = goal;
    goal_node.node_id = node_counter_++;
    goal_node.topo_class = 0;
    node_pool_.push_back(goal_node);
    
    // 🚀 BRIDGE NODE ALGORITHM: 修复图不连通问题
    // 目标: TGK成功率 85% → 95%
    // 策略: 检测孤岛节点,在起点-终点连线上插入桥接节点
    
    if (node_pool_.size() >= 2) {
        // 检查起点能否直接看到终点
        if (!isPathFree(start, goal)) {
            ROS_DEBUG("[TopoGraphSearch] Direct path blocked, checking graph connectivity");
            
            // 简单连通性检查: 起点能否通过现有节点到达终点
            // 统计起点和终点的可见节点数量
            int start_connections = 0;
            int goal_connections = 0;
            
            for (size_t i = 1; i < node_pool_.size() - 1; ++i) {
                if (canConnect(start, node_pool_[i].pos)) {
                    start_connections++;
                }
                if (canConnect(node_pool_[i].pos, goal)) {
                    goal_connections++;
                }
            }
            
            ROS_DEBUG("[TopoGraphSearch] Connectivity check: start→%d nodes, goal←%d nodes",
                     start_connections, goal_connections);
            
            // 🔧 改进: 降低桥接节点触发阈值,在连接稀疏时也触发 (< 3个连接)
            if (start_connections < 3 || goal_connections < 3) {
                ROS_WARN("[TopoGraphSearch] 🔧 Graph connectivity issue detected (start:%d, goal:%d), adding bridge nodes",
                        start_connections, goal_connections);
                
                // 在起点-终点连线上尝试插入桥接节点
                Vector3d dir = (goal - start).normalized();
                double dist = (goal - start).norm();
                
                // 尝试在1/4, 1/2, 3/4位置插入桥接节点
                vector<double> bridge_ratios = {0.25, 0.5, 0.75};
                int bridges_added = 0;
                
                for (double ratio : bridge_ratios) {
                    Vector3d bridge_pos = start + ratio * dist * dir;
                    
                    // 检查桥接位置是否无碰撞
                    if (!grid_map_->getInflateOccupancy(bridge_pos)) {
                        // 检查桥接节点是否能改善连通性
                        bool helpful = false;
                        
                        // 桥接节点应该能连接起点或终点
                        if (isPathFree(start, bridge_pos) || isPathFree(bridge_pos, goal)) {
                            helpful = true;
                        }
                        
                        if (helpful) {
                            TopoNode bridge_node;
                            bridge_node.pos = bridge_pos;
                            bridge_node.node_id = node_counter_++;
                            bridge_node.topo_class = -2;  // 标记为桥接节点
                            
                            // 插入到goal节点之前
                            node_pool_.insert(node_pool_.end() - 1, bridge_node);
                            bridges_added++;
                            
                            ROS_INFO("[TopoGraphSearch] ✅ Added bridge node at [%.2f, %.2f, %.2f]",
                                   bridge_pos.x(), bridge_pos.y(), bridge_pos.z());
                        }
                    }
                }
                
                if (bridges_added > 0) {
                    ROS_INFO("[TopoGraphSearch] 🔧 Added %d bridge nodes to improve connectivity",
                            bridges_added);
                }
            }
        }
    }
    
    // 🚀 CORRIDOR ID LABELING: 为每个节点标记所属走廊
    // 目标: 实现拓扑感知的K-shortest paths
    // 策略: 根据节点相对起点-终点连线的侧向偏移量分类
    
    if (node_pool_.size() >= 2) {
        Vector3d sg_dir = (goal - start).normalized();
        Vector3d lateral_dir = Vector3d(-sg_dir.y(), sg_dir.x(), 0.0).normalized();  // 垂直方向
        Vector3d midpoint = (start + goal) / 2.0;
        
        // 🔧 FIX 1: 修复走廊分类 - 不强制起点/终点,使用实际侧向偏移量
        for (size_t i = 0; i < node_pool_.size(); ++i) {
            Vector3d to_node = node_pool_[i].pos - midpoint;
            double lateral_offset = to_node.dot(lateral_dir);
            
            // 🔧 FIX 2: 根据侧向偏移量分配走廊ID (对齐BiasedSampler的±0/±5/±10m)
            // 分类边界: -7.5, -2.5, +2.5, +7.5 (走廊中心±2.5m)
            if (lateral_offset < -7.5) {
                node_pool_[i].corridor_id = -10;
            } else if (lateral_offset < -2.5) {
                node_pool_[i].corridor_id = -5;
            } else if (lateral_offset < 2.5) {
                node_pool_[i].corridor_id = 0;  // 主走廊
            } else if (lateral_offset < 7.5) {
                node_pool_[i].corridor_id = 5;
            } else {
                node_pool_[i].corridor_id = 10;
            }
            
            ROS_DEBUG("[TopoGraphSearch] Node %zu: corridor_id=%d, lateral_offset=%.2f",
                     i, node_pool_[i].corridor_id, lateral_offset);
        }
        
        // 统计每个走廊的节点数
        map<int, int> corridor_counts;
        for (const auto& node : node_pool_) {
            corridor_counts[node.corridor_id]++;
        }
        
        ROS_INFO("[TopoGraphSearch] 🗺️ Corridor distribution (all nodes):");
        for (const auto& pair : corridor_counts) {
            ROS_INFO("  Corridor %+3d: %2d nodes", pair.first, pair.second);
        }
    }
    
    ROS_INFO("[TopoGraphSearch] Graph built with %zu nodes", node_pool_.size());
    return node_pool_.size() >= 2;
}

bool TopoGraphSearch::astarSearch(const Vector3d& start,
                                  const Vector3d& goal,
                                  vector<Vector3d>& path) {
    if (node_pool_.size() < 2) {
        ROS_WARN("[TopoGraphSearch] Node pool too small: %zu", node_pool_.size());
        return false;
    }
    
    // Priority queue for A*
    priority_queue<TopoNode, vector<TopoNode>, TopoNodeComparator> open_set;
    
    // Visited set
    vector<bool> closed_set(node_pool_.size(), false);
    vector<TopoNode> node_states = node_pool_;
    
    // Initialize all nodes with infinite cost
    for (size_t i = 0; i < node_states.size(); ++i) {
        node_states[i].g_cost = std::numeric_limits<double>::max();
        node_states[i].parent_id = -1;
    }
    
    // Initialize start node
    int start_id = 0;  // First node is start
    node_states[start_id].g_cost = 0.0;
    node_states[start_id].h_cost = heuristic(node_states[start_id].pos, goal);
    node_states[start_id].parent_id = -1;
    open_set.push(node_states[start_id]);
    
    int goal_id = node_pool_.size() - 1;  // Last node is goal
    
    ROS_INFO("[TopoGraphSearch] A* search: %zu nodes, start_id=%d, goal_id=%d", 
             node_pool_.size(), start_id, goal_id);
    
    int iter = 0;
    int connections_tested = 0;
    while (!open_set.empty() && iter < max_search_nodes_) {
        iter++;
        
        // Get node with lowest f_cost
        TopoNode current = open_set.top();
        open_set.pop();
        
        int current_id = current.node_id;
        
        // Check if already visited
        if (closed_set[current_id]) {
            continue;
        }
        
        closed_set[current_id] = true;
        
        // Check if reached goal
        if (current_id == goal_id) {
            // Reconstruct path
            path.clear();
            int id = goal_id;
            vector<int> path_node_ids_temp;  // 临时记录路径节点ID
            while (id != -1) {
                path.push_back(node_states[id].pos);
                path_node_ids_temp.push_back(id);
                id = node_states[id].parent_id;
            }
            reverse(path.begin(), path.end());
            reverse(path_node_ids_temp.begin(), path_node_ids_temp.end());
            
            // 🔧 FIX 9: 验证路径是否经过被阻塞的节点(调试用)
            bool path_uses_blocked = false;
            for (int node_id : path_node_ids_temp) {
                if (node_pool_[node_id].is_blocked) {
                    path_uses_blocked = true;
                    ROS_ERROR("[TopoGraphSearch] ❌ BUG: Path uses blocked node %d (corridor %+d)!",
                              node_id, node_pool_[node_id].corridor_id);
                }
            }
            
            if (!path_uses_blocked) {
                ROS_INFO("[TopoGraphSearch] ✅ A* found valid path in %d iterations (no blocked nodes used)", iter);
            }
            
            return true;
        }
        
        // Expand neighbors
        for (size_t i = 0; i < node_pool_.size(); ++i) {
            if (closed_set[i]) continue;
            if (i == static_cast<size_t>(current_id)) continue;  // Skip self
            
            // 🔧 CRITICAL FIX: 跳过被阻塞的节点 (K-shortest paths)
            // 这样阻塞节点后,A*会自动寻找绕过的路径
            if (node_pool_[i].is_blocked) {
                ROS_DEBUG("[TopoGraphSearch] Skipping blocked node %zu (corridor %+d)", 
                         i, node_pool_[i].corridor_id);
                continue;
            }
            
            // Check if can connect
            connections_tested++;
            if (!canConnect(node_states[current_id].pos, node_pool_[i].pos)) {
                continue;
            }
            
            // Calculate tentative g_cost
            double tentative_g = node_states[current_id].g_cost + 
                                edgeCost(node_states[current_id].pos, node_pool_[i].pos);
            
            // Update if this path is better
            if (tentative_g < node_states[i].g_cost) {
                node_states[i].g_cost = tentative_g;
                node_states[i].h_cost = heuristic(node_states[i].pos, goal);
                node_states[i].parent_id = current_id;
                open_set.push(node_states[i]);
            }
        }
    }
    
    ROS_WARN("[TopoGraphSearch] A* failed to find path after %d iterations, tested %d connections", 
             iter, connections_tested);
    ROS_WARN("[TopoGraphSearch] Graph has %zu nodes, start can see goal: %s",
             node_pool_.size(), 
             isPathFree(node_states[start_id].pos, node_states[goal_id].pos) ? "YES" : "NO");
    return false;
}

void TopoGraphSearch::extractMultiplePaths(const Vector3d& start,
                                          const Vector3d& goal,
                                          vector<vector<Vector3d>>& paths) {
    paths.clear();
    
    // Step 1: Find first path
    vector<Vector3d> first_path;
    if (!astarSearch(start, goal, first_path)) {
        ROS_WARN("[TopoGraphSearch] Failed to find first path");
        // Fallback: try direct path
        if (isPathFree(start, goal)) {
            vector<Vector3d> direct_path = {start, goal};
            paths.push_back(direct_path);
            ROS_INFO("[TopoGraphSearch] Using direct fallback path");
        }
        return;
    }
    
    smoothPath(first_path);
    paths.push_back(first_path);
    ROS_INFO("[TopoGraphSearch] Path 1: %zu waypoints", first_path.size());
    
    // 🚀 CORRIDOR-AWARE K-SHORTEST PATHS: 拓扑感知的多路径生成
    // 目标: 多路径生成率 0% → 60%
    // 策略: 识别path1的走廊 → 阻塞整个走廊 → 强制使用其他走廊
    int max_paths = max_topo_paths_;  // Target: 3-5 paths (configurable)
    
    // Step 1: 识别第1条路径的主走廊
    map<int, int> corridor_usage;  // 统计path1使用的走廊
    for (const auto& waypoint : first_path) {
        // 找距离最近的节点,获取其走廊ID
        int nearest_node_id = -1;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < node_pool_.size(); i++) {
            double dist = (node_pool_[i].pos - waypoint).norm();
            if (dist < min_dist) {
                min_dist = dist;
                nearest_node_id = i;
            }
        }
        
        if (nearest_node_id >= 0) {
            corridor_usage[node_pool_[nearest_node_id].corridor_id]++;
        }
    }
    
    // 🔧 FIX 1: 修复主走廊识别 - 排除起点/终点的影响
    // 问题: 起点/终点都在走廊0,导致所有路径都被识别为走廊0
    // 解决: 只统计中间waypoint(排除first和last)
    int start_corridor = node_pool_[0].corridor_id;
    int goal_corridor = node_pool_[node_pool_.size() - 1].corridor_id;
    
    // 如果起点/终点在同一走廊,减去它们的贡献
    if (start_corridor == goal_corridor && corridor_usage[start_corridor] >= 2) {
        corridor_usage[start_corridor] -= 2;  // 减去起点和终点
    }
    
    // 找使用最多的走廊
    int main_corridor = 0;
    int max_usage = 0;
    for (const auto& pair : corridor_usage) {
        if (pair.second > max_usage) {
            max_usage = pair.second;
            main_corridor = pair.first;
        }
    }
    
    ROS_INFO("[TopoGraphSearch] 🎯 Path 1 main corridor: %+d (used %d times, start/goal in corridor %+d)", 
             main_corridor, max_usage, start_corridor);
    
    // Step 2: 渐进式阻塞走廊
    vector<int> forbidden_corridors;
    forbidden_corridors.push_back(main_corridor);  // 第1次阻塞主走廊
    
    // 🔧 FIX 3: 记录每条路径使用的节点,便于调试
    vector<vector<int>> path_node_ids;  // 每条路径使用的节点ID列表
    
    // 🔧 FIX 4: 永远不阻塞起点和终点
    int start_id = 0;
    int goal_id = node_pool_.size() - 1;
    
    for (int attempt = 1; attempt < max_paths; attempt++) {
        // 🔧 FIX 5: 累积阻塞 - 阻塞所有forbidden走廊的节点
        vector<int> newly_blocked;
        
        for (size_t i = 0; i < node_pool_.size(); i++) {
            // 🚀 CRITICAL: 永远不阻塞起点和终点!
            if (i == static_cast<size_t>(start_id) || i == static_cast<size_t>(goal_id)) {
                continue;
            }
            
            if (node_pool_[i].is_blocked) continue;  // 已被阻塞
            
            // 检查是否在forbidden走廊内
            bool should_block = false;
            for (int forbidden : forbidden_corridors) {
                if (node_pool_[i].corridor_id == forbidden) {
                    should_block = true;
                    break;
                }
            }
            
            if (should_block) {
                node_pool_[i].is_blocked = true;
                newly_blocked.push_back(i);
            }
        }
        
        // 统计总阻塞情况
        int total_blockable = node_pool_.size() - 2;  // 排除起点/终点
        int total_blocked = 0;
        map<int, int> blocked_by_corridor;
        for (size_t i = 0; i < node_pool_.size(); i++) {
            if (i == static_cast<size_t>(start_id) || i == static_cast<size_t>(goal_id)) {
                continue;  // 跳过起点/终点
            }
            if (node_pool_[i].is_blocked) {
                total_blocked++;
                blocked_by_corridor[node_pool_[i].corridor_id]++;
            }
        }
        
        if (newly_blocked.empty()) {
            ROS_INFO("[TopoGraphSearch] ⚠️ Attempt %d: No new nodes to block", attempt);
        }
        
        ROS_INFO("[TopoGraphSearch] 🚧 Attempt %d: Forbidden corridors %s → %d/%d nodes blocked (%.1f%%)", 
                 attempt,
                 [&forbidden_corridors]() {
                     string s = "[";
                     for (size_t i = 0; i < forbidden_corridors.size(); i++) {
                         s += to_string(forbidden_corridors[i]);
                         if (i < forbidden_corridors.size()-1) s += ", ";
                     }
                     return s + "]";
                 }().c_str(),
                 total_blocked, total_blockable,
                 100.0 * total_blocked / total_blockable);
        
        // 显示每个走廊的阻塞情况
        if (!blocked_by_corridor.empty()) {
            ROS_INFO("[TopoGraphSearch]    Blocked distribution:");
            for (const auto& pair : blocked_by_corridor) {
                ROS_INFO("      Corridor %+3d: %d nodes blocked", pair.first, pair.second);
            }
        }
        
        // 尝试寻找替代路径
        vector<Vector3d> alt_path;
        if (astarSearch(start, goal, alt_path)) {
            // 🔧 REMOVED FIX 10: 允许2节点路径（起点→终点直线）
            // 原因: 直线是最短路径baseline，应保留用于对比绕行路径
            // 走廊序列比较会自然区分 [0] vs [-5,0]，无需额外拒绝
            
            // 🔧 FIX 2: 在smooth之前先检查路径差异
            // 问题: smooth后两条不同的路径可能变成相同的直线
            // 解决: 保存smooth前的原始路径用于相似度计算
            vector<Vector3d> raw_alt_path = alt_path;  // 保存原始路径
            
            // 验证路径完整性
            if (alt_path.size() < 2) {
                ROS_DEBUG("[TopoGraphSearch] Rejected incomplete alternative path with %zu waypoints", alt_path.size());
            } else {
                // 🔧 FIX 7: 记录路径使用的节点ID,验证是否真的绕行
                vector<int> used_nodes;
                for (const auto& waypoint : alt_path) {
                    int nearest_node_id = -1;
                    double min_dist = std::numeric_limits<double>::max();
                    for (size_t i = 0; i < node_pool_.size(); i++) {
                        double dist = (node_pool_[i].pos - waypoint).norm();
                        if (dist < min_dist) {
                            min_dist = dist;
                            nearest_node_id = i;
                        }
                    }
                    if (nearest_node_id >= 0 && 
                        std::find(used_nodes.begin(), used_nodes.end(), nearest_node_id) == used_nodes.end()) {
                        used_nodes.push_back(nearest_node_id);
                    }
                }
                
                // 统计使用的走廊
                map<int, int> used_corridors;
                for (int node_id : used_nodes) {
                    used_corridors[node_pool_[node_id].corridor_id]++;
                }
                
                ROS_INFO("[TopoGraphSearch] 🛣️ Alt path %d uses %zu nodes across %zu corridors:",
                         attempt, used_nodes.size(), used_corridors.size());
                for (const auto& pair : used_corridors) {
                    ROS_INFO("    Corridor %+3d: %d nodes", pair.first, pair.second);
                }
                
                // � OPTIMIZED: 改用走廊序列判断路径差异
                // 旧方法: Hausdorff距离比较密集waypoint → 难以调优阈值
                // 新方法: 走廊序列比较 → "只要绕行方式不同就算新路径"
                // 示例: [0] vs [-5,0] vs [+5,0] → 3条不同路径
                
                // 提取当前路径和已有路径的走廊序列
                vector<int> alt_corridors = extractCorridorSequence(raw_alt_path);
                
                ROS_INFO("[TopoGraphSearch] �️ Alt path %d corridor sequence: %s",
                         attempt, 
                         [&alt_corridors]() {
                             string s = "[";
                             for (size_t i = 0; i < alt_corridors.size(); i++) {
                                 s += to_string(alt_corridors[i]);
                                 if (i < alt_corridors.size()-1) s += ", ";
                             }
                             return s + "]";
                         }().c_str());
                
                bool is_different = true;
                
                for (size_t i = 0; i < paths.size(); i++) {
                    vector<int> existing_corridors = extractCorridorSequence(paths[i]);
                    
                    if (!isCorridorSequenceDifferent(alt_corridors, existing_corridors)) {
                        // 走廊序列相同 → 拓扑相同
                        is_different = false;
                        ROS_INFO("[TopoGraphSearch] ❌ Rejected: same corridor sequence as path %zu", i+1);
                        break;
                    } else {
                        ROS_INFO("[TopoGraphSearch] ✅ Different from path %zu", i+1);
                    }
                }
                
                // 🔧 FIX 5: 额外检查 - 替代路径是否使用了forbidden走廊的节点（除起点/终点）
                // 如果替代路径的中间节点仍大量使用forbidden走廊,说明阻塞不够彻底
                bool uses_forbidden_nodes = false;
                int forbidden_node_count = 0;
                for (int node_id : used_nodes) {
                    // 跳过起点和终点
                    if (node_id == 0 || node_id == static_cast<int>(node_pool_.size()) - 1) {
                        continue;
                    }
                    // 检查中间节点是否在forbidden走廊
                    for (int forbidden : forbidden_corridors) {
                        if (node_pool_[node_id].corridor_id == forbidden) {
                            forbidden_node_count++;
                            break;
                        }
                    }
                }
                
                if (forbidden_node_count > used_nodes.size() / 3) {  // 超过1/3的节点在forbidden走廊
                    uses_forbidden_nodes = true;
                    ROS_INFO("[TopoGraphSearch] ⚠️ Alt path uses %d/%zu nodes from forbidden corridors",
                             forbidden_node_count, used_nodes.size());
                }
                
                if (uses_forbidden_nodes) {
                    is_different = false;
                    ROS_INFO("[TopoGraphSearch] ❌ Rejected: too many nodes from forbidden corridors");
                }
                
                if (is_different) {
                    smoothPath(alt_path);
                    paths.push_back(alt_path);
                    path_node_ids.push_back(used_nodes);
                    ROS_INFO("[TopoGraphSearch] ✅ Path %zu accepted: %zu waypoints", 
                             paths.size(), alt_path.size());
                    
                    // 🔧 FIX 6: 修复新路径主走廊识别
                    // 问题: used_corridors已经统计好了,直接用它找主走廊
                    // 旧代码重复计算,且可能有误
                    int new_corridor = 0;
                    int max_new_usage = 0;
                    for (const auto& pair : used_corridors) {
                        // 只考虑非起点/终点的走廊
                        // 起点/终点可能在任何走廊,但不应该影响主走廊判断
                        if (pair.second > max_new_usage) {
                            max_new_usage = pair.second;
                            new_corridor = pair.first;
                        }
                    }
                    
                    // 🔧 FIX 7: 避免重复添加相同走廊
                    bool already_forbidden = false;
                    for (int forbidden : forbidden_corridors) {
                        if (forbidden == new_corridor) {
                            already_forbidden = true;
                            break;
                        }
                    }
                    
                    if (!already_forbidden && new_corridor != main_corridor) {
                        forbidden_corridors.push_back(new_corridor);
                        ROS_INFO("[TopoGraphSearch] 🎯 Path %zu main corridor: %+d → added to forbidden list", 
                                 paths.size(), new_corridor);
                    } else if (already_forbidden) {
                        ROS_INFO("[TopoGraphSearch] 📌 Path %zu main corridor: %+d (already forbidden)", 
                                 paths.size(), new_corridor);
                    } else {
                        ROS_INFO("[TopoGraphSearch] 📌 Path %zu main corridor: %+d (same as Path 1)", 
                                 paths.size(), new_corridor);
                    }
                    
                } else {
                    ROS_DEBUG("[TopoGraphSearch] Attempt %d: Path too similar to existing paths", attempt);
                }
            }
        } else {
            ROS_WARN("[TopoGraphSearch] Attempt %d: A* failed to find path (graph may be disconnected)", 
                     attempt);
        }
    }
    
    ROS_INFO("[TopoGraphSearch] Generated %zu topological paths", paths.size());
}

double TopoGraphSearch::heuristic(const Vector3d& pos, const Vector3d& goal) {
    // Euclidean distance
    return (goal - pos).norm();
}

double TopoGraphSearch::edgeCost(const Vector3d& from, const Vector3d& to) {
    double dist = (to - from).norm();
    
    // Add penalty for proximity to obstacles
    // 🔧 Phase 4: Use getDistanceWithGrad (our ESDF API)
    Vector3d mid = (from + to) / 2.0;
    Vector3d edt_grad;
    double edt_dist = grid_map_->getDistanceWithGrad(mid, edt_grad);
    
    // 🔧 Phase 4.5.1.10: Filter abnormal ESDF values (unobserved regions)
    // ESDF returns 10000.0m for unobserved regions → treat as safe
    if (edt_dist > 100.0) {
        return dist;  // No penalty for unobserved regions (conservative)
    }
    
    // Normal region: penalize proximity to obstacles
    // Use 1.0m threshold (more lenient than 0.5m) for better path diversity
    double obs_penalty = 0.0;
    if (edt_dist < 1.0) {
        obs_penalty = (1.0 - edt_dist) * 2.0;
    }
    
    return dist + obs_penalty;
}

bool TopoGraphSearch::canConnect(const Vector3d& from, const Vector3d& to) {
    // 🚀 ADAPTIVE CONNECTION RADIUS: 动态调整连接半径
    // 目标: 适应稀疏/密集环境,提升连通性
    
    // Check distance constraint with adaptive radius
    double dist = (to - from).norm();
    
    // 🔧 动态半径策略:
    // - 稀疏环境 (corner < 10): 25m (扩大探索)
    // - 密集环境 (corner > 30): 15m (避免过度连接)
    // - 正常环境: 20m (默认)
    double adaptive_radius = connection_radius_;
    
    size_t corner_count = node_pool_.size();
    if (corner_count < 10) {
        adaptive_radius = 25.0;  // 稀疏环境,增大半径
    } else if (corner_count > 30) {
        adaptive_radius = 15.0;  // 密集环境,减小半径
    }
    
    if (dist > adaptive_radius) {
        return false;
    }
    
    // Check collision-free path
    return isPathFree(from, to);
}

bool TopoGraphSearch::isPathFree(const Vector3d& from, const Vector3d& to) {
    Vector3d dir = to - from;
    double dist = dir.norm();
    
    if (dist < 1e-6) {
        return true;  // Same point
    }
    
    dir.normalize();
    
    // � CRITICAL FIX: 放宽路径检查,匹配原始 TGK-Planner
    // 原 TGK 使用动力学约束检查而非密集几何检查
    // 降低步长要求: 0.08/0.12m → 0.3/0.5m
    double step = (dist < 3.0) ? 0.3 : 0.5;
    int num_checks = static_cast<int>(dist / step);
    if (num_checks < 1) num_checks = 1;
    
    for (int i = 0; i <= num_checks; ++i) {
        Vector3d check_pt = from + (dist * i / num_checks) * dir;
        if (grid_map_->getInflateOccupancy(check_pt)) {
            return false;
        }
    }
    
    return true;
}

void TopoGraphSearch::smoothPath(vector<Vector3d>& path) {
    if (path.size() <= 2) {
        return;
    }
    
    // Remove redundant waypoints using line-of-sight check
    vector<Vector3d> smoothed_path;
    smoothed_path.push_back(path[0]);
    
    size_t current_idx = 0;
    while (current_idx < path.size() - 1) {
        // Try to connect to farthest visible point
        for (int i = path.size() - 1; i > static_cast<int>(current_idx); --i) {
            if (isPathFree(path[current_idx], path[i])) {
                smoothed_path.push_back(path[i]);
                current_idx = i;
                break;
            }
        }
        
        // If no line-of-sight, move to next point
        if (current_idx < path.size() - 1 && 
            (smoothed_path.back() - path[current_idx]).norm() < 0.01) {
            current_idx++;
            smoothed_path.push_back(path[current_idx]);
        }
    }
    
    path = smoothed_path;
    
    ROS_INFO("[TopoGraphSearch] Path smoothed: %zu waypoints", path.size());
}

bool TopoGraphSearch::arePathsSimilar(const vector<Vector3d>& path1,
                                     const vector<Vector3d>& path2) {
    // Simple similarity check based on average distance
    if (path1.empty() || path2.empty()) {
        return false;
    }
    
    // Sample points along both paths and compare
    int num_samples = 10;
    double total_dist = 0.0;
    
    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1);
        
        size_t idx1 = static_cast<size_t>(t * (path1.size() - 1));
        size_t idx2 = static_cast<size_t>(t * (path2.size() - 1));
        
        total_dist += (path1[idx1] - path2[idx2]).norm();
    }
    
    double avg_dist = total_dist / num_samples;
    
    return avg_dist < path_pruning_threshold_;
}

double TopoGraphSearch::calculatePathSimilarity(const vector<Vector3d>& path1,
                                                const vector<Vector3d>& path2) {
    // 🚀 IMPROVED: 使用Hausdorff距离计算路径相似度
    // 🔧 FIX 8: 排除起点/终点，只比较中间waypoint的拓扑差异
    // 目标: 当起点/终点相同时，避免它们过度影响相似度
    
    if (path1.empty() || path2.empty()) {
        return 0.0;
    }
    
    // 🔧 关键改进: 只比较中间waypoint (排除起点/终点)
    vector<Vector3d> middle_path1, middle_path2;
    
    if (path1.size() > 2) {
        for (size_t i = 1; i < path1.size() - 1; ++i) {
            middle_path1.push_back(path1[i]);
        }
    }
    
    if (path2.size() > 2) {
        for (size_t i = 1; i < path2.size() - 1; ++i) {
            middle_path2.push_back(path2[i]);
        }
    }
    
    // 如果没有中间waypoint (只有起点终点), 使用完整路径
    const vector<Vector3d>& compare_path1 = middle_path1.empty() ? path1 : middle_path1;
    const vector<Vector3d>& compare_path2 = middle_path2.empty() ? path2 : middle_path2;
    
    // 计算路径长度作为归一化基准
    double path1_length = 0.0;
    for (size_t i = 0; i < path1.size() - 1; ++i) {
        path1_length += (path1[i + 1] - path1[i]).norm();
    }
    
    if (path1_length < 1e-6) {
        return 1.0;  // 退化路径
    }
    
    // 双向Hausdorff距离 (只在中间waypoint之间计算)
    double max_dist_1to2 = 0.0;
    for (const auto& p1 : compare_path1) {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& p2 : compare_path2) {
            double dist = (p1 - p2).norm();
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        if (min_dist > max_dist_1to2) {
            max_dist_1to2 = min_dist;
        }
    }
    
    // 反向: path2到path1
    double max_dist_2to1 = 0.0;
    for (const auto& p2 : compare_path2) {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& p1 : compare_path1) {
            double dist = (p2 - p1).norm();
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        if (min_dist > max_dist_2to1) {
            max_dist_2to1 = min_dist;
        }
    }
    
    // 双向Hausdorff距离 = max(单向距离)
    double hausdorff_dist = std::max(max_dist_1to2, max_dist_2to1);
    
    // 归一化到[0,1]
    // hausdorff_dist = 0 → similarity = 1.0 (完全相同)
    // hausdorff_dist ≥ path_length → similarity = 0.0 (完全不同)
    double similarity = 1.0 - std::min(1.0, hausdorff_dist / path1_length);
    
    ROS_DEBUG("[TopoGraphSearch] Similarity calculation: middle waypoints only (%zu vs %zu points)",
             compare_path1.size(), compare_path2.size());
    
    return similarity;
}

bool TopoGraphSearch::isCorridorSequenceDifferent(const vector<int>& corridors1,
                                                   const vector<int>& corridors2) {
    // 🚀 NEW: 基于走廊序列判断拓扑差异
    // 用户建议: "只要绕行方式不同应该就算一条新路径"
    // 策略: 走廊序列不同 = 拓扑不同
    
    // 简单比较：序列完全相同 → 相同路径
    if (corridors1.size() != corridors2.size()) {
        ROS_DEBUG("[TopoGraphSearch] 🛣️ Corridor sequences differ in length: %zu vs %zu",
                 corridors1.size(), corridors2.size());
        return true;  // 不同长度 = 不同拓扑
    }
    
    for (size_t i = 0; i < corridors1.size(); i++) {
        if (corridors1[i] != corridors2[i]) {
            ROS_DEBUG("[TopoGraphSearch] 🛣️ Corridor sequences differ at position %zu: %d vs %d",
                     i, corridors1[i], corridors2[i]);
            return true;  // 序列不同 = 不同拓扑
        }
    }
    
    ROS_DEBUG("[TopoGraphSearch] 🛣️ Corridor sequences are identical");
    return false;  // 序列完全相同 = 相同拓扑
}

vector<int> TopoGraphSearch::extractCorridorSequence(const vector<Vector3d>& path) {
    // 🚀 NEW: 提取路径经过的走廊序列
    // 返回: 去重后的走廊ID序列，例如 [0] 或 [-5, 0] 或 [+5, +10]
    
    vector<int> corridor_sequence;
    
    for (const auto& waypoint : path) {
        // 找最近的节点
        int nearest_node_id = -1;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < node_pool_.size(); i++) {
            double dist = (node_pool_[i].pos - waypoint).norm();
            if (dist < min_dist) {
                min_dist = dist;
                nearest_node_id = i;
            }
        }
        
        if (nearest_node_id >= 0) {
            int corridor_id = node_pool_[nearest_node_id].corridor_id;
            
            // 去重: 只添加不同的走廊
            if (corridor_sequence.empty() || corridor_sequence.back() != corridor_id) {
                corridor_sequence.push_back(corridor_id);
            }
        }
    }
    
    return corridor_sequence;
}

} // namespace ego_planner
