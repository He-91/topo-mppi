#include "path_searching/topo_graph_search.h"
#include <algorithm>
#include <cmath>

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
            while (id != -1) {
                path.push_back(node_states[id].pos);
                id = node_states[id].parent_id;
            }
            reverse(path.begin(), path.end());
            
            ROS_INFO("[TopoGraphSearch] A* found path in %d iterations", iter);
            return true;
        }
        
        // Expand neighbors
        for (size_t i = 0; i < node_pool_.size(); ++i) {
            if (closed_set[i]) continue;
            if (i == static_cast<size_t>(current_id)) continue;  // Skip self
            
            // 🔧 CRITICAL FIX: 跳过被阻塞的节点 (K-shortest paths)
            // 这样阻塞节点后,A*会自动寻找绕过的路径
            if (node_pool_[i].is_blocked) continue;
            
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
    
    // 🚀 ENHANCED K-SHORTEST PATHS: 改进多路径生成
    // 目标: 多路径生成率 13% → 40%
    // 策略: 阻塞走廊 + 放宽相似度 + 多点阻塞
    // 🔧 FIX: 使用is_blocked标记,避免删除节点导致索引错乱 → bad_alloc崩溃
    int max_paths = max_topo_paths_;  // Target: 3-5 paths (configurable)
    
    for (int attempt = 1; attempt < max_paths && first_path.size() >= 3; attempt++) {
        // 🔧 改进1: 阻塞多个位置 (走廊阻塞)
        // 不只阻塞1个点,而是阻塞第1条路径的多个关键节点
        vector<int> blocked_node_ids;
        
        // 选择5个阻塞位置: 0.15, 0.35, 0.5, 0.65, 0.85 (增强覆盖)
        vector<double> block_ratios = {0.15, 0.35, 0.5, 0.65, 0.85};
        
        for (double ratio : block_ratios) {
            size_t block_idx = static_cast<size_t>(first_path.size() * ratio);
            if (block_idx >= first_path.size()) continue;
            
            Vector3d blocked_pos = first_path[block_idx];
            
            // 找最近的关键点
            int blocked_node_id = -1;
            double min_dist = std::numeric_limits<double>::max();
            for (size_t i = 1; i < node_pool_.size() - 1; i++) {  // Skip start(0), goal(last)
                // 避免重复阻塞
                bool already_blocked = false;
                for (int id : blocked_node_ids) {
                    if (static_cast<size_t>(id) == i) {
                        already_blocked = true;
                        break;
                    }
                }
                if (already_blocked) continue;
                
                double dist = (node_pool_[i].pos - blocked_pos).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    blocked_node_id = i;
                }
            }
            
            // 🔧 改进2: 阻塞半径从3m扩大到8m (覆盖更大走廊)
            if (blocked_node_id >= 0 && min_dist <= 8.0) {
                blocked_node_ids.push_back(blocked_node_id);
            }
        }
        
        if (blocked_node_ids.empty()) {
            ROS_INFO("[TopoGraphSearch] ⚠️ Attempt %d: No nodes to block (path too short or no matching nodes)", attempt);
            continue;
        }
        
        // 🔧 CRITICAL FIX: 使用标记而不是删除节点!
        // 旧代码: node_pool_.erase() → 索引错乱 → goal_id错误 → bad_alloc崩溃
        // 新代码: is_blocked标记 → 索引稳定 → 正常工作
        for (int id : blocked_node_ids) {
            node_pool_[id].is_blocked = true;  // 标记为阻塞
        }
        
        ROS_INFO("[TopoGraphSearch] 🚧 Attempt %d: Blocked %zu nodes at positions along first path", 
                 attempt, blocked_node_ids.size());
        
        // 尝试寻找替代路径
        vector<Vector3d> alt_path;
        if (astarSearch(start, goal, alt_path)) {
            // 验证路径完整性
            if (alt_path.size() < 2) {
                ROS_DEBUG("[TopoGraphSearch] Rejected incomplete alternative path with %zu waypoints", alt_path.size());
            } else {
                // 🔧 改进3: 相似度阈值 0.7 → 0.5 (允许更多非同伦路径)
                // 允许更多拓扑相似但几何不同的路径
                bool is_different = true;
                for (const auto& existing_path : paths) {
                    double similarity = calculatePathSimilarity(alt_path, existing_path);
                    ROS_INFO("[TopoGraphSearch] 🔍 Path similarity check: attempt %d, similarity=%.3f (threshold=0.5)", 
                             attempt, similarity);
                    if (similarity > 0.5) {  // 0.5相似度阈值 (放宽)
                        is_different = false;
                        ROS_INFO("[TopoGraphSearch] ❌ Rejected similar path (similarity=%.3f > 0.5)", similarity);
                        break;
                    }
                }
                
                if (is_different) {
                    smoothPath(alt_path);
                    paths.push_back(alt_path);
                    ROS_INFO("[TopoGraphSearch] ✅ Path %zu: %zu waypoints (blocked %zu nodes)", 
                             paths.size(), alt_path.size(), blocked_node_ids.size());
                } else {
                    ROS_DEBUG("[TopoGraphSearch] Attempt %d: Path too similar to existing paths", attempt);
                }
            }
        } else {
            ROS_DEBUG("[TopoGraphSearch] Attempt %d: A* failed after blocking %zu nodes", 
                     attempt, blocked_node_ids.size());
        }
        
        // 恢复节点 (清除阻塞标记)
        for (int id : blocked_node_ids) {
            node_pool_[id].is_blocked = false;
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
    // 🚀 NEW: 计算路径相似度 (0.0 = 完全不同, 1.0 = 完全相同)
    // 用于更精细的多路径筛选
    
    if (path1.empty() || path2.empty()) {
        return 0.0;
    }
    
    // 方法: 在两条路径上采样,计算平均距离,归一化到[0, 1]
    int num_samples = 20;  // 增加采样点提高精度
    double total_dist = 0.0;
    double path_length = 0.0;
    
    // 计算path1的总长度作为归一化基准
    for (size_t i = 0; i < path1.size() - 1; ++i) {
        path_length += (path1[i + 1] - path1[i]).norm();
    }
    
    if (path_length < 1e-6) {
        return 1.0;  // 退化路径,认为相同
    }
    
    // 在路径上采样并比较
    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / (num_samples - 1);
        
        size_t idx1 = static_cast<size_t>(t * (path1.size() - 1));
        size_t idx2 = static_cast<size_t>(t * (path2.size() - 1));
        
        total_dist += (path1[idx1] - path2[idx2]).norm();
    }
    
    double avg_dist = total_dist / num_samples;
    
    // 归一化: dist / path_length
    // dist=0 → similarity=1.0
    // dist>path_length → similarity=0.0
    double similarity = 1.0 - std::min(1.0, avg_dist / path_length);
    
    return similarity;
}

} // namespace ego_planner
