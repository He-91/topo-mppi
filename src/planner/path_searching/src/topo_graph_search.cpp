#include "path_searching/topo_graph_search.h"
#include <algorithm>
#include <cmath>

using namespace std;
using namespace Eigen;

namespace ego_planner {

TopoGraphSearch::TopoGraphSearch()
    : max_search_nodes_(1000),
      connection_radius_(3.0),
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
    // For now, just find one best path
    // TODO: Implement k-shortest paths algorithm for multiple topo paths
    
    vector<Vector3d> best_path;
    if (astarSearch(start, goal, best_path)) {
        smoothPath(best_path);
        paths.push_back(best_path);
        ROS_INFO("[TopoGraphSearch] Extracted path with %zu waypoints", best_path.size());
    } else {
        ROS_WARN("[TopoGraphSearch] A* search failed, trying direct path");
        // Fallback: try direct path
        if (isPathFree(start, goal)) {
            vector<Vector3d> direct_path = {start, goal};
            paths.push_back(direct_path);
            ROS_INFO("[TopoGraphSearch] Using direct fallback path");
        }
    }
    
    // Try to find alternative paths by penalizing nodes in the first path
    // This is a simplified approach - TGK uses more sophisticated method
    vector<bool> penalized_nodes(node_pool_.size(), false);
    
    // Mark middle nodes of best path as penalized
    if (!best_path.empty() && best_path.size() > 2) {
        for (size_t i = 1; i < best_path.size() - 1; ++i) {
            for (size_t j = 0; j < node_pool_.size(); ++j) {
                if ((node_pool_[j].pos - best_path[i]).norm() < connection_radius_ * 0.3) {
                    penalized_nodes[j] = true;
                }
            }
        }
    }
    
    // Search for alternative paths (simplified)
    for (int attempt = 0; attempt < max_topo_paths_ - 1; ++attempt) {
        vector<Vector3d> alt_path;
        
        // Temporarily increase cost for penalized nodes
        // (In a full implementation, modify the graph structure)
        
        // For now, just check if we can find direct or simple alternatives
        // This is a placeholder for more sophisticated multi-path search
        
        break;  // Exit after first path for now
    }
}

double TopoGraphSearch::heuristic(const Vector3d& pos, const Vector3d& goal) {
    // Euclidean distance
    return (goal - pos).norm();
}

double TopoGraphSearch::edgeCost(const Vector3d& from, const Vector3d& to) {
    double dist = (to - from).norm();
    
    // Add penalty for proximity to obstacles
    // 🔧 Phase 4: Use getDistanceWithGrad (our ESDF API)
    double obs_penalty = 0.0;
    Vector3d mid = (from + to) / 2.0;
    Vector3d edt_grad;
    double edt_dist = grid_map_->getDistanceWithGrad(mid, edt_grad);
    
    if (edt_dist < 0.5) {
        obs_penalty = (0.5 - edt_dist) * 2.0;
    }
    
    return dist + obs_penalty;
}

bool TopoGraphSearch::canConnect(const Vector3d& from, const Vector3d& to) {
    // Check distance constraint
    double dist = (to - from).norm();
    if (dist > connection_radius_) {
        return false;
    }
    
    // Check collision-free path
    return isPathFree(from, to);
}

bool TopoGraphSearch::isPathFree(const Vector3d& from, const Vector3d& to) {
    Vector3d dir = to - from;
    double dist = dir.norm();
    dir.normalize();
    
    double step = 0.05;  // 5cm resolution
    int num_checks = static_cast<int>(dist / step);
    
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

} // namespace ego_planner
