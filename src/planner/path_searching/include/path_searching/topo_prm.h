#ifndef _TOPO_PRM_H_
#define _TOPO_PRM_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>
#include <vector>
#include <memory>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace ego_planner {

struct TopoPath {
    std::vector<Eigen::Vector3d> path;
    double cost;
    int path_id;
    
    TopoPath() : cost(0.0), path_id(-1) {}
    TopoPath(const std::vector<Eigen::Vector3d>& p, double c, int id) 
        : path(p), cost(c), path_id(id) {}
};

// 🚀 NEW: Graph node for PRM-based topology planning (Fast-Planner style)
struct GraphNode {
    Eigen::Vector3d pos;
    int id;
    std::vector<GraphNode*> neighbors;
    
    GraphNode() : id(-1) {}
    GraphNode(const Eigen::Vector3d& p, int node_id) : pos(p), id(node_id) {}
};

class TopoPRM {
private:
    GridMap::Ptr grid_map_;
    ros::Publisher topo_paths_pub_;
    std::string frame_id_;
    
    // Parameters
    double step_size_;
    double search_radius_;
    int max_sample_num_;
    double collision_check_resolution_;
    
    // 🚀 NEW: Fast-Planner PRM parameters
    int max_raw_paths_;           // 最大原始路径数 (DFS搜索限制)
    int reserve_num_;             // 保留的最短路径数
    double clearance_;            // 节点最小安全距离
    double sample_inflate_;       // 椭球采样膨胀系数
    double ratio_to_short_;       // 相对最短路径的长度比率阈值
    int discretize_points_num_;   // 拓扑去重时的离散化点数
    
    // 🚀 NEW: PRM graph data structures
    std::vector<GraphNode*> graph_nodes_;
    std::vector<std::vector<Eigen::Vector3d>> raw_paths_;
    
    // Shared utility functions
    bool isPathValid(const std::vector<Eigen::Vector3d>& path);
    bool isLineCollisionFree(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
    
    // 🚀 NEW: Fast-Planner PRM methods
    // Week 1: 椭球采样
    std::vector<Eigen::Vector3d> sampleFreeSpaceInEllipsoid(
        const Eigen::Vector3d& start, const Eigen::Vector3d& goal, int num_samples);
    bool isPointFree(const Eigen::Vector3d& pt, double min_clearance);
    
    // Week 2: 可见性图构建
    void buildVisibilityGraph(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                               const std::vector<Eigen::Vector3d>& sample_points);
    void clearGraph();
    
    // Week 3: DFS多路径搜索
    void depthFirstSearch(std::vector<GraphNode*>& visited, GraphNode* goal_node);
    std::vector<std::vector<Eigen::Vector3d>> searchMultiplePaths(
        GraphNode* start_node, GraphNode* goal_node);
    
    // Week 4: 拓扑去重
    bool sameTopoPath(const std::vector<Eigen::Vector3d>& path1,
                      const std::vector<Eigen::Vector3d>& path2);
    std::vector<Eigen::Vector3d> discretizePath(const std::vector<Eigen::Vector3d>& path, int pt_num);
    std::vector<std::vector<Eigen::Vector3d>> pruneEquivalentPaths(
        const std::vector<std::vector<Eigen::Vector3d>>& paths);
    
    // 辅助函数
    int shortestPathIndex(const std::vector<std::vector<Eigen::Vector3d>>& paths);
    double pathLength(const std::vector<Eigen::Vector3d>& path);
    std::vector<std::vector<Eigen::Vector3d>> selectShortPaths(
        const std::vector<std::vector<Eigen::Vector3d>>& paths);
    
    // Legacy 4-direction topological path generation
    std::vector<TopoPath> findTopoPathsLegacy(const Eigen::Vector3d& start, 
                                              const Eigen::Vector3d& goal);
    
    // Four-directional obstacle avoidance (legacy)
    std::vector<Eigen::Vector3d> generateAlternativePath(const Eigen::Vector3d& start,
                                                        const Eigen::Vector3d& goal,
                                                        const Eigen::Vector3d& obstacle_center,
                                                        int direction); // 0=up, 1=down, 2=left, 3=right
    
    // Fast-Planner inspired path generation methods
    std::vector<Eigen::Vector3d> generateCircularPath(const Eigen::Vector3d& start,
                                                     const Eigen::Vector3d& goal,
                                                     const Eigen::Vector3d& obstacle_center,
                                                     int side); // -1=left, 1=right
    
    std::vector<Eigen::Vector3d> generateVerticalPath(const Eigen::Vector3d& start,
                                                     const Eigen::Vector3d& goal,
                                                     const Eigen::Vector3d& obstacle_center,
                                                     int vertical); // -1=under, 1=over
    
    std::vector<Eigen::Vector3d> generateTangentPoints(const Eigen::Vector3d& start,
                                                      const Eigen::Vector3d& goal,
                                                      const Eigen::Vector3d& obstacle_center);
    
    // 🚀 NEW: Dynamic obstacle size estimation
    double estimateObstacleSize(const Eigen::Vector3d& obstacle_center);
    
    // Cost calculation
    double calculatePathCost(const std::vector<Eigen::Vector3d>& path);
    double calculateSmoothnessCost(const std::vector<Eigen::Vector3d>& path);
    double calculateObstacleCost(const std::vector<Eigen::Vector3d>& path);
    
    // Visualization
    void visualizeTopoPaths(const std::vector<TopoPath>& paths);
    void publishPath(const std::vector<Eigen::Vector3d>& path, int id, 
                    double r, double g, double b, double scale = 0.1);

public:
    typedef std::shared_ptr<TopoPRM> Ptr;
    
    TopoPRM();
    ~TopoPRM();
    
    void init(ros::NodeHandle& nh, GridMap::Ptr grid_map);
    
    // Main interface
    bool searchTopoPaths(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                        std::vector<TopoPath>& topo_paths);
    
    TopoPath selectBestPath(const std::vector<TopoPath>& paths);
    
    // Parameters
    void setStepSize(double step_size) { step_size_ = step_size; }
    void setSearchRadius(double radius) { search_radius_ = radius; }
    void setMaxSampleNum(int num) { max_sample_num_ = num; }
};

} // namespace ego_planner

#endif