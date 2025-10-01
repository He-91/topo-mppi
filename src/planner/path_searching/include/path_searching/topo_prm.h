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
    
    // Topological path generation
    std::vector<TopoPath> findTopoPaths(const Eigen::Vector3d& start, 
                                       const Eigen::Vector3d& goal);
    
    bool isPathValid(const std::vector<Eigen::Vector3d>& path);
    bool isLineCollisionFree(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
    
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