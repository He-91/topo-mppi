#ifndef _BIAS_SAMPLER_H_
#define _BIAS_SAMPLER_H_

#include <Eigen/Eigen>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <plan_env/grid_map.h>

namespace ego_planner {

/**
 * @brief Topological sampling space structure
 * Represents a sampling region guided by obstacles
 */
struct SamplingSpace {
    Eigen::Vector3d center;           // Center of sampling space
    std::vector<Eigen::Vector3d> corners;  // Key corner points (obstacle vertices)
    double radius;                    // Sampling radius
    int topo_id;                      // Topological class identifier
    
    SamplingSpace() : radius(0.0), topo_id(-1) {}
};

/**
 * @brief BiasSampler - Topologically-guided sampling framework
 * Extracted from TGK-Planner, simplified to pure geometry (no dynamics)
 * 
 * Core Ideas:
 * 1. Detect obstacle corners as topological key points
 * 2. Build sampling framework around these corners
 * 3. Guide path search along topological channels
 */
class BiasSampler {
public:
    typedef std::shared_ptr<BiasSampler> Ptr;
    
    BiasSampler();
    ~BiasSampler();
    
    /**
     * @brief Initialize sampler with grid map
     */
    void init(ros::NodeHandle& nh, GridMap::Ptr grid_map);
    
    /**
     * @brief Find sampling spaces between start and goal
     * @param start Start position
     * @param goal Goal position
     * @param spaces Output: detected sampling spaces
     * @return Success flag
     */
    bool findSamplingSpaces(const Eigen::Vector3d& start,
                           const Eigen::Vector3d& goal,
                           std::vector<SamplingSpace>& spaces);
    
    /**
     * @brief Sample a point from a given sampling space
     * @param space The sampling space
     * @param sample Output: sampled point
     * @return Success flag
     */
    bool sampleFromSpace(const SamplingSpace& space, Eigen::Vector3d& sample);
    
    /**
     * @brief Check if point is in collision-free space
     */
    bool isCollisionFree(const Eigen::Vector3d& pos);
    
    /**
     * @brief Get topological key points (obstacle corners)
     * These points define the topological structure of the environment
     */
    std::vector<Eigen::Vector3d> getTopoKeyPoints(const Eigen::Vector3d& start,
                                                   const Eigen::Vector3d& goal);
    
    // Parameter setters
    void setCornerDetectionRadius(double radius) { corner_detection_radius_ = radius; }
    void setSamplingRadius(double radius) { sampling_radius_ = radius; }
    void setResolution(double res) { resolution_ = res; }
    
private:
    GridMap::Ptr grid_map_;
    
    // Parameters
    double corner_detection_radius_;  // Radius for corner detection
    double sampling_radius_;          // Radius for sampling around corners
    double resolution_;               // Sampling resolution
    double collision_check_res_;      // Collision check resolution
    int max_corner_num_;             // Maximum number of corners to detect
    
    /**
     * @brief Detect obstacle corners in a region
     * A corner is a local geometry feature where obstacles create channels
     */
    std::vector<Eigen::Vector3d> detectObstacleCorners(
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& goal);
    
    /**
     * @brief Check if a point is an obstacle corner
     * Uses local geometry analysis (gradient-based)
     */
    bool isCornerPoint(const Eigen::Vector3d& pos);
    
    /**
     * @brief Build sampling space around a corner point
     */
    SamplingSpace buildSamplingSpace(const Eigen::Vector3d& corner, int topo_id);
    
    /**
     * @brief Check line collision (for corner detection)
     */
    bool isLineFree(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
    
    /**
     * @brief Calculate topological class ID based on relative position
     */
    int calculateTopoClass(const Eigen::Vector3d& corner,
                          const Eigen::Vector3d& start,
                          const Eigen::Vector3d& goal);
};

} // namespace ego_planner

#endif
