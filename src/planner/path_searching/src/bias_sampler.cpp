#include "path_searching/bias_sampler.h"
#include <cmath>
#include <algorithm>
#include <random>

using namespace std;
using namespace Eigen;

namespace ego_planner {

BiasSampler::BiasSampler() 
    : corner_detection_radius_(3.0),
      sampling_radius_(2.0),
      resolution_(0.1),
      collision_check_res_(0.05),
      max_corner_num_(20) {
}

BiasSampler::~BiasSampler() {
}

void BiasSampler::init(ros::NodeHandle& nh, GridMap::Ptr grid_map) {
    grid_map_ = grid_map;
    
    // Load parameters
    nh.param("bias_sampler/corner_detection_radius", corner_detection_radius_, 3.0);
    nh.param("bias_sampler/sampling_radius", sampling_radius_, 2.0);
    nh.param("bias_sampler/resolution", resolution_, 0.1);
    nh.param("bias_sampler/max_corner_num", max_corner_num_, 20);
    
    ROS_INFO("[BiasSampler] Initialized with:");
    ROS_INFO("  - Corner detection radius: %.2f", corner_detection_radius_);
    ROS_INFO("  - Sampling radius: %.2f", sampling_radius_);
    ROS_INFO("  - Resolution: %.2f", resolution_);
}

bool BiasSampler::findSamplingSpaces(const Vector3d& start,
                                     const Vector3d& goal,
                                     vector<SamplingSpace>& spaces) {
    spaces.clear();
    
    // Step 1: Detect obstacle corners between start and goal
    vector<Vector3d> corners = detectObstacleCorners(start, goal);
    
    if (corners.empty()) {
        ROS_INFO("[BiasSampler] No obstacle corners detected - path may be direct");
        return true;  // Not an error - direct path possible
    }
    
    ROS_INFO("[BiasSampler] Detected %zu obstacle corners", corners.size());
    
    // Step 2: Build sampling spaces around each corner
    for (size_t i = 0; i < corners.size(); ++i) {
        int topo_id = calculateTopoClass(corners[i], start, goal);
        SamplingSpace space = buildSamplingSpace(corners[i], topo_id);
        spaces.push_back(space);
    }
    
    ROS_INFO("[BiasSampler] Created %zu sampling spaces", spaces.size());
    return true;
}

vector<Vector3d> BiasSampler::detectObstacleCorners(const Vector3d& start,
                                                     const Vector3d& goal) {
    vector<Vector3d> corners;
    
    // Define search region around direct path
    Vector3d dir = (goal - start).normalized();
    double dist = (goal - start).norm();
    
    // Search in a cylinder around the direct path
    double search_radius = corner_detection_radius_;
    int num_samples = static_cast<int>(dist / resolution_);
    
    // Sample points along the path and around it
    for (int i = 0; i < num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        Vector3d center_pt = start + t * dist * dir;
        
        // Sample radially around this center point
        for (double theta = 0; theta < 2 * M_PI; theta += M_PI / 4) {
            for (double r = resolution_; r < search_radius; r += resolution_) {
                Vector3d offset = r * Vector3d(cos(theta), sin(theta), 0.0);
                Vector3d sample_pt = center_pt + offset;
                
                // Check if this point is a corner
                if (isCornerPoint(sample_pt)) {
                    // Avoid duplicates
                    bool is_duplicate = false;
                    for (const auto& corner : corners) {
                        if ((sample_pt - corner).norm() < resolution_ * 2.0) {
                            is_duplicate = true;
                            break;
                        }
                    }
                    
                    if (!is_duplicate) {
                        corners.push_back(sample_pt);
                        
                        if (corners.size() >= static_cast<size_t>(max_corner_num_)) {
                            ROS_WARN("[BiasSampler] Reached max corner number limit");
                            return corners;
                        }
                    }
                }
            }
        }
        
        // Also check in vertical direction
        for (double z = -search_radius; z <= search_radius; z += resolution_) {
            Vector3d sample_pt = center_pt + Vector3d(0, 0, z);
            
            if (isCornerPoint(sample_pt)) {
                bool is_duplicate = false;
                for (const auto& corner : corners) {
                    if ((sample_pt - corner).norm() < resolution_ * 2.0) {
                        is_duplicate = true;
                        break;
                    }
                }
                
                if (!is_duplicate) {
                    corners.push_back(sample_pt);
                    
                    if (corners.size() >= static_cast<size_t>(max_corner_num_)) {
                        return corners;
                    }
                }
            }
        }
    }
    
    return corners;
}

bool BiasSampler::isCornerPoint(const Vector3d& pos) {
    // A corner point is at the boundary between free and occupied space
    // AND has high gradient variation (concave/convex geometry)
    
    // Check 1: Must be collision-free
    if (!isCollisionFree(pos)) {
        return false;
    }
    
    // Check 2: Must be close to obstacles (near boundary)
    // 🔧 Phase 4: Use getDistanceWithGrad (our ESDF API)
    Vector3d grad;
    double dist = grid_map_->getDistanceWithGrad(pos, grad);
    
    // 🔧 Phase 4.5: 放宽距离阈值（原来1.0m太严格，改为3.0m）
    // 原因：0 key points → TGK失败 → 降级到legacy
    // 修复：接受更远的点作为潜在角点
    if (dist > sampling_radius_ * 1.5) {  // 2.0m × 1.5 = 3.0m
        return false;  // Too far from obstacles
    }
    
    // Check 3: Must have multiple obstacle directions nearby (corner feature)
    // Sample 8 directions around the point
    vector<bool> occupied_dirs;
    double check_dist = resolution_ * 2.0;
    
    for (double theta = 0; theta < 2 * M_PI; theta += M_PI / 4) {
        Vector3d dir(cos(theta), sin(theta), 0.0);
        Vector3d check_pt = pos + check_dist * dir;
        occupied_dirs.push_back(!isCollisionFree(check_pt));
    }
    
    // Count transitions from free to occupied
    int transitions = 0;
    for (size_t i = 0; i < occupied_dirs.size(); ++i) {
        if (occupied_dirs[i] != occupied_dirs[(i + 1) % occupied_dirs.size()]) {
            transitions++;
        }
    }
    
    // A corner should have at least 2 transitions (meaning multiple obstacle edges meet)
    return transitions >= 2;
}

SamplingSpace BiasSampler::buildSamplingSpace(const Vector3d& corner, int topo_id) {
    SamplingSpace space;
    space.center = corner;
    space.radius = sampling_radius_;
    space.topo_id = topo_id;
    
    // Store corner as a key point
    space.corners.push_back(corner);
    
    return space;
}

bool BiasSampler::sampleFromSpace(const SamplingSpace& space, Vector3d& sample) {
    static random_device rd;
    static mt19937 gen(rd());
    uniform_real_distribution<double> dis(-1.0, 1.0);
    
    int max_attempts = 20;
    for (int i = 0; i < max_attempts; ++i) {
        // Sample uniformly in a sphere
        Vector3d offset(dis(gen), dis(gen), dis(gen));
        offset.normalize();
        offset *= space.radius * abs(dis(gen));
        
        sample = space.center + offset;
        
        // Check if sample is collision-free
        if (isCollisionFree(sample)) {
            return true;
        }
    }
    
    ROS_WARN("[BiasSampler] Failed to find collision-free sample after %d attempts", max_attempts);
    return false;
}

bool BiasSampler::isCollisionFree(const Vector3d& pos) {
    return !grid_map_->getInflateOccupancy(pos);
}

bool BiasSampler::isLineFree(const Vector3d& p1, const Vector3d& p2) {
    Vector3d dir = p2 - p1;
    double dist = dir.norm();
    dir.normalize();
    
    int num_checks = static_cast<int>(dist / collision_check_res_);
    for (int i = 0; i <= num_checks; ++i) {
        Vector3d check_pt = p1 + (dist * i / num_checks) * dir;
        if (!isCollisionFree(check_pt)) {
            return false;
        }
    }
    
    return true;
}

int BiasSampler::calculateTopoClass(const Vector3d& corner,
                                    const Vector3d& start,
                                    const Vector3d& goal) {
    // Classify topological relationship based on corner's position
    // relative to the start-goal line
    
    Vector3d sg_dir = (goal - start).normalized();
    Vector3d sc_vec = corner - start;
    
    // Project corner onto start-goal line
    double proj = sc_vec.dot(sg_dir);
    Vector3d proj_pt = start + proj * sg_dir;
    Vector3d perp_vec = corner - proj_pt;
    
    // Classify based on perpendicular distance and direction
    double perp_dist = perp_vec.norm();
    
    if (perp_dist < resolution_) {
        return 0;  // On the line
    }
    
    // Use cross product to determine left/right
    Vector3d cross = sg_dir.cross(perp_vec);
    
    if (cross.z() > 0) {
        return 1;  // Left side
    } else {
        return 2;  // Right side
    }
}

vector<Vector3d> BiasSampler::getTopoKeyPoints(const Vector3d& start,
                                               const Vector3d& goal) {
    // Get all corner points as topological key points
    return detectObstacleCorners(start, goal);
}

} // namespace ego_planner
