/**#ifndef _TOPO_PRM_H_

* Adapted from Fast-Planner (HKUST Aerial Robotics Group)#define _TOPO_PRM_H_

* Original: https://github.com/HKUST-Aerial-Robotics/Fast-Planner

* #include <iostream>

* Modified for EGO-Planner integration:#include <ros/ros.h>

* - Namespace: fast_planner → ego_planner#include <ros/console.h>

* - Environment: EDTEnvironment → GridMap  #include <Eigen/Eigen>

* - Added searchTopoPaths() adapter interface#include <plan_env/grid_map.h>

* #include <queue>

* Copyright 2019 Boyu Zhou, <bzhouai at connect dot ust dot hk>#include <vector>

* Integration modifications: 2025#include <memory>

*/#include <visualization_msgs/Marker.h>

#include <visualization_msgs/MarkerArray.h>

#ifndef _TOPO_PRM_H_

#define _TOPO_PRM_H_namespace ego_planner {



#include <plan_env/grid_map.h>struct TopoPath {

#include <Eigen/Eigen>    std::vector<Eigen::Vector3d> path;

#include <ros/ros.h>    double cost;

#include <random>    int path_id;

#include <memory>    

#include <vector>    TopoPath() : cost(0.0), path_id(-1) {}

#include <list>    TopoPath(const std::vector<Eigen::Vector3d>& p, double c, int id) 

        : path(p), cost(c), path_id(id) {}

namespace ego_planner {};



using std::vector;class TopoPRM {

using std::list;private:

using std::shared_ptr;    GridMap::Ptr grid_map_;

    ros::Publisher topo_paths_pub_;

/* ========== GraphNode: PRM图节点 ========== */    std::string frame_id_;

class GraphNode {    

public:    // Parameters

  enum NODE_TYPE { Guard = 1, Connector = 2 };    double step_size_;

  enum NODE_STATE { NEW = 1, CLOSE = 2, OPEN = 3 };    double search_radius_;

    int max_sample_num_;

  GraphNode() {}    double collision_check_resolution_;

  GraphNode(Eigen::Vector3d pos, NODE_TYPE type, int id) {    

    pos_ = pos;    // Shared utility functions

    type_ = type;    bool isPathValid(const std::vector<Eigen::Vector3d>& path);

    state_ = NEW;    bool isLineCollisionFree(const Eigen::Vector3d& start, const Eigen::Vector3d& end);

    id_ = id;    

  }    // Legacy 4-direction topological path generation

  ~GraphNode() {}    std::vector<TopoPath> findTopoPaths(const Eigen::Vector3d& start, 

                                       const Eigen::Vector3d& goal);

  vector<shared_ptr<GraphNode>> neighbors_;    

  Eigen::Vector3d pos_;    // Four-directional obstacle avoidance (legacy)

  NODE_TYPE type_;    std::vector<Eigen::Vector3d> generateAlternativePath(const Eigen::Vector3d& start,

  NODE_STATE state_;                                                        const Eigen::Vector3d& goal,

  int id_;                                                        const Eigen::Vector3d& obstacle_center,

                                                        int direction); // 0=up, 1=down, 2=left, 3=right

  typedef shared_ptr<GraphNode> Ptr;    

};    // Fast-Planner inspired path generation methods

    std::vector<Eigen::Vector3d> generateCircularPath(const Eigen::Vector3d& start,

/* ========== RayCaster: 射线投射(用于碰撞检测) ========== */                                                     const Eigen::Vector3d& goal,

class RayCaster {                                                     const Eigen::Vector3d& obstacle_center,

private:                                                     int side); // -1=left, 1=right

  Eigen::Vector3d start_;    

  Eigen::Vector3d end_;    std::vector<Eigen::Vector3d> generateVerticalPath(const Eigen::Vector3d& start,

  Eigen::Vector3d direction_;                                                     const Eigen::Vector3d& goal,

  Eigen::Vector3d min_;                                                     const Eigen::Vector3d& obstacle_center,

  Eigen::Vector3d max_;                                                     int vertical); // -1=under, 1=over

  int x_;    

  int y_;    std::vector<Eigen::Vector3d> generateTangentPoints(const Eigen::Vector3d& start,

  int z_;                                                      const Eigen::Vector3d& goal,

  int endX_;                                                      const Eigen::Vector3d& obstacle_center);

  int endY_;    

  int endZ_;    // Cost calculation

  double maxDist_;    double calculatePathCost(const std::vector<Eigen::Vector3d>& path);

  double dx_;    double calculateSmoothnessCost(const std::vector<Eigen::Vector3d>& path);

  double dy_;    double calculateObstacleCost(const std::vector<Eigen::Vector3d>& path);

  double dz_;    

  int stepX_;    // Visualization

  int stepY_;    void visualizeTopoPaths(const std::vector<TopoPath>& paths);

  int stepZ_;    void publishPath(const std::vector<Eigen::Vector3d>& path, int id, 

  double tMaxX_;                    double r, double g, double b, double scale = 0.1);

  double tMaxY_;

  double tMaxZ_;public:

  double tDeltaX_;    typedef std::shared_ptr<TopoPRM> Ptr;

  double tDeltaY_;    

  double tDeltaZ_;    TopoPRM();

  double dist_;    ~TopoPRM();

    

  int step_num_;    void init(ros::NodeHandle& nh, GridMap::Ptr grid_map);

    

public:    // Main interface

  RayCaster() {}    bool searchTopoPaths(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,

  ~RayCaster() {}                        std::vector<TopoPath>& topo_paths);

    

  // 设置起终点(voxel坐标)    TopoPath selectBestPath(const std::vector<TopoPath>& paths);

  bool setInput(const Eigen::Vector3d& start, const Eigen::Vector3d& end);    

      // Parameters

  // 迭代下一个体素    void setStepSize(double step_size) { step_size_ = step_size; }

  bool step(Eigen::Vector3d& ray_pt);    void setSearchRadius(double radius) { search_radius_ = radius; }

};    void setMaxSampleNum(int num) { max_sample_num_ = num; }

};

/* ========== TopologyPRM: 拓扑路径规划器 ========== */

class TopologyPRM {} // namespace ego_planner

private:

  /* ===== 环境 ===== */#endif
  GridMap::Ptr grid_map_;

  /* ===== 采样器 ===== */
  std::random_device rd_;
  std::default_random_engine eng_;
  std::uniform_real_distribution<double> rand_pos_;

  Eigen::Vector3d sample_r_;       // 采样区域半径
  Eigen::Vector3d translation_;    // 采样区域中心
  Eigen::Matrix3d rotation_;       // 采样区域旋转

  /* ===== 图数据结构 ===== */
  list<GraphNode::Ptr> graph_;                      // PRM图 (0:start, 1:goal, 2-n:其他节点)
  vector<vector<Eigen::Vector3d>> raw_paths_;       // 原始路径
  vector<vector<Eigen::Vector3d>> short_paths_;     // shortcut后路径
  vector<vector<Eigen::Vector3d>> final_paths_;     // 最终精选路径
  vector<Eigen::Vector3d> start_pts_, end_pts_;     // 起终点段

  /* ===== 射线投射(用于碰撞检测) ===== */
  vector<RayCaster> casters_;
  Eigen::Vector3d offset_;   // GridMap坐标偏移

  /* ===== 参数 ===== */
  double max_sample_time_;   // 最大采样时间(秒)
  int max_sample_num_;       // 最大采样点数
  int max_raw_path_;         // 最大原始路径数
  int max_raw_path2_;        // DFS后筛选路径数
  int short_cut_num_;        // shortcut迭代次数
  Eigen::Vector3d sample_inflate_;  // 采样区域膨胀
  double resolution_;        // GridMap分辨率
  double clearance_;         // 最小障碍物间隙
  double ratio_to_short_;    // 路径长度比阈值
  int reserve_num_;          // 保留路径数
  bool parallel_shortcut_;   // 是否并行shortcut

  /* ===== 核心算法 ===== */
  // 创建PRM图
  list<GraphNode::Ptr> createGraph(Eigen::Vector3d start, Eigen::Vector3d end);
  
  // DFS搜索所有路径
  vector<vector<Eigen::Vector3d>> searchPaths();
  void depthFirstSearch(vector<GraphNode::Ptr>& vis);
  
  // 路径shortcut优化
  void shortcutPaths();
  void shortcutPath(vector<Eigen::Vector3d> path, int path_id, int iter_num = 1);
  
  // 等价路径剪枝
  vector<vector<Eigen::Vector3d>> pruneEquivalent(vector<vector<Eigen::Vector3d>>& paths);
  bool sameTopoPath(const vector<Eigen::Vector3d>& path1, 
                    const vector<Eigen::Vector3d>& path2, double thresh);
  
  // 选择最短路径
  vector<vector<Eigen::Vector3d>> selectShortPaths(vector<vector<Eigen::Vector3d>>& paths, int step);
  int shortestPath(vector<vector<Eigen::Vector3d>>& paths);

  /* ===== 辅助函数 ===== */
  // 采样
  Eigen::Vector3d getSample();
  
  // 可见性检测
  vector<GraphNode::Ptr> findVisibGuard(Eigen::Vector3d pt);
  bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt);
  bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
                 double thresh, Eigen::Vector3d& pc, int caster_id = 0);
  
  // 图剪枝
  void pruneGraph();
  
  // 路径离散化
  vector<Eigen::Vector3d> discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2);
  vector<Eigen::Vector3d> discretizePath(vector<Eigen::Vector3d> path);
  vector<Eigen::Vector3d> discretizePath(const vector<Eigen::Vector3d>& path, int pt_num);
  
  // 路径长度
  double pathLength(const vector<Eigen::Vector3d>& path);

public:
  typedef shared_ptr<TopologyPRM> Ptr;

  TopologyPRM();
  ~TopologyPRM();

  /* ===== 初始化 ===== */
  void init(ros::NodeHandle& nh);
  void setEnvironment(const GridMap::Ptr& env);

  /* ===== Fast-Planner核心接口 ===== */
  void findTopoPaths(Eigen::Vector3d start, Eigen::Vector3d end, 
                     vector<Eigen::Vector3d> start_pts, vector<Eigen::Vector3d> end_pts,
                     list<GraphNode::Ptr>& graph,
                     vector<vector<Eigen::Vector3d>>& raw_paths,
                     vector<vector<Eigen::Vector3d>>& filtered_paths,
                     vector<vector<Eigen::Vector3d>>& select_paths);

  /* ===== EGO-Planner适配接口 ===== */
  // 兼容原有planner_manager调用
  bool searchTopoPaths(const Eigen::Vector3d& start,
                      const Eigen::Vector3d& goal,
                      vector<vector<Eigen::Vector3d>>& topo_paths);
};

}  // namespace ego_planner

#endif
