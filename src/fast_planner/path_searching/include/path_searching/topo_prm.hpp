#ifndef _TOPO_PRM_H
#define _TOPO_PRM_H
#include <memory>
#include <vector>
#include <list>
#include <random>
#include <iostream>
#include <algorithm>
#include <Eigen/Core>
#include "rclcpp/rclcpp.hpp"
#include "plan_env/edt_environment.hpp" // Changed from .h to .hpp
#include "plan_env/raycast.h" // Changed from .h to .hpp


namespace fast_planner {

/* ---------- used for iterating all topo combination ---------- */
class TopoIterator {
private:
  std::vector<int> path_nums_;
  std::vector<int> cur_index_;
  int combine_num_;
  int cur_num_;

  void increase(int bit_num) {
    cur_index_[bit_num] += 1;
    if (cur_index_[bit_num] >= path_nums_[bit_num]) {
      cur_index_[bit_num] = 0;
      increase(bit_num + 1);
    }
  }

public:
  TopoIterator(std::vector<int> pn) {
    path_nums_ = pn;
    cur_index_.resize(path_nums_.size());
    std::fill(cur_index_.begin(), cur_index_.end(), 0);
    cur_num_ = 0;

    combine_num_ = 1;
    for (int i = 0; i < path_nums_.size(); ++i) {
      combine_num_ *= path_nums_[i] > 0 ? path_nums_[i] : 1;
    }
    std::cout << "[Topo]: merged path num: " << combine_num_ << std::endl;
  }
  TopoIterator() {
  }
  ~TopoIterator() {
  }

  bool nextIndex(std::vector<int>& index) {
    index = cur_index_;
    cur_num_ += 1;

    if (cur_num_ == combine_num_) return false;

    // go to next combination
    increase(0);
    return true;
  }
};

/* ---------- node of topo graph ---------- */
class GraphNode {
public:
  enum NODE_TYPE { Guard = 1, Connector = 2 };

  enum NODE_STATE { NEW = 1, CLOSE = 2, OPEN = 3 };

  GraphNode() {}
  GraphNode(Eigen::Vector3d pos, NODE_TYPE type, int id) {
    pos_ = pos;
    type_ = type;
    state_ = NEW;
    id_ = id;
  }
  ~GraphNode() {}

  std::vector<std::shared_ptr<GraphNode>> neighbors_;
  Eigen::Vector3d pos_;
  NODE_TYPE type_;
  NODE_STATE state_;
  int id_;

  typedef std::shared_ptr<GraphNode> Ptr;
};

class TopologyPRM {
private:
  EDTEnvironment::Ptr edt_environment_;  // environment representation

  // sampling generator
  std::random_device rd_;
  std::default_random_engine eng_;
  std::uniform_real_distribution<double> rand_pos_;

  Eigen::Vector3d sample_r_;
  Eigen::Vector3d translation_;
  Eigen::Matrix3d rotation_;

  // roadmap data structure, 0:start, 1:goal, 2-n: others
  std::list<GraphNode::Ptr> graph_;
  std::vector<std::vector<Eigen::Vector3d>> raw_paths_;
  std::vector<std::vector<Eigen::Vector3d>> short_paths_;
  std::vector<std::vector<Eigen::Vector3d>> final_paths_;
  std::vector<Eigen::Vector3d> start_pts_, end_pts_;

  // raycasting
  std::vector<RayCaster> casters_;
  Eigen::Vector3d offset_;

  // parameter
  double max_sample_time_;
  int max_sample_num_;
  int max_raw_path_, max_raw_path2_;
  int short_cut_num_;
  Eigen::Vector3d sample_inflate_;
  double resolution_;

  double ratio_to_short_;
  int reserve_num_;

  bool parallel_shortcut_;

  /* create topological roadmap */
  /* path searching, shortening, pruning and merging */
  std::list<GraphNode::Ptr> createGraph(const Eigen::Vector3d& start, const Eigen::Vector3d& end);
  std::vector<std::vector<Eigen::Vector3d>> searchPaths();
  void shortcutPaths();
  std::vector<std::vector<Eigen::Vector3d>> pruneEquivalent(std::vector<std::vector<Eigen::Vector3d>>& paths);
  std::vector<std::vector<Eigen::Vector3d>> selectShortPaths(std::vector<std::vector<Eigen::Vector3d>>& paths, int step);

  /* ---------- helper ---------- */
  inline Eigen::Vector3d getSample();
  std::vector<GraphNode::Ptr> findVisibGuard(const Eigen::Vector3d& pt);  // find pairs of visibile guard
  bool needConnection(const GraphNode::Ptr& g1, const GraphNode::Ptr& g2,
                      const Eigen::Vector3d& pt);  // test redundancy with existing
                                            // connection between two guard
  bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh,
                 Eigen::Vector3d& pc, int caster_id = 0);
  bool triangleVisib(Eigen::Vector3d pt, Eigen::Vector3d p1, Eigen::Vector3d p2);
  void pruneGraph();

  void depthFirstSearch(std::vector<GraphNode::Ptr>& vis);

  std::vector<Eigen::Vector3d> discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2);
  std::vector<std::vector<Eigen::Vector3d>> discretizePaths(std::vector<std::vector<Eigen::Vector3d>>& path);

  std::vector<Eigen::Vector3d> discretizePath(std::vector<Eigen::Vector3d> path);
  void shortcutPath(std::vector<Eigen::Vector3d> path, int path_id, int iter_num = 1);

  std::vector<Eigen::Vector3d> discretizePath(const std::vector<Eigen::Vector3d>& path, int pt_num);
  bool sameTopoPath(const std::vector<Eigen::Vector3d>& path1, const std::vector<Eigen::Vector3d>& path2,
                    double thresh);
  Eigen::Vector3d getOrthoPoint(const std::vector<Eigen::Vector3d>& path);

  int shortestPath(std::vector<std::vector<Eigen::Vector3d>>& paths);

public:
  double clearance_;

  TopologyPRM();
  ~TopologyPRM();

  void init(const rclcpp::Node::SharedPtr& nh);

  void setEnvironment(const EDTEnvironment::Ptr& env);

  void findTopoPaths(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const std::vector<Eigen::Vector3d>& start_pts,
                     const std::vector<Eigen::Vector3d>& end_pts, std::list<GraphNode::Ptr>& graph,
                     std::vector<std::vector<Eigen::Vector3d>>& raw_paths,
                     std::vector<std::vector<Eigen::Vector3d>>& filtered_paths,
                     std::vector<std::vector<Eigen::Vector3d>>& select_paths);

  double pathLength(const std::vector<Eigen::Vector3d>& path);
  std::vector<Eigen::Vector3d> pathToGuidePts(std::vector<Eigen::Vector3d>& path, int pt_num);
};

}  // namespace fast_planner

#endif