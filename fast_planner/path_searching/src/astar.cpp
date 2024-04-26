#include <path_searching/astar.h>
#include <sstream>
#include <iostream>
#include <algorithm>

using namespace std;
using namespace Eigen;

namespace fast_planner {
Astar::~Astar() {
  for (int i = 0; i < allocate_num_; i++) {
    delete path_node_pool_[i];
  }
}

int Astar::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool dynamic, double time_start) {
  /* ---------- initialize ---------- */
  NodePtr cur_node = path_node_pool_[0];
  cur_node->parent = nullptr;
  cur_node->position = start_pt;
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;

  Eigen::Vector3d end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal;

  end_index = posToIndex(end_pt);
  cur_node->f_score = lambda_heu_ * getEuclHeu(cur_node->position, end_pt);
  cur_node->node_state = IN_OPEN_SET;

  open_set_.push(cur_node);
  use_node_num_ += 1;

  if (dynamic) {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
    // RCLCPP_INFO(get_logger(), "time start: %f", time_start);
  } else
    expanded_nodes_.insert(cur_node->index, cur_node);

  NodePtr neighbor = nullptr;
  NodePtr terminate_node = nullptr;

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    /* ---------- get lowest f_score node ---------- */
    cur_node = open_set_.top();
    // RCLCPP_INFO(get_logger(), "pos: %f %f %f", cur_node->state.head(3).transpose());
    // RCLCPP_INFO(get_logger(), "time: %f", cur_node->time);
    // RCLCPP_INFO(get_logger(), "dist: %f", edt_environment_->evaluateCoarseEDT(cur_node->state.head(3), cur_node->time));

    /* ---------- determine termination ---------- */

    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 && abs(cur_node->index(2) - end_index(2)) <= 1;

    if (reach_end) {
      // RCLCPP_INFO(get_logger(), "[Astar]:---------------------- %d", use_node_num_);
      // RCLCPP_INFO(get_logger(), "use node num: %d", use_node_num_);
      // RCLCPP_INFO(get_logger(), "iter num: %d", iter_num_);
      terminate_node = cur_node;
      retrievePath(terminate_node);
      has_path_ = true;

      return REACH_END;
    }

    /* ---------- pop node and add to close set ---------- */
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    /* ---------- init neighbor expansion ---------- */

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d pro_pos;
    double pro_t;

    vector<Eigen::Vector3d> inputs;
    Eigen::Vector3d d_pos;

    /* ---------- expansion loop ---------- */
    for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
      for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
        for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) {
          d_pos << dx, dy, dz;

          if (d_pos.norm() < 1e-3) continue;

          pro_pos = cur_pos + d_pos;

          /* ---------- check if in feasible space ---------- */
          /* inside map range */
          if (pro_pos(0) <= origin_(0) || pro_pos(0) >= map_size_3d_(0) || pro_pos(1) <= origin_(1) ||
              pro_pos(1) >= map_size_3d_(1) || pro_pos(2) <= origin_(2) ||
              pro_pos(2) >= map_size_3d_(2)) {
            // RCLCPP_INFO(get_logger(), "outside map");
            continue;
          }

          /* not in close set */
          Eigen::Vector3i pro_id = posToIndex(pro_pos);
          int pro_t_id = timeToIndex(pro_t);

          NodePtr pro_node =
              dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);

          if (pro_node != nullptr && pro_node->node_state == IN_CLOSE_SET) {
            // RCLCPP_INFO(get_logger(), "in closeset");
            continue;
          }

          /* collision free */
          // double dist = dynamic ?
          // edt_environment_->evaluateCoarseEDT(pro_pos, cur_node->time + dt) :
          //                         edt_environment_->evaluateCoarseEDT(pro_pos,
          //                         -1.0);
          double dist = edt_environment_->evaluateCoarseEDT(pro_pos, -1.0);
          if (dist <= margin_) {
            continue;
          }

          /* ---------- compute cost ---------- */
          double time_to_goal, tmp_g_score, tmp_f_score;
          tmp_g_score = d_pos.squaredNorm() + cur_node->g_score;
          tmp_f_score = tmp_g_score + lambda_heu_ * getEuclHeu(pro_pos, end_pt);

          if (pro_node == nullptr) {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->position = pro_pos;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic) {
              pro_node->time = cur_node->time + 1.0;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              // RCLCPP_INFO(get_logger(), "run out of memory.");
              return NO_PATH;
            }
          } else if (pro_node->node_state == IN_OPEN_SET) {
            if (tmp_g_score < pro_node->g_score) {
              // pro_node->index = pro_id;
              pro_node->position = pro_pos;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->parent = cur_node;
              if (dynamic) pro_node->time = cur_node->time + 1.0;
            }
          } else {
            // RCLCPP_INFO(get_logger(), "error type in searching: %c", pro_node->node_state);
          }

          /* ----------  ---------- */
        }
  }

  /* ---------- open set empty, no path ---------- */
  // RCLCPP_INFO(get_logger(), "open set empty, no path!");
  // RCLCPP_INFO(get_logger(), "use node num: %d", use_node_num_);
  // RCLCPP_INFO(get_logger(), "iter num: %d", iter_num_);
  return NO_PATH;
}

void Astar::setParam(rclcpp::Node::SharedPtr& nh) {
  nh->get_parameter("astar/resolution_astar", resolution_);
  nh->get_parameter("astar/time_resolution", time_resolution_);
  nh->get_parameter("astar/lambda_heu", lambda_heu_);
  nh->get_parameter("astar/margin", margin_);
  nh->get_parameter("astar/allocate_num", allocate_num_);
  tie_breaker_ = 1.0 + 1.0 / 10000;

  // RCLCPP_INFO(get_logger(), "margin: %f", margin_);
}

void Astar::retrievePath(NodePtr end_node) {
  NodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != nullptr) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

std::vector<Eigen::Vector3d> Astar::getPath() {
  vector<Eigen::Vector3d> path;
  for (int i = 0; i < path_nodes_.size(); ++i) {
    path.push_back(path_nodes_[i]->position);
  }
  return path;
}

double Astar::getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));

  double h;
  int diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  return tie_breaker_ * h;
}

double Astar::getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));

  return tie_breaker_ * (dx + dy + dz);
}

double Astar::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2) {
  return tie_breaker_ * (x2 - x1).norm();
}

void Astar::init() {
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;
  edt_environment_->getMapRegion(origin_, map_size_3d_);

  // RCLCPP_INFO(get_logger(), "origin_: %f %f %f", origin_.transpose());
  // RCLCPP_INFO(get_logger(), "map size: %f %f %f", map_size_3d_.transpose());

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new Node;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
}

void Astar::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
}

void Astar::reset() {
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++) {
    NodePtr node = path_node_pool_[i];
    node->parent = nullptr;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
}

std::vector<NodePtr> Astar::getVisitedNodes() {
  vector<NodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector3i Astar::posToIndex(Eigen::Vector3d pt) {
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

  // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
  // origin_(1)) * inv_resolution_),
  //     floor((pt(2) - origin_(2)) * inv_resolution_);

  return idx;
}

int Astar::timeToIndex(double time) {
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

}  // namespace fast_planner
