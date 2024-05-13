#ifndef FAST_PLANNER__BSPLINE_OPTIMIZER_HPP_
#define FAST_PLANNER__BSPLINE_OPTIMIZER_HPP_

#include "bspline_opt/bspline_optimizer.h"
#include <nlopt.hpp>

namespace fast_planner {

const int BsplineOptimizer::SMOOTHNESS  = (1 << 0);
const int BsplineOptimizer::DISTANCE    = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::ENDPOINT    = (1 << 3);
const int BsplineOptimizer::GUIDE       = (1 << 4);
const int BsplineOptimizer::WAYPOINTS   = (1 << 6);

const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE;
const int BsplineOptimizer::NORMAL_PHASE =
    BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE | BsplineOptimizer::FEASIBILITY;

void BsplineOptimizer::setParam(std::shared_ptr<FastPlanner> nh) {

  nh->get_parameter("optimization/lambda1", lambda1_);
  nh->get_parameter("optimization/lambda2", lambda2_);
  nh->get_parameter("optimization/lambda3", lambda3_);
  nh->get_parameter("optimization/lambda4", lambda4_);
  nh->get_parameter("optimization/lambda5", lambda5_);
  nh->get_parameter("optimization/lambda6", lambda6_);
  nh->get_parameter("optimization/lambda7", lambda7_);
  nh->get_parameter("optimization/lambda8", lambda8_);

  nh->get_parameter("optimization/dist0", dist0_);
  nh->get_parameter("optimization/max_vel", max_vel_);
  nh->get_parameter("optimization/max_acc", max_acc_);
  nh->get_parameter("optimization/visib_min", visib_min_);
  nh->get_parameter("optimization/dlmin", dlmin_);
  nh->get_parameter("optimization/wnl", wnl_);

  nh->get_parameter("optimization/max_iteration_num1", max_iteration_num_[0]);
  nh->get_parameter("optimization/max_iteration_num2", max_iteration_num_[1]);
  nh->get_parameter("optimization/max_iteration_num3", max_iteration_num_[2]);
  nh->get_parameter("optimization/max_iteration_num4", max_iteration_num_[3]);
  nh->get_parameter("optimization/max_iteration_time1", max_iteration_time_[0]);
  nh->get_parameter("optimization/max_iteration_time2", max_iteration_time_[1]);
  nh->get_parameter("optimization/max_iteration_time3", max_iteration_time_[2]);
  nh->get_parameter("optimization/max_iteration_time4", max_iteration_time_[3]);

  nh->get_parameter("optimization/algorithm1", algorithm1_);
  nh->get_parameter("optimization/algorithm2", algorithm2_);
  nh->get_parameter("optimization/order", order_);
}

void BsplineOptimizer::setEnvironment(const std::shared_ptr<EDTEnvironment> env) {
  edt_environment_ = env;
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd& points) {
  control_points_ = points;
  dim_            = control_points_.cols();
}

void BsplineOptimizer::setBsplineInterval(const double& ts) { bspline_interval_ = ts; }

void BsplineOptimizer::setTerminateCond(const int& max_num_id, const int& max_time_id) {
  max_num_id_  = max_num_id;
  max_time_id_ = max_time_id;
}

void BsplineOptimizer::setCostFunction(const int& cost_code) {
  cost_function_ = cost_code;

  // print optimized cost function
  std::string cost_str;
  if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";
  if (cost_function_ & DISTANCE) cost_str += " dist  |";
  if (cost_function_ & FEASIBILITY) cost_str += " feasi |";
  if (cost_function_ & ENDPOINT) cost_str += " endpt |";
  if (cost_function_ & GUIDE) cost_str += " guide |";
  if (cost_function_ & WAYPOINTS) cost_str += " waypt |";

  RCLCPP_INFO_STREAM(rclcpp::get_logger("BsplineOptimizer"), "cost func: " << cost_str);
}

void BsplineOptimizer::setGuidePath(const std::vector<Eigen::Vector3d>& guide_pt) { guide_pts_ = guide_pt; }

void BsplineOptimizer::setWaypoints(const std::vector<Eigen::Vector3d>& waypts,
                                    const std::vector<int>&             waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

Eigen::MatrixXd BsplineOptimizer::BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                                      const int& cost_function, int max_num_id,
                                                      int max_time_id) {
  setControlPoints(points);
  setBsplineInterval(ts);
  setCostFunction(cost_function);
  setTerminateCond(max_num_id, max_time_id);

  optimize();
  return this->control_points_;
}

void BsplineOptimizer::optimize() {
  /* initialize solver */
  iter_num_        = 0;
  min_cost_        = std::numeric_limits<double>::max();
  const int pt_num = control_points_.rows();
  g_q_.resize(pt_num);
  g_smoothness_.resize(pt_num);
  g_distance_.resize(pt_num);
  g_feasibility_.resize(pt_num);
  g_endpoint_.resize(pt_num);
  g_waypoints_.resize(pt_num);
  g_guide_.resize(pt_num);

  if (cost_function_ & ENDPOINT) {
    variable_num_ = dim_ * (pt_num - order_);
    // end position used for hard constraint
    end_pt_ = (1 / 6.0) *
        (control_points_.row(pt_num - 3) + 4 * control_points_.row(pt_num - 2) +
         control_points_.row(pt_num - 1));
  } else {
    variable_num_ = std::max(0, dim_ * (pt_num - 2 * order_));
  }

  std::cout << "Variable Num " << variable_num_<< std::endl;
  std::cout << "Is quadratic " << isQuadratic()<< std::endl;

  /* do optimization using NLopt slover */
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);
  opt.set_maxeval(max_iteration_num_[max_num_id_]);
  opt.set_maxtime(max_iteration_time_[max_time_id_]);
  opt.set_xtol_rel(1e-5);

  std::vector<double> q(variable_num_);
  for (int i = order_; i < pt_num; ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      q[dim_ * (i - order_) + j] = control_points_(i, j);
    }
  }

  if (dim_ != 1) {
    std::vector<double> lb(variable_num_), ub(variable_num_);
    const double   bound = 10.0;
    for (int i = 0; i < variable_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
  }

  try {
    double        final_cost;
    RCLCPP_WARN(rclcpp::get_logger("BsplineOptimizer"), "%f", final_cost);
    std::cout << "Vector size: " << q.size() << std::endl;
    for (size_t i = 0; i < q.size(); ++i) {
        std::cout << "q[" << i << "]: " << q[i] << std::endl;
    }
    nlopt::result result = opt.optimize(q, final_cost);
    // RCLCPP_WARN(rclcpp::get_logger("BsplineOptimizer"), "%f", result);

    /* retrieve the optimization result */
  } catch (std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("BsplineOptimizer"), "[Optimization]: nlopt exception ANNA");
    std::cout << e.what() << std::endl;
  }

  for (int i = order_; i < control_points_.rows(); ++i) {
    if (!(cost_function_ & ENDPOINT) && i >= pt_num - order_) continue;
    for (int j = 0; j < dim_; j++) {
      control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
    }
  }

  if (!(cost_function_ & GUIDE)) RCLCPP_INFO_STREAM(rclcpp::get_logger("BsplineOptimizer"), "iter num: " << iter_num_);
}

void BsplineOptimizer::calcSmoothnessCost(const std::vector<Eigen::Vector3d>& q, double& cost,
                                          std::vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);
  Eigen::Vector3d jerk, temp_j;

  for (int i = 0; i < q.size() - order_; i++) {
    /* evaluate jerk */
    jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    cost += jerk.squaredNorm();
    temp_j = 2.0 * jerk;
    /* jerk gradient */
    gradient[i + 0] += -temp_j;
    gradient[i + 1] += 3.0 * temp_j;
    gradient[i + 2] += -3.0 * temp_j;
    gradient[i + 3] += temp_j;
  }
}

void BsplineOptimizer::calcDistanceCost(const std::vector<Eigen::Vector3d>& q, double& cost,
                                        std::vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  double          dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);

  int end_idx = (cost_function_ & ENDPOINT) ? q.size() : q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);
    if (dist_grad.norm() > 1e-4) dist_grad.normalize();

    if (dist < dist0_) {
      cost += pow(dist - dist0_, 2);
      gradient[i] += 2.0 * (dist - dist0_) * dist_grad;
    }
  }
}

void BsplineOptimizer::calcFeasibilityCost(const std::vector<Eigen::Vector3d>& q, double& cost,
                                           std::vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  /* abbreviation */
  double ts, vm2, am2, ts_inv2, ts_inv4;
  vm2 = max_vel_ * max_vel_;
  am2 = max_acc_ * max_acc_;

  ts      = bspline_interval_;
  ts_inv2 = 1 / ts / ts;
  ts_inv4 = ts_inv2 * ts_inv2;

  /* velocity feasibility */
  for (int i = 0; i < q.size() - 1; i++) {
    Eigen::Vector3d vi = q[i + 1] - q[i];

    for (int j = 0; j < 3; j++) {
      double vd = vi(j) * vi(j) * ts_inv2 - vm2;
      if (vd > 0.0) {
        cost += pow(vd, 2);

        double temp_v = 4.0 * vd * ts_inv2;
        gradient[i + 0](j) += -temp_v * vi(j);
        gradient[i + 1](j) += temp_v * vi(j);
      }
    }
  }

  /* acceleration feasibility */
  for (int i = 0; i < q.size() - 2; i++) {
    Eigen::Vector3d ai = q[i + 2] - 2 * q[i + 1] + q[i];

    for (int j = 0; j < 3; j++) {
      double ad = ai(j) * ai(j) * ts_inv4 - am2;
      if (ad > 0.0) {
        cost += pow(ad, 2);

        double temp_a = 4.0 * ad * ts_inv4;
        gradient[i + 0](j) += temp_a * ai(j);
        gradient[i + 1](j) += -2 * temp_a * ai(j);
        gradient[i + 2](j) += temp_a * ai(j);
      }
    }
  }
}

void BsplineOptimizer::calcEndpointCost(const std::vector<Eigen::Vector3d>& q, double& cost,
                                        std::vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  // zero cost and gradient in hard constraints
  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - end_pt_;
  cost += dq.squaredNorm();

  gradient[q.size() - 3] += 2 * dq * (1 / 6.0);
  gradient[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient[q.size() - 1] += 2 * dq * (1 / 6.0);
}

void BsplineOptimizer::calcWaypointsCost(const std::vector<Eigen::Vector3d>& q, double& cost,
                                         std::vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int             idx   = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient[idx + 2] += dq * (2.0 / 6.0);
  }
}

/* use the uniformly sampled points on a geomertic path to guide the
 * trajectory. For each control points to be optimized, it is assigned a
 * guiding point on the path and the distance between them is penalized */
void BsplineOptimizer::calcGuideCost(const std::vector<Eigen::Vector3d>& q, double& cost,
                                     std::vector<Eigen::Vector3d>& gradient) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient.begin(), gradient.end(), zero);

  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Eigen::Vector3d gpt = guide_pts_[i - order_];
    cost += (q[i] - gpt).squaredNorm();
    gradient[i] += 2 * (q[i] - gpt);
  }
}

void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                   double& f_combine) {
  /* convert the NLopt format vector to control points. */

  // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store each control point
  // For 1D case, the second and third elements are zero, and similar for the 2D case.
  for (int i = 0; i < order_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i][j] = control_points_(i, j);
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i][j] = 0.0;
    }
  }

  for (int i = 0; i < variable_num_ / dim_; i++) {
    for (int j = 0; j < dim_; ++j) {
      g_q_[i + order_][j] = x[dim_ * i + j];
    }
    for (int j = dim_; j < 3; ++j) {
      g_q_[i + order_][j] = 0.0;
    }
  }

  if (!(cost_function_ & ENDPOINT)) {
    for (int i = 0; i < order_; i++) {

      for (int j = 0; j < dim_; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] =
            control_points_(control_points_.rows() - order_ + i, j);
      }
      for (int j = dim_; j < 3; ++j) {
        g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
      }
    }
  }

  double cost_smoothness, cost_distance, cost_feasibility, cost_endpoint, cost_guide, cost_waypoints;

  std::vector<Eigen::Vector3d> g_smoothness(g_q_.size(), Eigen::Vector3d::Zero()),
      g_distance(g_q_.size(), Eigen::Vector3d::Zero()),
      g_feasibility(g_q_.size(), Eigen::Vector3d::Zero()),
      g_endpoint(g_q_.size(), Eigen::Vector3d::Zero()),
      g_waypoints(g_q_.size(), Eigen::Vector3d::Zero()),
      g_guide(g_q_.size(), Eigen::Vector3d::Zero());

  calcSmoothnessCost(g_q_, cost_smoothness, g_smoothness);
  calcDistanceCost(g_q_, cost_distance, g_distance);
  calcFeasibilityCost(g_q_, cost_feasibility, g_feasibility);

  cost_waypoints = 0.0;
  if (cost_function_ & WAYPOINTS) {
    calcWaypointsCost(g_q_, cost_waypoints, g_waypoints);
  }

  cost_guide = 0.0;
  if (cost_function_ & GUIDE) {
    calcGuideCost(g_q_, cost_guide, g_guide);
  }

  f_combine = lambda1_ * cost_smoothness + lambda2_ * cost_distance + lambda3_ * cost_feasibility;

  if (cost_function_ & ENDPOINT) {
    calcEndpointCost(g_q_, cost_endpoint, g_endpoint);
    f_combine += lambda4_ * cost_endpoint;
  }

  if (cost_function_ & GUIDE) {
    f_combine += lambda5_ * cost_guide;
  }

  if (cost_function_ & WAYPOINTS) {
    f_combine += lambda6_ * cost_waypoints;
  }

  // accumulate gradient
  if (!grad.empty()) {
    for (int i = 0; i < grad.size(); i++) {
      for (int j = 0; j < 3; j++) {
        grad[i] = lambda1_ * g_smoothness[i][j] + lambda2_ * g_distance[i][j] +
            lambda3_ * g_feasibility[i][j];

        if (cost_function_ & ENDPOINT) {
          grad[i] += lambda4_ * g_endpoint[i][j];
        }

        if (cost_function_ & GUIDE) {
          grad[i] += lambda5_ * g_guide[i][j];
        }

        if (cost_function_ & WAYPOINTS) {
          grad[i] += lambda6_ * g_waypoints[i][j];
        }
      }
    }
  }
}

double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double            cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_      = cost;
    opt->best_variable_ = x;
  }
  return cost;
}

vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints() { return this->control_points_; }

bool BsplineOptimizer::isQuadratic() {
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }
  return false;
}

}  // namespace fast_planner

#endif  // FAST_PLANNER__BSPLINE_OPTIMIZER_HPP_
