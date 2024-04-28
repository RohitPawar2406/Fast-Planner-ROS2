#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "plan_manage/planner_manager.h"
//rclcpp::Clock(RCL_ROS_TIME).now()
namespace fast_planner {

FastPlannerManager::FastPlannerManager() {}

FastPlannerManager::~FastPlannerManager() {
  std::cout << "des manager" << std::endl;
}

void FastPlannerManager::initPlanModules(rclcpp::Node::SharedPtr& nh) {
  /* read algorithm parameters */

  nh->get_parameter_or("manager/max_vel", pp_.max_vel_, -1.0);
  nh->get_parameter_or("manager/max_acc", pp_.max_acc_, -1.0);
  nh->get_parameter_or("manager/max_jerk", pp_.max_jerk_, -1.0);
  nh->get_parameter_or("manager/dynamic_environment", pp_.dynamic_, -1);
  nh->get_parameter_or("manager/clearance_threshold", pp_.clearance_, -1.0);
  nh->get_parameter_or("manager/local_segment_length", pp_.local_traj_len_, -1.0);
  nh->get_parameter_or("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);

  bool use_geometric_path, use_kinodynamic_path, use_topo_path, use_optimization, use_active_perception;
  nh->get_parameter_or("manager/use_geometric_path", use_geometric_path, false);
  nh->get_parameter_or("manager/use_kinodynamic_path", use_kinodynamic_path, false);
  nh->get_parameter_or("manager/use_topo_path", use_topo_path, false);
  nh->get_parameter_or("manager/use_optimization", use_optimization, false);

  local_data_.traj_id_ = 0;
  sdf_map_.reset(new SDFMap);
  sdf_map_->initMap(nh);
  edt_environment_.reset(new EDTEnvironment);
  edt_environment_->setMap(sdf_map_);

  if (use_geometric_path) {
    geo_path_finder_.reset(new Astar);
    geo_path_finder_->setParam(nh);
    geo_path_finder_->setEnvironment(edt_environment_);
    geo_path_finder_->init();
  }

  if (use_kinodynamic_path) {
    kino_path_finder_.reset(new KinodynamicAstar);
    kino_path_finder_->setParam(nh);
    kino_path_finder_->setEnvironment(edt_environment_);
    kino_path_finder_->init();
  }

  if (use_optimization) {
    bspline_optimizers_.resize(10);
    for (int i = 0; i < 10; ++i) {
      bspline_optimizers_[i].reset(new BsplineOptimizer);
      bspline_optimizers_[i]->setParam(nh);
      bspline_optimizers_[i]->setEnvironment(edt_environment_);
    }
  }

  if (use_topo_path) {
    topo_prm_.reset(new TopologyPRM);
    topo_prm_->setEnvironment(edt_environment_);
    topo_prm_->init(nh);
  }
}

void FastPlannerManager::setGlobalWaypoints(std::vector<Eigen::Vector3d>& waypoints) {
  plan_data_.global_waypoints_ = waypoints;
}

bool FastPlannerManager::checkTrajCollision(double& distance) {
  double t_now = (rclcpp::Clock().now() - local_data_.start_time_).seconds();

  double tm, tmp;
  local_data_.position_traj_.getTimeSpan(tm, tmp);
  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now);

  double          radius = 0.0;
  Eigen::Vector3d fut_pt;
  double          fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);

    double dist = edt_environment_->evaluateCoarseEDT(fut_pt, -1.0);
    if (dist < 0.1) {
      distance = radius;
      return false;
    }

    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

bool FastPlannerManager::kinodynamicReplan( Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc, Eigen::Vector3d end_pt, Eigen::Vector3d end_vel) {

  std::cout << "[kino replan]: -----------------------" << std::endl;
  std::cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << ", "
            << start_acc.transpose() << "\ngoal:" << end_pt.transpose() << ", " << end_vel.transpose()
            << std::endl;

  if ((start_pt - end_pt).norm() < 0.2) {
    std::cout << "Close goal" << std::endl;
    return false;
  }

  auto t1 = rclcpp::Clock().now();
  double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

  Eigen::Vector3d init_pos = start_pt;
  Eigen::Vector3d init_vel = start_vel;
  Eigen::Vector3d init_acc = start_acc;

  // kinodynamic path searching

  t1 = rclcpp::Clock().now();

  kino_path_finder_->reset();

  int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);

  if (status == KinodynamicAstar::NO_PATH) {
    std::cout << "[kino replan]: kinodynamic search fail!" << std::endl;

    // retry searching with discontinuous initial state
    kino_path_finder_->reset();
    status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);

    if (status == KinodynamicAstar::NO_PATH) {
      std::cout << "[kino replan]: Can't find path." << std::endl;
      return false;
    } else {
      std::cout << "[kino replan]: retry search success." << std::endl;
    }

  } else {
    std::cout << "[kino replan]: kinodynamic search success." << std::endl;
  }

  plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

  t_search = (rclcpp::Clock().now() - t1).seconds();

  // parameterize the path to bspline

  double ts = pp_.ctrl_pt_dist / pp_.max_vel_;
  std::vector<Eigen::Vector3d> point_set, start_end_derivatives;
  kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  NonUniformBspline init(ctrl_pts, 3, ts);

  // bspline trajectory optimization

  t1 = rclcpp::Clock().now();

  int cost_function = BsplineOptimizer::NORMAL_PHASE;

  if (status != KinodynamicAstar::REACH_END) {
    cost_function |= BsplineOptimizer::ENDPOINT;
  }

  ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

  t_opt = (rclcpp::Clock().now() - t1).seconds();

  // iterative time adjustment

  t1 = rclcpp::Clock().now();
  NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);

  double to = pos.getTimeSum();
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  bool feasible = pos.checkFeasibility(false);

  int iter_num = 0;
  while (!feasible && rclcpp::ok()) {

    feasible = pos.reallocateTime();

    if (++iter_num >= 3) break;
  }

  double tn = pos.getTimeSum();

  std::cout << "[kino replan]: Reallocate ratio: " << tn / to << std::endl;
  if (tn / to > 3.0) {
    std::cout << "reallocate error." << std::endl;
  }

  t_adjust = (rclcpp::Clock().now() - t1).seconds();

  // save planned results

  local_data_.position_traj_ = pos;

  double t_total = t_search + t_opt + t_adjust;
  std::cout << "[kino replan]: time: " << t_total << ", search: " << t_search << ", optimize: " << t_opt
            << ", adjust time:" << t_adjust << std::endl;

  pp_.time_search_   = t_search;
  pp_.time_optimize_ = t_opt;
  pp_.time_adjust_   = t_adjust;

  updateTrajInfo();

  return true;
}

////////////////////////////////////

bool FastPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos) {
  plan_data_.clearTopoPaths();

  // Generate global reference trajectory

  std::vector<Eigen::Vector3d> points = plan_data_.global_waypoints_;
  if (points.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("INFO:planner_manager"), "No global waypoints!");
  }

  points.insert(points.begin(), start_pos);

  // Insert intermediate points if too far
  std::vector<Eigen::Vector3d> inter_points;
  const double dist_thresh = 4.0;

  for (size_t i = 0; i < points.size() - 1; ++i) {
    inter_points.push_back(points.at(i));
    double dist = (points.at(i + 1) - points.at(i)).norm();

    if (dist > dist_thresh) {
      int id_num = static_cast<int>(std::floor(dist / dist_thresh)) + 1;

      for (int j = 1; j < id_num; ++j) {
        Eigen::Vector3d inter_pt =
            points.at(i) * (1.0 - static_cast<double>(j) / id_num) + points.at(i + 1) * static_cast<double>(j) / id_num;
        inter_points.push_back(inter_pt);
      }
    }
  }

  inter_points.push_back(points.back());
  if (inter_points.size() == 2) {
    Eigen::Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
    inter_points.insert(inter_points.begin() + 1, mid);
  }

  // Write position matrix
  size_t pt_num = inter_points.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (size_t i = 0; i < pt_num; ++i) pos.row(i) = inter_points[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd time(pt_num - 1);
  for (size_t i = 0; i < pt_num - 1; ++i) {
    time(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_);
  }

  time(0) *= 2.0;
  time(0) = std::max(1.0, time(0));
  time(time.rows() - 1) *= 2.0;
  time(time.rows() - 1) = std::max(1.0, time(time.rows() - 1));

  PolynomialTraj gl_traj = minSnapTraj(pos, zero, zero, zero, zero, time);

  auto time_now = rclcpp::Clock(RCL_ROS_TIME).now();
  global_data_.setGlobalTraj(gl_traj, time_now);

  // Truncate a local trajectory

  double dt, duration;
  Eigen::MatrixXd ctrl_pts = reparamLocalTraj(0.0, dt, duration);
  NonUniformBspline bspline(ctrl_pts, 3, dt);

  global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
  local_data_.position_traj_ = bspline;
  local_data_.start_time_ = time_now;
  RCLCPP_INFO(rclcpp::get_logger("INFO:planner_manager"), "Global trajectory generated.");

  updateTrajInfo();

  return true;
}

bool FastPlannerManager::topoReplan(bool collide) {
  rclcpp::Time t1, t2;

  /* truncate a new local segment for replanning */
  rclcpp::Time time_now = rclcpp::Clock(RCL_ROS_TIME).now();
  double t_now = (time_now - global_data_.global_start_time_).seconds();
  double local_traj_dt, local_traj_duration;
  double time_inc = 0.0;

  Eigen::MatrixXd ctrl_pts = reparamLocalTraj(t_now, local_traj_dt, local_traj_duration);
  NonUniformBspline init_traj(ctrl_pts, 3, local_traj_dt);
  local_data_.start_time_ = time_now;

  if (!collide) {  // simply truncate the segment and do nothing
    refineTraj(init_traj, time_inc);
    local_data_.position_traj_ = init_traj;
    global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + time_inc + t_now, time_inc);

  } else {
    plan_data_.initial_local_segment_ = init_traj;
    std::vector<Eigen::Vector3d> colli_start, colli_end, start_pts, end_pts;
    findCollisionRange(colli_start, colli_end, start_pts, end_pts);

    if (colli_start.size() == 1 && colli_end.size() == 0) {
      RCLCPP_WARN(rclcpp::get_logger("WARN: planner_manager"), "Init traj ends in obstacle, no replanning.");
      local_data_.position_traj_ = init_traj;
      global_data_.setLocalTraj(init_traj, t_now, local_traj_duration + t_now, 0.0);

    } else {
      NonUniformBspline best_traj;

      // local segment is in collision, call topological replanning
      /* search topological distinctive paths */
      RCLCPP_INFO(rclcpp::get_logger("INFO:planner_manager"), "[Topo]: ---------");
      plan_data_.clearTopoPaths();
      list<GraphNode::Ptr> graph;
      std::vector<std::vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;
      topo_prm_->findTopoPaths(colli_start.front(), colli_end.back(), start_pts, end_pts, graph, raw_paths, filtered_paths, select_paths);

      if (select_paths.size() == 0) {
        RCLCPP_WARN(rclcpp::get_logger("WARN: planner_manager"), "No path.");
        return false;
      }
      plan_data_.addTopoPaths(graph, raw_paths, filtered_paths, select_paths);

      /* optimize trajectory using different topo paths */
      RCLCPP_INFO(rclcpp::get_logger("INFO:planner_manager"), "[Optimize]: ---------");
      t1 = rclcpp::Clock(RCL_ROS_TIME).now();

      plan_data_.topo_traj_pos1_.resize(select_paths.size());
      plan_data_.topo_traj_pos2_.resize(select_paths.size());
      std::vector<std::thread> optimize_threads;
      for (int i = 0; i < select_paths.size(); ++i) {
        optimize_threads.emplace_back(&FastPlannerManager::optimizeTopoBspline, this, t_now,
                                      local_traj_duration, select_paths[i], i);
        // optimizeTopoBspline(t_now, local_traj_duration,
        // select_paths[i], origin_len, i);
      }
      for (int i = 0; i < select_paths.size(); ++i) optimize_threads[i].join();

      double t_opt = (rclcpp::Clock(RCL_ROS_TIME).now() - t1).seconds();
      std::cout << "[planner]: optimization time: " << t_opt << std::endl;
      selectBestTraj(best_traj);
      refineTraj(best_traj, time_inc);

      local_data_.position_traj_ = best_traj;
      global_data_.setLocalTraj(local_data_.position_traj_, t_now,
                                local_traj_duration + time_inc + t_now, time_inc);
    }
  }
  updateTrajInfo();
  return true;
}

void FastPlannerManager::selectBestTraj(NonUniformBspline& traj) {
  vector<NonUniformBspline>& trajs = plan_data_.topo_traj_pos2_;
  std::sort(trajs.begin(), trajs.end(),
       [&](NonUniformBspline& tj1, NonUniformBspline& tj2) { return tj1.getJerk() < tj2.getJerk(); });
  traj = trajs[0];
}


void FastPlannerManager::refineTraj(NonUniformBspline& best_traj, double& time_inc) {
  rclcpp::Time t1 = rclcpp::Clock(RCL_ROS_TIME).now();
  time_inc = 0.0;
  double dt, t_inc;
  const int max_iter = 1;

  // int cost_function = BsplineOptimizer::NORMAL_PHASE | BsplineOptimizer::VISIBILITY;
  Eigen::MatrixXd ctrl_pts = best_traj.getControlPoint();
  int cost_function = BsplineOptimizer::NORMAL_PHASE;

  best_traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  double ratio = best_traj.checkRatio();
  std::cout << "ratio: " << ratio << std::endl;
  reparamBspline(best_traj, ratio, ctrl_pts, dt, t_inc);
  time_inc += t_inc;

  ctrl_pts = bspline_optimizers_[0]->BsplineOptimizeTraj(ctrl_pts, dt, cost_function, 1, 1);
  best_traj = NonUniformBspline(ctrl_pts, 3, dt);
  RCLCPP_WARN_STREAM(rclcpp::get_logger("WARN: planner_manager"), "[Refine]: cost " << (rclcpp::Clock(RCL_ROS_TIME).now() - t1).seconds()
                                    << " seconds, time change is: " << time_inc);
}

void FastPlannerManager::updateTrajInfo() {

  local_data_.velocity_traj_     = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_         = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_          = local_data_.position_traj_.getTimeSum();
  local_data_.traj_id_ += 1;
}

void FastPlannerManager::reparamBspline(NonUniformBspline& bspline, double ratio,
                                        Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc) {
  int prev_num = bspline.getControlPoint().rows();
  double time_origin = bspline.getTimeSum();
  int seg_num = bspline.getControlPoint().rows() - 3;
  // double length = bspline.getLength(0.1);
  // int seg_num = ceil(length / pp_.ctrl_pt_dist);

  ratio = std::min(1.01, ratio);
  bspline.lengthenTime(ratio);
  double duration = bspline.getTimeSum();
  dt = duration / double(seg_num);
  time_inc = duration - time_origin;

  std::vector<Eigen::Vector3d> point_set;
  for (double time = 0.0; time <= duration + 1e-4; time += dt) {
    point_set.push_back(bspline.evaluateDeBoorT(time));
  }
  NonUniformBspline::parameterizeToBspline(dt, point_set, plan_data_.local_start_end_derivative_,
                                           ctrl_pts);
  // RCLCPP_WARN(rclcpp::get_logger("WARN: planner_manager"), "prev: %d, new: %d", prev_num, ctrl_pts.rows());
}


void FastPlannerManager::optimizeTopoBspline(double start_t, double duration,
                                             std::vector<Eigen::Vector3d> guide_path, int traj_id) {
  rclcpp::Time t1 = rclcpp::Clock(RCL_ROS_TIME).now();
  double tm1, tm2, tm3;

  t1 = rclcpp::Clock(RCL_ROS_TIME).now();

  // Parameterize B-spline according to the length of guide path
  int seg_num = topo_prm_->pathLength(guide_path) / pp_.ctrl_pt_dist;
  Eigen::MatrixXd ctrl_pts;
  double dt;

  ctrl_pts = reparamLocalTraj(start_t, duration, seg_num, dt);
  // RCLCPP_INFO(this->get_logger(), "ctrl pt num: %d", ctrl_pts.rows());

  // Discretize the guide path and align it with B-spline control points
  std::vector<Eigen::Vector3d> guide_pt;
  guide_pt = topo_prm_->pathToGuidePts(guide_path, static_cast<int>(ctrl_pts.rows()) - 2);

  guide_pt.pop_back();
  guide_pt.pop_back();
  guide_pt.erase(guide_pt.begin(), guide_pt.begin() + 2);

  // RCLCPP_INFO(rclcpp::get_logger("INFO:planner_manager"), "guide pt num: %d", guide_pt.size());
  if (static_cast<int>(guide_pt.size()) != static_cast<int>(ctrl_pts.rows()) - 6)
    RCLCPP_WARN(rclcpp::get_logger("WARN: planner_manager"), "what guide");

  tm1 = (rclcpp::Clock(RCL_ROS_TIME).now() - t1).seconds();
  t1  = rclcpp::Clock(RCL_ROS_TIME).now();

  // First phase, path-guided optimization

  bspline_optimizers_[traj_id]->setGuidePath(guide_pt);
  Eigen::MatrixXd opt_ctrl_pts1 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      ctrl_pts, dt, BsplineOptimizer::GUIDE_PHASE, 0, 1);

  plan_data_.topo_traj_pos1_[traj_id] = NonUniformBspline(opt_ctrl_pts1, 3, dt);

  tm2 = (rclcpp::Clock(RCL_ROS_TIME).now() - t1).seconds();
  t1  = rclcpp::Clock(RCL_ROS_TIME).now();

  // Second phase, normal optimization

  Eigen::MatrixXd opt_ctrl_pts2 = bspline_optimizers_[traj_id]->BsplineOptimizeTraj(
      opt_ctrl_pts1, dt, BsplineOptimizer::NORMAL_PHASE, 1, 1);

  plan_data_.topo_traj_pos2_[traj_id] = NonUniformBspline(opt_ctrl_pts2, 3, dt);

  tm3 = (rclcpp::Clock(RCL_ROS_TIME).now() - t1).seconds();
  RCLCPP_INFO(rclcpp::get_logger("INFO:planner_manager"), "optimization %d cost %lf, %lf, %lf seconds.", traj_id, tm1, tm2, tm3);
}


Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(double start_t, double& dt, double& duration) {
  /* Get the sample points local traj within radius */

  std::vector<Eigen::Vector3d> point_set;
  std::vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByRadius(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist, point_set,
                               start_end_derivative, dt, duration);

  /* Parameterization of B-spline */

  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  plan_data_.local_start_end_derivative_ = start_end_derivative;
  // RCLCPP_INFO(rclcpp::get_logger("INFO:planner_manager"), "ctrl pts: %d", ctrl_pts.rows());

  return ctrl_pts;
}

Eigen::MatrixXd FastPlannerManager::reparamLocalTraj(double start_t, double duration, int seg_num,
                                                     double& dt) {
  std::vector<Eigen::Vector3d> point_set;
  std::vector<Eigen::Vector3d> start_end_derivative;

  global_data_.getTrajByDuration(start_t, duration, seg_num, point_set, start_end_derivative, dt);
  plan_data_.local_start_end_derivative_ = start_end_derivative;

  /* Parameterization of B-spline */
  Eigen::MatrixXd ctrl_pts;
  NonUniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  // RCLCPP_INFO(rclcpp::get_logger("INFO:planner_manager"), "ctrl pts: %d", ctrl_pts.rows());

  return ctrl_pts;
}
void FastPlannerManager::findCollisionRange(std::vector<Eigen::Vector3d>& colli_start,
                                            std::vector<Eigen::Vector3d>& colli_end,
                                            std::vector<Eigen::Vector3d>& start_pts,
                                            std::vector<Eigen::Vector3d>& end_pts) {
  bool               last_safe = true, safe;
  double             t_m, t_mp;
  NonUniformBspline* initial_traj = &plan_data_.initial_local_segment_;
  initial_traj->getTimeSpan(t_m, t_mp);

  /* find range of collision */
  double t_s = -1.0, t_e;
  for (double tc = t_m; tc <= t_mp + 1e-4; tc += 0.05) {

    Eigen::Vector3d ptc = initial_traj->evaluateDeBoor(tc);
    safe = edt_environment_->evaluateCoarseEDT(ptc, -1.0) < topo_prm_->clearance_ ? false : true;

    if (last_safe && !safe) {
      colli_start.push_back(initial_traj->evaluateDeBoor(tc - 0.05));
      if (t_s < 0.0) t_s = tc - 0.05;
    } else if (!last_safe && safe) {
      colli_end.push_back(ptc);
      t_e = tc;
    }

    last_safe = safe;
  }

  if (colli_start.empty()) return;

  if (colli_start.size() == 1 && colli_end.empty()) return;

  /* find start and end safe segment */
  double dt = initial_traj->getInterval();
  int    sn = std::ceil((t_s - t_m) / dt);
  dt        = (t_s - t_m) / sn;

  for (double tc = t_m; tc <= t_s + 1e-4; tc += dt) {
    start_pts.push_back(initial_traj->evaluateDeBoor(tc));
  }

  dt = initial_traj->getInterval();
  sn = std::ceil((t_mp - t_e) / dt);
  dt = (t_mp - t_e) / sn;

  if (dt > 1e-4) {
    for (double tc = t_e; tc <= t_mp + 1e-4; tc += dt) {
      end_pts.push_back(initial_traj->evaluateDeBoor(tc));
    }
  } else {
    end_pts.push_back(initial_traj->evaluateDeBoor(t_mp));
  }
}

void FastPlannerManager::planYaw(const Eigen::Vector3d& start_yaw) {
  RCLCPP_INFO(rclcpp::get_logger("INFO:planner_manager"), "plan yaw");
  auto t1 = rclcpp::Clock().now();
  // calculate waypoints of heading

  auto&  pos      = local_data_.position_traj_;
  double duration = pos.getTimeSum();

  double dt_yaw  = 0.3;
  int    seg_num = std::ceil(duration / dt_yaw);
  dt_yaw         = duration / seg_num;

  const double            forward_t = 2.0;
  double                  last_yaw  = start_yaw(0);
  std::vector<Eigen::Vector3d> waypts;
  std::vector<int>             waypt_idx;

  // seg_num -> seg_num - 1 points for constraint excluding the boundary states

  for (int i = 0; i < seg_num; ++i) {
    double          tc = i * dt_yaw;
    Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
    double          tf = std::min(duration, tc + forward_t);
    Eigen::Vector3d pf = pos.evaluateDeBoorT(tf);
    Eigen::Vector3d pd = pf - pc;

    Eigen::Vector3d waypt;
    if (pd.norm() > 1e-6) {
      waypt(0) = atan2(pd(1), pd(0));
      waypt(1) = waypt(2) = 0.0;
      calcNextYaw(last_yaw, waypt(0));
    } else {
      waypt = waypts.back();
    }
    waypts.push_back(waypt);
    waypt_idx.push_back(i);
  }

  // calculate initial control points with boundary state constraints

  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw, 1.0,
      dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;
  yaw.block(0, 0, 3, 1) = states2pts * start_yaw;

  Eigen::Vector3d end_v = local_data_.velocity_traj_.evaluateDeBoorT(duration - 0.1);
  Eigen::Vector3d end_yaw(atan2(end_v(1), end_v(0)), 0, 0);
  calcNextYaw(last_yaw, end_yaw(0));
  yaw.block(seg_num, 0, 3, 1) = states2pts * end_yaw;

  // solve
  bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
  int cost_func = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::WAYPOINTS;
  yaw           = bspline_optimizers_[1]->BsplineOptimizeTraj(yaw, dt_yaw, cost_func, 1, 1);

  // update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_    = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  std::vector<double> path_yaw;
  for (int i = 0; i < waypts.size(); ++i) path_yaw.push_back(waypts[i][0]);
  plan_data_.path_yaw_    = path_yaw;
  plan_data_.dt_yaw_      = dt_yaw;
  plan_data_.dt_yaw_path_ = dt_yaw;

  std::cout << "plan heading: " << (rclcpp::Clock().now() - t1).seconds() << std::endl;
}


void FastPlannerManager::calcNextYaw(const double& last_yaw, double& yaw) {
  // round yaw to [-PI, PI]

  double round_last = last_yaw;

  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;

  if (std::fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}


}  // namespace fast_planner
