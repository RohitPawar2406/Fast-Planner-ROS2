#ifndef _TOPO_REPLAN_FSM_H_
#define _TOPO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.hpp>
#include <plan_env/obj_predictor.hpp>
#include <plan_env/sdf_map.h>
#include "quadrotor_msgs/msg/bspline.hpp"
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.hpp>

using std::vector;

namespace fast_planner {

class TopoReplanFSM {
private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* planning utils */
  std::shared_ptr<FastPlannerManager> planner_manager_;
  std::shared_ptr<PlanningVisualization> visualization_;

  /* parameters */
  int target_type_;  // 1 manual select, 2 hard code
  double replan_distance_threshold_, replan_time_threshold_;
  double waypoints_[50][3];
  int waypoint_num_;
  bool act_map_;

  /* planning data */
  bool trigger_, have_target_, have_odom_, collide_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d target_point_, end_vel_;                        // target state
  int current_wp_;

  /* ROS2 utils */
  rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_, new_pub_;
  rclcpp::Publisher<quadrotor_msgs::msg::Bspline>::SharedPtr bspline_pub_;

  /* helper functions */
  bool callSearchAndOptimization();    // front-end and back-end method
  bool callTopologicalTraj(int step);  // topo path guided gradient-based
                                       // optimization; 1: new, 2: replan
  void changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call);
  void printFSMExecState();

  /* ROS2 functions */
  void execFSMCallback();
  void checkCollisionCallback();
  void waypointCallback(const nav_msgs::msg::Path::SharedPtr& msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr& msg);

public:
  TopoReplanFSM() {}
  ~TopoReplanFSM() {}

  void init(rclcpp::Node::SharedPtr &nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif
