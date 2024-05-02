#ifndef _KINO_REPLAN_FSM_H_
#define _KINO_REPLAN_FSM_H_

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

class Test {
private:
  /* data */
  int test_;
  std::vector<int> test_vec_;
  rclcpp::Node::SharedPtr nh_;

public:
  Test(const int& v) {
    test_ = v;
  }
  Test(rclcpp::Node::SharedPtr node) {
    nh_ = node;
  }
  ~Test() {
  }
  void print() {
    std::cout << "test: " << test_ << std::endl;
  }
};

class KinoReplanFSM {

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* planning utils */
  FastPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  int target_type_;  // 1 mannual select, 2 hard code
  double no_replan_thresh_, replan_thresh_;
  double waypoints_[50][3];
  int waypoint_num_;

  /* planning data */
  bool trigger_, have_target_, have_odom_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d end_pt_, end_vel_;                              // target state
  int current_wp_;

  /* ROS utils */
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_, vis_timer_, test_something_timer_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  //rclcpp::Publisher replan_pub_, new_pub_, bspline_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr new_pub_ ;
  rclcpp::Publisher<quadrotor_msgs::msg::Bspline>::SharedPtr  bspline_pub_;

  /* helper functions */
  bool callKinodynamicReplan();        // front-end and back-end method
  bool callTopologicalTraj(int step);  // topo path guided gradient-based
                                       // optimization; 1: new, 2: replan
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();

  /* ROS functions */
  void execFSMCallback();
  void checkCollisionCallback();
  void waypointCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

public:
  KinoReplanFSM(/* args */) {
  }
  ~KinoReplanFSM() {
  }

  void init(rclcpp::Node::SharedPtr& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif
