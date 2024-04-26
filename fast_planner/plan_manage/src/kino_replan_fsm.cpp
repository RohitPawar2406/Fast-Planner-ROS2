#include "plan_manage/kino_replan_fsm.hpp"
#include <geometry_msgs/msg/Point.hpp>
#include <std_msgs/msg/Empty.hpp>

namespace fast_planner {

void KinoReplanFSM::init() {
    this->declare_parameter("fsm/flight_type", -1);
    this->declare_parameter("fsm/thresh_replan", -1.0);
    this->declare_parameter("fsm/thresh_no_replan", -1.0);
    this->declare_parameter("fsm/waypoint_num", -1);

    this->get_parameter("fsm/flight_type", target_type_);
    this->get_parameter("fsm/thresh_replan", replan_thresh_);
    this->get_parameter("fsm/thresh_no_replan", no_replan_thresh_);
    this->get_parameter("fsm/waypoint_num", waypoint_num_);

    waypoints_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++) {
        this->declare_parameter("fsm/waypoint" + std::to_string(i) + "_x", -1.0);
        this->declare_parameter("fsm/waypoint" + std::to_string(i) + "_y", -1.0);
        this->declare_parameter("fsm/waypoint" + std::to_string(i) + "_z", -1.0);

        this->get_parameter("fsm/waypoint" + std::to_string(i) + "_x", waypoints_[i][0]);
        this->get_parameter("fsm/waypoint" + std::to_string(i) + "_y", waypoints_[i][1]);
        this->get_parameter("fsm/waypoint" + std::to_string(i) + "_z", waypoints_[i][2]);
    }

    planner_manager_ = std::make_shared<FastPlannerManager>();
    planner_manager_->initPlanModules(this->shared_from_this());
    visualization_ = std::make_shared<PlanningVisualization>(this->shared_from_this());

    exec_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&KinoReplanFSM::execFSMCallback, this));
    safety_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&KinoReplanFSM::checkCollisionCallback, this));

    waypoint_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/waypoint_generator/waypoints", 1, std::bind(&KinoReplanFSM::waypointCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_world", 1, std::bind(&KinoReplanFSM::odometryCallback, this, std::placeholders::_1));

    replan_pub_ = this->create_publisher<std_msgs::msg::Empty>("/planning/replan", 10);
    new_pub_ = this->create_publisher<std_msgs::msg::Empty>("/planning/new", 10);
    bspline_pub_ = this->create_publisher<plan_manage::msg::Bspline>("/planning/bspline", 10);
}

void KinoReplanFSM::waypointCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  if (msg->poses[0].pose.position.z < -0.1) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Triggered!");
  trigger_ = true;

  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;

  } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0) = waypoints_[current_wp_][0];
    end_pt_(1) = waypoints_[current_wp_][1];
    end_pt_(2) = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;

  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void KinoReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call) {
  std::vector<std::string> state_str = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ"};
  int pre_s = static_cast<int>(exec_state_);
  exec_state_ = new_state;
  RCLCPP_INFO(this->get_logger(), "[%s]: from %s to %s", pos_call.c_str(), state_str[pre_s].c_str(), state_str[static_cast<int>(new_state)].c_str());
}

void KinoReplanFSM::printFSMExecState() {
  std::vector<std::string> state_str = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ"};
  RCLCPP_INFO(this->get_logger(), "[FSM]: state: %s", state_str[static_cast<int>(exec_state_)].c_str());
}

void KinoReplanFSM::execFSMCallback() {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) RCLCPP_INFO(this->get_logger(), "no odom.");
    if (!trigger_) RCLCPP_INFO(this->get_logger(), "wait for goal.");
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT:
      if (!have_odom_ || !trigger_) return;
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    case WAIT_TARGET:
      if (!have_target_) return;
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      break;
    case GEN_NEW_TRAJ:
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();
      start_yaw_(0) = atan2(odom_orient_.toRotationMatrix()(1, 0), odom_orient_.toRotationMatrix()(0, 0));
      start_yaw_(1) = start_yaw_(2) = 0.0;
      if (callKinodynamicReplan()) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    case EXEC_TRAJ:
      if (manageTrajectoryExecution()) {
        changeFSMExecState(WAIT_TARGET, "FSM");
      }
      break;
    case REPLAN_TRAJ:
      if (replanTrajectory()) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
  }
}

void KinoReplanFSM::checkCollisionCallback() {
  if (!have_target_) return;

  double dist = planner_manager_->evaluateTrajectorySafety(end_pt_, info->duration_);
  if (dist <= 0.3) {
    if (tryToFindSafeGoal()) {
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
      publishReplan();
    }
  }

  if (exec_state_ == EXEC_TRAJ && !planner_manager_->checkTrajectorySafety()) {
    RCLCPP_WARN(this->get_logger(), "current traj in collision.");
    changeFSMExecState(REPLAN_TRAJ, "SAFETY");
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {
  auto plan_success = planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);
  if (plan_success) {
    planner_manager_->planYaw(start_yaw_);
    publishTrajectory();
    return true;
  } else {
    RCLCPP_INFO(this->get_logger(), "generate new traj fail.");
    return false;
  }
}

}  // namespace fast_planner
