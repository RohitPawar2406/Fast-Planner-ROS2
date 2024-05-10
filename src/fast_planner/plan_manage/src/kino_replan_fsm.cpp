#include "plan_manage/kino_replan_fsm.h"
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/empty.hpp>

namespace fast_planner {

KinoReplanFSM::KinoReplanFSM() {
   std::cout << "KinoReplanFSM hit ! " << std::endl;
}

KinoReplanFSM::~KinoReplanFSM() {
  std::cout << "KinoReplanFSM ended ! " << std::endl;
}

void KinoReplanFSM::init(std::shared_ptr<FastPlanner> nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;

  nh->declare_parameter("fsm/flight_type", -1);
  nh->declare_parameter("fsm/thresh_replan", -1.0);
  nh->declare_parameter("fsm/thresh_no_replan", -1.0);
  nh->declare_parameter("fsm/waypoint_num", -1);

  nh->get_parameter("fsm/flight_type", target_type_);
  nh->get_parameter("fsm/thresh_replan", replan_thresh_);
  nh->get_parameter("fsm/thresh_no_replan", no_replan_thresh_);
  nh->get_parameter("fsm/waypoint_num", waypoint_num_);

  //waypoints_.resize(waypoint_num_);
  nh->declare_parameter("fsm/waypoint0_x", -1.0);
  nh->declare_parameter("fsm/waypoint0_y", -1.0);
  nh->declare_parameter("fsm/waypoint0_z", -1.0);
  nh->declare_parameter("fsm/waypoint1_x", -1.0);
  nh->declare_parameter("fsm/waypoint1_y", -1.0);
  nh->declare_parameter("fsm/waypoint1_z", -1.0);
  nh->declare_parameter("fsm/waypoint2_x", -1.0);
  nh->declare_parameter("fsm/waypoint2_y", -1.0);
  nh->declare_parameter("fsm/waypoint2_z", -1.0);


  //////////////////////// DOUBTS - NEXT 3 lines
  planner_manager_ = std::make_shared<FastPlannerManager>();
  planner_manager_->initPlanModules(nh);
  std::shared_ptr<PlanningVisualization> visualization_ = std::make_shared<PlanningVisualization>(nh);

  exec_timer_ = nh->create_wall_timer(std::chrono::milliseconds(10), std::bind(&KinoReplanFSM::execFSMCallback, this));
  safety_timer_ = nh->create_wall_timer(std::chrono::milliseconds(50), std::bind(&KinoReplanFSM::checkCollisionCallback, this));

  waypoint_sub_ = nh->create_subscription<nav_msgs::msg::Path>(
      "/waypoint_generator/waypoints", 1, std::bind(&KinoReplanFSM::waypointCallback, this, std::placeholders::_1));
  odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_world", 1, std::bind(&KinoReplanFSM::odometryCallback, this, std::placeholders::_1));

  replan_pub_ = nh->create_publisher<std_msgs::msg::Empty>("/planning/replan", 10);
  new_pub_ = nh->create_publisher<std_msgs::msg::Empty>("/planning/new", 10);
  bspline_pub_ = nh->create_publisher<quadrotor_msgs::msg::Bspline>("/planning/bspline", 10);
}

void KinoReplanFSM::waypointCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  if (msg->poses[0].pose.position.z < -0.1) {
    return;
  }

  RCUTILS_LOG_INFO("Triggered!");
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
  // RCLCPP_INFO(this->get_logger(), "[%s]: from %s to %s", pos_call.c_str(), state_str[pre_s].c_str(), state_str[static_cast<int>(new_state)].c_str());
  RCUTILS_LOG_INFO("[%s]: from %s to %s", pos_call.c_str(), state_str[pre_s].c_str(), state_str[static_cast<int>(new_state)].c_str());
}

void KinoReplanFSM::printFSMExecState() {
  std::vector<std::string> state_str = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ"};
  //RCLCPP_INFO(this->get_logger(), "[FSM]: state: %s", state_str[static_cast<int>(exec_state_)].c_str());
  RCUTILS_LOG_INFO("[FSM]: state: %s", state_str[static_cast<int>(exec_state_)].c_str());
}

void KinoReplanFSM::execFSMCallback() {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) RCUTILS_LOG_INFO("no odom");
    if (!trigger_) RCUTILS_LOG_INFO("wait for goal");
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT:{
      if (!have_odom_ || !trigger_) return;
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;}

    case WAIT_TARGET:{
      if (!have_target_) return;
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      break;}

    case GEN_NEW_TRAJ:{
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
      break;}
      case EXEC_TRAJ: {
            /* determine if need to replan */
            LocalTrajData* info     = &planner_manager_->local_data_;
            rclcpp::Time time_now = rclcpp::Clock().now();
            double         t_cur    = (time_now - info->start_time_).seconds();
            t_cur                   = min(info->duration_, t_cur);

            Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

            /* && (end_pt_ - pos).norm() < 0.5 */
            if (t_cur > info->duration_ - 1e-2) {
              have_target_ = false;
              changeFSMExecState(WAIT_TARGET, "FSM");
              return;

            } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
              // cout << "near end" << endl;
              return;

            } else if ((info->start_pos_ - pos).norm() < replan_thresh_) {
              // cout << "near start" << endl;
              return;

            } else {
              changeFSMExecState(REPLAN_TRAJ, "FSM");
            }
            break;
          }
    case REPLAN_TRAJ:{
      LocalTrajData* info     = &planner_manager_->local_data_;
      rclcpp::Time time_now = rclcpp::Clock().now();
      double         t_cur    = (time_now - info->start_time_).seconds();

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      std_msgs::msg::Empty replan_msg;
      replan_pub_->publish(replan_msg);

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void KinoReplanFSM::checkCollisionCallback() {
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool            new_goal = false;
      const double    dr = 0.5, dtheta = 30, dz = 0.3;
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        cout << "change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");

        std_msgs::msg::Empty emt;
        replan_pub_->publish(emt);
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      RCUTILS_LOG_WARN("current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {
  auto plan_success = planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);
  if (plan_success) {

    planner_manager_->planYaw(start_yaw_);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    quadrotor_msgs::msg::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();

    bspline_pub_->publish(bspline);

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(1, 1, 0, 0.4));
    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(1.0,0, 0.0, 1), true, 0.2, Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

}  // namespace fast_planner
