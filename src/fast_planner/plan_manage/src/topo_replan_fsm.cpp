#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "quadrotor_msgs/msg/bspline.hpp"

#include "plan_manage/topo_replan_fsm.h"

using namespace std;
using namespace Eigen;

namespace fast_planner {

void TopoReplanFSM::init(rclcpp::Node::SharedPtr &nh) {
    this->declare_parameter<int>("fsm/flight_type", -1);
    this->declare_parameter<double>("fsm/thresh_replan", -1.0);
    this->declare_parameter<double>("fsm/thresh_no_replan", -1.0);
    this->declare_parameter<int>("fsm/waypoint_num", -1);
    this->declare_parameter<bool>("fsm/act_map", false);

    this->get_parameter("fsm/flight_type", target_type_);
    this->get_parameter("fsm/thresh_replan", replan_time_threshold_);
    this->get_parameter("fsm/thresh_no_replan", replan_distance_threshold_);
    this->get_parameter("fsm/waypoint_num", waypoint_num_);
    this->get_parameter("fsm/act_map", act_map_);

    waypoints_.resize(waypoint_num_, Vector3d());
    for (int i = 0; i < waypoint_num_; i++) {
        this->declare_parameter<double>("fsm/waypoint" + to_string(i) + "_x", -1.0);
        this->declare_parameter<double>("fsm/waypoint" + to_string(i) + "_y", -1.0);
        this->declare_parameter<double>("fsm/waypoint" + to_string(i) + "_z", -1.0);

        this->get_parameter("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0]);
        this->get_parameter("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1]);
        this->get_parameter("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2]);
    }

std::shared_ptr<FastPlannerManager> planner_manager_ = make_shared<FastPlannerManager>();

std::shared_ptr<PlanningVisualization> visualization_ = make_shared<PlanningVisualization>(this->shared_from_this());

    exec_timer_ = this->create_wall_timer(10ms, std::bind(&TopoReplanFSM::execFSMCallback, this));
    safety_timer_ = this->create_wall_timer(50ms, std::bind(&TopoReplanFSM::checkCollisionCallback, this));

    waypoint_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/waypoint_generator/waypoints", 1, std::bind(&TopoReplanFSM::waypointCallback, this, placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_world", 1, std::bind(&TopoReplanFSM::odometryCallback, this, placeholders::_1));

    replan_pub_ = this->create_publisher<std_msgs::msg::Empty>("/planning/replan", 20);
    new_pub_ = this->create_publisher<std_msgs::msg::Empty>("/planning/new", 20);
    bspline_pub_ = this->create_publisher<quadrotor_msgs::msg::Bspline>("/planning/bspline", 20);
}

void TopoReplanFSM::waypointCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses[0].pose.position.z < -0.1) return;
    RCLCPP_INFO(this->get_logger(), "Triggered!");

    vector<Vector3d> global_wp;
    if (target_type_ == TARGET_TYPE::REFENCE_PATH) {
        for (int i = 0; i < waypoint_num_; ++i) {
            Vector3d pt(waypoints_[i][0], waypoints_[i][1], waypoints_[i][2]);
            global_wp.push_back(pt);
        }
    } else {
        target_point_(0) = msg->poses[0].pose.position.x;
        target_point_(1) = msg->poses[0].pose.position.y;
        target_point_(2) = (target_type_ == TARGET_TYPE::MANUAL_TARGET) ? 1.0 : waypoints_[current_wp_][2];
        current_wp_ = (current_wp_ + 1) % waypoint_num_;

        RCLCPP_INFO_STREAM(this->get_logger(), "Target Point: " << target_point_.transpose());
        global_wp.push_back(target_point_);
        visualization_->drawGoal(target_point_, 0.3, Vector4d(1, 0, 0, 1.0));
    }

    planner_manager_->setGlobalWaypoints(global_wp);
    end_vel_.setZero();
    have_target_ = true;
    trigger_ = true;

    if (exec_state_ == WAIT_TARGET) {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
}

void TopoReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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

void TopoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, std::string pos_call) {
    static const vector<std::string> state_str = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "REPLAN_NEW"};
    int pre_s = static_cast<int>(exec_state_);
    exec_state_ = new_state;
    RCLCPP_INFO(this->get_logger(), "[%s]: from %s to %s", pos_call.c_str(), state_str[pre_s].c_str(), state_str[static_cast<int>(new_state)].c_str());
}

void TopoReplanFSM::printFSMExecState() {
    static const vector<std::string> state_str = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "REPLAN_NEW"};
    RCLCPP_INFO(this->get_logger(), "State: %s", state_str[static_cast<int>(exec_state_)].c_str());
}

void TopoReplanFSM::execFSMCallback() {
        static int fsm_num = 0;
        fsm_num++;
        if (fsm_num == 100) {
            printFSMExecState();
            if (!have_odom_) RCLCPP_INFO(this->get_logger(), "no odom.");
            if (!trigger_) RCLCPP_INFO(this->get_logger(), "no trigger.");
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

                publishEmptyMessage(new_pub_);
                if (callTopologicalTraj(1)) {
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
                    RCLCPP_WARN(this->get_logger(), "Replan fail, retrying...");
                }
                break;
            case REPLAN_NEW:
                if (initializeNewPlan()) {
                    changeFSMExecState(EXEC_TRAJ, "FSM");
                } else {
                    changeFSMExecState(GEN_NEW_TRAJ, "FSM");
                }
                break;
        }
    }


void TopoReplanFSM::checkCollisionCallback() {
  auto info = &planner_manager_->local_data_;

  /* ---------- check goal safety ---------- */
  // if (have_target_)
  if (false) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(target_point_, /* time to program start */ info->duration_) :
        edt_env->evaluateCoarseEDT(target_point_, -1.0);

    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool new_goal = false;
      const double dr = 0.5, dtheta = 30, dz = 0.3;

      double new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = target_point_(0) + r * cos(theta / 57.3);
            new_y = target_point_(1) + r * sin(theta / 57.3);
            new_z = target_point_(2) + nz;
            Eigen::Vector3d new_pt(new_x, new_y, new_z);

            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset target_point_ */
              goal(0) = new_x;
              goal(1) = new_y;
              goal(2) = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        RCLCPP_INFO(get_logger(), "change goal, replan.");
        target_point_ = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_NEW, "SAFETY");
        }

        visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        RCLCPP_INFO(get_logger(), "goal near collision, keep retry");
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ) {
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      if (dist > 0.5) {
        RCLCPP_WARN(get_logger(), "current traj %lf m to collision", dist);
        collide_ = true;
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
      } else {
        RCLCPP_ERROR(get_logger(), "current traj %lf m to collision, emergency stop!", dist);
        replan_pub_->publish(std_msgs::msg::Empty());
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "SAFETY");
      }
    } else {
      collide_ = false;
    }
  }
}

bool TopoReplanFSM::callSearchAndOptimization() {}

bool TopoReplanFSM::callTopologicalTraj(int step) {
  bool plan_success;

  if (step == 1) {
    plan_success = planner_manager_->planGlobalTraj(start_pt_);
  } else {
    plan_success = planner_manager_->topoReplan(collide_);
  }

  if (plan_success) {

    planner_manager_->planYaw(start_yaw_);

    auto locdat = &planner_manager_->local_data_;

    /* publish newest trajectory to server */

    /* publish traj */
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = locdat->start_time_;
    bspline.traj_id    = locdat->traj_id_;

    Eigen::MatrixXd pos_pts = locdat->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = locdat->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = locdat->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = locdat->yaw_traj_.getInterval();

    bspline_pub_->publish(bspline);

    /* visualize new trajectories */

    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawPolynomialTraj(planner_manager_->global_data_.global_traj_, 0.05,
                                       Eigen::Vector4d(0, 0, 0, 1), 0);
    visualization_->drawBspline(locdat->position_traj_, 0.08, Eigen::Vector4d(1.0, 0.0, 0.0, 1), false,
                                0.15, Eigen::Vector4d(1.0, 1.0, 1.0, 1), 99, 99);
    visualization_->drawBsplinesPhase2(plan_data->topo_traj_pos2_, 0.075);
    visualization_->drawYawTraj(locdat->position_traj_, locdat->yaw_traj_, plan_data->dt_yaw_);

    return true;
  } else {
    return false;
  }
}
}