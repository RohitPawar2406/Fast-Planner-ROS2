
#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/msg/odometry.hpp"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "quadrotor_msgs/msg/bspline.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

using NonUniformBspline = fast_planner::NonUniformBspline;


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TrajServer : public rclcpp::Node
{
  public:
    TrajServer()
    : Node("traj_server")
    {   
        cmd.kx[0] = pos_gain[0];
        cmd.kx[1] = pos_gain[1];
        cmd.kx[2] = pos_gain[2];

        cmd.kv[0] = vel_gain[0];
        cmd.kv[1] = vel_gain[1];
        cmd.kv[2] = vel_gain[2];
        bspline_sub = this->create_subscription<quadrotor_msgs::msg::Bspline>("planning/bspline", 10, std::bind(&TrajServer::bsplineCallback, this, std::placeholders::_1));
        replan_sub = this->create_subscription<std_msgs::msg::Empty>("planning/replan", 10, std::bind(&TrajServer::replanCallback, this, std::placeholders::_1));
        new_sub = this->create_subscription<std_msgs::msg::Empty>("planning/new", 10, std::bind(&TrajServer::newCallback,    this, std::placeholders::_1));
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom_world", 50, std::bind(&TrajServer::odomCallbck, this, std::placeholders::_1));

        cmd_vis_pub = this->create_publisher<visualization_msgs::msg::Marker>("planning/position_cmd_vis", 10);
        pos_cmd_pub = this->create_publisher<quadrotor_msgs::msg::PositionCommand>("/position_cmd", 50);
        traj_pub = this->create_publisher<visualization_msgs::msg::Marker>("planning/travel_traj", 10);

        cmd_timer = this->create_wall_timer(10ms, std::bind(&TrajServer::cmdCallback, this));
        vis_timer = this->create_wall_timer(250ms, std::bind(&TrajServer::visCallback, this));

    }

   void displayTrajWithColor(const std::vector<Eigen::Vector3d>& path, double resolution, const Eigen::Vector4d& color, int id) {
    auto mk = std::make_unique<visualization_msgs::msg::Marker>();
    mk->header.frame_id = "world";
    mk->header.stamp = this->now();
    mk->type = visualization_msgs::msg::Marker::SPHERE_LIST;
    mk->action = visualization_msgs::msg::Marker::DELETE;
    mk->id = id;

    traj_pub->publish(std::move(mk));

    mk = std::make_unique<visualization_msgs::msg::Marker>();
    mk->header.frame_id = "world";
    mk->header.stamp = this->now();
    mk->type = visualization_msgs::msg::Marker::SPHERE_LIST;
    mk->action = visualization_msgs::msg::Marker::ADD;
    mk->pose.orientation.x = 0.0;
    mk->pose.orientation.y = 0.0;
    mk->pose.orientation.z = 0.0;
    mk->pose.orientation.w = 1.0;

    mk->color.r = color(0);
    mk->color.g = color(1);
    mk->color.b = color(2);
    mk->color.a = color(3);

    mk->scale.x = resolution;
    mk->scale.y = resolution;
    mk->scale.z = resolution;

    for (const auto& point : path) {
        geometry_msgs::msg::Point pt;
        pt.x = point(0);
        pt.y = point(1);
        pt.z = point(2);
        mk->points.push_back(pt);
    }
    traj_pub->publish(std::move(mk));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

  void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id, const Eigen::Vector4d& color) 
  {
    auto mk_state = std::make_unique<visualization_msgs::msg::Marker>();
    mk_state->header.frame_id = "world";
    mk_state->header.stamp = this->now();
    mk_state->id = id;
    mk_state->type = visualization_msgs::msg::Marker::ARROW;
    mk_state->action = visualization_msgs::msg::Marker::ADD;

    mk_state->pose.orientation.w = 1.0;
    mk_state->scale.x = 0.1;
    mk_state->scale.y = 0.2;
    mk_state->scale.z = 0.3;

    geometry_msgs::msg::Point pt;
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);
    mk_state->points.push_back(pt);

    pt.x = pos(0) + vec(0);
    pt.y = pos(1) + vec(1);
    pt.z = pos(2) + vec(2);
    mk_state->points.push_back(pt);

    mk_state->color.r = color(0);
    mk_state->color.g = color(1);
    mk_state->color.b = color(2);
    mk_state->color.a = color(3);

    cmd_vis_pub->publish(std::move(mk_state));
}


private:
  void bsplineCallback(const quadrotor_msgs::msg::Bspline::SharedPtr msg) {
      Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
      Eigen::VectorXd knots(msg->knots.size());
      for (size_t i = 0; i < msg->knots.size(); ++i) {
          knots(i) = msg->knots[i];}

      for (size_t i = 0; i < msg->pos_pts.size(); ++i) {
          pos_pts(i, 0) = msg->pos_pts[i].x;
          pos_pts(i, 1) = msg->pos_pts[i].y;
          pos_pts(i, 2) = msg->pos_pts[i].z;
      }

      fast_planner::NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
      pos_traj.setKnot(knots);

      // parse yaw traj
      Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
      for (size_t i = 0; i < msg->yaw_pts.size(); ++i) {
          yaw_pts(i, 0) = msg->yaw_pts[i];
      }

      fast_planner::NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

      start_time_ = msg->start_time;
      traj_id_ = msg->traj_id;

      traj_.clear();
      traj_.push_back(pos_traj);
      traj_.push_back(traj_[0].getDerivative());
      traj_.push_back(traj_[1].getDerivative());
      traj_.push_back(yaw_traj);
      traj_.push_back(yaw_traj.getDerivative());

      traj_duration_ = traj_[0].getTimeSum();

      receive_traj_ = true;
    }

  void replanCallback(const std_msgs::msg::Empty::SharedPtr msg) {
      /* reset duration */
      const double time_out = 0.01;
      rclcpp::Time time_now = this->now();
      double t_stop = (time_now - start_time_).seconds() + time_out;
      traj_duration_ = std::min(t_stop, traj_duration_);
  }


  void newCallback(const std_msgs::msg::Empty::SharedPtr msg) {
  traj_cmd_.clear();
  traj_real_.clear();
  }


  void odomCallbck(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (msg->child_frame_id == "X" || msg->child_frame_id == "O") return;

    //odom = *msg;

    traj_real_.push_back(
        Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

    if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
  }

    void visCallback() {
        // Update the callback implementation
        displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
    }

  void cmdCallback() {
      /* no publishing before receive traj_ */
      //std::cout<<receive_traj_<<"****************"<<std::endl;
      if (!receive_traj_) return;

      rclcpp::Time time_now = this->now();
      double t_cur = (time_now - start_time_).seconds();

      Eigen::Vector3d pos, vel, acc, pos_f;
      double yaw, yawdot;
      //std::cout<<"############ "<<t_cur <<" ####### " <<traj_duration_ << " ############ "<<t_cur <<std::endl;
      if (t_cur < traj_duration_ && t_cur >= 0.0) {
        //std::cout<<"************** " <<receive_traj_<<"***************"<<endl;
        pos = traj_[0].evaluateDeBoorT(t_cur);
        vel = traj_[1].evaluateDeBoorT(t_cur);
        acc = traj_[2].evaluateDeBoorT(t_cur);
        yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
        yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
        // std::cout << "pos: [" << pos[0] << ", " << pos[1] << ", " << pos[2] << "]" << std::endl;
        // std::cout << "vel: [" << vel[0] << ", " << vel[1] << ", " << vel[2] << "]" << std::endl;
        double tf = std::min(traj_duration_, t_cur + 2.0);
        pos_f = traj_[0].evaluateDeBoorT(tf);

      } else if (t_cur >= traj_duration_) {
        //std::cout<<"999999999999900000 " <<receive_traj_<<" 9999999999999 "<<endl;
        /* hover when finish traj_ */
        pos = traj_[0].evaluateDeBoorT(traj_duration_);
        vel.setZero();
        acc.setZero();
        yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
        yawdot = traj_[4].evaluateDeBoorT(traj_duration_)[0];

        pos_f = pos;

      } else {
        std::cout << "[Traj server]: invalid time." << std::endl;
      }

      cmd.header.stamp = time_now;
      cmd.header.frame_id = "world";
      cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
      cmd.trajectory_id = traj_id_;

      cmd.position.x = pos(0);
      cmd.position.y = pos(1);
      cmd.position.z = pos(2);

      cmd.velocity.x = vel(0);
      cmd.velocity.y = vel(1);
      cmd.velocity.z = vel(2);

      cmd.acceleration.x = acc(0);
      cmd.acceleration.y = acc(1);
      cmd.acceleration.z = acc(2);

      cmd.yaw = yaw;
      cmd.yaw_dot = yawdot;

      auto pos_err = pos_f - pos;
      // if (pos_err.norm() > 1e-3) {
      //   cmd.yaw = atan2(pos_err(1), pos_err(0));
      // } else {
      //   cmd.yaw = last_yaw_;
      // }
      // cmd.yaw_dot = 1.0;

      last_yaw_ = cmd.yaw;
      pos_cmd_pub->publish(cmd);

      // draw cmd

      // drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
      // drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));

      Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
      drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
      // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));

      traj_cmd_.push_back(pos);
      if (traj_cmd_.size() > 10000) traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}


    rclcpp::Subscription<quadrotor_msgs::msg::Bspline>::SharedPtr bspline_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr replan_sub, new_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_vis_pub, traj_pub;
    rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;

    rclcpp::TimerBase::SharedPtr cmd_timer, vis_timer;
    rclcpp::Time start_time_;
    int traj_id_;
    double traj_duration_;
    bool receive_traj_ = false;
    vector<Eigen::Vector3d> traj_cmd_, traj_real_;
    double last_yaw_ = 0.0;
    vector<NonUniformBspline> traj_;
    quadrotor_msgs::msg::PositionCommand cmd;

    double pos_gain[3] = { 5.7, 5.7, 6.2 };
    double vel_gain[3] = { 3.4, 3.4, 4.0 };
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajServer>());
  rclcpp::shutdown();
  return 0;
}