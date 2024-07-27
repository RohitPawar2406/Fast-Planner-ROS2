#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include "sample_waypoints.h"
#include <vector>
#include <deque>
#include <boost/format.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include <typeinfo>

using namespace std;
using bfmt = boost::format;

class WaypointGenerator : public rclcpp::Node {
public:
    WaypointGenerator() : Node("waypoint_generator") {
        waypoint_type = "manual";
        is_odom_ready = false;
        trigged_time = this->now();
        auto odom_callback = [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->odom_callback(msg);
        };
        auto goal_callback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            this->goal_callback(msg);
        };
        auto traj_start_trigger_callback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            this->traj_start_trigger_callback(msg);
        };

        sub1 = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, odom_callback);
        sub2 = this->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10, goal_callback);
        sub3 = this->create_subscription<geometry_msgs::msg::PoseStamped>("traj_start_trigger", 10, traj_start_trigger_callback);
        pub1 = this->create_publisher<nav_msgs::msg::Path>("/waypoint_generator/waypoints", 10);
        pub2 = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoint_generator/waypoints_vis", 10);
        this->declare_parameter<string>("waypoint_type", "PLEASE_DECALRE");
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub2;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub3;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub1;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub2;
    string waypoint_type;
    bool is_odom_ready;
    nav_msgs::msg::Odometry odom;
    nav_msgs::msg::Path waypoints;
    std::deque<nav_msgs::msg::Path> waypointSegments;
    rclcpp::Time trigged_time;

void load_seg( int segid, const rclcpp::Time& time_base) {
    std::string seg_str = boost::str(bfmt("seg%d/") % segid);
    double yaw;
    double time_from_start;
    RCLCPP_INFO(this->get_logger(), "Getting segment %d", segid);
    this->get_parameter(seg_str + "yaw", yaw);
    RCLCPP_EXPORT(this->get_logger(), (yaw > -3.1499999) && (yaw < 3.14999999));
    this->get_parameter(seg_str + "time_from_start", time_from_start);
    RCLCPP_EXPORT(this->get_logger(), time_from_start >= 0.0);

    std::vector<double> ptx;
    std::vector<double> pty;
    std::vector<double> ptz;

    this->get_parameter(seg_str + "x", ptx);
    this->get_parameter(seg_str + "y", pty);
    this->get_parameter(seg_str + "z", ptz);

    RCLCPP_EXPORT(this->get_logger(), ptx.size());
    RCLCPP_EXPORT(this->get_logger(), ptx.size() == pty.size() && ptx.size() == ptz.size());

    nav_msgs::msg::Path path_msg;

    path_msg.header.stamp = time_base + rclcpp::Duration::from_seconds(time_from_start);

    //double baseyaw = tf2::getYaw(odom.pose.pose.orientation);
    tf2::Quaternion quat;
    tf2::fromMsg(odom.pose.pose.orientation, quat);
    double roll, pitch, baseyaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, baseyaw);
    
    for (size_t k = 0; k < ptx.size(); ++k) {
        geometry_msgs::msg::PoseStamped pt;
        pt.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), baseyaw + yaw));
        Eigen::Vector2d dp(ptx.at(k), pty.at(k));
        Eigen::Vector2d rdp;
        rdp.x() = std::cos(-baseyaw-yaw)*dp.x() + std::sin(-baseyaw-yaw)*dp.y();
        rdp.y() =-std::sin(-baseyaw-yaw)*dp.x() + std::cos(-baseyaw-yaw)*dp.y();
        pt.pose.position.x = rdp.x() + odom.pose.pose.position.x;
        pt.pose.position.y = rdp.y() + odom.pose.pose.position.y;
        pt.pose.position.z = ptz.at(k) + odom.pose.pose.position.z;
        path_msg.poses.push_back(pt);
    }

    waypointSegments.push_back(path_msg);
}

void load_waypoints(const rclcpp::Time& time_base) {
    int seg_cnt = 0;
    waypointSegments.clear();
    this->get_parameter("segment_cnt", seg_cnt);
    for (int i = 0; i < seg_cnt; ++i) {
        load_seg(i, time_base);
        if (i > 0) {
            //RCLCPP_EXPORT(this->get_logger(), waypointSegments[i - 1].header.stamp < waypointSegments[i].header.stamp);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Overall load %zu segments", waypointSegments.size());
}

void publish_waypoints() {
    waypoints.header.frame_id = "world";
    waypoints.header.stamp = this->now();
    pub1->publish(waypoints);
    geometry_msgs::msg::PoseStamped init_pose;
    init_pose.header = odom.header;
    init_pose.pose = odom.pose.pose;
    waypoints.poses.insert(waypoints.poses.begin(), init_pose);
    //pub2->publish(waypoints);
    waypoints.poses.clear();
}

void publish_waypoints_vis() {
    nav_msgs::msg::Path wp_vis = waypoints;
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.header.frame_id = "world";
    poseArray.header.stamp = this->now();

    {
        geometry_msgs::msg::Pose init_pose;
        init_pose = odom.pose.pose;
        poseArray.poses.push_back(init_pose);
    }

    for (auto it = waypoints.poses.begin(); it != waypoints.poses.end(); ++it) {
        geometry_msgs::msg::Pose p;
        p = it->pose;
        poseArray.poses.push_back(p);
    }
    pub2->publish(poseArray);
}

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    is_odom_ready = true;
    odom = *msg;

    // RCLCPP_INFO(this->get_logger(), "INSIDE ODOM CALLBACK -------------");
    if (waypointSegments.size()) {
        rclcpp::Time odom_time = this->get_clock()->now();
        rclcpp::Time expected_time = waypointSegments.front().header.stamp;
        //cout<<"--------- "<<typeid(odom.header.stamp).name()<<" ====== "<<expected_time;
        if (odom_time >= expected_time) {
            waypoints = waypointSegments.front();

            std::stringstream ss;
            ss << bfmt("Series send %.3f from start:\n") % trigged_time.seconds();
            for (auto& pose_stamped : waypoints.poses) {
                ss << bfmt("P[%.2f, %.2f, %.2f] q(%.2f,%.2f,%.2f,%.2f)") %
                            pose_stamped.pose.position.x % pose_stamped.pose.position.y %
                            pose_stamped.pose.position.z % pose_stamped.pose.orientation.w %
                            pose_stamped.pose.orientation.x % pose_stamped.pose.orientation.y %
                            pose_stamped.pose.orientation.z << std::endl;
            }
            RCLCPP_INFO(this->get_logger(), ss.str());

            publish_waypoints_vis();
            publish_waypoints();

            waypointSegments.pop_front();
        }
    }
}

void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

    trigged_time = this->now();
    //auto nh = this->get_node_parameters_interface();
    this->get_parameter("waypoint_type", waypoint_type);
    
    if (waypoint_type == "circle") {
        waypoints = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == "eight") {
        waypoints = eight();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == "point") {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == "series") {
        load_waypoints(trigged_time);
    } else if (waypoint_type == "manual_lonely_waypoint") {
        if (msg->pose.position.z > -0.1) {
            geometry_msgs::msg::PoseStamped pt = *msg;
            RCLCPP_WARN(this->get_logger(), "INSIDE11");
            waypoints.poses.clear();
            waypoints.poses.push_back(pt);
            publish_waypoints_vis();
            publish_waypoints();
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid goal in manual-lonely-waypoint mode.");
        }
    } else {
        if (msg->pose.position.z > 0) {
            geometry_msgs::msg::PoseStamped pt = *msg;
            if (waypoint_type == "noyaw") {
                tf2::Quaternion quat;
                tf2::fromMsg(odom.pose.pose.orientation, quat);
                double roll, pitch, yaw;
                tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

                //double yaw = tf2::getYaw(odom.pose.pose.orientation);
                pt.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
            }
            waypoints.poses.push_back(pt);
            publish_waypoints_vis();
        } else if (msg->pose.position.z > -1.0) {
            if (waypoints.poses.size() >= 1) {
                waypoints.poses.erase(std::prev(waypoints.poses.end()));
            }
            publish_waypoints_vis();
        } else {
            if (waypoints.poses.size() >= 1) {
                publish_waypoints_vis();
                publish_waypoints();
            }
        }
    }
}

void traj_start_trigger_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!is_odom_ready) {
        RCLCPP_ERROR(this->get_logger(), "No odom!");
        return;
    }

    RCLCPP_WARN(this->get_logger(), "Trigger!");
    trigged_time = this->now();
    //auto nh = this->get_node_parameters_interface();
    this->get_parameter("waypoint_type",waypoint_type);

    RCLCPP_ERROR(this->get_logger(), "Pattern %s generated!", waypoint_type.c_str());
    if (waypoint_type == "free") {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == "circle") {
        waypoints = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == "eight") {
        waypoints = eight();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == "point") {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == "series") {
        load_waypoints( trigged_time);
    }
}
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointGenerator>());
    rclcpp::shutdown();
    return 0;
}
