#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include "sample_waypoints.hpp"
#include <vector>
#include <deque>
#include <boost/format.hpp>
//#include <Eigen/Dense>

using namespace std;
using bfmt = boost::format;

rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub1;
rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub2;
string waypoint_type = string("manual");
bool is_odom_ready;
nav_msgs::msg::Odometry odom;
nav_msgs::msg::Path waypoints;

// series waypoint needed
std::deque<nav_msgs::msg::Path> waypointSegments;
rclcpp::Time trigged_time;

void load_seg(const rclcpp::Node::SharedPtr& nh, int segid, const rclcpp::Time& time_base) {
    std::string seg_str = boost::str(bfmt("seg%d/") % segid);
    double yaw;
    double time_from_start;
    RCLCPP_INFO(nh->get_logger(), "Getting segment %d", segid);
    RCLCPP_ASSERT(nh->get_parameter(seg_str + "yaw", yaw));
    RCLCPP_ASSERT_MSG((yaw > -3.1499999) && (yaw < 3.14999999), "yaw=%.3f", yaw);
    RCLCPP_ASSERT(nh->get_parameter(seg_str + "time_from_start", time_from_start));
    RCLCPP_ASSERT(time_from_start >= 0.0);

    std::vector<double> ptx;
    std::vector<double> pty;
    std::vector<double> ptz;

    RCLCPP_ASSERT(nh->get_parameter(seg_str + "x", ptx));
    RCLCPP_ASSERT(nh->get_parameter(seg_str + "y", pty));
    RCLCPP_ASSERT(nh->get_parameter(seg_str + "z", ptz));

    RCLCPP_ASSERT(ptx.size());
    RCLCPP_ASSERT(ptx.size() == pty.size() && ptx.size() == ptz.size());

    nav_msgs::msg::Path path_msg;

    path_msg.header.stamp = time_base + rclcpp::Duration::from_seconds(time_from_start);

    double baseyaw = tf2::getYaw(odom.pose.pose.orientation);
    
    for (size_t k = 0; k < ptx.size(); ++k) {
        geometry_msgs::msg::PoseStamped pt;
        pt.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, baseyaw + yaw));
        double dp_x = ptx.at(k);
        double dp_y = pty.at(k);
        double rdp_x = std::cos(-baseyaw - yaw) * dp_x + std::sin(-baseyaw - yaw) * dp_y;
        double rdp_y = -std::sin(-baseyaw - yaw) * dp_x + std::cos(-baseyaw - yaw) * dp_y;
        pt.pose.position.x = rdp_x + odom.pose.pose.position.x;
        pt.pose.position.y = rdp_y + odom.pose.pose.position.y;
        pt.pose.position.z = ptz.at(k) + odom.pose.pose.position.z;
        path_msg.poses.push_back(pt);
    }

    waypointSegments.push_back(path_msg);
}


void load_waypoints(const rclcpp::Node::SharedPtr& nh, const rclcpp::Time& time_base) {
    int seg_cnt = 0;
    waypointSegments.clear();
    RCLCPP_ASSERT(nh->get_parameter("segment_cnt", seg_cnt));
    for (int i = 0; i < seg_cnt; ++i) {
        load_seg(nh, i, time_base);
        if (i > 0) {
            RCLCPP_ASSERT(waypointSegments[i - 1].header.stamp < waypointSegments[i].header.stamp);
        }
    }
    RCLCPP_INFO(nh->get_logger(), "Overall load %zu segments", waypointSegments.size());
}

void publish_waypoints() {
    waypoints.header.frame_id = std::string("world");
    waypoints.header.stamp = rclcpp::Clock().now();
    pub1->publish(waypoints);
    geometry_msgs::msg::PoseStamped init_pose;
    init_pose.header = odom.header;
    init_pose.pose = odom.pose.pose;
    waypoints.poses.insert(waypoints.poses.begin(), init_pose);
    waypoints.poses.clear();
}

void publish_waypoints_vis() {
    nav_msgs::msg::Path wp_vis = waypoints;
    geometry_msgs::msg::PoseArray poseArray;
    poseArray.header.frame_id = std::string("world");
    poseArray.header.stamp = rclcpp::Clock().now();

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

    if (waypointSegments.size()) {
        rclcpp::Time expected_time = waypointSegments.front().header.stamp;
        if (odom.header.stamp >= expected_time) {
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
            RCLCPP_INFO(nh->get_logger(), ss.str());

            publish_waypoints_vis();
            publish_waypoints();

            waypointSegments.pop_front();
        }
    }
}

void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    trigged_time = rclcpp::Clock().now();
    rclcpp::Node::SharedPtr n = std::make_shared<rclcpp::Node>("waypoint_generator");
    n->get_parameter("waypoint_type", waypoint_type);
    
    if (waypoint_type == string("circle")) {
        waypoints = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("eight")) {
        waypoints = eight();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("point")) {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("series")) {
        load_waypoints(n, trigged_time);
    } else if (waypoint_type == string("manual-lonely-waypoint")) {
        if (msg->pose.position.z > -0.1) {
            geometry_msgs::msg::PoseStamped pt = *msg;
            waypoints.poses.clear();
            waypoints.poses.push_back(pt);
            publish_waypoints_vis();
            publish_waypoints();
        } else {
            RCLCPP_WARN(n->get_logger(), "invalid goal in manual-lonely-waypoint mode.");
        }
    } else {
        if (msg->pose.position.z > 0) {
            geometry_msgs::msg::PoseStamped pt = *msg;
            if (waypoint_type == string("noyaw")) {
                double yaw = tf2::getYaw(odom.pose.pose.orientation);
                pt.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, yaw));
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

void traj_start_trigger_callback(const geometry_msgs::msg::PoseStamped msg) {
    if (!is_odom_ready) {
        RCLCPP_ERROR(nh->get_logger(), "No odom!");
        return;
    }

    RCLCPP_WARN(nh->get_logger(), "Trigger!");
    trigged_time = rclcpp::Clock().now();

    rclcpp::Node::SharedPtr n = std::make_shared<rclcpp::Node>("waypoint_generator");
    n->get_parameter("waypoint_type", waypoint_type);

    RCLCPP_ERROR_STREAM(n->get_logger(), "Pattern " << waypoint_type << " generated!");
    if (waypoint_type == string("free")) {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("circle")) {
        waypoints = circle();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("eight")) {
        waypoints = eight();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("point")) {
        waypoints = point();
        publish_waypoints_vis();
        publish_waypoints();
    } else if (waypoint_type == string("series")) {
        load_waypoints(n, trigged_time);
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nh = std::make_shared<rclcpp::Node>("waypoint_generator");
    nh->get_parameter("waypoint_type", waypoint_type);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1 =
        nh->create_subscription<nav_msgs::msg::Odometry>("odom", 10, odom_callback);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub2 =
        nh->create_subscription<geometry_msgs::msg::PoseStamped>("goal", 10, goal_callback);

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub3 =
        n->create_subscription<geometry_msgs::msg::PoseStamped>("traj_start_trigger", 10, traj_start_trigger_callback);

    pub1 = n->create_publisher<nav_msgs::msg::Path>("waypoints", 50);
    pub2 = n->create_publisher<geometry_msgs::msg::PoseArray>("waypoints_vis", 10);

    trigged_time = rclcpp::Time(0);

    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}
