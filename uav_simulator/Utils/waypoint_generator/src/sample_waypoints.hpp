#ifndef SAMPLE_WAYPOINTS_H
#define SAMPLE_WAYPOINTS_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

nav_msgs::msg::Path point()
{
    double h = 1.0;
    double scale = 7.0;
    nav_msgs::msg::Path waypoints;
    geometry_msgs::msg::PoseStamped pt;
    pt.pose.orientation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    pt.pose.position.y = scale * 0.0;
    pt.pose.position.x = scale * 2.0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);

    pt.pose.position.y = scale * 0.0;
    pt.pose.position.x = scale * 4.0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);

    pt.pose.position.y = scale * 0.25;
    pt.pose.position.x = scale * 5.0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);

    pt.pose.position.y = scale * 0.5;
    pt.pose.position.x = scale * 5.3;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);

    pt.pose.position.y = scale * 0.75;
    pt.pose.position.x = scale * 5.0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);

    pt.pose.position.y = scale * 1.0;
    pt.pose.position.x = scale * 4.0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);

    pt.pose.position.y = scale * 1.0;
    pt.pose.position.x = scale * 2.0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);

    pt.pose.position.y = scale * 1.0;
    pt.pose.position.x = scale * 0.0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);

    return waypoints;
}

nav_msgs::msg::Path circle()
{
    double h = 1.0;
    double scale = 5.0;
    nav_msgs::msg::Path waypoints;
    geometry_msgs::msg::PoseStamped pt;
    pt.pose.orientation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    pt.pose.position.y = -1.2 * scale;
    pt.pose.position.x = 2.5 * scale;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);

    // Add other points similarly...

    return waypoints;
}

nav_msgs::msg::Path eight()
{
    double offset_x = 0.0;
    double offset_y = 0.0;
    double r = 10.0;
    double h = 2.0;
    nav_msgs::msg::Path waypoints;
    geometry_msgs::msg::PoseStamped pt;
    pt.pose.orientation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    // Populate waypoints for figure 8 trajectory...

    return waypoints;
}

#endif
