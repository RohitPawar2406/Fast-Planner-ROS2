//#if

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

nav_msgs::msg::Path point()
{
    double h = 1.0;
    double scale = 7.0;
    nav_msgs::msg::Path waypoints;
    geometry_msgs::msg::PoseStamped pt;
    pt.pose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());

    pt.pose.position.y = scale * 0.0;
    pt.pose.position.x = scale * 2.0;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);

    // Add more waypoints...
    pt.pose.position.y =  scale * 0.0;
    pt.pose.position.x =  scale * 4.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y =  scale * 0.25;
    pt.pose.position.x =  scale * 5.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y =  scale * 0.5;
    pt.pose.position.x =  scale * 5.3;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y =  scale * 0.75;
    pt.pose.position.x =  scale * 5.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt); 

    pt.pose.position.y =  scale * 1.0;
    pt.pose.position.x =  scale * 4.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y =  scale * 1.0;
    pt.pose.position.x =  scale * 2.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y =  scale * 1.0;
    pt.pose.position.x =  scale * 0.0;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);     
    return waypoints;
}

nav_msgs::msg::Path circle()
{
    double h = 1.0;
    double scale = 5.0;
    nav_msgs::msg::Path waypoints;
    geometry_msgs::msg::PoseStamped pt;
    pt.pose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());

    pt.pose.position.y = -1.2 * scale;
    pt.pose.position.x = 2.5 * scale;
    pt.pose.position.z = h;
    waypoints.poses.push_back(pt);

    // Add more waypoints...
    pt.pose.position.y = -2.4 * scale;
    pt.pose.position.x =  5.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      
    pt.pose.position.y =  0.0 * scale;
    pt.pose.position.x =  5.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = -1.2 * scale;
    pt.pose.position.x =  2.5 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y = -2.4 * scale;
    pt.pose.position.x =  0. * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    pt.pose.position.y =  0.0 * scale;
    pt.pose.position.x =  0.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);       

    pt.pose.position.y = -1.2 * scale;
    pt.pose.position.x =  2.5 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y = -2.4 * scale;
    pt.pose.position.x =  5.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      
    pt.pose.position.y =  0.0 * scale;
    pt.pose.position.x =  5.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    
    pt.pose.position.y = -1.2 * scale;
    pt.pose.position.x =  2.5 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);      

    pt.pose.position.y = -2.4 * scale;
    pt.pose.position.x =  0. * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);  
    pt.pose.position.y =  0.0 * scale;
    pt.pose.position.x =  0.0 * scale;
    pt.pose.position.z =  h;
    waypoints.poses.push_back(pt);     
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
    pt.pose.orientation = tf2::toMsg(tf2::Quaternion::getIdentity());
    for(int i=0; i< 1; ++i)
    {
        // First loop
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y = -r + offset_y;
        pt.pose.position.z =  h/2;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0 ;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y =  r ;
        pt.pose.position.z =  h/2;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*4 + offset_x * 4;
        pt.pose.position.y =  0 ;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);       
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y = -r ;
        pt.pose.position.z =  h/2;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0 ;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r + offset_x * 2;
        pt.pose.position.y =  r ;
        pt.pose.position.z =  h/2;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  0  + offset_x;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);
        // Second loop
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y = -r;
        pt.pose.position.z =  h / 2 * 3;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y =  r;
        pt.pose.position.z =  h / 2 * 3;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r*4 + offset_x * 4;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);       
        pt.pose.position.x =  r*3 + offset_x * 3;
        pt.pose.position.y = -r;
        pt.pose.position.z =  h / 2 * 3;
        waypoints.poses.push_back(pt);      
        pt.pose.position.x =  r*2 + offset_x * 2;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  r + offset_x;
        pt.pose.position.y =  r + offset_y;
        pt.pose.position.z =  h / 2 * 3;
        waypoints.poses.push_back(pt);  
        pt.pose.position.x =  0;
        pt.pose.position.y =  0;
        pt.pose.position.z =  h;
        waypoints.poses.push_back(pt);  
    }

    return waypoints;
}

//#endif