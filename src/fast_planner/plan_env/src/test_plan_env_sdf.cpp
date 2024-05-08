#include "plan_env/test_plan_env_sdf.hpp"

void Node2::Node2()
{
    subs = node_class_->create_subscription<std_msgs::msg::String>(
        "/topic_name", 10, messageCallback);
}

void Node2::messageCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received message: '%s'", msg->data.c_str());
}