#include "plan_env/test_plan_env_sdf.hpp"

NodeClass_Client::NodeClass_Client()
{}

void NodeClass_Client::init() {
    subscription_ = node_class_->create_subscription<std_msgs::msg::String>(
        "topic_subs", 10, std::bind(&NodeClass_Client::topic_callback, this, std::placeholders::_1));
}


void NodeClass_Client::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(node_class_->get_logger(), "I heard: '%s'", msg->data.c_str());
}