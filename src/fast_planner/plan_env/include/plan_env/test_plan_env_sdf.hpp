#ifndef TEST_PLAN_ENV_SDF_HPP_
#define TEST_PLAN_ENV_SDF_HPP_

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "plan_env/test_plan_env.hpp"

class Node2
{
    public:
        Node2();
        void messageCallback(const std_msgs::msg::String::SharedPtr msg); 
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subs;
        std::shared_ptr<NodeClass> node_class_ = std::make_shared<NodeClass>();
        
};

#endif 