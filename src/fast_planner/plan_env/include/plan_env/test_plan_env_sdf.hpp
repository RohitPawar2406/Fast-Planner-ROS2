#ifndef TEST_PLAN_ENV_SDF_HPP_
#define TEST_PLAN_ENV_SDF_HPP_

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "plan_env/test_plan_env.hpp"

class NodeClass_Client
{
    public:
        NodeClass_Client();
        void init(std::shared_ptr<NodeClass> node_class);
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    private:
        void topic_callback(const std_msgs::msg::String::SharedPtr msg) const; 
        std::shared_ptr<NodeClass> node_class_;
        
};

#endif 
