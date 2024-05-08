#ifndef TEST_PLAN_ENV_HPP_
#define TEST_PLAN_ENV_HPP_
#include <rclcpp/rclcpp.hpp>

#ifndef NODE_CLASS_HPP_
#define NODE_CLASS_HPP_

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

class NodeClass : public rclcpp::Node
{
public:
    NodeClass();

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

#endif


#endif 