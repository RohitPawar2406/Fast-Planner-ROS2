#ifndef FAST_PLANNER_H_
#define FAST_PLANNER_H_


#include "rclcpp/rclcpp.hpp"

#include <string>
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;

class FastPlanner : public rclcpp::Node
{
    public:
        FastPlanner();

    private:
        void timer_callback();

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
};

#endif