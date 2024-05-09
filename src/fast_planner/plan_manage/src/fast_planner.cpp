#include <plan_manage/fast_planner.h>

namespace backward {
    backward::SignalHandling sh;
}


FastPlanner::FastPlanner()
    : Node("fast_planner_node"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&FastPlanner::timer_callback, this));
}

void FastPlanner::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}
