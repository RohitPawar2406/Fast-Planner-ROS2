#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>

#include "plan_manage/kino_replan_fsm.h"
#include "plan_manage/topo_replan_fsm.h"
#include "plan_manage/fast_planner_node.h"

#include "plan_manage/backward.hpp"

#include <string>
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace fast_planner;

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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FastPlanner>();

  node->declare_parameter<float>("bspline/limit_vel", 0);
  node->declare_parameter<float>("bspline/limit_acc", 0);
  node->declare_parameter<float>("bspline/limit_ratio", 0);

  node->declare_parameter<int>("planner_node/planner", -1);
  rclcpp::Parameter planner_param = node->get_parameter("planner_node/planner"); 

  int planner = planner_param.as_int();
  RCLCPP_INFO(node->get_logger(), "The planner value is (int) : %s",
                planner_param.value_to_string().c_str());

  // # Initialize the kino Class
  std::shared_ptr<KinoReplanFSM> kino_replan = std::make_shared<KinoReplanFSM>();
  
  if (planner == 1) {
      kino_replan->init(node);
  } else if (planner == 2) {
      std::cout << ("TOPO Commented for now ");
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
  
}
