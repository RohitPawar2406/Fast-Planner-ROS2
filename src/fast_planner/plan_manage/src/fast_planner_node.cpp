#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>

#include "plan_manage/kino_replan_fsm.h"
#include "plan_manage/topo_replan_fsm.h"

#include "plan_manage/backward.hpp"

#include <string>
#include "std_msgs/msg/string.hpp"

namespace backward {
    backward::SignalHandling sh;
}

void call_back(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(rclcpp::get_logger("logger"), "Received: %s", msg->data.c_str());
}

using namespace fast_planner;

int main(int argc, char** argv) {

    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("fast_planner_node");

    // std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> publisher_;
    // publisher_ =node->create_publisher<std_msgs::msg::String>("asd", 1);
    // rclcpp::Rate loop_rate(10);
    // std_msgs::msg::String str;

    // size_t count_ = 0;

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

    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> publisher_;
    publisher_ = node->create_subscription<std_msgs::msg::String>("/asdddd", 10, call_back);

    // while(rclcpp::ok()){
    //     str.data = "Hello! " + std::to_string(count_++);
    //     publisher_->publish(str);
    //     RCLCPP_INFO(rclcpp::get_logger("logger"), "Published: %s", str.data.c_str());
    //     loop_rate.sleep();
    //     rclcpp::spin_some(node);
    // }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
