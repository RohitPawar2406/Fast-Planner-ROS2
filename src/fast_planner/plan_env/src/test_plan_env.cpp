// #include "plan_env/sdf_map.h"

// int main(int argc, char** argv) {

//     rclcpp::init(argc,argv);
//     std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_plan_env_library");
//     // SDFMap::Ptr sdf_map_;
//     // sdf_map_.reset(new SDFMap);
//     SDFMap sdf_map_;
//     sdf_map_.initMap(node);

//     // FPM.initPlanModules(node);
//     // KinoReplanFSM KRF;
//     // KRF.init(node);
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;

// }


#include"plan_env/test_plan_env.hpp"
#include"plan_env/test_plan_env_sdf.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

NodeClass::NodeClass()
: Node("my_node_class"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&NodeClass::timer_callback, this));

    // Create a shared pointer to NodeClass_Client
    auto node_class_client_ptr = std::make_shared<NodeClass_Client>();

    // Call the init function of NodeClass_Client
    node_class_client_ptr->init();

}

void NodeClass::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NodeClass>());
  rclcpp::shutdown();
  return 0;
}