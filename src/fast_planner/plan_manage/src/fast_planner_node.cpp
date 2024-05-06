#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>

#include "plan_manage/kino_replan_fsm.h"
#include "plan_manage/topo_replan_fsm.h"

#include "plan_manage/backward.hpp"
namespace backward {
    backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {

    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("fast_planner_node");

    std::shared_ptr<KinoReplanFSM> kino_replan = std::make_shared<KinoReplanFSM>();
    node->declare_parameter<int>("planner_node/planner", -1);
    rclcpp::Parameter planner_param = node->get_parameter("planner_node/planner"); 
    int planner = planner_param.as_int();
    RCLCPP_INFO(node->get_logger(), "The planner value is (int) : %s",
                planner_param.value_to_string().c_str());
    if (planner == 1) {
        kino_replan->init(node);
    } else if (planner == 2) {
        std::cout << ("TOPO Commented for now ");
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
