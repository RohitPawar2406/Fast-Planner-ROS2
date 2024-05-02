#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/Marker.hpp"

#include "plan_manage/kino_replan_fsm.hpp"
#include "plan_manage/topo_replan_fsm.hpp"

#include "plan_manage/backward.hpp"
namespace backward {
    backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("fast_planner_node");

    int planner;
    node->declare_parameter<int>("planner_node.planner", -1);
    node->get_parameter("planner_node.planner", planner);

    std::shared_ptr<TopoReplanFSM> topo_replan = std::make_shared<TopoReplanFSM>(rclcpp::NodeOptions());
    std::shared_ptr<KinoReplanFSM> kino_replan = std::make_shared<KinoReplanFSM>(rclcpp::NodeOptions());

    if (planner == 1) {
        kino_replan->init();
    } else if (planner == 2) {
        topo_replan->init();
    }

    rclcpp::sleep_for(std::chrono::seconds(1));
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
