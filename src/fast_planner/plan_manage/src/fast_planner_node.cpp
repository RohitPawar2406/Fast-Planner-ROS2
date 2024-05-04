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
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("fast_planner_node");

    int planner;
    node->declare_parameter<int>("planner_node/planner", -1);
    node->get_parameter("planner_node/planner", planner);

        // topo_replan->init(node);
    // std::shared_ptr<TopoReplanFSM> topo_replan = std::make_shared<TopoReplanFSM>();
    std::shared_ptr<KinoReplanFSM> kino_replan = std::make_shared<KinoReplanFSM>();

    if (planner == 1) {
        kino_replan->init(node);
    } else if (planner == 2) {
        std::cout << ("TOPO Commented for now ");
    }

    // rclcpp::executors::SingleThreadedExecutor executor;
    // executor.add_node(node);
    // executor.spin();

    rclcpp::executors::SingleThreadedExecutor exec;
	exec.add_node(node);

	auto spin_exec = [&exec]() {
    	exec.spin();
  	};

    // rclcpp::sleep_for(std::chrono::seconds(1));
    // rclcpp::spin(node);


    // rclcpp::shutdown();
    // return 0;
}
