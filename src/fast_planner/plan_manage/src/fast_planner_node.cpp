#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>

#include "plan_manage/kino_replan_fsm.h"
#include "plan_manage/topo_replan_fsm.h"

#include "plan_manage/backward.hpp"
namespace backward {
    backward::SignalHandling sh;
}

using namespace fast_planner;
// class MinimalPublisher : public rclcpp::Node
// {
//   public:
//     MinimalPublisher()
//     : Node("fast_planner_node")
//     {
//         int planner;
//         auto node_ = this;
//         std::shared_ptr<KinoReplanFSM> kino_replan = std::make_shared<KinoReplanFSM>();
//         this->declare_parameter<int>("planner_node/planner", -1);
//         this->get_parameter("planner_node/planner", planner); 
//         if (planner == 1) {
//             // kino_replan->init(node_->node_());
//         } 
//         // else if (planner == 2) {
//         //     std::cout << ("TOPO Commented for now ");
//     }
    

//   private:
// };


// Example code.
class ABCD {
public:
    ABCD() {}

    void init(std::shared_ptr<rclcpp::Node> node) {
        std::cout << "init hit " << std::endl;
        node->get_parameter_or("param_name", param_value_, std::string("default_value"));
    }

private:
  float t;
  std::string param_value_;

};
int main(int argc, char** argv) {

    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("fast_planner_node");
    FastPlannerManager FPM;
    FPM.initPlanModules(node);
    ABCD t;
    t.init(node);

    // FPM.initPlanModules(node);
    // KinoReplanFSM KRF;
    // KRF.init(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

    //   void setParam(rclcpp::Node::SharedPtr& nh);
    // //auto node = rclcpp::Node::make_shared("fast_planner_node");



    //     // topo_replan->init(node);
    // // std::shared_ptr<TopoReplanFSM> topo_replan = std::make_shared<TopoReplanFSM>();
    


    // // rclcpp::executors::SingleThreadedExecutor executor;
    // // executor.add_node(node);
    // // executor.spin();
    // //auto node = std::make_shared<nav2_smoother::SmootherServer>();
    // rclcpp::spin(std::make_shared<MinimalPublisher>());
    // rclcpp::shutdown();

    // // rclcpp::sleep_for(std::chrono::seconds(1));
    // // rclcpp::spin(node);


    // // rclcpp::shutdown();
    // return 0;
}
