#include "plan_env/sdf_map.h"

int main(int argc, char** argv) {

    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_plan_env_library");
    // SDFMap::Ptr sdf_map_;
    // sdf_map_.reset(new SDFMap);
    SDFMap sdf_map_;
    sdf_map_.initMap(node);

    // FPM.initPlanModules(node);
    // KinoReplanFSM KRF;
    // KRF.init(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}