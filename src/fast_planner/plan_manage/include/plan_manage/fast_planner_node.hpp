#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>

#include "plan_manage/kino_replan_fsm.h"
#include "plan_manage/topo_replan_fsm.h"

#include "plan_manage/backward.hpp"

using namespace fast_planner;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher(): Node("fast_planner_node"){};
    
private:
};

