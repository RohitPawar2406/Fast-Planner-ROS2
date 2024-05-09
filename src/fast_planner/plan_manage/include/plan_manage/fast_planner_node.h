#ifndef FAST_PLANNER_H_
#define FAST_PLANNER_H_

#include "plan_manage/kino_replan_fsm.h"
//#include "plan_manage/topo_replan_fsm.h"

#include "plan_manage/backward.hpp"
#include "std_msgs/msg/string.hpp"

using namespace fast_planner;


class FastPlanner : public rclcpp::Node
{
    public:
        FastPlanner();

    private:
        void timer_callback();

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
};


#endif