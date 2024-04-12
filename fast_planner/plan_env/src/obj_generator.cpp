/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/

#include "visualization_msgs/msg/marker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include <plan_env/linear_obj_model.hpp>

using namespace std;

int obj_num;
double _xy_size, _h_size, _vel, _yaw_dot, _acc_r1, _acc_r2, _acc_z, _scale1, _scale2, _interval;

rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obj_pub; // visualize marker
vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_pubs; // obj pose (from optitrack)
vector<LinearObjModel> obj_models;

random_device rd;
default_random_engine eng(rd());
uniform_real_distribution<double> rand_pos;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_vel;
uniform_real_distribution<double> rand_acc_r;
uniform_real_distribution<double> rand_acc_t;
uniform_real_distribution<double> rand_acc_z;
uniform_real_distribution<double> rand_color;
uniform_real_distribution<double> rand_scale;
uniform_real_distribution<double> rand_yaw_dot;
uniform_real_distribution<double> rand_yaw;

rclcpp::Time time_update, time_change;

void updateCallback(const rclcpp::TimerBase& timer);
void visualizeObj(int id);




using namespace std;

class DynamicObjNode : public rclcpp::Node {
public:
    DynamicObjNode() : Node("dynamic_obj") {
        this->declare_parameter<int>("obj_generator.obj_num", 10);
        this->declare_parameter<double>("obj_generator.xy_size", 15.0);
        this->declare_parameter<double>("obj_generator.h_size", 5.0);
        this->declare_parameter<double>("obj_generator.vel", 5.0);
        this->declare_parameter<double>("obj_generator.yaw_dot", 5.0);
        this->declare_parameter<double>("obj_generator.acc_r1", 4.0);
        this->declare_parameter<double>("obj_generator.acc_r2", 6.0);
        this->declare_parameter<double>("obj_generator.acc_z", 3.0);
        this->declare_parameter<double>("obj_generator.scale1", 1.5);
        this->declare_parameter<double>("obj_generator.scale2", 2.5);
        this->declare_parameter<double>("obj_generator.interval", 2.5);

        this->get_parameter("obj_generator.obj_num", obj_num);
        this->get_parameter("obj_generator.xy_size", _xy_size);
        this->get_parameter("obj_generator.h_size", _h_size);
        this->get_parameter("obj_generator.vel", _vel);
        this->get_parameter("obj_generator.yaw_dot", _yaw_dot);
        this->get_parameter("obj_generator.acc_r1", _acc_r1);
        this->get_parameter("obj_generator.acc_r2", _acc_r2);
        this->get_parameter("obj_generator.acc_z", _acc_z);
        this->get_parameter("obj_generator.scale1", _scale1);
        this->get_parameter("obj_generator.scale2", _scale2);
        this->get_parameter("obj_generator.interval", _interval);

        obj_pub = this->create_publisher<visualization_msgs::msg::Marker>("/dynamic/obj", 10);
        for (int i = 0; i < obj_num; ++i) {
            auto pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/dynamic/pose_" + to_string(i), 10);
            pose_pubs.push_back(pose_pub);
        }

        update_timer = this->create_wall_timer(33ms, std::bind(&DynamicObjNode::updateCallback, this));
        RCLCPP_INFO(this->get_logger(), "Initialization complete with %d moving objects.", obj_num);
    }

private:
    void updateCallback() {
        auto time_now = this->get_clock()->now();
        rclcpp::Duration dtc = time_now - time_change;
        if (dtc.seconds() > _interval) {
            for (int i = 0; i < obj_num; ++i) {
                double vx, vy, vz, yd;
                vx = rand_vel(eng);
                vy = rand_vel(eng);
                vz = 0.0;
                yd = rand_yaw_dot(eng);

                obj_models[i].setInput(Eigen::Vector3d(vx, vy, vz));
                obj_models[i].setYawDot(yd);
            }
            time_change = time_now;
        }

        rclcpp::Duration dt = time_now - time_update;
        time_update = time_now;
        for (int i = 0; i < obj_num; ++i) {
            obj_models[i].update(dt.seconds());
            visualizeObj(i);
        }
    }

    void visualizeObj(int id) {
        auto& model = obj_models[id];
        auto pos = model.getPosition();
        auto color = model.getColor();
        auto scale = model.getScale();
        auto yaw = model.getYaw();

        Eigen::Quaterniond qua(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

        visualization_msgs::msg::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = this->get_clock()->now();
        mk.type = visualization_msgs::msg::Marker::CUBE;
        mk.action = visualization_msgs::msg::Marker::ADD;
        mk.id = id;

        mk.scale.x = scale(0);
        mk.scale.y = scale(1);
        mk.scale.z = scale(2);
        mk.color.a = 0.5;
        mk.color.r = color(0);
        mk.color.g = color(1);
        mk.color.b = color(2);

        mk.pose.orientation.w = qua.w();
        mk.pose.orientation.x = qua.x();
        mk.pose.orientation.y = qua.y();
        mk.pose.orientation.z = qua.z();

        mk.pose.position.x = pos(0);
        mk.pose.position.y = pos(1);
        mk.pose.position.z = pos(2);

        obj_pub->publish(mk);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = pos(0);
        pose.pose.position.y = pos(1);
        pose.pose.position.z = pos(2);
        pose.pose.orientation = mk.pose.orientation;

        pose_pubs[id]->publish(pose);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obj_pub;
    vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_pubs;
    vector<LinearObjModel> obj_models;

    random_device rd;
    default_random_engine eng{rd()};
    uniform_real_distribution<double> rand_vel{-_vel, _vel}, rand_yaw_dot{-_yaw_dot, _yaw_dot};

    rclcpp::Time time_update, time_change;
    rclcpp::TimerBase::SharedPtr update_timer;

    int obj_num;
    double _xy_size, _h_size, _vel, _yaw_dot, _acc_r1, _acc_r2, _acc_z, _scale1, _scale2, _interval;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicObjNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
