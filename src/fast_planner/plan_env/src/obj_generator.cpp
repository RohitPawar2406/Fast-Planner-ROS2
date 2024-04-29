#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <vector>
#include <random>
#include "plan_env/linear_obj_model.hpp"  // Ensure you have a ROS2 compatible version of this

using namespace std;
using namespace std::chrono_literals;

class DynamicObjNode : public rclcpp::Node {
public:
    int obj_num;
    double _xy_size, _h_size, _vel, _yaw_dot, _acc_r1, _acc_r2, _acc_z, _scale1, _scale2, _interval;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obj_pub;
    vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_pubs;
    vector<LinearObjModel> obj_models;

    random_device rd;
    default_random_engine eng{rd()};
    uniform_real_distribution<double> rand_pos, rand_h, rand_vel, rand_acc_r, rand_acc_t, rand_acc_z, rand_color, rand_scale, rand_yaw_dot, rand_yaw;

    rclcpp::TimerBase::SharedPtr update_timer;
    DynamicObjNode() : Node("dynamic_obj") {
        this->declare_parameter<int>("obj_num", 10);
        this->declare_parameter<double>("xy_size", 15.0);
        this->declare_parameter<double>("h_size", 5.0);
        this->declare_parameter<double>("vel", 5.0);
        this->declare_parameter<double>("yaw_dot", 5.0);
        this->declare_parameter<double>("acc_r1", 4.0);
        this->declare_parameter<double>("acc_r2", 6.0);
        this->declare_parameter<double>("acc_z", 3.0);
        this->declare_parameter<double>("scale1", 1.5);
        this->declare_parameter<double>("scale2", 2.5);
        this->declare_parameter<double>("interval", 2.5);

        this->get_parameter("obj_num", obj_num);
        this->get_parameter("xy_size", _xy_size);
        this->get_parameter("h_size", _h_size);
        this->get_parameter("vel", _vel);
        this->get_parameter("yaw_dot", _yaw_dot);
        this->get_parameter("acc_r1", _acc_r1);
        this->get_parameter("acc_r2", _acc_r2);
        this->get_parameter("acc_z", _acc_z);
        this->get_parameter("scale1", _scale1);
        this->get_parameter("scale2", _scale2);
        this->get_parameter("interval", _interval);
        obj_pub = this->create_publisher<visualization_msgs::msg::Marker>("/dynamic/obj", 10);

        
        for (int i = 0; i < obj_num; ++i) {
            auto pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/dynamic/pose_" + to_string(i), 10);
            pose_pubs.push_back(pose_pub);
        }

        update_timer = this->create_wall_timer(33ms, bind(&DynamicObjNode::updateCallback, this));
        initializeModels();
    }

    void initializeModels() {
        rand_color = uniform_real_distribution<double>(0.0, 1.0);
        rand_pos = uniform_real_distribution<double>(-_xy_size, _xy_size);
        rand_h = uniform_real_distribution<double>(0.0, _h_size);
        rand_vel = uniform_real_distribution<double>(-_vel, _vel);
        rand_acc_t = uniform_real_distribution<double>(0.0, 6.28);
        rand_acc_r = uniform_real_distribution<double>(_acc_r1, _acc_r2);
        rand_acc_z = uniform_real_distribution<double>(-_acc_z, _acc_z);
        rand_scale = uniform_real_distribution<double>(_scale1, _scale2);
        rand_yaw = uniform_real_distribution<double>(0.0, 2 * M_PI);
        rand_yaw_dot = uniform_real_distribution<double>(-_yaw_dot, _yaw_dot);

        for (int i = 0; i < obj_num; ++i) {
            LinearObjModel model;
            Eigen::Vector3d pos(rand_pos(eng), rand_pos(eng), rand_h(eng));
            Eigen::Vector3d vel(rand_vel(eng), rand_vel(eng), 0.0);
            Eigen::Vector3d color(rand_color(eng), rand_color(eng), rand_color(eng));
            Eigen::Vector3d scale(rand_scale(eng), 1.5 * rand_scale(eng), rand_scale(eng));
            double yaw = rand_yaw(eng);
            double yaw_dot = rand_yaw_dot(eng);

            double r, t, z;
            r = rand_acc_r(eng);
            t = rand_acc_t(eng);
            z = rand_acc_z(eng);
            Eigen::Vector3d acc(r * cos(t), r * sin(t), z);

            model.initialize(pos, vel, acc, yaw, yaw_dot, color, scale);
            model.setLimits(Eigen::Vector3d(_xy_size, _xy_size, _h_size), Eigen::Vector2d(0.0, _vel), Eigen::Vector2d(0, 0));
            obj_models.push_back(model);
        }
    }

    void updateCallback() {
        auto time_now = this->get_clock()->now();

        for (int i = 0; i < obj_num; ++i) {
            obj_models[i].update(this->get_clock()->now().seconds() - last_update_time_);
            visualizeObj(i);
        }
        last_update_time_ = this->get_clock()->now().seconds();
    }

    void visualizeObj(int id) {
        auto& model = obj_models[id];
        auto pos = model.getPosition();
        auto color = model.getColor();
        auto scale = model.getScale();
        double yaw = model.getYaw();

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
        pose.pose.orientation.w = qua.w();
        pose.pose.orientation.x = qua.x();
        pose.pose.orientation.y = qua.y();
        pose.pose.orientation.z = qua.z();

        pose_pubs[id]->publish(pose);
    }

    double last_update_time_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<DynamicObjNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
