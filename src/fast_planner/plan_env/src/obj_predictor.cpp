#include "plan_env/obj_predictor.hpp"
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace fast_planner {

/* ============================== obj history_ ============================== */

int ObjHistory::queue_size_;
int ObjHistory::skip_num_;
rclcpp::Time ObjHistory::global_start_time_;
auto node_time = rclcpp::Node::make_shared("time_node");
void ObjHistory::init(int id) {
  clear();
  skip_ = 0;
  obj_idx_ = id;
}

void ObjHistory::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  ++skip_;
  if (skip_ < ObjHistory::skip_num_) return;

  Eigen::Vector4d pos_t;
  pos_t(0) = msg->pose.position.x, pos_t(1) = msg->pose.position.y, pos_t(2) = msg->pose.position.z;
  pos_t(3) = (node_time->get_clock()->now() - ObjHistory::global_start_time_).seconds();

  history_.push_back(pos_t);
  int h1 = history_.size();
  double h2 = (double) h1;

  if (h2 > queue_size_) history_.pop_front();

  skip_ = 0;
}

/* ============================== obj predictor ============================== */

ObjPredictor::ObjPredictor(/* args */) {
}

ObjPredictor::ObjPredictor(rclcpp::Node::SharedPtr node) : node_handle_(node) {
}

ObjPredictor::~ObjPredictor() {
}

void ObjPredictor::init() {
  /* get param */
  node_handle_->get_parameter("prediction/obj_num", obj_num_);
  node_handle_->get_parameter("prediction/lambda", lambda_);
  node_handle_->get_parameter("prediction/predict_rate", predict_rate_);

  predict_trajs_ = std::make_shared<std::vector<PolynomialPrediction>>(obj_num_);

  obj_scale_ = std::make_shared<std::vector<Eigen::Vector3d>>(obj_num_);
  scale_init_.resize(obj_num_);
  for (int i = 0; i < obj_num_; i++)
    scale_init_[i] = false;

  /* subscribe to pose */
  for (int i = 0; i < obj_num_; i++) {
    auto obj_his = std::make_shared<ObjHistory>();
    obj_his->init(i);
    obj_histories_.push_back(obj_his);

    auto pose_sub = node_handle_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/dynamic/pose_" + std::to_string(i), 10, [this, i](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          obj_histories_[i]->poseCallback(msg);
        });

    pose_subs_.push_back(pose_sub);
  }

  marker_sub_ = node_handle_->create_subscription<visualization_msgs::msg::Marker>(
      "/dynamic/obj", 10, [this](visualization_msgs::msg::Marker::SharedPtr msg) {
        markerCallback(msg);
      });

  /* update prediction */
  predict_timer_ = node_handle_->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / predict_rate_)), [this]() { predictCallback(); });
}

ObjPrediction ObjPredictor::getPredictionTraj() {
  return predict_trajs_;
}

ObjScale ObjPredictor::getObjScale() {
  return obj_scale_;
}

void ObjPredictor::predictPolyFit() {
  // Implement polynomial prediction
  for (int i = 0; i < obj_num_; i++) {
    /* ---------- write A and b ---------- */
    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 1> temp;
    Eigen::Matrix<double, 6, 1> bm[3];  // poly coefficent
    vector<Eigen::Matrix<double, 6, 1>> pm(3);

    A.setZero();
    for (int i = 0; i < 3; ++i)
      bm[i].setZero();

    /* ---------- estimation error ---------- */
    list<Eigen::Vector4d> his;
    obj_histories_[i]->getHistory(his);
    for (list<Eigen::Vector4d>::iterator it = his.begin(); it != his.end(); ++it) {
      Eigen::Vector3d qi = (*it).head(3);
      double ti = (*it)(3);

      /* A */
      temp << 1.0, ti, pow(ti, 2), pow(ti, 3), pow(ti, 4), pow(ti, 5);
      for (int j = 0; j < 6; ++j)
        A.row(j) += 2.0 * pow(ti, j) * temp.transpose();

      /* b */
      for (int dim = 0; dim < 3; ++dim)
        bm[dim] += 2.0 * qi(dim) * temp;
    }

    /* ---------- acceleration regulator ---------- */
    double t1 = his.front()(3);
    double t2 = his.back()(3);

    temp << 0.0, 0.0, 2 * t1 - 2 * t2, 3 * pow(t1, 2) - 3 * pow(t2, 2), 4 * pow(t1, 3) - 4 * pow(t2, 3),
        5 * pow(t1, 4) - 5 * pow(t2, 4);
    A.row(2) += -4 * lambda_ * temp.transpose();

    temp << 0.0, 0.0, pow(t1, 2) - pow(t2, 2), 2 * pow(t1, 3) - 2 * pow(t2, 3),
        3 * pow(t1, 4) - 3 * pow(t2, 4), 4 * pow(t1, 5) - 4 * pow(t2, 5);
    A.row(3) += -12 * lambda_ * temp.transpose();

    temp << 0.0, 0.0, 20 * pow(t1, 3) - 20 * pow(t2, 3), 45 * pow(t1, 4) - 45 * pow(t2, 4),
        72 * pow(t1, 5) - 72 * pow(t2, 5), 100 * pow(t1, 6) - 100 * pow(t2, 6);
    A.row(4) += -4.0 / 5.0 * lambda_ * temp.transpose();

    temp << 0.0, 0.0, 35 * pow(t1, 4) - 35 * pow(t2, 4), 84 * pow(t1, 5) - 84 * pow(t2, 5),
        140 * pow(t1, 6) - 140 * pow(t2, 6), 200 * pow(t1, 7) - 200 * pow(t2, 7);
    A.row(5) += -4.0 / 7.0 * lambda_ * temp.transpose();

    /* ---------- solve ---------- */
    for (int j = 0; j < 3; j++) {
      pm[j] = A.colPivHouseholderQr().solve(bm[j]);
    }

    /* ---------- update prediction container ---------- */
    predict_trajs_->at(i).setPolynomial(pm);
    predict_trajs_->at(i).setTime(t1, t2);
  }
}

void ObjPredictor::predictCallback() {
  // predictPolyFit();
  predictConstVel();
}

void ObjPredictor::markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
  int idx = msg->id;
  (*obj_scale_)[idx](0) = msg->scale.x;
  (*obj_scale_)[idx](1) = msg->scale.y;
  (*obj_scale_)[idx](2) = msg->scale.z;

  scale_init_[idx] = true;

  int finish_num = 0;
  for (int i = 0; i < obj_num_; i++) {
    if (scale_init_[i]) finish_num++;
  }

  if (finish_num == obj_num_) {
    marker_sub_.reset();
  }
}

void ObjPredictor::predictConstVel() {
  for (int i = 0; i < obj_num_; i++) {
    /* ---------- get the last two point ---------- */
    std::list<Eigen::Vector4d> his;
    obj_histories_[i]->getHistory(his);
    std::list<Eigen::Vector4d>::iterator list_it = his.end();

    Eigen::Vector3d q1, q2;
    double t1, t2;

    --list_it;
    q2 = (*list_it).head(3);
    t2 = (*list_it)(3);

    --list_it;
    q1 = (*list_it).head(3);
    t1 = (*list_it)(3);

    Eigen::Matrix<double, 2, 3> p01, q12;
    q12.row(0) = q1.transpose();
    q12.row(1) = q2.transpose();

    Eigen::Matrix<double, 2, 2> At12;
    At12 << 1, t1, 1, t2;

    p01 = At12.inverse() * q12;

    std::vector<Eigen::Matrix<double, 6, 1>> polys(3);
    for (int j = 0; j < 3; ++j) {
      polys[j].setZero();
      polys[j].head(2) = p01.col(j);
    }

    predict_trajs_->at(i).setPolynomial(polys);
    predict_trajs_->at(i).setTime(t1, t2);
  }
}

}  // namespace fast_planner
