#ifndef BSPLINE_OPTIMIZER_HPP_
#define BSPLINE_OPTIMIZER_HPP_

#include <Eigen/Eigen>
#include <vector>

#include "plan_env/edt_environment.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fast_planner {

class BsplineOptimizer {

public:
  static const int SMOOTHNESS;
  static const int DISTANCE;
  static const int FEASIBILITY;
  static const int ENDPOINT;
  static const int GUIDE;
  static const int WAYPOINTS;

  static const int GUIDE_PHASE;
  static const int NORMAL_PHASE;

  BsplineOptimizer() {}
  ~BsplineOptimizer() {}

  /* main API */ 
  void setEnvironment(const std::shared_ptr<EDTEnvironment> env);
  void setParam(rclcpp::Node::SharedPtr& nh);
  Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd& points, const double& ts,
                                      const int& cost_function, int max_num_id, int max_time_id);

  /* helper function */

  // required inputs
  void setControlPoints(const Eigen::MatrixXd& points);
  void setBsplineInterval(const double& ts);
  void setCostFunction(const int& cost_function);
  void setTerminateCond(const int& max_num_id, const int& max_time_id);

  // optional inputs
  void setGuidePath(const std::vector<Eigen::Vector3d>& guide_pt);
  void setWaypoints(const std::vector<Eigen::Vector3d>& waypts,
                    const std::vector<int>&             waypt_idx);  // N-2 constraints at most

  void optimize();

  Eigen::MatrixXd         getControlPoints();
  std::vector<Eigen::Vector3d> matrixToVectors(const Eigen::MatrixXd& ctrl_pts);

private:
  std::shared_ptr<EDTEnvironment> edt_environment_;

  // main input
  Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
  double          bspline_interval_;   // B-spline knot span
  Eigen::Vector3d end_pt_;             // end of the trajectory
  int             dim_;                // dimension of the B-spline
                                       //
  std::vector<Eigen::Vector3d> guide_pts_;  // geometric guiding path points, N-6
  std::vector<Eigen::Vector3d> waypoints_;  // waypts constraints
  std::vector<int>             waypt_idx_;  // waypts constraints index
                                       //
  int    max_num_id_, max_time_id_;    // stopping criteria
  int    cost_function_;               // used to determine objective function
  bool   dynamic_;                     // moving obstacles ?
  double start_time_;                  // global time for moving obstacles

  /* optimization parameters */
  int    order_;                  // bspline degree
  double lambda1_;                // jerk smoothness weight
  double lambda2_;                // distance weight
  double lambda3_;                // feasibility weight
  double lambda4_;                // end point weight
  double lambda5_;                // guide cost weight
  double lambda6_;                // visibility cost weight
  double lambda7_;                // waypoints cost weight
  double lambda8_;                // acc smoothness
                                  //
  double dist0_;                  // safe distance
  double max_vel_, max_acc_;      // dynamic limits
  double visib_min_;              // threshold of visibility
  double wnl_;                    //
  double dlmin_;                  //
                                  //
  int    algorithm1_;             // optimization algorithms for quadratic cost
  int    algorithm2_;             // optimization algorithms for general cost
  int    max_iteration_num_[4];   // stopping criteria that can be used
  double max_iteration_time_[4];  // stopping criteria that can be used

  /* intermediate variables */
  /* buffer for gradient of cost function, to avoid repeated allocation and
   * release of memory */
  std::vector<Eigen::Vector3d> g_q_;
  std::vector<Eigen::Vector3d> g_smoothness_;
  std::vector<Eigen::Vector3d> g_distance_;
  std::vector<Eigen::Vector3d> g_feasibility_;
  std::vector<Eigen::Vector3d> g_endpoint_;
  std::vector<Eigen::Vector3d> g_guide_;
  std::vector<Eigen::Vector3d> g_waypoints_;

  int                 variable_num_;   // optimization variables
  int                 iter_num_;       // iteration of the solver
  std::vector<double> best_variable_;  //
  double              min_cost_;       //

  std::vector<Eigen::Vector3d> block_pts_;  // blocking points to compute visibility

  /* cost function */
  /* calculate each part of cost function with control points q as input */

  static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
  void          combineCost(const std::vector<double>& x, std::vector<double>& grad, double& cost);

  // q contains all control points
  void calcSmoothnessCost(const std::vector<Eigen::Vector3d>& q, double& cost,
                          std::vector<Eigen::Vector3d>& gradient);
  void calcDistanceCost(const std::vector<Eigen::Vector3d>& q, double& cost,
                        std::vector<Eigen::Vector3d>& gradient);
  void calcFeasibilityCost(const std::vector<Eigen::Vector3d>& q, double& cost,
                           std::vector<Eigen::Vector3d>& gradient);
  void calcEndpointCost(const std::vector<Eigen::Vector3d>& q, double& cost,
                        std::vector<Eigen::Vector3d>& gradient);
  void calcGuideCost(const std::vector<Eigen::Vector3d>& q, double& cost, std::vector<Eigen::Vector3d>& gradient);
  void calcVisibilityCost(const std::vector<Eigen::Vector3d>& q, double& cost,
                          std::vector<Eigen::Vector3d>& gradient);
  void calcWaypointsCost(const std::vector<Eigen::Vector3d>& q, double& cost,
                         std::vector<Eigen::Vector3d>& gradient);
  void calcViewCost(const std::vector<Eigen::Vector3d>& q, double& cost, std::vector<Eigen::Vector3d>& gradient);
  bool isQuadratic();

  /* for benckmark evaluation only */
public:
  std::vector<double> vec_cost_;
  std::vector<double> vec_time_;
  rclcpp::Time        time_start_;

  void getCostCurve(std::vector<double>& cost, std::vector<double>& time) {
    cost = vec_cost_;
    time = vec_time_;
  }

  typedef std::unique_ptr<BsplineOptimizer> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace fast_planner
#endif  // BSPLINE_OPTIMIZER_HPP_
