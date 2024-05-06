#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
// #include <bspline/non_uniform_bspline.h>
// #include <poly_traj/polynomial_traj.h>
#include <iostream>
// #include <path_searching/topo_prm.hpp> NOT TO DO
// #include <plan_env/obj_predictor.hpp>


namespace fast_planner {

class PlanningVisualization {
private:
    enum TRAJECTORY_PLANNING_ID {
        GOAL = 1,
        PATH = 200,
        BSPLINE = 300,
        BSPLINE_CTRL_PT = 400,
        POLY_TRAJ = 500
    };

    enum TOPOLOGICAL_PATH_PLANNING_ID {
        GRAPH_NODE = 1,
        GRAPH_EDGE = 100,
        RAW_PATH = 200,
        FILTERED_PATH = 300,
        SELECT_PATH = 400
    };

    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr topo_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr predict_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visib_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontier_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr yaw_pub_;
    std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> pubs_;

    int last_topo_path1_num_;
    int last_topo_path2_num_;
    int last_bspline_phase1_num_;
    int last_bspline_phase2_num_;
    int last_frontier_num_;

public:
    PlanningVisualization() {}
    ~PlanningVisualization() {}
    PlanningVisualization(std::shared_ptr<rclcpp::Node> nh);

    // draw basic shapes
    void displaySphereList(const std::vector<Eigen::Vector3d>& list, double resolution,
                           const Eigen::Vector4d& color, int id, int pub_id = 0);
    void displayCubeList(const std::vector<Eigen::Vector3d>& list, double resolution,
                         const Eigen::Vector4d& color, int id, int pub_id = 0);
    void displayLineList(const std::vector<Eigen::Vector3d>& list1, const std::vector<Eigen::Vector3d>& list2,
                         double line_width, const Eigen::Vector4d& color, int id, int pub_id = 0);

    // draw a piece-wise straight line path
    void drawGeometricPath(const std::vector<Eigen::Vector3d>& path, double resolution,
                           const Eigen::Vector4d& color, int id = 0);

    // draw a polynomial trajectory
    // void drawPolynomialTraj(PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d& color,
    //                         int id = 0);

    // draw a bspline trajectory
    // void drawBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color,
    //                  bool show_ctrl_pts = false, double size2 = 0.1,
    //                  const Eigen::Vector4d& color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0,
    //                  int id2 = 0);

    // draw a set of bspline trajectories generated in different phases
    // void drawBsplinesPhase1(std::vector<NonUniformBspline>& bsplines, double size);
    // void drawBsplinesPhase2(std::vector<NonUniformBspline>& bsplines, double size);

    // draw topological graph and paths
    // void drawTopoGraph(std::list<GraphNode::Ptr>& graph, double point_size, double line_width,
    //                    const Eigen::Vector4d& color1, const Eigen::Vector4d& color2,
    //                    const Eigen::Vector4d& color3, int id = 0);

    void drawTopoPathsPhase1(std::vector<std::vector<Eigen::Vector3d>>& paths, double line_width);
    void drawTopoPathsPhase2(std::vector<std::vector<Eigen::Vector3d>>& paths, double line_width);

    void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id = 0);
    //void drawPrediction(ObjPrediction pred, double resolution, const Eigen::Vector4d& color, int id = 0);

    Eigen::Vector4d getColor(double h, double alpha = 1.0);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    // SECTION developing
    // void drawYawTraj(NonUniformBspline& pos, NonUniformBspline& yaw, const double& dt);
    // void drawYawPath(NonUniformBspline& pos, const std::vector<double>& yaw, const double& dt);
};

}  // namespace fast_planner

#endif  // _PLANNING_VISUALIZATION_H_
