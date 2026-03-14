#ifndef GRAPH_SLAM_H_
#define GRAPH_SLAM_H_

#include "graph_slam/associationSolver.hpp"
#include "graph_slam/types_graph_slam.h"
#include "graph_slam/custom_types.hpp"

#include "lart_common.h"
#include "lart_msgs/msg/dynamics.hpp"
#include "lart_msgs/msg/cone_array.hpp"
#include "lart_msgs/msg/mission.hpp"
#include "lart_msgs/msg/slam_stats.hpp"
#include "lart_msgs/msg/cone.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <map>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>


#define ASSOCIATION_MODE 1
#define CONES_TOPIC "/mapping/cones" // observations
#define DYNAMICS_TOPIC "/acu_origin/dynamics" //rpm and all
#define IMU_TOPIC "/imu/angular_velocity"
#define MISSION_TOPIC "/mission"
#define MAP_MARKERS_TOPIC "/slam/map/markers"
#define MAP_TOPIC "/slam/map"
#define POSE_TOPIC "/slam/pose"
#define POSE_MARKER_TOPIC "/slam/pose_marker"
#define STATS_TOPIC "/slam/stats"



class GraphSLAM : public rclcpp::Node
{
public:
    GraphSLAM();
    ~GraphSLAM();

    void observations_callback(const lart_msgs::msg::ConeArray::SharedPtr msg);
    void dynamics_callback(const lart_msgs::msg::Dynamics::SharedPtr msg);
    void imu_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void mission_callback(const lart_msgs::msg::Mission::SharedPtr msg);
    std::tuple<double, double, double> compute_predicted_pose(float velocity, float omega_z);
private:
    //Subscriptions
    rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr observations_subscriber_;
    rclcpp::Subscription<lart_msgs::msg::Dynamics>::SharedPtr dynamics_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<lart_msgs::msg::Mission>::SharedPtr mission_subscriber_;
    
    //Publishers
    rclcpp::Publisher<lart_msgs::msg::SlamStats>::SharedPtr slam_stats_publisher_;

    rclcpp::Publisher<lart_msgs::msg::ConeArray>::SharedPtr map_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_markers_publisher_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_marker_publisher_;

    struct GridPos {
        float x, y;
        bool operator<(const GridPos& other) const {
            if (x != other.x) return x < other.x;
            return y < other.y;
        }
    };

    g2o::SparseOptimizer optimizer_;
    using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
    using SlamLinearSolver = g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;
    long landmark_id_counter_ = -1;
    long pose_id_counter_ = 5000;
    float angular_velocity_ = 0.0;
    float velocity_ = 0.0;
    std::chrono::steady_clock::time_point last_predict_time_{};
    Eigen::Vector3d current_pose_{0.0, 0.0, 0.0}; // x, y, theta

    // Information matrix parameters
    const double base_depth_uncertainty_ = 0.1; // Base longitudinal uncertainty in meters
    const double base_lateral_uncertainty_ = 0.05; // Base lateral
    const double k_depth = 0.0012;  //longitudinal uncertainty
    const double k_lateral = 0.04; //lateral uncertainty
    const double depth_weight = 1.5; //exponential weight for depth uncertainty

    // Stats variables
    long frame_count_ = 0;
    long observation_count_ = 0;
    double time_sum_ = 0.0;
    bool is_robot_moving_= false;

    //Lap logic variables
    lart_msgs::msg::Mission current_mission_;
    bool mission_set_ = false;
    int16_t current_lap_ = -1;
    double current_lap_distance_ = 0.0;
    float lap_margin_x_ = 1.0;
    float lap_margin_y_ = 3.0;
    float lap_margin_ = 10.0;
    void check_lap_completion();

    bool localization_mode_ = false;
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    void broadcast_transform();
    void publish_map();

    
protected:
    AssociationSolver *association_solver_;
};

#endif