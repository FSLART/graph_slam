#ifndef GRAPH_SLAM_H_
#define GRAPH_SLAM_H_

#include <rclcpp/rclcpp.hpp>
#include "graph_slam/associationSolver.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "lart_msgs/msg/cone_array.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "lart_common.h"
#include <chrono>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>

#include "graph_optimizer_sparse_incremental.h"

#include "graph_slam/types_graph_slam.h"

#define ASSOCIATION_MODE 0
#define CONES_TOPIC "/mapping/cones"
#define DYNAMICS_TOPIC "/acu_origin/dynamics"
#define IMU_TOPIC "/imu/angular_velocity"

class GraphSLAM : public rclcpp::Node
{
public:
    GraphSLAM();
    ~GraphSLAM();

    void observations_callback(const lart_msgs::msg::ConeArray::SharedPtr msg);
    void dynamics_callback(const lart_msgs::msg::Dynamics::SharedPtr msg);
    void imu_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    std::tuple<double, double, double> compute_predicted_pose(float velocity, float omega_z);
private:
    rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr observations_subscriber_;
    rclcpp::Subscription<lart_msgs::msg::Dynamics>::SharedPtr dynamics_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_subscriber_;

    g2o::SparseOptimizerIncremental optimizer_;
    g2o::G2oSlamInterface slamInterface_;
    using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
    using SlamLinearSolver = g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;
    long landmark_id_counter_ = -1;
    long pose_id_counter_ = 5000;
    float angular_velocity_ = 0.0;
    float velocity_ = 0.0;
    std::chrono::steady_clock::time_point last_predict_time_{};
    Eigen::Vector3d current_pose_{0.0, 0.0, 0.0}; // x, y, theta

    const double base_depth_uncertainty_ = 0.1; // Base longitudinal uncertainty in meters
    const double base_lateral_uncertainty_ = 0.05; // Base lateral
    const double k_depth = 0.0012;  //longitudinal uncertainty
    const double k_lateral = 0.04; //lateral uncertainty
    const double depth_weight = 1.5; //exponential weight for depth uncertainty

protected:
    AssociationSolver *association_solver_;
};

#endif