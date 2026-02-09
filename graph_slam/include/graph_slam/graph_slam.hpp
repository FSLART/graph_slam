#ifndef GRAPH_SLAM_H_
#define GRAPH_SLAM_H_

#include <rclcpp/rclcpp.hpp>
#include "graph_slam/associationSolver.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "lart_msgs/msg/cone_array.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include "graph_slam/g2o_types.hpp"

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
private:
    rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr observations_subscriber_;
    rclcpp::Subscription<lart_msgs::msg::Dynamics>::SharedPtr dynamics_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_subscriber_;

    g2o::SparseOptimizer optimizer_;
    using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
    using SlamLinearSolver = g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;
    int landmark_id_counter_ = -1;

protected:
    AssociationSolver *association_solver_;
};

#endif