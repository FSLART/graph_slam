#ifndef GRAPH_SLAM_H_
#define GRAPH_SLAM_H_

#include <rclcpp/rclcpp.hpp>
#include "graph_slam/associationSolver.hpp"

#include "lart_msgs/msg/cone_array.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include "graph_slam/types_graph_slam.h"

#define ASSOCIATION_MODE 0
#define CONES_TOPIC "/mapping/cones"

class GraphSLAM : public rclcpp::Node
{
public:
    GraphSLAM();
    ~GraphSLAM();

    void observations_callback(const lart_msgs::msg::ConeArray::SharedPtr msg);

private:
    rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr observations_subscriber_;

    g2o::SparseOptimizer optimizer_;
    using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
    using SlamLinearSolver = g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;
    int landmark_id_counter_ = -1;

protected:
    AssociationSolver *association_solver_;
};

#endif