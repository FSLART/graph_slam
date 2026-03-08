#ifndef ASSOCIATION_SOLVER_H_
#define ASSOCIATION_SOLVER_H_

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <stdexcept>
#include <cmath>
#include <limits>
#include <map>
#include <algorithm>
#include <iostream>

#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>

#include "lart_msgs/msg/cone_array.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "graph_slam/custom_types.hpp"

#define ASSOCIATION_EUCLIDIAN_DISTANCE_THRESHOLD_SQUARED 2.0  // in meters

class AssociationSolver
{
public:
    AssociationSolver(int mode);
    ~AssociationSolver();

    std::pair<std::vector<int>, std::vector<graph_slam_types::Cone>> associate(const std::vector<graph_slam_types::Cone> &observations,
                               const std::vector<graph_slam_types::Cone> &map_cones,
                               const Eigen::Vector3d &pose);

    class AssociationBackend;

    Eigen::Matrix2d get_info_matrix(double x, double y);

private:
    std::unique_ptr<AssociationBackend> backend_;

    std::vector<double> x_grid = {-3.0, 0.0, 3.0};
    std::vector<double> y_grid = {2.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 14.0};

    std::map<std::pair<double, double>, std::pair<double, double>> table = {
        {{-3, 14}, {0.88, 0.03}}, {{0, 14}, {0.14, 0.31}}, {{3, 14}, {0.14, 0.45}},
        {{-3, 12}, {0.11, 0.10}}, {{0, 12}, {0.17, 0.34}}, {{3, 12}, {0.21, 0.41}},
        {{-3, 10}, {0.11, 0.10}}, {{0, 10}, {0.56, 0.25}}, {{3, 10}, {0.13, 0.29}},
        {{-3, 8},  {0.01, 0.10}}, {{0, 8},  {0.14, 0.16}}, {{3, 8},  {0.08, 0.19}},
        {{-3, 6},  {0.11, 0.12}}, {{0, 6},  {0.01, 0.10}}, {{3, 6},  {0.09, 0.07}},
        {{-3, 5},  {0.12, 0.10}}, {{0, 5},  {0.20, 0.06}}, {{3, 5},  {0.14, 0.04}},
        {{-3, 4},  {0.21, 0.14}}, {{0, 4},  {0.12, 0.04}}, {{3, 4},  {0.24, 0.01}},
        {{-3, 2},  {0.39, 0.15}}, {{0, 2},  {0.20, 0.10}}, {{3, 2},  {0.43, 0.10}}
    };

    double interpolate(double x, double y, double x1, double x2, double y1, double y2,
                       double q11, double q21, double q12, double q22);

};

#endif