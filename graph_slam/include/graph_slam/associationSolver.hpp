#ifndef ASSOCIATION_SOLVER_H_
#define ASSOCIATION_SOLVER_H_

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include <stdexcept>
#include <cmath>
#include <limits>

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

private:
    std::unique_ptr<AssociationBackend> backend_;
};

#endif