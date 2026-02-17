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
#include "geometry_msgs/msg/pose_stamped.hpp"

#define ASSOCIATION_EUCLIDIAN_DISTANCE_THRESHOLD_SQUARED 1.2  // meters

class AssociationSolver
{
public:
    AssociationSolver(int mode);
    ~AssociationSolver();

    std::pair<std::vector<int>, lart_msgs::msg::ConeArray> associate(const lart_msgs::msg::ConeArray &observations,
                               const lart_msgs::msg::ConeArray &map_cones,
                               const Eigen::Vector3d &pose);

    class AssociationBackend;

private:
    std::unique_ptr<AssociationBackend> backend_;
};

#endif