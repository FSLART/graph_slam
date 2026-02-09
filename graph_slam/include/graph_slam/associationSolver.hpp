#ifndef ASSOCIATION_SOLVER_H_
#define ASSOCIATION_SOLVER_H_

#include <memory>
#include <vector>

#include "lart_msgs/msg/cone_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#define ASSOCIATION_EUCLIDIAN_DISTANCE_THRESHOLD_SQUARED 1.0  // meters

class AssociationSolver
{
public:
    AssociationSolver(int mode);
    ~AssociationSolver();

    std::pair<std::vector<int>, lart_msgs::msg::ConeArray> associate(const lart_msgs::msg::ConeArray &observations,
                               const lart_msgs::msg::ConeArray &map_cones,
                               const geometry_msgs::msg::PoseStamped &pose);

    class AssociationBackend;

private:
    std::unique_ptr<AssociationBackend> backend_;
};

#endif