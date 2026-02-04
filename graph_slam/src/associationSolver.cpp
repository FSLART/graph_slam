#include "graph_slam/associationSolver.hpp"

#include <stdexcept>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"

namespace
{
    // Simple 2D Euclidean distance between two geometry_msgs points
    inline double euclideanDistance2D(const geometry_msgs::msg::Point &a,
                                      const geometry_msgs::msg::Point &b)
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        return dx * dx + dy * dy;
    }
} // namespace

// ================== Backend interface =======================================

class AssociationSolver::AssociationBackend
{
public:
    virtual ~AssociationBackend() = default;
    // Returns a vector aligned with observations.cones:
    //  - element i is the index of the matched map cone, or -1 if no match
    virtual std::vector<int> associate(const lart_msgs::msg::ConeArray &observations,
                                       const lart_msgs::msg::ConeArray &map_cones,
                                       const geometry_msgs::msg::PoseStamped &pose) = 0;
};

// ================== Nearest Neighbor backend =================================

class NearestNeighborBackend : public AssociationSolver::AssociationBackend
{
public:
    std::vector<int> associate(const lart_msgs::msg::ConeArray &observations,
                               const lart_msgs::msg::ConeArray &map_cones,
                               const geometry_msgs::msg::PoseStamped &pose) override
    {
        // If there is no map yet or no observations, everything is "new"
        if (map_cones.cones.empty())
        {
            RCLCPP_DEBUG(rclcpp::get_logger("association_solver"),
                         "Map is empty, all observations are unmatched.");
            
            // All observations are unmatched -> filled with -1
            return std::vector<int>(observations.cones.size(), -1);
        }

        // Transform observations from local vehicle frame into global/map frame
        const auto &pose_msg = pose.pose;
        const double yaw = pose.pose.orientation.w;
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);

        std::vector<int> matches(observations.cones.size(), -1);

        // For each observed cone (in local frame), find nearest cone in map (global)
        for (std::size_t i = 0; i < observations.cones.size(); ++i)
        {
            const auto &obs_cone_local = observations.cones[i];

            // Transform to global coordinates (2D)
            geometry_msgs::msg::Point obs_global;
            const double x_l = obs_cone_local.position.x;
            const double y_l = obs_cone_local.position.y;

            obs_global.x = pose_msg.position.x + cy * x_l - sy * y_l;
            obs_global.y = pose_msg.position.y + sy * x_l + cy * y_l;
            obs_global.z = 0.0;

            int best_index = -1;
            double best_dist = std::numeric_limits<double>::max();

            for (std::size_t j = 0; j < map_cones.cones.size(); ++j)
            {
                if(observations.cones[i].class_type.data != map_cones.cones[j].class_type.data)
                {
                    // Skip cones of different color
                    continue;
                }
                const auto &map_cone = map_cones.cones[j];
                const double d_squared = euclideanDistance2D(obs_global, map_cone.position);

                if (d_squared < best_dist)
                {
                    best_dist = d_squared;
                    best_index = static_cast<int>(j);
                }
            }

            //TODO : Use variable threshold based on observation uncertainty
            //TODO : Threshold lower in function of the number of cones (they are probably closer)  
            if (best_index != -1 && best_dist <= ASSOCIATION_EUCLIDIAN_DISTANCE_THRESHOLD_SQUARED)
            {
                // Observation is considered to correspond to an existing map cone
                matches[i] = best_index;
                RCLCPP_DEBUG(rclcpp::get_logger("association_solver"),
                            "Observation %zu associated to map cone %d (dist = %.3f m)",
                            i, best_index, best_dist);
            }
            else
            {
                // No sufficiently close cone found in the map
                RCLCPP_DEBUG(rclcpp::get_logger("association_solver"),
                            "Observation %zu not associated (best dist = %.3f m)",
                            i, best_dist);
            }
        }

        // Return full match array, aligned with observations
        return matches;
    }
};

// ================== Mahalanobis backend ======================================

class MahalanobisBackend : public AssociationSolver::AssociationBackend
{
public:
    std::vector<int> associate(const lart_msgs::msg::ConeArray &observations,
                               const lart_msgs::msg::ConeArray &map_cones,
                               const geometry_msgs::msg::PoseStamped &pose) override
    {
        // TODO: implement Mahalanobis distance-based association
        // d^2 = (z - h(x))^T S^{-1} (z - h(x))
        (void)observations;
        (void)map_cones;
        (void)pose;

        // For now, treat all observations as unmatched
        return std::vector<int>(observations.cones.size(), -1);
    }
};

// ================== AssociationSolver front-end ==============================

AssociationSolver::AssociationSolver(int mode)
{
    switch (mode)
    {
    case 0:
        backend_ = std::make_unique<NearestNeighborBackend>();
        break;
    case 1:
        backend_ = std::make_unique<MahalanobisBackend>();
        break;
    default:
        throw std::invalid_argument("Unknown association mode");
    }
}

AssociationSolver::~AssociationSolver() = default;

std::vector<int> AssociationSolver::associate(const lart_msgs::msg::ConeArray &observations,
                                              const lart_msgs::msg::ConeArray &map_cones,
                                              const geometry_msgs::msg::PoseStamped &pose)
{
    if (backend_)
        return backend_->associate(observations, map_cones, pose);

    return {};
}