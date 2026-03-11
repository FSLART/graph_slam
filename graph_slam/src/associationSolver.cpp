#include "graph_slam/associationSolver.hpp"
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

    inline std::vector<graph_slam_types::Cone> obsToGlobal(const std::vector<graph_slam_types::Cone> &cone_array, const Eigen::Vector3d &pose)
    {
        std::vector<graph_slam_types::Cone> positions;
        const double yaw = pose[2];
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);
        const double car_x = pose[0];
        const double car_y = pose[1];

        for (std::size_t i = 0; i < cone_array.size(); ++i){
            const auto &obs_cone_local = cone_array[i];

            // Transform to global coordinates (2D)
            graph_slam_types::Cone obs_global;
            const double x_l = obs_cone_local.x;
            const double y_l = obs_cone_local.y;

            obs_global.x = car_x + cy * x_l - sy * y_l;
            obs_global.y = car_y + sy * x_l + cy * y_l;
            obs_global.type = obs_cone_local.type;

            positions.push_back(obs_global);
        }

        return positions;
    }
} // namespace

// ================== Backend interface =======================================

class AssociationSolver::AssociationBackend
{
public:
    virtual ~AssociationBackend() = default;
    // Returns a vector aligned with observations.cones:
    //  - element i is the index of the matched map cone, or -1 if no match
    virtual std::pair<std::vector<int>, std::vector<graph_slam_types::Cone>> associate(const std::vector<graph_slam_types::Cone> &observations,
                                       const std::vector<graph_slam_types::Cone> &map_cones,
                                       const Eigen::Vector3d &pose) = 0;
};

// ================== Nearest Neighbor backend =================================

class NearestNeighborBackend : public AssociationSolver::AssociationBackend
{
public:
    std::pair<std::vector<int>, std::vector<graph_slam_types::Cone>> associate(const std::vector<graph_slam_types::Cone> &observations,
                               const std::vector<graph_slam_types::Cone> &map_cones,
                               const Eigen::Vector3d &pose) override
    {

        std::vector<graph_slam_types::Cone> obs_global = obsToGlobal(observations, pose);
        // If there is no map yet or no observations, everything is "new"
        if (map_cones.empty())
        {
            RCLCPP_DEBUG(rclcpp::get_logger("association_solver"),
                         "Map is empty, all observations are unmatched.");
            
            // All observations are unmatched -> filled with -1
            return {std::vector<int>(observations.size(), -1), obs_global};
        }

        std::vector<int> matches(observations.size(), -1);

        // For each observed cone (in local frame), find nearest cone in map (global)
        for (std::size_t i = 0; i < obs_global.size(); ++i)
        {
            // Transform to global coordinates (2D)
            geometry_msgs::msg::Point global_cone;
            global_cone.x = obs_global[i].x;
            global_cone.y = obs_global[i].y;

            int best_index = -1;
            double best_dist = std::numeric_limits<double>::max();

            for (std::size_t j = 0; j < map_cones.size(); ++j)
            {
                if(obs_global[i].type != map_cones[j].type)
                {
                    // Skip cones of different color
                    continue;
                }
                geometry_msgs::msg::Point map_cone;
                map_cone.x = map_cones[j].x;
                map_cone.y = map_cones[j].y;

                const double d_squared = euclideanDistance2D(global_cone, map_cone);

                if (d_squared < best_dist)
                {
                    best_dist = d_squared;
                    best_index = static_cast<int>(map_cones[j].id);
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
        return {matches, obs_global};
    }
};

// ================== Mahalanobis backend ======================================

class MahalanobisBackend : public AssociationSolver::AssociationBackend
{
public:
    std::pair<std::vector<int>, std::vector<graph_slam_types::Cone>> associate(const std::vector<graph_slam_types::Cone> &observations,
                               const std::vector<graph_slam_types::Cone> &map_cones,
                               const Eigen::Vector3d &pose) override
    {
        auto start_time = std::chrono::steady_clock::now();
        std::vector<graph_slam_types::Cone> obs_global = obsToGlobal(observations, pose);
        std::vector<int> associations(observations.size(), -1);

        // Chi-squared threshold for 2 degrees of freedom (x, y) 
        const double threshold = 4.3;

        for (size_t i = 0; i < obs_global.size(); ++i) {
            double min_dist = std::numeric_limits<double>::max();
            int best_idx = -1;

            Eigen::Vector2d z(obs_global[i].x, obs_global[i].y);

            for (size_t j = 0; j < map_cones.size(); ++j) {
                // Only match cones of the same type (color)
                if (obs_global[i].type != map_cones[j].type) continue;

                Eigen::Vector2d z_hat(map_cones[j].x, map_cones[j].y);
                Eigen::Vector2d diff = z - z_hat;

                // Mahalanobis distance: d^2 = diff^T * S^-1 * diff
                // Note: Since S^-1 is the information matrix, we can use it directly!
                double d2 = diff.transpose() * obs_global[i].information * diff;

                if (d2 < threshold && d2 < min_dist) {
                    min_dist = d2;
                    best_idx = j;
                }
            }

            if (best_idx != -1) {
                associations[i] = map_cones[best_idx].id;
            }
        }

        auto end_time = std::chrono::steady_clock::now();
        auto duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        RCLCPP_INFO(rclcpp::get_logger("association_solver"), "Mahalanobis association took %.3f ms.", duration_ms);
        return {associations, obs_global};
    }
};

// ================== ICP backend ==============================================

class ICPBackend : public AssociationSolver::AssociationBackend
{
public:
    std::pair<std::vector<int>, std::vector<graph_slam_types::Cone>> associate(const std::vector<graph_slam_types::Cone> &observations,
                               const std::vector<graph_slam_types::Cone> &map_cones,
                               const Eigen::Vector3d &pose) override
    {
        std::vector<graph_slam_types::Cone> obs_global = obsToGlobal(observations, pose);
        
        if (map_cones.empty()) {
            return {std::vector<int>(observations.size(), -1), obs_global};
        }

        // 1. Convert ROS messages to PCL Clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obs(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& cone : obs_global)
            cloud_obs->push_back(pcl::PointXYZ(cone.x, cone.y, 0));

        for (const auto& cone : map_cones)
            cloud_map->push_back(pcl::PointXYZ(cone.x, cone.y, 0));

        // 2. Setup and Run ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_obs);
        icp.setInputTarget(cloud_map);
        
        // Settings: Max 1.5m to find a neighbor, stop if change is tiny
        icp.setMaxCorrespondenceDistance(2); 
        icp.setTransformationEpsilon(1e-5);
        icp.setMaximumIterations(10);

        pcl::PointCloud<pcl::PointXYZ> aligned_obs;
        icp.align(aligned_obs);

        std::vector<int> matches(observations.size(), -1);

        if (icp.hasConverged()) {
            // 3. Match based on ALIGNED positions
            for (size_t i = 0; i < aligned_obs.size(); ++i) {
                int best_index = -1;
                double best_dist_sq = std::numeric_limits<double>::max();

                for (size_t j = 0; j < map_cones.size(); ++j) {
                    // Still respect color classes
                    if(obs_global[i].type != map_cones[j].type)
                        continue;

                    double dx = aligned_obs[i].x - map_cones[j].x;
                    double dy = aligned_obs[i].y - map_cones[j].y;
                    double d_sq = dx*dx + dy*dy;

                    if (d_sq < best_dist_sq) {
                        best_dist_sq = d_sq;
                        best_index = static_cast<int>(map_cones[j].type);
                    }
                }

                // Use a tighter threshold (e.g., 0.4m) since we've already aligned clouds
                if (best_index != -1 && best_dist_sq < 0.70) { // 0.4^2 = 0.16
                    matches[i] = best_index;
                }
            }
        }

        return {matches, obs_global};
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
    case 2:
        backend_ = std::make_unique<ICPBackend>();
        break;
    default:
        throw std::invalid_argument("Unknown association mode");
    }
}

AssociationSolver::~AssociationSolver() = default;

std::pair<std::vector<int>, std::vector<graph_slam_types::Cone>> AssociationSolver::associate(const std::vector<graph_slam_types::Cone> &observations,
                                              const std::vector<graph_slam_types::Cone> &map_cones,
                                              const Eigen::Vector3d &pose)
{
    if (backend_)
        return backend_->associate(observations, map_cones, pose);

    return std::pair<std::vector<int>, std::vector<graph_slam_types::Cone>>{};
}

Eigen::Matrix2d AssociationSolver::get_info_matrix(double x, double y)
{
    // 1. Clamp and Grid Search (Same as before)
    x = std::clamp(x, x_grid.front(), x_grid.back());
    y = std::clamp(y, y_grid.front(), y_grid.back());

    auto it_x = std::lower_bound(x_grid.begin(), x_grid.end(), x);
    int x1_idx = std::distance(x_grid.begin(), it_x) - (it_x == x_grid.begin() ? 0 : 1);
    int x2_idx = std::min(x1_idx + 1, (int)x_grid.size() - 1);

    auto it_y = std::lower_bound(y_grid.begin(), y_grid.end(), y);
    int y1_idx = std::distance(y_grid.begin(), it_y) - (it_y == y_grid.begin() ? 0 : 1);
    int y2_idx = std::min(y1_idx + 1, (int)y_grid.size() - 1);

    // 2. Interpolate Errors
    auto get_err = [&](int xi, int yi) { return table.at({x_grid[xi], y_grid[yi]}); };
    
    double dx = interpolate(x, y, x_grid[x1_idx], x_grid[x2_idx], y_grid[y1_idx], y_grid[y2_idx], 
                           get_err(x1_idx, y1_idx).first, get_err(x2_idx, y1_idx).first,
                           get_err(x1_idx, y2_idx).first, get_err(x2_idx, y2_idx).first);

    double dy = interpolate(x, y, x_grid[x1_idx], x_grid[x2_idx], y_grid[y1_idx], y_grid[y2_idx], 
                           get_err(x1_idx, y1_idx).second, get_err(x2_idx, y1_idx).second,
                           get_err(x1_idx, y2_idx).second, get_err(x2_idx, y2_idx).second);

    // 3. Robust Variance Calculation
    // min_sigma: The best possible precision you trust (e.g., 0.1 units)
    // max_sigma: The worst precision before you consider a measurement "useless"
    const double min_sigma = 0.1; 
    const double max_sigma = 2.0; 

    // Add noise floor and clamp the error values before squaring
    double sigma_x = std::clamp(dx, min_sigma, max_sigma);
    double sigma_y = std::clamp(dy, min_sigma, max_sigma);

    Eigen::Matrix2d info = Eigen::Matrix2d::Zero();
    info(0, 0) = 1.0 / (sigma_x * sigma_x);
    info(1, 1) = 1.0 / (sigma_y * sigma_y);

    return info;
}

double AssociationSolver::interpolate(double x, double y, double x1, double x2, double y1, double y2, double q11, double q21, double q12, double q22){
    if (x1 == x2 && y1 == y2) return q11;
    if (x1 == x2) return q11 + (q12 - q11) * (y - y1) / (y2 - y1);
    if (y1 == y2) return q11 + (q21 - q11) * (x - x1) / (x2 - x1);

    double r1 = ((x2 - x) / (x2 - x1)) * q11 + ((x - x1) / (x2 - x1)) * q21;
    double r2 = ((x2 - x) / (x2 - x1)) * q12 + ((x - x1) / (x2 - x1)) * q22;
    return ((y2 - y) / (y2 - y1)) * r1 + ((y - y1) / (y2 - y1)) * r2;

}
