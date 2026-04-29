#ifndef GRAPH_SLAM_H_
#define GRAPH_SLAM_H_

#include "graph_slam/associationSolver.hpp"
#include "graph_slam/types_graph_slam.h"
#include "graph_slam/map_manager.hpp"

#include "lart_common.h"
#include "lart_msgs/msg/dynamics.hpp"
#include "lart_msgs/msg/mission.hpp"
#include "lart_msgs/msg/slam_stats.hpp"

// #include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <map>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>
#include <g2o/core/robust_kernel_impl.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#define ASSOCIATION_MODE 1


#define SKIDPAD_MAP "/maps/skidpad.yaml.default"

#define ONLINE_FLAG true

struct LandmarkKDInfo {
    long vertex_id;
    int color;
};

class GraphSLAM
{
public:
    GraphSLAM();
    ~GraphSLAM();

    visualization_msgs::msg::MarkerArray process_observations(const lart_msgs::msg::ConeArray::SharedPtr msg);
    void process_dynamics(const lart_msgs::msg::Dynamics::SharedPtr msg);
    void set_angular_velocity(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void set_mission(const lart_msgs::msg::Mission::SharedPtr msg);
    void compute_predicted_pose();
    Eigen::Vector3d get_current_pose();
    g2o::SparseOptimizer optimizer_;
    int get_lap(){return current_lap_;};
private:
    // SLAM G2O Solvers
    using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
    using SlamLinearSolver = g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;
    //Vertex Ids
    long landmark_id_counter_ = -1;
    long pose_id_counter_ = 5000;
    //Pose estimation
    Eigen::Vector3d current_pose_{0.0, 0.0, 0.0}; // x, y, theta
    float velocity_ = 0.0;
    float angular_velocity_ = 0.0;
    std::chrono::steady_clock::time_point last_predict_time_{};
    //Mutexes
    std::mutex pose_mutex_;
    std::mutex pose_id_mutex_;
    std::mutex optimizer_mutex_;
    // Bookkeeping for new vertices and edges in each optimization step
    g2o::HyperGraph::VertexSet new_vertices;
    g2o::HyperGraph::EdgeSet   new_edges;
    // Stats variables
    long observation_count_ = 0;
    float time_sum_ = 0.0;
    bool is_robot_moving_= false;
    bool initialized_once = false;
    //Lap logic variables
    lart_msgs::msg::Mission current_mission_;
    bool mission_set_ = false;
    int16_t current_lap_ = -1;
    float current_lap_distance_ = 0.0;
    float lap_margin_x_ = 0.5;
    float lap_margin_y_ = 3.0;
    float lap_margin_ = 10.0;
    void check_lap_completion();
    
    bool localization_mode_ = false;
    visualization_msgs::msg::MarkerArray final_map_;

    // KD-tree variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::KdTreeFLANN<pcl::PointXYZ> map_kdtree_;
    // std::vector<long> map_kdtree_vertex_ids_;
    u_int8_t count_locliz_updts_ = 0;

    std::vector<LandmarkKDInfo> map_kdtree_landmarks_;

    bool map_kdtree_ready_ = false;

    void build_map_kdtree();
    
    void update_graph(g2o::HyperGraph::VertexSet& vset, g2o::HyperGraph::EdgeSet& eset);
    void localize_in_map(std::vector<graph_slam_types::Cone>& observations, long current_pose_id, Eigen::Vector3d robot_pose);
    visualization_msgs::msg::MarkerArray get_map(std::vector<graph_slam_types::Cone> cones = {});

protected:
    AssociationSolver *association_solver_;
};

#endif