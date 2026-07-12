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
#include <deque>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/sparse_block_matrix.h>
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

#define WINDOW_SIZE 50

#define SKIDPAD_MAP "/maps/skidpad.yaml.default"

#define ONLINE_FLAG true

#define IMU_NOISE 0.01

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
    void compute_predicted_pose(double stamp_sec);
    // Step 1: set odometry-edge noise params (tunable via ROS params in the node) -- replaces 35*I.
    void set_odom_noise_params(double sigma_v_frac, double sigma_theta_coeff, double slip_inflation);
    // Fix F.1: iteration cap for the one-time final map optimization at the mapping->localization
    // switch (the existing terminate action's gain threshold still stops it earlier on convergence).
    void set_final_optimize_max_iters(int max_iters);
    // Step 3 (Fix D interim): assumed /mapping/cones pipeline latency [s]. Cone frames are
    // unstamped, so observations are anchored to the pose this long before the newest predict.
    void set_obs_latency(double latency_sec);
    // Observation anchoring mode (A/B + robustness knob): 0 = NEWEST (anchor to the latest pose,
    // pre-Step-3 behaviour), 1 = CONST (ignore any stamp, use latest predict - obs_latency),
    // 2 = STAMP (per-frame header stamp = capture time; falls back to CONST when unstamped).
    void set_anchor_mode(int mode);
    // Camera extrinsic (camera -> rear-axle base frame): obs_base = R(yaw)*obs + [x, y]. Applied
    // at cone intake before the pose+R(theta)*obs global step. In PacSim the front sensor is at the
    // CoG (perception.yaml pose [0,0,0]) so x == lr (0.6975 m); real car camera ~0.69 m fwd of the
    // rear axle. 0/0/0 leaves observations in the base frame (pre-refactor behavior). See the frame
    // policy at the top of graph_slam.cpp -- keep x DISTINCT from the harness GT-transform lr.
    void set_camera_extrinsic(double x_m, double y_m, double yaw_rad);
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
    double last_predict_stamp_sec_ = -1.0; // Step 0: last dynamics header stamp [s]; <0 = uninitialized

    // Camera extrinsic: camera -> rear-axle base frame (see set_camera_extrinsic). 0/0/0 keeps
    // observations in the base frame (pre-refactor behavior, for a clean A/B). Sim default (front
    // sensor at CoG) is 0.6975/0/0; real car ~0.69/0/0.
    double camera_extrinsic_x_   = 0.69;
    double camera_extrinsic_y_   = 0.0;
    double camera_extrinsic_yaw_ = 0.0;

    // Step 1: odometry-edge information (real per-DoF, real units) -- replaces the 35*I magic
    // number. Tunable via ROS params (graph_slam_node). See graph_slam_refactor_plan.md Step 1.
    double odom_sigma_v_frac_      = 0.025;   // wheel-speed / rolling-radius scale error (fraction of v)
    double odom_sigma_theta_coeff_ = 1.22e-4; // gyro angle-random-walk coeff [rad/sqrt(s)] (MTi-680G)
    double odom_slip_inflation_    = 1.0;     // x sigma_ds when slip detected (detection: TODO Step 1b)
    double odom_sigma_ds_floor_    = 1e-3;    // min per-step translation sigma [m] (avoids inf info at v~0)
    double odom_sigma_theta_floor_ = 1e-5;    // min per-step heading sigma [rad]

    // Fix F.1: final map optimization at the mode switch runs to convergence under this cap
    // (was a single hardcoded iteration, which froze the map at its dead-reckoning warp).
    int final_optimize_max_iters_ = 50;
    g2o::SparseOptimizerTerminateAction* terminate_action_ = nullptr;

    // Step 3 (Fix D interim): stamped pose history for anchoring observations at capture time.
    // /mapping/cones carries no stamp, so capture time is approximated as
    // (latest predict stamp - obs_latency_sec_). vertex_id is -1 in localization mode (no
    // graph vertices are created there). Guarded by pose_mutex_.
    struct StampedPose { double t; Eigen::Vector3d pose; long vertex_id; };
    std::deque<StampedPose> pose_history_;
    double obs_latency_sec_ = 0.09;   // measured ~90 ms on ground_truth_bag2 (REFACTOR_LOG)
    int obs_anchor_mode_ = 2;         // 0=NEWEST, 1=CONST, 2=STAMP (default: per-frame stamp)
    bool lookup_pose_at(double t, Eigen::Vector3d& pose, long& vertex_id);

    // Step 2: pose covariance Sigma (x,y,theta). Grown in compute_predicted_pose (F*S*F'+Q),
    // reset/updated at map corrections (computeMarginals + Kalman). Initialized in the ctor.
    Eigen::Matrix3d pose_cov_;

    //Mutexes
    std::mutex pose_mutex_;
    std::mutex pose_id_mutex_;
    std::mutex optimizer_mutex_;

    // Bookkeeping for new vertices and edges in each optimization step
    g2o::HyperGraph::VertexSet new_vertices;
    g2o::HyperGraph::EdgeSet   new_edges;
    int new_poses_since_optimize_ = 0;

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
    std::vector<LandmarkKDInfo> map_kdtree_landmarks_;
    bool map_kdtree_ready_ = false;
    void build_map_kdtree();
    
    //optimize graph
    void update_graph();

    // Camera -> rear-axle base frame transform for one observation (see set_camera_extrinsic).
    graph_slam_types::Cone camera_to_base(const graph_slam_types::Cone& obs) const;

    //Pose correction
    void localize_in_map(std::vector<graph_slam_types::Cone>& observations, Eigen::Vector3d robot_pose);

    // Map publish
    visualization_msgs::msg::MarkerArray get_map(std::vector<graph_slam_types::Cone> cones = {});

protected:
    AssociationSolver *association_solver_;
};

#endif