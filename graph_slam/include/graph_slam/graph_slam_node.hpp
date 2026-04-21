#ifndef GRAPH_SLAM_NODE_HPP_
#define GRAPH_SLAM_NODE_HPP_

// #include <rclcpp/rclcpp.hpp>
#include "graph_slam/graph_slam.hpp"

// #include <geometry_msgs/msg/vector3_stamped.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
// #include <visualization_msgs/msg/marker.hpp>

// #include "lart_msgs/msg/dynamics.hpp"
// #include "lart_msgs/msg/cone_array.hpp"
// #include "lart_msgs/msg/mission.hpp"
// #include "lart_msgs/msg/slam_stats.hpp"
// #include "lart_msgs/msg/cone.hpp"

#define CONES_TOPIC "/mapping/cones" // observations
#define DYNAMICS_TOPIC "/acu_origin/dynamics" //rpm and all
#define IMU_TOPIC "/imu/angular_velocity"
#define MISSION_TOPIC "/pc_origin/system_status/critical_as/mission"
#define MAP_MARKERS_TOPIC "/slam/map/markers"
#define MAP_TOPIC "/slam/map"
#define POSE_TOPIC "/slam/pose"
#define POSE_MARKER_TOPIC "/slam/pose_marker"
#define STATS_TOPIC "/slam/stats"

class GraphSLAM_Node : public rclcpp::Node
{
public:
    GraphSLAM_Node();
    // Callbacks
    void observations_callback(const lart_msgs::msg::ConeArray::SharedPtr msg);
    void dynamics_callback(const lart_msgs::msg::Dynamics::SharedPtr msg);
    void imu_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void mission_callback(const lart_msgs::msg::Mission::SharedPtr msg);

private:

    //Subscriptions
    rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr observations_subscriber_;
    rclcpp::Subscription<lart_msgs::msg::Dynamics>::SharedPtr dynamics_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<lart_msgs::msg::Mission>::SharedPtr mission_subscriber_;
    
    //Publishers
    rclcpp::Publisher<lart_msgs::msg::SlamStats>::SharedPtr slam_stats_publisher_;
    rclcpp::Publisher<lart_msgs::msg::ConeArray>::SharedPtr map_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_markers_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    void broadcast_transform();
    lart_msgs::msg::ConeArray create_map_markers(visualization_msgs::msg::MarkerArray cones);
protected:
    GraphSLAM *graph_slam_solver_;
};

#endif // GRAPH_SLAM_NODE_HPP_