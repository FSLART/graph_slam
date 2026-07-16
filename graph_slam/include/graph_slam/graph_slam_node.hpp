#ifndef GRAPH_SLAM_NODE_HPP_
#define GRAPH_SLAM_NODE_HPP_

#include "graph_slam/graph_slam.hpp"

#include "topics.h"

#include "super_node/parent_node.hpp"

// GraphSLAM_Node is a managed lifecycle node: it extends super_node::ParentNode
// so the race_director can drive its state (Unconfigured/Inactive/Active) through
// the standard lifecycle services. Resource creation lives in configure_impl(),
// and the periodic pose/tf broadcast only runs while the node is Active.
class GraphSLAM_Node : public super_node::ParentNode
{
public:
    GraphSLAM_Node();
    // Callbacks
    void observations_callback(const lart_msgs::msg::ConeArray::SharedPtr msg);
    void dynamics_callback(const lart_msgs::msg::Dynamics::SharedPtr msg);
    void imu_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void mission_callback(const lart_msgs::msg::Mission::SharedPtr msg);

protected:
    // Lifecycle hooks required by ParentNode.
    CallbackReturn configure_impl() override;
    CallbackReturn activate_impl() override;
    CallbackReturn deactivate_impl() override;
    CallbackReturn cleanup_impl() override;
    CallbackReturn shutdown_impl() override;

private:

    //Subscriptions
    rclcpp::Subscription<lart_msgs::msg::ConeArray>::SharedPtr observations_subscriber_;
    rclcpp::Subscription<lart_msgs::msg::Dynamics>::SharedPtr dynamics_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<lart_msgs::msg::Mission>::SharedPtr mission_subscriber_;

    //Publishers (lifecycle-managed: only emit while the node is Active)
    rclcpp_lifecycle::LifecyclePublisher<lart_msgs::msg::SlamStats>::SharedPtr slam_stats_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<lart_msgs::msg::ConeArray>::SharedPtr map_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_markers_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Periodic pose/tf broadcast timer, replacing the old detached while(ok())
    // thread. Created on activate, cancelled on deactivate, so it only runs Active.
    rclcpp::TimerBase::SharedPtr broadcast_timer_;
    rclcpp::CallbackGroup::SharedPtr observations_cb_group_;
    rclcpp::CallbackGroup::SharedPtr other_cb_group_;

    void broadcast_transform();
    lart_msgs::msg::ConeArray create_map_cones(visualization_msgs::msg::MarkerArray cones);

protected:
    GraphSLAM *graph_slam_solver_ = nullptr;
};

#endif // GRAPH_SLAM_NODE_HPP_
