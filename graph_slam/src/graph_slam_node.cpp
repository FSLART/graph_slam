#include "graph_slam/graph_slam_node.hpp"

using std::placeholders::_1;
using namespace std;

GraphSLAM_Node::GraphSLAM_Node() : Node("graph_slam_node"){
    RCLCPP_INFO(this->get_logger(), "GraphSLAM node has been started.");

    #ifdef __LART_T24__
        RCLCPP_WARN(this->get_logger(), "Running on T24 hardware.");
    #else
        RCLCPP_WARN(this->get_logger(), "Running on T26 hardware");
    #endif

    graph_slam_solver_ = new GraphSLAM();

    auto observations_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto other_callbacks_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions obs_options;
    obs_options.callback_group = observations_callback_group;

    rclcpp::SubscriptionOptions other_options;
    other_options.callback_group = other_callbacks_group;

    // Subscribe to the cone observations topic
    observations_subscriber_ = this->create_subscription<lart_msgs::msg::ConeArray>(
        CONES_TOPIC, 10,
        bind(&GraphSLAM_Node::observations_callback, this, _1), obs_options);

    // Subscribe to the dynamics topic
    dynamics_subscriber_ = this->create_subscription<lart_msgs::msg::Dynamics>(
        DYNAMICS_TOPIC, 10,
        bind(&GraphSLAM_Node::dynamics_callback, this, _1), other_options);

    //Subscribe to angular velocity topic 
    imu_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        IMU_TOPIC, 10,
        bind(&GraphSLAM_Node::imu_callback, this, _1), other_options);

    mission_subscriber_ = this->create_subscription<lart_msgs::msg::Mission>(
        MISSION_TOPIC, 10,
        bind(&GraphSLAM_Node::mission_callback, this, _1), other_options);

    slam_stats_publisher_ = this->create_publisher<lart_msgs::msg::SlamStats>(STATS_TOPIC, 10);
    
    map_markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(MAP_MARKERS_TOPIC, 10);

    map_publisher_ = this->create_publisher<lart_msgs::msg::ConeArray>(MAP_TOPIC, 10);

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(POSE_TOPIC, 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    std::thread broadcast_transform_thread ([this]() {
        rclcpp::Rate rate(50);
        while (rclcpp::ok()) {
            this->broadcast_transform();
            rate.sleep();
        }
    });
    broadcast_transform_thread.detach();
}

void GraphSLAM_Node::broadcast_transform(){
    Eigen::Vector3d current_pose = this->graph_slam_solver_->get_current_pose();

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "base_footprint";
    transformStamped.transform.translation.x = current_pose[0];
    transformStamped.transform.translation.y = current_pose[1];
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose[2]);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transformStamped);
    
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = current_pose[0];
    pose_msg.pose.position.y = current_pose[1];
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), current_pose[2]));

    this->pose_publisher_->publish(pose_msg);
}

void GraphSLAM_Node::observations_callback(const lart_msgs::msg::ConeArray::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Received cone observations");
    visualization_msgs::msg::MarkerArray map_cones_markers = this->graph_slam_solver_->process_observations(msg);
    lart_msgs::msg::ConeArray map_cones_msg_ = this->create_map_markers(map_cones_markers);

    this->map_markers_publisher_->publish(map_cones_markers);
    this->map_publisher_->publish(map_cones_msg_);

    //Publish lap count, current observations count and total observations count
    lart_msgs::msg::SlamStats stats_msg;
    stats_msg.cones_count_all = map_cones_msg_.cones.size();
    stats_msg.cones_count_current = msg->cones.size();
    stats_msg.lap_count = this->graph_slam_solver_->get_lap();
    this->slam_stats_publisher_->publish(stats_msg);
}

void GraphSLAM_Node::dynamics_callback(const lart_msgs::msg::Dynamics::SharedPtr msg){
    RCLCPP_DEBUG(this->get_logger(), "Received dynamics message");
    this->graph_slam_solver_->process_dynamics(msg);
}

void GraphSLAM_Node::imu_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg){
    RCLCPP_DEBUG(this->get_logger(), "Received IMU angular velocity message");
    this->graph_slam_solver_->set_angular_velocity(msg);
}

void GraphSLAM_Node::mission_callback(const lart_msgs::msg::Mission::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "Received mission update: %d", msg->data);
    this->graph_slam_solver_->set_mission(msg);
}

lart_msgs::msg::ConeArray GraphSLAM_Node::create_map_markers(visualization_msgs::msg::MarkerArray cones){
    
    lart_msgs::msg::ConeArray map_cones_msg;
    for (const auto& marker : cones.markers) {
        lart_msgs::msg::Cone cone;
        cone.position.x = marker.pose.position.x;
        cone.position.y = marker.pose.position.y;
        cone.cone_id.data = marker.id;
        if(marker.color.r == 1.0f && marker.color.g == 1.0f && marker.color.b == 0.0f){
            cone.class_type.data = lart_msgs::msg::Cone::YELLOW;
        } else if(marker.color.r == 0.0f && marker.color.g == 0.0f && marker.color.b == 1.0f){
            cone.class_type.data = lart_msgs::msg::Cone::BLUE;
        } else if(marker.color.r == 1.0f && marker.color.g == 0.5f && marker.color.b == 0.0f){
            cone.class_type.data = lart_msgs::msg::Cone::ORANGE_SMALL;
        } else if(marker.color.r == 1.0f && marker.color.g == 0.2f && marker.color.b == 0.0f){
            cone.class_type.data = lart_msgs::msg::Cone::ORANGE_BIG;
        }else {
            cone.class_type.data = lart_msgs::msg::Cone::UNKNOWN;
        }
        map_cones_msg.cones.push_back(cone);
    }
    return map_cones_msg;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GraphSLAM_Node>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}