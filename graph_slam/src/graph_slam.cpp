#include "graph_slam/graph_slam.hpp"

using std::placeholders::_1;
using namespace g2o;

GraphSLAM::GraphSLAM() : Node("graph_slam_node")
{
    RCLCPP_INFO(this->get_logger(), "GraphSLAM node has been started.");

    association_solver_ = new AssociationSolver(ASSOCIATION_MODE);

    observations_subscriber_ = this->create_subscription<lart_msgs::msg::ConeArray>(
        CONES_TOPIC, 10,
        std::bind(&GraphSLAM::observations_callback, this, _1));

    
}

GraphSLAM::~GraphSLAM()
{
    delete association_solver_;
    RCLCPP_INFO(this->get_logger(), "GraphSLAM node has been terminated.");
}

void GraphSLAM::observations_callback(const lart_msgs::msg::ConeArray::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received ConeArray with %zu cones.", msg->cones.size());

    // TODO : replace placeholders with real values
    geometry_msgs::msg::PoseStamped current_pose_ = geometry_msgs::msg::PoseStamped(); 
    lart_msgs::msg::ConeArray map_cones_ = lart_msgs::msg::ConeArray();

    const auto matches = association_solver_->associate(*msg, map_cones_, current_pose_);

    (void)matches; // To avoid unused variable warning for now
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GraphSLAM>());
    rclcpp::shutdown();
    return 0;
}