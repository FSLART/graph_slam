#include "graph_slam/graph_slam.hpp"

using std::placeholders::_1;

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

    association_solver_->associate(*msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GraphSLAM>());
    rclcpp::shutdown();
    return 0;
}