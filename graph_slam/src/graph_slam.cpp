#include "graph_slam/graph_slam.hpp"

using std::placeholders::_1;
using namespace g2o;

GraphSLAM::GraphSLAM() : Node("graph_slam_node")
{
    RCLCPP_INFO(this->get_logger(), "GraphSLAM node has been started.");

    association_solver_ = new AssociationSolver(ASSOCIATION_MODE);

    // Subscribe to the cone observations topic
    observations_subscriber_ = this->create_subscription<lart_msgs::msg::ConeArray>(
        CONES_TOPIC, 10,
        std::bind(&GraphSLAM::observations_callback, this, _1));

    // Subscribe to the dynamics topic
    dynamics_subscriber_ = this->create_subscription<lart_msgs::msg::Dynamics>(
        DYNAMICS_TOPIC, 10,
        std::bind(&GraphSLAM::dynamics_callback, this, _1));

    //Subscribe to angular velocity topic 
    imu_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        IMU_TOPIC, 10,
        std::bind(&GraphSLAM::imu_callback, this, _1));
    
    auto linearSolver = std::make_unique<SlamLinearSolver>();

    OptimizationAlgorithmGaussNewton* solver =
      new OptimizationAlgorithmGaussNewton(
          std::make_unique<SlamBlockSolver>(std::move(linearSolver)));
    
    optimizer_.setAlgorithm(solver);
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

    // const auto matches = association_solver_->associate(*msg, map_cones_, current_pose_);
    std::pair<std::vector<int>, lart_msgs::msg::ConeArray> association_result = association_solver_->associate(*msg, map_cones_, current_pose_);
    
    const auto matches = association_result.first;
    const auto obs_global = association_result.second;

    optimizer_.vertices();

    for (std::size_t i = 0; i < msg->cones.size(); ++i){
        if (matches[i] != -1){
            VertexLandmark2D* landmark = new VertexLandmark2D();
            landmark->setId(++landmark_id_counter_);
            landmark->setEstimate(Eigen::Vector2d(obs_global.cones[i].position.x, obs_global.cones[i].position.y));
            landmark->setColor(msg->cones[i].class_type.data);
            this->optimizer_.addVertex(landmark);

            EdgeSE2* edge = new EdgeSE2();
            edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(pose_id_counter_)));
            edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(landmark_id_counter_)));
            edge->setMeasurement(g2o::SE2(msg->cones[i].position.x, msg->cones[i].position.y, 0.0)); // Assuming zero orientation for simplicity
            edge->setInformation(Eigen::Matrix3d::Identity()); // Placeholder information matrix
            this->optimizer_.addEdge(edge); 

            RCLCPP_DEBUG(this->get_logger(), "Observation %zu associated with map cone %d.", i, matches[i]);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Observation %zu is a new cone.", i);
        }
    }

}

void GraphSLAM::dynamics_callback(const lart_msgs::msg::Dynamics::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received Dynamics message.");
    // TODO: Implement dynamics callback
    (void)msg; // To avoid unused parameter warning
}

void GraphSLAM::imu_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received IMU angular velocity message.");
    // TODO: Implement IMU callback
    (void)msg; // To avoid unused parameter warning
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GraphSLAM>());
    rclcpp::shutdown();
    return 0;
}