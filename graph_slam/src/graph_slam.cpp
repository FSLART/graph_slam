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

    const auto matches = association_solver_->associate(*msg, map_cones_, current_pose_);

    for (std::size_t i = 0; i < msg->cones.size(); ++i){
        if (matches[i] != -1){
            VertexLandmark2D* landmark = new VertexLandmark2D();
            landmark->setId(landmark_id_counter_++);
            landmark->setEstimate(Eigen::Vector2d(msg->cones[i].position.x, msg->cones[i].position.y));
            landmark->setColor(msg->cones[i].class_type.data);
            this->optimizer_.addVertex(landmark);
            RCLCPP_DEBUG(this->get_logger(), "Observation %zu associated with map cone %d.", i, matches[i]);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Observation %zu is a new cone.", i);
        }
    }

}

void GraphSLAM::dynamics_callback(const lart_msgs::msg::Dynamics::SharedPtr msg)
{
    float current_rpm = (float)msg->rpm;
    float ms_speed = TIRE_PERIMETER_M * (current_rpm / TRANSMISSION_RATIO / 60.0);
    RCLCPP_DEBUG(this->get_logger(), "Received Dynamics message: %f", ms_speed);

}

void GraphSLAM::imu_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
    this->angular_velocity_ = msg->vector.z;
    RCLCPP_DEBUG(this->get_logger(), "Received IMU angular velocity message: %f", this->angular_velocity_);
}

void GraphSLAM::compute_predicted_pose(float velocity, float omega_z)
{
    auto now = std::chrono::steady_clock::now();
    if (last_predict_time_.time_since_epoch().count() == 0) {
        last_predict_time_ = now;
        return;
    }

    double dt = std::chrono::duration<double>(now - last_predict_time_).count();
    last_predict_time_ = now;

    double v = static_cast<double>(velocity);
    double w = static_cast<double>(omega_z);
    double theta = current_pose_[2];

    double dx = 0.0;
    double dy = 0.0;
    if (std::abs(w) > 0.01) {
        dx = -(v / w) * std::sin(theta) + (v / w) * std::sin(theta + w * dt);
        dy =  (v / w) * std::cos(theta) - (v / w) * std::cos(theta + w * dt);
    } else {
        dx = v * std::cos(theta) * dt;
        dy = v * std::sin(theta) * dt;
    }

    current_pose_[0] += dx;
    current_pose_[1] += dy;
    current_pose_[2] += w * dt;

    // Normalize angle to [-pi, pi]
    current_pose_[2] = std::atan2(std::sin(current_pose_[2]), std::cos(current_pose_[2]));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GraphSLAM>());
    rclcpp::shutdown();
    return 0;
}