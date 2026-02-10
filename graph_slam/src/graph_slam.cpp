#include "graph_slam/graph_slam.hpp"

using std::placeholders::_1;
using namespace g2o;
using namespace std;

GraphSLAM::GraphSLAM() : Node("graph_slam_node")
{
    RCLCPP_INFO(this->get_logger(), "GraphSLAM node has been started.");

    association_solver_ = new AssociationSolver(ASSOCIATION_MODE);

    // Subscribe to the cone observations topic
    observations_subscriber_ = this->create_subscription<lart_msgs::msg::ConeArray>(
        CONES_TOPIC, 10,
        bind(&GraphSLAM::observations_callback, this, _1));

    // Subscribe to the dynamics topic
    dynamics_subscriber_ = this->create_subscription<lart_msgs::msg::Dynamics>(
        DYNAMICS_TOPIC, 10,
        bind(&GraphSLAM::dynamics_callback, this, _1));

    //Subscribe to angular velocity topic 
    imu_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        IMU_TOPIC, 10,
        bind(&GraphSLAM::imu_callback, this, _1));
    
    auto linearSolver = std::make_unique<SlamLinearSolver>();

    OptimizationAlgorithmGaussNewton* solver =
      new OptimizationAlgorithmGaussNewton(
          std::make_unique<SlamBlockSolver>(move(linearSolver)));
    
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
    const long current_pose_id = pose_id_counter_;
    const auto robot_pose_ =this->current_pose_; 
    lart_msgs::msg::ConeArray map_cones_ = lart_msgs::msg::ConeArray();
    const auto &verts = optimizer_.vertices();
    for (const auto &kv : verts) {
        auto *v_landmark = dynamic_cast<VertexLandmark2D*>(kv.second);
        if (!v_landmark) {
            continue; // skip non-landmark vertices
        }

        const Eigen::Vector2d &est = v_landmark->estimate();
        lart_msgs::msg::Cone cone;
        cone.position.x = est[0];
        cone.position.y = est[1];
        cone.position.z = 0.0;
        cone.class_type.data = v_landmark->color();
        map_cones_.cones.push_back(cone);
    }
    
    pair<vector<int>, lart_msgs::msg::ConeArray> association_result = association_solver_->associate(*msg, map_cones_, robot_pose_);
    
    const auto matches = association_result.first;
    const auto obs_global = association_result.second;

    for (size_t i = 0; i < msg->cones.size(); ++i){
        long landmark_id = -1;
        if (matches[i] != -1){
            landmark_id= matches[i];

            RCLCPP_DEBUG(this->get_logger(), "Observation %zu associated with map cone %d.", i, matches[i]);
        } else {
            VertexLandmark2D* landmark = new VertexLandmark2D();
            landmark->setId(++landmark_id_counter_);
            landmark->setEstimate(Eigen::Vector2d(obs_global.cones[i].position.x, obs_global.cones[i].position.y));
            landmark->setColor(msg->cones[i].class_type.data);
            this->optimizer_.addVertex(landmark);

            landmark_id = landmark_id_counter_;

            RCLCPP_DEBUG(this->get_logger(), "Observation %zu is a new cone.", i);
        }

        EdgeSE2PointXY* edge = new EdgeSE2PointXY();
        edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(current_pose_id)));//use the last pose inserted
        edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(landmark_id)));
        edge->setMeasurement(Eigen::Vector2d(msg->cones[i].position.x, msg->cones[i].position.y));
        edge->setInformation(Eigen::Matrix2d::Identity()); // Placeholder information matrix
        this->optimizer_.addEdge(edge);
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
    auto now = chrono::steady_clock::now();
    if (last_predict_time_.time_since_epoch().count() == 0) {
        last_predict_time_ = now;
        return;
    }

    double dt = chrono::duration<double>(now - last_predict_time_).count();
    last_predict_time_ = now;

    double v = static_cast<double>(velocity);
    double w = static_cast<double>(omega_z);
    double theta = current_pose_[2];

    double dx = 0.0;
    double dy = 0.0;
    if (abs(w) > 0.01) {
        dx = -(v / w) * sin(theta) + (v / w) * sin(theta + w * dt);
        dy =  (v / w) * cos(theta) - (v / w) * cos(theta + w * dt);
    } else {
        dx = v * cos(theta) * dt;
        dy = v * sin(theta) * dt;
    }

    current_pose_[0] += dx;
    current_pose_[1] += dy;
    current_pose_[2] += w * dt;

    // Normalize angle to [-pi, pi]
    current_pose_[2] = atan2(sin(current_pose_[2]), cos(current_pose_[2]));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<GraphSLAM>());
    rclcpp::shutdown();
    return 0;
}