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

    SparseOptimizerTerminateAction* terminate_action = new SparseOptimizerTerminateAction();

    //Set stop criteria for optimization
    terminate_action->setMaxIterations(10);
    terminate_action->setGainThreshold(1e-4);
    optimizer_.addPostIterationAction(terminate_action);

    // Enable verbose output for debugging
    optimizer_.setVerbose(true);
    
    VertexSE2* initial_pose = new VertexSE2();
    initial_pose->setId(pose_id_counter_);
    initial_pose->setEstimate(SE2(0, 0, 0));
    initial_pose->setFixed(true); // Fix the initial pose to anchor the graph
    optimizer_.addVertex(initial_pose);

}

GraphSLAM::~GraphSLAM()
{
    RCLCPP_INFO(this->get_logger(), "average processing time per ConeArray: %.3f ms", time_sum_ / observation_count_);
    this->optimizer_.save("final_graph.g2o");

    const auto &verts = optimizer_.vertices();
    std::vector<std::pair<int, VertexLandmark2D*>> landmarks_to_remove;
    landmarks_to_remove.reserve(verts.size());
    for (const auto &kv : verts) {
        auto *v_landmark = dynamic_cast<VertexLandmark2D*>(kv.second);
        if (v_landmark && v_landmark->edges().size() < 4) {
            landmarks_to_remove.emplace_back(kv.first, v_landmark);
        }
    }

    for (const auto &item : landmarks_to_remove) {
        const int vertex_id = item.first;
        VertexLandmark2D* v_landmark = item.second;
        this->optimizer_.removeVertex(v_landmark);
        RCLCPP_DEBUG(this->get_logger(), "Removed landmark vertex ID %d due to insufficient constraints.", vertex_id);
    }

    this->optimizer_.initializeOptimization();
    const int iterations = this->optimizer_.optimize(10);
    RCLCPP_INFO(this->get_logger(), "Graph optimization finished (%d iterations).", iterations);
    this->optimizer_.save("optimized_graph.g2o");
    delete association_solver_;
    RCLCPP_INFO(this->get_logger(), "GraphSLAM node has been terminated.");
}

void GraphSLAM::observations_callback(const lart_msgs::msg::ConeArray::SharedPtr msg)
{
    auto start_time = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(this->get_logger(), "Received ConeArray with %zu cones.", msg->cones.size());
    this->observation_count_++;

    // TODO : replace placeholders with real values
    const long current_pose_id = pose_id_counter_;
    const auto robot_pose_ =this->current_pose_; 
    g2o::OptimizableGraph::Vertex* v_pose = optimizer_.vertex(current_pose_id);
    const auto &verts = optimizer_.vertices();
    if (v_pose){
        lart_msgs::msg::ConeArray map_cones_ = lart_msgs::msg::ConeArray();
        for (const auto &kv : verts) {
            auto *v_landmark = dynamic_cast<VertexLandmark2D*>(kv.second);
            if (!v_landmark) {
                continue; // skip non-landmark vertices
            }
            const Eigen::Vector2d &est = v_landmark->estimate();

            double d = (est - Eigen::Vector2d(robot_pose_[0], robot_pose_[1])).norm();
            if (d > 15) {
                continue; // Skip landmarks that are too far away, likely outliers
            }
    
            lart_msgs::msg::Cone cone;
            cone.position.x = est[0];
            cone.position.y = est[1];
            cone.position.z = 0.0;
            cone.class_type.data = v_landmark->color();
            cone.cone_id.data = v_landmark->id();
            map_cones_.cones.push_back(cone);
        }
        
        pair<vector<int>, lart_msgs::msg::ConeArray> association_result = association_solver_->associate(*msg, map_cones_, robot_pose_);
        
        const auto matches = association_result.first;
        const auto obs_global = association_result.second;
    
        for (size_t i = 0; i < msg->cones.size(); ++i){
            long landmark_id = -1;
            double x = msg->cones[i].position.x;
            double y = msg->cones[i].position.y;
            double d = std::sqrt(x*x + y*y);

            if (d > 10 )
                continue; // Skip observations that are too far away, likely outliers

            if (matches[i] != -1){
                landmark_id= matches[i];
    
                dynamic_cast<VertexLandmark2D*>(optimizer_.vertex(landmark_id))->setEstimate(Eigen::Vector2d(obs_global.cones[i].position.x, obs_global.cones[i].position.y));
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
    
            Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
    
    
            double sigma_x = k_depth * std::pow(d, depth_weight) + base_depth_uncertainty_;
            double sigma_y = k_lateral * d + base_lateral_uncertainty_;
    
            information(0, 0) = (1.0 / (sigma_x * sigma_x))/2; // Inverse of sigma_x^2
            information(1, 1) = (1.0 / (sigma_y * sigma_y))/2; // Inverse of sigma_y^2
    
    
            EdgeSE2PointXY* edge = new EdgeSE2PointXY();
            edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(current_pose_id)));//use the last pose inserted
            edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(landmark_id)));
            edge->setMeasurement(Eigen::Vector2d(msg->cones[i].position.x, msg->cones[i].position.y));

            // RCLCPP_INFO(this->get_logger(), "information matrix [[%.4f, 0], [0, %.4f]]", information(0, 0), information(1, 1));
            edge->setInformation(information); // Use the computed information matrix

            this->optimizer_.addEdge(edge);
        }
    }else {
        RCLCPP_WARN(this->get_logger(), "Current pose vertex not found in the graph. Probably no pose initialized.");
    }

    // //print all landmarks in the graph
    // RCLCPP_INFO(this->get_logger(), "Current landmarks in the graph:");
    // for (const auto &kv : verts) {
    //     auto *v_landmark = dynamic_cast<VertexLandmark2D*>(kv.second);
    //     if (v_landmark) {
    //         const Eigen::Vector2d &est = v_landmark->estimate();
    //         RCLCPP_INFO(this->get_logger(), "Landmark ID: %d, Position: (%.2f, %.2f), Color: %d", v_landmark->id(), est[0], est[1], v_landmark->color());
    //     }
    // }
    auto end_time = std::chrono::steady_clock::now();
    auto duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    time_sum_ += duration_ms;
    RCLCPP_INFO(this->get_logger(), "Processing ConeArray took %.3f ms.", duration_ms);

}

void GraphSLAM::dynamics_callback(const lart_msgs::msg::Dynamics::SharedPtr msg)
{
    if (frame_count_ % 5 != 0) {
        frame_count_++;
        return; // Skip this callback to reduce frequency
    }
    frame_count_ ++;
    float current_rpm = (float)msg->rpm;
    float ms_speed = TIRE_PERIMETER_M * (current_rpm / TRANSMISSION_RATIO / 60.0);
    this->velocity_ = ms_speed;

    tuple<double, double, double> deltas = this->compute_predicted_pose(this->velocity_, this->angular_velocity_); // Assuming velocity is 0 for prediction, can be replaced with actual velocity if available
    
    VertexSE2* current_pose_vertex = dynamic_cast<VertexSE2*>(optimizer_.vertex(pose_id_counter_));

    VertexSE2* new_pose_vertex =  new VertexSE2();
    new_pose_vertex->setId(++pose_id_counter_);
    new_pose_vertex->setEstimate(SE2(current_pose_[0], current_pose_[1], current_pose_[2]));
    optimizer_.addVertex(new_pose_vertex);

    EdgeSE2* odom_edge = new EdgeSE2();
    odom_edge->setVertex(0, current_pose_vertex);
    odom_edge->setVertex(1, new_pose_vertex);
    odom_edge->setMeasurement(SE2(get<0>(deltas), get<1>(deltas), get<2>(deltas)));
    odom_edge->setInformation(Eigen::Matrix3d::Identity()*120);
    optimizer_.addEdge(odom_edge);
    RCLCPP_DEBUG(this->get_logger(), "Received Dynamics message: %f", ms_speed);

}

void GraphSLAM::imu_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
    this->angular_velocity_ = msg->vector.z;
    
    RCLCPP_DEBUG(this->get_logger(), "Received IMU angular velocity message: %f", this->angular_velocity_);
}

tuple<double,double,double> GraphSLAM::compute_predicted_pose(float velocity, float omega_z)
{
    auto now = chrono::steady_clock::now();
    if (last_predict_time_.time_since_epoch().count() == 0) {
        last_predict_time_ = now;
        return make_tuple(0.0, 0.0, 0.0); // No movement on the first call
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

    return make_tuple(dx, dy, w * dt);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<GraphSLAM>());
    rclcpp::shutdown();
    return 0;
}