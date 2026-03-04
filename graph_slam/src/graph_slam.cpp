#include "graph_slam/graph_slam.hpp"

using std::placeholders::_1;
using namespace g2o;
using namespace std;

GraphSLAM::GraphSLAM() : Node("graph_slam_node")
{
    RCLCPP_INFO(this->get_logger(), "GraphSLAM node has been started.");

    this->current_mission_.data = lart_msgs::msg::Mission::MANUAL;

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

    mission_subscriber_ = this->create_subscription<lart_msgs::msg::Mission>(
        "/mission", 10,
        bind(&GraphSLAM::mission_callback, this, _1));

    slam_stats_publisher_ = this->create_publisher<lart_msgs::msg::SlamStats>("/slam/stats", 10);
    
    map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/slam/map", 10);
    
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

    
    VertexSE2* initial_pose = new VertexSE2();
    initial_pose->setId(pose_id_counter_);
    initial_pose->setFixed(true); // Fix the initial pose to anchor the graph
    initial_pose->setEstimate(SE2(0, 0, 0));
    this->current_pose_ = Eigen::Vector3d(0, 0, 0);
    
    optimizer_.addVertex(initial_pose);
    
    // Enable verbose output for debugging
    optimizer_.setVerbose(true);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&GraphSLAM::broadcast_transform, this));
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

    //Commented out for online use

    // this->optimizer_.initializeOptimization();
    // const int iterations = this->optimizer_.optimize(10);
    // RCLCPP_INFO(this->get_logger(), "Graph optimization finished (%d iterations).", iterations);
    // this->optimizer_.save("optimized_graph.g2o");

    delete association_solver_;
    RCLCPP_INFO(this->get_logger(), "GraphSLAM node has been terminated.");
}

void GraphSLAM::broadcast_transform()
{
    if (optimizer_.vertices().empty()) {
        return; // No vertices in the graph, skip broadcasting
    }

    VertexSE2* current_pose_vertex = dynamic_cast<VertexSE2*>(optimizer_.vertex(pose_id_counter_));
    if (!current_pose_vertex) {
        RCLCPP_WARN(this->get_logger(), "Current pose vertex not found in the graph. Cannot broadcast transform.");
        return;
    }

    SE2 pose_estimate = current_pose_vertex->estimate();
    
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->get_clock()->now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "base_footprint";
    transformStamped.transform.translation.x = pose_estimate.translation()[0];
    transformStamped.transform.translation.y = pose_estimate.translation()[1];
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, pose_estimate.rotation().angle());
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transformStamped);
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
                this->new_vertices.insert(landmark); // Add new landmark vertex for update bookeeping
    
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
            this->new_edges.insert(edge); // Add new edge for update bookkeeping
        }
        // Update current pose estimate for next iteration
        update_graph(this->new_vertices, this->new_edges);

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
    // RCLCPP_INFO(this->get_logger(), "Processing ConeArray took %.3f ms.", duration_ms);
    this->check_lap_completion();
    RCLCPP_INFO(this->get_logger(), "Current pose: (%.2f, %.2f, %.2f), Lap: %d", current_pose_[0], current_pose_[1], current_pose_[2], current_lap_);

    const auto &verts_map = optimizer_.vertices();
    visualization_msgs::msg::MarkerArray map_markers_;
    for (const auto &kv : verts_map) {
        auto *v_landmark = dynamic_cast<VertexLandmark2D*>(kv.second);
        if (v_landmark) {
            const Eigen::Vector2d &est = v_landmark->estimate();
    
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = this->get_clock()->now();
            marker.header.frame_id = "world";
            marker.ns = "graph_slam";
            marker.id = v_landmark->id();
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.lifetime = rclcpp::Duration(0, 500000000); // Marker will last for 0.5 seconds
            marker.pose.position.x = est[0];
            marker.pose.position.y = est[1];
            marker.pose.position.z = 0.0;
            marker.scale.x = 0.2; // Diameter of the sphere
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0; // Fully opaque
            // Set color based on cone class type
            switch (v_landmark->color()) {
                case 1: // Blue cone
                    marker.color.r = 0.0f;
                    marker.color.g = 0.0f;
                    marker.color.b = 1.0f;
                    break;
                case 2: // Yellow cone
                    marker.color.r = 1.0f;
                    marker.color.g = 1.0f;
                    marker.color.b = 0.0f;
                    break;
                case 3: // Orange cone
                    marker.color.r = 1.0f;
                    marker.color.g = 0.5f; // Orange is a mix of red and yellow
                    marker.color.b = 0.0f;
                    break;
                default:
                    // Default to white if unknown color type
                    marker.color.r = 1.0f;
                    marker.color.g = 1.0f;
                    marker.color.b = 1.0f;
            }
            map_markers_.markers.push_back(marker);
        }
    }
    map_publisher_->publish(map_markers_);
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
    this->new_vertices.insert(new_pose_vertex); // Add new pose vertex for update bookkeeping

    EdgeSE2* odom_edge = new EdgeSE2();
    odom_edge->setVertex(0, current_pose_vertex);
    odom_edge->setVertex(1, new_pose_vertex);
    odom_edge->setMeasurement(SE2(get<0>(deltas), get<1>(deltas), get<2>(deltas)));
    odom_edge->setInformation(Eigen::Matrix3d::Identity()*120);
    optimizer_.addEdge(odom_edge);
    this->new_edges.insert(odom_edge); // Add new edge for update bookkeeping
    
    //Update graph with new odom information
    update_graph(this->new_vertices, this->new_edges);

    RCLCPP_DEBUG(this->get_logger(), "Received Dynamics message: %f", ms_speed);

}

void GraphSLAM::imu_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
    this->angular_velocity_ = msg->vector.z;
    RCLCPP_DEBUG(this->get_logger(), "Received IMU angular velocity message: %f", this->angular_velocity_);
}

void GraphSLAM::mission_callback(const lart_msgs::msg::Mission::SharedPtr msg)
{
    if(!mission_set_){
        this->current_mission_.data = msg->data;
        mission_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Mission set to %d", this->current_mission_.data);
    } else {
        RCLCPP_WARN(this->get_logger(), "Mission already set. Ignoring new mission message.");
    }
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

    float distance_delta = sqrt(dx*dx + dy*dy);
    this->current_lap_distance_ += distance_delta;

    current_pose_[0] += dx;
    current_pose_[1] += dy;
    current_pose_[2] += w * dt;

    // Normalize angle to [-pi, pi]
    current_pose_[2] = atan2(sin(current_pose_[2]), cos(current_pose_[2]));

    return make_tuple(dx, dy, w * dt);
}

void GraphSLAM::check_lap_completion()
{
    if ((this->current_lap_distance_ < lap_margin_ && this->current_lap_ != -1) || !this->mission_set_) {
        return; // you ain't got no motion
    }

    float x = current_pose_[0];
    float y = current_pose_[1];
    float theta = current_pose_[2];

    bool lap_completed = false;

     // Check if we are close to the starting line (e.g., within 1 meter)
    switch (current_mission_.data){
        case lart_msgs::msg::Mission::ACCELERATION:
            if (abs(x-75.0) < lap_margin_x_) {
                lap_completed = true;
            }
        break;
        case lart_msgs::msg::Mission::SKIDPAD:
            if (abs(x-15.0) < lap_margin_x_ && abs(y) < lap_margin_y_) {
                lap_completed = true;
            }
        break;
        case lart_msgs::msg::Mission::AUTOCROSS:
        case lart_msgs::msg::Mission::TRACKDRIVE:
            if (abs(x-6.0) < lap_margin_x_ && abs(y) < lap_margin_y_) {
                lap_completed = true;
            }
        break;
    }

    if (lap_completed) {
        this->current_lap_++;
        this->current_lap_distance_ = 0.0; // Reset distance for the next lap
        if (this->current_lap_ == 1) {
            // TODO: call full optimization after the first lap is completed
        }
    }
}

void GraphSLAM::update_graph(g2o::HyperGraph::VertexSet vset, g2o::HyperGraph::EdgeSet eset){

    if(eset.size() < 9){
        return; // Not enough new information to warrant an update
    }

    if(!this->initialized_once){
        this->optimizer_.initializeOptimization(0);
        this->optimizer_.optimize(10); // TODO: tune the number of iterations for the initial optimization
        this->initialized_once = true;
        return; 
    }

    // Preform a partial update
    this->optimizer_.updateInitialization(vset, eset);
    this->optimizer_.optimize(5); // TODO: tune the number of iterations for

    // Clear the sets after the update
    this->new_vertices.clear();
    this->new_edges.clear();

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<GraphSLAM>());
    rclcpp::shutdown();
    return 0;
}