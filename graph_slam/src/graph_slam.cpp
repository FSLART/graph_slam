#include "graph_slam/graph_slam.hpp"

using std::placeholders::_1;
using namespace g2o;
using namespace std;

GraphSLAM::GraphSLAM()
{
    // this->current_mission_.data = lart_msgs::msg::Mission::MANUAL;
    this->current_mission_.data = 6;
    this->mission_set_ = true;

    association_solver_ = new AssociationSolver(ASSOCIATION_MODE);

    auto linearSolver = std::make_unique<SlamLinearSolver>();

    OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(
    std::make_unique<SlamBlockSolver>(std::move(linearSolver)));
    
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
}

GraphSLAM::~GraphSLAM()
{
    RCLCPP_INFO(rclcpp::get_logger("graph_slam_solver"), "average processing time per ConeArray: %.3f ms", time_sum_ / observation_count_);
    this->optimizer_.save("final_graph.g2o");
    if(this->current_mission_.data == lart_msgs::msg::Mission::AUTOCROSS || this->current_mission_.data == lart_msgs::msg::Mission::TRACKDRIVE)
        MapManager::save_map(this->current_mission_.data, this->optimizer_);
    delete association_solver_;
    RCLCPP_INFO(rclcpp::get_logger("graph_slam_solver"), "GraphSLAM node has been terminated.");
}

void GraphSLAM::build_map_kdtree()
{

    // Caller must hold optimizer_mutex_.
    map_cloud_->clear();
    map_kdtree_landmarks_.clear();

    // Reserve necessary space
    map_cloud_->points.reserve(optimizer_.vertices().size());
    map_kdtree_landmarks_.reserve(optimizer_.vertices().size());

    for (const auto& [id, v] : optimizer_.vertices()) {
        auto* vl = dynamic_cast<VertexLandmark2D*>(v);
        if (!vl) {
            continue;
        }
        const Eigen::Vector2d& est = vl->estimate();
        map_cloud_->points.emplace_back(static_cast<float>(est.x()), static_cast<float>(est.y()), 0.0f);

        // prepare payload
        LandmarkKDInfo info;
        info.vertex_id = vl->id();
        info.color = vl->color();
        
        // save id+color
        map_kdtree_landmarks_.push_back(info);
    }

    map_cloud_->width = static_cast<uint32_t>(map_cloud_->points.size());
    map_cloud_->height = 1;
    map_cloud_->is_dense = true;

    if (map_cloud_->points.empty()) {
        map_kdtree_ready_ = false;
        RCLCPP_WARN(rclcpp::get_logger("graph_slam_solver"), "KD-tree build requested but map has 0 landmarks.");
        return;
    }

    map_kdtree_.setInputCloud(map_cloud_);
    map_kdtree_ready_ = true;
    RCLCPP_INFO(rclcpp::get_logger("graph_slam_solver"), "KD-tree built with %zu landmarks.", map_cloud_->points.size());
}

void GraphSLAM::localize_in_map(std::vector<graph_slam_types::Cone>& observations, long current_pose_id, Eigen::Vector3d robot_pose)
{
    if (!map_kdtree_ready_) {
        RCLCPP_WARN(rclcpp::get_logger("graph_slam_solver"), "Localization requested but KD-tree is not ready.");
        return;
    }

    // Testing to see what happens if the rate of update is slower
    if(this->count_locliz_updts_ < 10){
        this->count_locliz_updts_++;
        return;
    }
    this->count_locliz_updts_ = 0;

    // Transform observations to global frame
    std::vector<graph_slam_types::Cone> obs_global;

    for (const auto& obs : observations) {
        graph_slam_types::Cone global_obs;
        double cos_theta = std::cos(robot_pose[2]);
        double sin_theta = std::sin(robot_pose[2]);
        global_obs.x = robot_pose[0] + obs.x * cos_theta - obs.y * sin_theta;
        global_obs.y = robot_pose[1] + obs.x * sin_theta + obs.y * cos_theta;
        global_obs.type = obs.type;
        obs_global.push_back(global_obs);
    }

    // For each observation, find the nearest landmark in the map using the KD-tree
    std::vector<int> matches(observations.size(), -1); // Initialize all matches to -1 (no match)
    for (size_t i = 0; i < obs_global.size(); ++i) {
        const auto& obs = obs_global[i];
        pcl::PointXYZ search_point(static_cast<float>(obs.x), static_cast<float>(obs.y), 0.0f);

        std::vector<int> point_idx_nkn_search(1);
        std::vector<float> point_nkn_squared_distance(1);

        if (map_kdtree_.nearestKSearch(search_point, 1, point_idx_nkn_search, point_nkn_squared_distance) > 0) {

            int idx = point_idx_nkn_search[0];
            // const auto& matched_landmark_info = map_kdtree_landmarks_[idx];
            // matches[i] = static_cast<int>(matched_landmark_info.vertex_id);

            // Add edge between current pose and matched landmark
            g2o::EdgeSE2PointXY* edge = new g2o::EdgeSE2PointXY();

            {
                std::lock_guard<std::mutex> lock(optimizer_mutex_);
                edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(current_pose_id)));
                edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(map_kdtree_landmarks_[idx].vertex_id)));
            }

            // Set local measurement
            edge->setMeasurement(Eigen::Vector2d(observations[i].x, observations[i].y));
            observations[i].calculate_information(robot_pose[2]);
            edge->setInformation(observations[i].information);

            // Set robust kernel
            auto rk = new RobustKernelHuber();
            rk->setDelta(0.5);
            edge->setRobustKernel(rk);

            {
                std::lock_guard<std::mutex> lock(optimizer_mutex_);
                this->optimizer_.addEdge(edge);
            }

        } else {
            RCLCPP_DEBUG(rclcpp::get_logger("graph_slam_solver"), "Observation %zu could not be localized to any landmark.", i);

        }
    }

    // Preform optimization
    {
    std::lock_guard<std::mutex> lock(optimizer_mutex_);

    optimizer_.initializeOptimization();
    optimizer_.optimize(1);
}

}

Eigen::Vector3d GraphSLAM::get_current_pose()
{
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return current_pose_;
}

visualization_msgs::msg::MarkerArray GraphSLAM::process_observations(const lart_msgs::msg::ConeArray::SharedPtr msg)
{
    std::vector<graph_slam_types::Cone> not_added_observations;
    if(!is_robot_moving_){
        RCLCPP_DEBUG(rclcpp::get_logger("graph_slam_solver"), "Robot is stationary. Skipping ConeArray processing.");
        for (const auto& cone_msg : msg->cones) {
            graph_slam_types::Cone cone;
            cone.x = cone_msg.position.x;
            cone.y = cone_msg.position.y;
            cone.type = cone_msg.class_type.data;
            not_added_observations.push_back(cone);
        }
        return this->get_map(not_added_observations); // Skip processing if the robot is not moving
    }
    
    auto start_time = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(rclcpp::get_logger("graph_slam_solver"), "Received ConeArray with %zu cones.", msg->cones.size());
    this->observation_count_++;

    const long current_pose_id = pose_id_counter_;
    const auto robot_pose_ =this->current_pose_;
    g2o::OptimizableGraph::Vertex* v_pose = nullptr;
    g2o::OptimizableGraph::VertexIDMap verts;
    {
        std::lock_guard<std::mutex> lock(optimizer_mutex_);
        v_pose = optimizer_.vertex(current_pose_id);
        verts = optimizer_.vertices();
    }
    std::vector<graph_slam_types::Cone> map_cones_;
    std::vector<graph_slam_types::Cone> observations;
    if (v_pose){

        //get observation cones
        for (const auto& cone_msg : msg->cones) {
            graph_slam_types::Cone cone;
            cone.x = cone_msg.position.x;
            cone.y = cone_msg.position.y;
            cone.type = cone_msg.class_type.data;
            observations.push_back(cone);
        }

        // Use localization mode
        if (localization_mode_) {
            localize_in_map(observations, current_pose_id, robot_pose_);
            return final_map_;
        }

        // Get map cones
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

            g2o::EdgeSE2PointXY* most_recent_edge = nullptr;
            int max_pose_id = -1;
            if (v_landmark->edges().empty()) {
                continue; // Skip landmarks with no edges, as we have no information about their uncertainty
            }
            for (auto* edge_base : v_landmark->edges()) {
                // 1. Safely check the type
                auto* e_se2xy = dynamic_cast<g2o::EdgeSE2PointXY*>(edge_base);
                if (!e_se2xy) continue;

                // 2. The robot pose is usually vertex(0) in an EdgeSE2PointXY
                int current_pose_id = e_se2xy->vertex(0)->id();

                // 3. Keep the one with the highest ID (most recent in time)
                if (current_pose_id > max_pose_id) {
                    max_pose_id = current_pose_id;
                    most_recent_edge = e_se2xy;
                }
            }

            const Eigen::Matrix2d &info = most_recent_edge->information();
    
            graph_slam_types::Cone cone;
            cone.x = est[0];
            cone.y = est[1];
            cone.type = v_landmark->color();
            cone.id = v_landmark->id();
            cone.information = info;
            map_cones_.push_back(cone);
        }

        pair<vector<int>, std::vector<graph_slam_types::Cone>> association_result = association_solver_->associate(observations, map_cones_, robot_pose_);
        
        const auto matches = association_result.first;
        const auto obs_global = association_result.second;

    
        for (size_t i = 0; i < observations.size(); ++i){
            long landmark_id = -1;
            double x = observations[i].x;
            double y = observations[i].y;
            double d = std::sqrt(x*x + y*y);

            if (d > 10 ){
                not_added_observations.push_back(obs_global[i]);
                continue; // Skip observations that are too far away, likely outliers
            }


            if (matches[i] != -1){
                landmark_id= matches[i];
    
                // dynamic_cast<VertexLandmark2D*>(optimizer_.vertex(landmark_id))->setEstimate(Eigen::Vector2d(obs_global[i].x, obs_global[i].y));
                RCLCPP_DEBUG(rclcpp::get_logger("graph_slam_solver"), "Observation %zu associated with map cone %d.", i, matches[i]);
            } else {
                VertexLandmark2D* landmark = new VertexLandmark2D();
                landmark->setId(++landmark_id_counter_);
                landmark->setEstimate(Eigen::Vector2d(obs_global[i].x, obs_global[i].y));
                landmark->setColor(observations[i].type);
                {
                    std::lock_guard<std::mutex> lock(optimizer_mutex_);
                    this->optimizer_.addVertex(landmark);
                }
                this->new_vertices.insert(landmark); // Add new landmark vertex for update bookeeping
    
                landmark_id = landmark_id_counter_;
    
                RCLCPP_DEBUG(rclcpp::get_logger("graph_slam_solver"), "Observation %zu is a new cone.", i);
            }
    
            EdgeSE2PointXY* edge = new EdgeSE2PointXY();

            {
                std::lock_guard<std::mutex> lock(optimizer_mutex_);
                edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(current_pose_id)));//use the last pose inserted
                edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer_.vertex(landmark_id)));
            }
            edge->setMeasurement(Eigen::Vector2d(observations[i].x, observations[i].y));

            // RCLCPP_INFO(rclcpp::get_logger("graph_slam_solver"), "information matrix [[%.4f, 0], [0, %.4f]]", information(0, 0), information(1, 1));
            observations[i].calculate_information(robot_pose_[2]);
            edge->setInformation(observations[i].information); // Use the computed information matrix

            // Add robust kernel hereinitialized_once
            auto rk = new RobustKernelHuber();
            rk->setDelta(0.5);
            edge->setRobustKernel(rk);

            {
                std::lock_guard<std::mutex> lock(optimizer_mutex_);
                this->optimizer_.addEdge(edge);
            }

            this->new_edges.insert(edge); // Add new edge for update bookkeeping
        }
        if (ONLINE_FLAG){
            update_graph(this->new_vertices, this->new_edges);
        }

    }else {
        RCLCPP_WARN(rclcpp::get_logger("graph_slam_solver"), "Current pose vertex not found in the graph. Probably no pose initialized.");
    }
    auto end_time = std::chrono::steady_clock::now();
    auto duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    time_sum_ += duration_ms;
    RCLCPP_INFO(rclcpp::get_logger("graph_slam_solver"), "Processing ConeArray took %.3f ms.", duration_ms);
    this->check_lap_completion();
    RCLCPP_INFO(rclcpp::get_logger("graph_slam_solver"), "Current pose: (%.2f, %.2f, %.2f), Lap: %d", current_pose_[0], current_pose_[1], current_pose_[2], current_lap_);
    return this->get_map(not_added_observations);
}

void GraphSLAM::process_dynamics(const lart_msgs::msg::Dynamics::SharedPtr msg)
{
    float current_rpm = (float)msg->rpm;
    float ms_speed = RPM_TO_MS(current_rpm);
    this->velocity_ = ms_speed;

    RCLCPP_DEBUG(rclcpp::get_logger("graph_slam_solver"), "Received Dynamics message: %f", ms_speed);
}

void GraphSLAM::set_angular_velocity(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
    this->angular_velocity_ = msg->vector.z;
    RCLCPP_DEBUG(rclcpp::get_logger("graph_slam_solver"), "Received IMU angular velocity message: %f", this->angular_velocity_);
}

void GraphSLAM::set_mission(const lart_msgs::msg::Mission::SharedPtr msg)
{
    if(!mission_set_){
        this->current_mission_.data = msg->data;
        mission_set_ = true;
        RCLCPP_INFO(rclcpp::get_logger("graph_slam_solver"), "Mission set to %d", this->current_mission_.data);
        if (this->current_mission_.data == lart_msgs::msg::Mission::SKIDPAD) {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("graph_slam");
            std::string map_path = package_share_dir + SKIDPAD_MAP;
            {
                std::lock_guard<std::mutex> lock(optimizer_mutex_);
                landmark_id_counter_ = MapManager::load_map(map_path, this->optimizer_);
                RCLCPP_INFO(rclcpp::get_logger("graph_slam_solver"), "Skidpad map loaded with %zu vertices.", this->optimizer_.vertices().size());
            }
        }
        if (this->current_mission_.data == lart_msgs::msg::Mission::ACCELERATION)
            this->current_lap_++ ;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("graph_slam_solver"), "Mission already set. Ignoring new mission message.");
    }
}


void GraphSLAM::compute_predicted_pose()
{
    if (this->velocity_ == 0.0 && !this->is_robot_moving_){
        return;
    }

    is_robot_moving_ = true;

    auto now = chrono::steady_clock::now();
    if (last_predict_time_.time_since_epoch().count() == 0) {
        last_predict_time_ = now;
        return;
    }

    double dt = chrono::duration<double>(now - last_predict_time_).count();
    last_predict_time_ = now;

    double v = static_cast<double>(this->velocity_);
    double w = static_cast<double>(this->angular_velocity_);
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

    {
        std::lock_guard<std::mutex> lock(optimizer_mutex_);
        VertexSE2* current_pose_vertex = dynamic_cast<VertexSE2*>(optimizer_.vertex(pose_id_counter_));
        
        VertexSE2* new_pose_vertex =  new VertexSE2();
        new_pose_vertex->setId(++pose_id_counter_);
        new_pose_vertex->setEstimate(SE2(current_pose_[0], current_pose_[1], current_pose_[2]));
        optimizer_.addVertex(new_pose_vertex);
        this->new_vertices.insert(new_pose_vertex); // Add new pose vertex for update bookkeeping
        
        EdgeSE2* odom_edge = new EdgeSE2();
        odom_edge->setVertex(0, current_pose_vertex);
        odom_edge->setVertex(1, new_pose_vertex);
        odom_edge->setMeasurement(SE2(dx, dy, w * dt));
        odom_edge->setInformation(Eigen::Matrix3d::Identity()*35);
        this->optimizer_.addEdge(odom_edge);
        this->new_edges.insert(odom_edge); // Add new edge for update bookkeeping
    }
}

void GraphSLAM::check_lap_completion()
{
    if ((this->current_lap_distance_ < lap_margin_ && this->current_lap_ != -1) || !this->mission_set_) {
        return; // you ain't got no motion
    }

    float x = current_pose_[0];
    float y = current_pose_[1];

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
            this->localization_mode_ = true; 
            {
                std::lock_guard<std::mutex> lock(optimizer_mutex_);
                
                std::vector<VertexLandmark2D*> to_remove;
                for (const auto& [id, v] : optimizer_.vertices()) {
                    auto* vl = dynamic_cast<VertexLandmark2D*>(v);
                    //vl->setFixed(true);
                    if (vl && vl->edges().size() < 5)
                        to_remove.push_back(vl);
                }

                for (auto* vl : to_remove){
                    std::vector<g2o::OptimizableGraph::Edge*> edges_to_remove;
                    for (auto* eb : vl->edges()) {
                        edges_to_remove.push_back(
                            dynamic_cast<g2o::OptimizableGraph::Edge*>(eb));
                    }

                    for (auto* e : edges_to_remove) {
                        optimizer_.removeEdge(e);
                    }

                    optimizer_.removeVertex(vl);
                }
                // this->initialized_once = false; //FIXME : THIS DOES NOT SOLVE THE PROBLEM
                this->new_vertices.clear();
                this->new_edges.clear();
                this->optimizer_.initializeOptimization();
                this->optimizer_.optimize(1, false);
                this->optimizer_.save("final_graph.g2o");
                if(this->current_mission_.data == lart_msgs::msg::Mission::AUTOCROSS || this->current_mission_.data == lart_msgs::msg::Mission::TRACKDRIVE)
                    MapManager::save_map(this->current_mission_.data, this->optimizer_);

                
                // Make every landmark vertex fixed to prevent drift in localization mode
                for (const auto& [id, v] : optimizer_.vertices()) {
                    auto* vl = dynamic_cast<VertexLandmark2D*>(v);
                    if (vl) {
                        vl->setFixed(true);
                    }
                }

                // Build a KD-tree for the current map
                this->build_map_kdtree();

                RCLCPP_INFO(rclcpp::get_logger("graph_slam_solver"),"Localization_mode = %d",localization_mode_);
            }
            // Build and save in cache the final vizualization map
            auto empty_observations = std::vector<graph_slam_types::Cone>{};
            this->final_map_ = this->get_map(empty_observations);
            RCLCPP_INFO(rclcpp::get_logger("graph_slam_solver"), "1ºLAP COMPLETED AND OTHER STUFF !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        }
    }
}

void GraphSLAM::update_graph(g2o::HyperGraph::VertexSet& vset, g2o::HyperGraph::EdgeSet& eset)
{
    //RCLCPP_INFO(rclcpp::get_logger("graph_slam_solver"), "Only %zu new edges and %zu new vertices since last update. Skipping graph update.", eset.size(), vset.size());
    std::lock_guard<std::mutex> lock(optimizer_mutex_);

    if(!this->initialized_once){
        RCLCPP_INFO(rclcpp::get_logger("graph_slam_solver"), "Performing initial graph optimization with %zu vertices and %zu edges.",
                    optimizer_.vertices().size(), optimizer_.edges().size());

        this->optimizer_.initializeOptimization();
        this->optimizer_.optimize(10); // initial batch solve
        
        this->initialized_once = true;
        this->new_vertices.clear();
        this->new_edges.clear();
        return;
    }

    if(vset.size() < 60){
        return; // Not enough new information to warrant an update
    }

    optimizer_.updateInitialization(vset, eset);

    optimizer_.computeActiveErrors();
    double chi_before = optimizer_.activeChi2();

    optimizer_.optimize(1, true); // one incremental step each time

    optimizer_.computeActiveErrors();
    double chi_after = optimizer_.activeChi2();

    RCLCPP_INFO(rclcpp::get_logger("graph_slam_solver"),"chi2 before %.4f after %.4f",chi_before, chi_after);

    // Clear the sets after the update
    this->new_vertices.clear();
    this->new_edges.clear();

}

visualization_msgs::msg::MarkerArray GraphSLAM::get_map(std::vector<graph_slam_types::Cone> not_in_map_observations)
{
    g2o::OptimizableGraph::VertexIDMap verts_map;
    {
        RCLCPP_INFO(rclcpp::get_logger("graph_slam_solver"),"MIAUMIAU123?");
        std::lock_guard<std::mutex> lock(optimizer_mutex_);
        verts_map = optimizer_.vertices();
    }
    auto now = rclcpp::Clock().now();

    auto make_marker = [&](int id, double x, double y) {
        visualization_msgs::msg::Marker m;
        m.header.stamp    = now;
        m.header.frame_id = "world";
        m.ns              = "graph_slam";
        m.id              = id;
        m.type            = visualization_msgs::msg::Marker::SPHERE;
        m.action          = visualization_msgs::msg::Marker::ADD;
        m.lifetime = rclcpp::Duration(0, 500'000'000);
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 0.0;
        m.color.a         = 0.5f;
        return m;
    };

    static const std::unordered_map<uint8_t, std::array<float,3>> kConeColors = {
        { lart_msgs::msg::Cone::YELLOW,       {1.0f, 1.0f, 0.0f} },
        { lart_msgs::msg::Cone::BLUE,         {0.0f, 0.0f, 1.0f} },
        { lart_msgs::msg::Cone::ORANGE_SMALL, {1.0f, 0.5f, 0.0f} },
        { lart_msgs::msg::Cone::ORANGE_BIG,   {1.0f, 0.2f, 0.0f} },
    };
    
    visualization_msgs::msg::MarkerArray map_markers_;
    int id_counter = 1000;
    for (const auto& cone : not_in_map_observations) {
        auto m    = make_marker(id_counter++, cone.x, cone.y);
        m.scale.x = m.scale.y = 0.2;
        m.scale.z = 0.4;
        m.color.r = m.color.g = m.color.b = 0.6f;
        map_markers_.markers.push_back(std::move(m));
    }

    for (const auto &kv : verts_map) {
        auto *v_landmark = dynamic_cast<VertexLandmark2D*>(kv.second);
        if (v_landmark) {
            const Eigen::Vector2d &est = v_landmark->estimate();
            g2o::EdgeSE2PointXY* most_recent_edge = nullptr;
            int max_pose_id = -1;
            if (v_landmark->edges().empty()) {
                continue; // Skip landmarks with no edges, as we have no information about their uncertainty
            }
            for (auto* edge_base : v_landmark->edges()) {
                auto* e_se2xy = dynamic_cast<g2o::EdgeSE2PointXY*>(edge_base);
                if (!e_se2xy) continue;

                int current_pose_id = e_se2xy->vertex(0)->id();

                if (current_pose_id > max_pose_id) {
                    max_pose_id = current_pose_id;
                    most_recent_edge = e_se2xy;
                }
            }
            const Eigen::Matrix2d &info = most_recent_edge->information();
            Eigen::Matrix2d cov = info.inverse();

            auto m    = make_marker(v_landmark->id(), est[0], est[1]);
            m.scale.x = std::sqrt(cov(0,0)) * 2.0;
            m.scale.y = std::sqrt(cov(1,1)) * 2.0;
            m.scale.z = 0.4;

            auto it = kConeColors.find(v_landmark->color());
            const auto& rgb = (it != kConeColors.end()) ? it->second : std::array{1.f,1.f,1.f};
            m.color.r = rgb[0]; m.color.g = rgb[1]; m.color.b = rgb[2];

            map_markers_.markers.push_back(std::move(m));        
        }
    }
    return map_markers_;
}

