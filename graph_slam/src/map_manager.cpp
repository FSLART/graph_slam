#include "graph_slam/map_manager.hpp"

using namespace g2o;

MapManager::MapManager()
{
}

long MapManager::load_map(const std::string& filename, SparseOptimizer& optimizer_)
{
    long landmark_id_counter = -1;
 
    Node config = LoadFile(filename);
    // loads the whole track node
    Node track_node = config["track"];

    std::vector<std::string> categories = {"left", "right", "unknown"};

    for (const std::string& cat : categories) {
        if (!track_node[cat]) continue; 

        const Node& cone_list = track_node[cat];
        
        for (size_t i = 0; i < cone_list.size(); ++i) {
            // Correctly access the nested 'position' and 'class' keys
            double x = cone_list[i]["position"][0].as<double>();
            double y = cone_list[i]["position"][1].as<double>();
            std::string color = cone_list[i]["class"].as<std::string>();

            // Skip 'invisible' markers if they exist in your YAML
            if (color == "invisible") continue;

            uint8_t color_id = getColorID(color);

            VertexLandmark2D* landmark = new VertexLandmark2D();
            landmark->setId(++landmark_id_counter);
            landmark->setEstimate(Eigen::Vector2d(x, y));
            landmark->setColor(color_id);
            landmark->setFixed(false);
            optimizer_.addVertex(landmark);
        }
    }

    return landmark_id_counter;
}

void MapManager::save_map(const int mission, SparseOptimizer& optimizer_)
{
    std::string package_name = "graph_slam"; // Change to your actual package name
    std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
    
    std::filesystem::path maps_dir = std::filesystem::path(package_share_directory) / "maps";

    if (!std::filesystem::exists(maps_dir)) {
        std::filesystem::create_directories(maps_dir);
    }

    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);

    std::stringstream ss;
    ss << std::put_time(&now_tm, "%Y-%m-%d_%H-%M"); 
    std::string date_hour_minute = ss.str();
    
    std::string file_name = "mission_" + std::to_string(mission) + "_" + date_hour_minute + "_map.yaml";
    std::filesystem::path full_path = maps_dir / file_name;

    YAML::Emitter out;

    // Start Mapping
    out << YAML::BeginMap;
    out << YAML::Key << "track";
    out << YAML::Value << YAML::BeginMap;

    // Version
    out << YAML::Key << "version";
    out << YAML::Value << "1.0";

    // Start Section
    out << YAML::Key << "start";
    out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "position" << YAML::Value << YAML::Flow << std::vector<double>{0.0, 0.0, 0.0};
    out << YAML::Key << "orientation" << YAML::Value << YAML::Flow << std::vector<double>{0.0, 0.0, 0.0};
    out << YAML::EndMap;

    // Define colors to ensure they exist in the YAML even if empty
    std::map<std::string, std::vector<YAML::Node>> landmarks;
    landmarks["left"] = {};
    landmarks["right"] = {};
    landmarks["unknown"] = {};

    // Populate landmarks from Optimizer
    const auto &verts_map = optimizer_.vertices();
    for (const auto &kv : verts_map) {
        auto *v_landmark = dynamic_cast<VertexLandmark2D*>(kv.second);
        if (v_landmark) {
            const Eigen::Vector2d &est = v_landmark->estimate();
            
            std::string color_str;

            switch (v_landmark->color()) {

                case lart_msgs::msg::Cone::YELLOW:
                    color_str = "yellow";
                    break;
                case lart_msgs::msg::Cone::BLUE:
                    color_str = "blue";
                    break;
                case lart_msgs::msg::Cone::ORANGE_SMALL:
                    color_str = "small-orange";
                    break;
                case lart_msgs::msg::Cone::ORANGE_BIG:
                    color_str = "big-orange";
                    break;
                default:
                    color_str = "unknown";
            }
            YAML::Node cone;
            cone["position"] = std::vector<double>{est[0], est[1], 0.0};
            cone["position"].SetStyle(YAML::EmitterStyle::Flow);
            cone["class"] = color_str;
            cone.SetStyle(YAML::EmitterStyle::Block);
            
            landmarks["unknown"].push_back(cone);
        }
    }

    // Write landmarks to the emitter
    for (auto const& [color, nodes] : landmarks) {
        out << YAML::Key << color;
        out << YAML::Value << YAML::BeginSeq;
        for (const auto& node : nodes) {
            out << node;
        }
        out << YAML::EndSeq;
    }

    out << YAML::EndMap; // End track
    out << YAML::EndMap; // End root

    // Write to file
    std::ofstream fout(full_path);
    fout << out.c_str();
    fout.close();
}

uint8_t MapManager::getColorID(const std::string& class_str) {
    static const std::unordered_map<std::string, uint8_t> color_map = {
        {"yellow",       lart_msgs::msg::Cone::YELLOW},
        {"blue",         lart_msgs::msg::Cone::BLUE},
        {"small-orange", lart_msgs::msg::Cone::ORANGE_SMALL},
        {"big-orange",   lart_msgs::msg::Cone::ORANGE_BIG},
        {"invisible",    lart_msgs::msg::Cone::UNKNOWN} // Or handle as a specific case
    };

    auto it = color_map.find(class_str);
    if (it != color_map.end()) {
        return it->second;
    }
    return lart_msgs::msg::Cone::UNKNOWN;
}