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

void MapManager::save_map(const std::string& filename, SparseOptimizer& optimizer_)
{
    // Implement map saving logic here
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