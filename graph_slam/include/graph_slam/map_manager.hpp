#ifndef MAP_MANAGER_HPP_
#define MAP_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include "yaml-cpp/yaml.h"

#include <g2o/core/sparse_optimizer.h>
#include "graph_slam/types_graph_slam.h"
#include "graph_slam/custom_types.hpp"
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>

#include "lart_msgs/msg/cone.hpp"

using namespace g2o;
using namespace YAML;
class MapManager
{
public:
    MapManager();
    ~MapManager();
    static long load_map(const std::string& filename, SparseOptimizer& optimizer_);
    static void save_map(const std::string& filename, SparseOptimizer& optimizer_);
    static uint8_t getColorID(const std::string& class_str);
};

#endif // MAP_MANAGER_HPP_