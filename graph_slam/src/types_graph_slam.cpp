#include "graph_slam/types_graph_slam.h"

#include "g2o/core/factory.h"

namespace g2o {
    G2O_REGISTER_TYPE_GROUP(graph_slam);

    G2O_REGISTER_TYPE_NAME("VertexLandmark2D", VertexLandmark2D);

}