#include "graph_slam/types_graph_slam.h"

#include "g2o/core/factory.h"

namespace g2o {

    bool VertexLandmark2D::read(std::istream& is) {
        if (!VertexPointXY::read(is)) {
            return false;
        }
        is >> color_;
        return is.good();
    }

    bool VertexLandmark2D::write(std::ostream& os) const {
        if (!VertexPointXY::write(os)) {
            return false;
        }
        os << " " << color_;
        return os.good();
    }

    G2O_REGISTER_TYPE_GROUP(graph_slam);

    G2O_REGISTER_TYPE_NAME("VertexLandmark2D", VertexLandmark2D);

}