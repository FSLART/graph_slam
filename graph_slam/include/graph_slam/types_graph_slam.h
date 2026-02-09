#ifndef TYPES_GRAPH_SLAM_H
#define TYPES_GRAPH_SLAM_H

#include <g2o/types/slam2d/vertex_point_xy.h>
#include "graph_slam/g2o_graph_slam_api.h"

namespace g2o {

class G2O_GRAPH_SLAM_API VertexLandmark2D : public VertexPointXY {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        VertexLandmark2D() {}

        int color() const { return color_; }
        void setColor(int color) { color_ = color; }
    private:
        int color_;

};

}

#endif