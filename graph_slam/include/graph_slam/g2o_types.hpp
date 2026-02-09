#ifndef GRAPH_SLAM_G2O_TYPES_HPP_
#define GRAPH_SLAM_G2O_TYPES_HPP_

#include <g2o/types/slam2d/types_slam2d.h>

namespace graph_slam
{
using VertexPose2D = g2o::VertexSE2;

using EdgeObservation2D = g2o::EdgeSE2PointXY;

using EdgeOdometry2D = g2o::EdgeSE2;

class VertexLandmark2D : public g2o::VertexPointXY
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexLandmark2D() : color_(0) {}

  void setColor(int c) { color_ = c; }
  int color() const { return color_; }

private:
  int color_;
};

} // namespace graph_slam

#endif // GRAPH_SLAM_G2O_TYPES_HPP_
