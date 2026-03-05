#ifndef CUSTOM_TYPES_HPP_
#define CUSTOM_TYPES_HPP_

namespace graph_slam_types
{
struct Cone
{
  long id{-1};
  float x{0.0f};
  float y{0.0f};
  int type{0};
};

}  // namespace graph_slam

#endif  // CUSTOM_TYPES_HPP_