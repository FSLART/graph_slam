#ifndef CUSTOM_TYPES_HPP_
#define CUSTOM_TYPES_HPP_

#include <Eigen/Dense>

namespace graph_slam_types
{
struct Cone
{
  long id{-1};
  float x{0.0f};
  float y{0.0f};
  int type{0};
  Eigen::Matrix2d information{Eigen::Matrix2d::Identity()};
  void calculate_information(){
    const double base_depth_uncertainty_ = 0.1; // Base longitudinal uncertainty in meters
    const double base_lateral_uncertainty_ = 0.05; // Base lateral uncertainty in meters
    const double k_depth = 0.03;  //longitudinal uncertainty
    const double k_lateral = 0.02; //lateral uncertainty
    const double depth_weight = 1.5; //exponential weight for depth uncertainty
    double d = std::sqrt(x*x + y*y);

    Eigen::Matrix2d information = Eigen::Matrix2d::Identity();

    double sigma_x = k_depth * std::pow(d, depth_weight) + base_depth_uncertainty_;
    double sigma_y = k_lateral * std::pow(d, 1.15) + base_lateral_uncertainty_;

    //Testing if this works
    double info_x = 1.0 / (sigma_x * sigma_x);
    double info_y = 1.0 / (sigma_y * sigma_y);

    info_x = std::max(info_x, 1e-6);
    info_y = std::max(info_y, 1e-6);

    information(0,0) = info_x;
    information(1,1) = info_y;

    this->information = information;

  }
};

}  // namespace graph_slam_types

#endif  // CUSTOM_TYPES_HPP_