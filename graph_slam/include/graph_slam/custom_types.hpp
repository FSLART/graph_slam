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

    double sigma_depth = k_depth * std::pow(d, depth_weight) + base_depth_uncertainty_;
    double sigma_lat   = k_lateral * std::pow(d, 1.15) + base_lateral_uncertainty_;

    // 2. Convert to Covariance (Variance)
    double var_depth = sigma_depth * sigma_depth;
    double var_lat   = sigma_lat * sigma_lat;

    // 3. Construct Information Matrix
    // If x and y are in the sensor's local polar frame:
    Eigen::Matrix2d obs_cov = Eigen::Matrix2d::Zero();
    obs_cov(0,0) = var_depth; 
    obs_cov(1,1) = var_lat;

    // 4. Invert to get Information
    // .inverse() is safe here because variances are clamped/positive
    this->information = obs_cov.inverse();
  }
};

}  // namespace graph_slam_types

#endif  // CUSTOM_TYPES_HPP_