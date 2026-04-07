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
  void calculate_information(double theta){
    const double base_depth_uncertainty_ = 0.1; // Base longitudinal uncertainty in meters
    const double base_lateral_uncertainty_ = 0.05; // Base lateral uncertainty in meters
    const double k_depth = 0.03;  //longitudinal uncertainty
    const double k_lateral = 0.02; //lateral uncertainty
    const double depth_weight = 1.5; //exponential weight for depth uncertainty
    double d = std::sqrt(x*x + y*y);

    // Eigen::Matrix2d information = Eigen::Matrix2d::Identity();

    double sigma_x = k_depth * std::pow(d, depth_weight) + base_depth_uncertainty_;
    double sigma_y = k_lateral * std::pow(d, 1.15) + base_lateral_uncertainty_;

    double v_d = sigma_x * sigma_x;
    double v_l = sigma_y * sigma_y;

    // 2. Get robot heading (theta) from the pose the edge originates from
    double c = cos(theta);
    double s = sin(theta);

    // 3. Rotate Covariance: R * Sigma_local * R^T
    double cov_xx = c * c * v_d + s * s * v_l;
    double cov_yy = s * s * v_d + c * c * v_l;
    double cov_xy = c * s * (v_d - v_l);

    Eigen::Matrix2d global_cov;
    global_cov << cov_xx, cov_xy,
                  cov_xy, cov_yy;

    this->information = global_cov.inverse();

  }
};

}  // namespace graph_slam_types

#endif  // CUSTOM_TYPES_HPP_