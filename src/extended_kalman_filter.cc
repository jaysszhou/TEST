#include "extended_kalman_filter.h"
#include <Eigen/Dense>
#include <iostream>

namespace Practice {
namespace {
constexpr double kProcessNoise = 1e1;      // 过程噪声
constexpr double kMeasurementNoise = 1e-1; // 测量噪声
constexpr double kEpsilon = 1e-6;          // 数值稳定性
constexpr size_t kDefaultWindowSize = 5;
} // namespace

void ExtendedKalmanFilter::Init() {
  state_ = Eigen::Vector3d::Zero(); // [pos_x, pos_y, curvevature]
  covariance_ = Eigen::Matrix3d::Identity() * 100;
  transition_f_ = Eigen::Matrix3d::Identity();
  process_noise_wk_ = Eigen::Matrix3d::Identity() * kProcessNoise;
  measurement_noise_vk_ = Eigen::Matrix2d::Identity() * kMeasurementNoise;
  observation_h_ = Eigen::Matrix<double, 2, 3>::Zero();
  observation_h_(0, 0) = 1;
  observation_h_(1, 1) = 1;

  window_size_ = kDefaultWindowSize;
  current_x_ = 0.0;
  recent_points_.clear();
  output_.clear();
}

void ExtendedKalmanFilter::Predict() {
  // 状态预测 x_k = f(x_k-1, u_k) + w_k
  // 协方差预测 P_k = F*P_k-1*F' + Q
  state_ = transition_f_ * state_;
  covariance_ = transition_f_ * covariance_ * transition_f_.transpose() +
                process_noise_wk_;
}

void ExtendedKalmanFilter::Update(const Eigen::Vector2d &point) {
  UpdateRecentPoints(point);

  Eigen::Vector2d residual = point - observation_h_ * state_;
  Eigen::Matrix2d residual_covariance =
      observation_h_ * covariance_ * observation_h_.transpose() +
      measurement_noise_vk_;

  // 计算卡尔曼增益 K_k = P_k*H^T*(H*P_k*H^T + Q)^-1
  const Eigen::Matrix<double, 3, 2> kalman_gain =
      covariance_ * observation_h_.transpose() * residual_covariance.inverse();
  state_ = state_ + kalman_gain * residual;

  covariance_ = (Eigen::Matrix3d::Identity() - kalman_gain * observation_h_) *
                covariance_;

  state_(2) = EstimateCurvature();
}

double ExtendedKalmanFilter::EstimateCurvature() {
  if (recent_points_.size() < 3) {
    return 0.0;
  }
  // 使用最近的点进行二次曲线拟合，估计曲率
  int n = recent_points_.size();
  const Eigen::Vector2d &p1 = recent_points_[n - 3];
  const Eigen::Vector2d &p2 = recent_points_[n - 2];
  const Eigen::Vector2d &p3 = recent_points_[n - 1];

  const double area = std::abs((p2(0) - p1(0)) * (p3(1) - p1(1)) -
                               (p3(0) - p1(0)) * (p2(1) - p1(1))) /
                      2.0;
  if (area < kEpsilon) {
    return 0.0;
  }
  const double a = (p2 - p1).norm();
  const double b = (p3 - p2).norm();
  const double c = (p1 - p3).norm();
  const double curvature = (4 * area) / (a * b * c + kEpsilon);
  return curvature;
}

void ExtendedKalmanFilter::UpdateRecentPoints(const Eigen::Vector2d &point) {
  recent_points_.push_back(point);
  if (recent_points_.size() > window_size_) {
    recent_points_.erase(recent_points_.begin());
  }
}

void ExtendedKalmanFilter::Process() {
  Init();
  if (data_.empty()) {
    return;
  }
  output_.push_back(Eigen::Vector2d(state_(0), state_(1)));
  UpdateRecentPoints(data_[0]);
  current_x_ = data_[0](0);
  for (const auto &point : data_) {
    current_x_ = point(0);
    Predict();
    Update(point);
    output_.emplace_back(state_(0), state_(1));
  }
}

} // namespace Practice