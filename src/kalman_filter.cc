#include "kalman_filter.h"

namespace Practice {
namespace {
constexpr double kProcessNoise = 1e-2;     // 过程噪声
constexpr double kMeasurementNoise = 1e-1; // 测量噪声
constexpr double kEpsilon = 1e-6;          // 数值稳定性
} // namespace

void KalmanFilter::Init() {
  state_ = Eigen::Vector3d::Zero(); // [pos_x, pos_y, curvevature]
  covariance_ = Eigen::Matrix3d::Identity() * 100;
  process_noise_wk_ = Eigen::Matrix3d::Identity() * kProcessNoise;
  measurement_noise_vk_ = Eigen::Matrix2d::Identity() * kMeasurementNoise;
  observation_h_ = Eigen::Matrix<double, 2, 3>::Zero();
  observation_h_(0, 0) = 1;
  observation_h_(1, 1) = 1;

  window_size_ = 5;
  current_x_ = 0.0;
  recent_points_.clear();
  output_.clear();
}

void KalmanFilter::Predict() {
  // 状态预测 x_k = F*x_k-1 + B*u_k + w_k
  // 协方差预测 P_k = F*P_k-1*F' + Q
  const double curvature = state_(2);
  transition_f_ = Eigen::Matrix3d::Identity();
  transition_f_(0, 2) = curvature * 0.1;
  transition_f_(1, 2) = curvature * 0.1;

  state_ = transition_f_ * state_;
  covariance_ = transition_f_ * covariance_ * transition_f_.transpose() +
                process_noise_wk_;
}

void KalmanFilter::Update(const Eigen::Vector2d &point) {
  UpdateRecentPoints(point);
  // 计算卡尔曼增益 K_k = P_k*H^T*(H*P_k*H^T + Q)^-1
  const Eigen::Matrix<double, 3, 2> kalman_gain =
      covariance_ * observation_h_.transpose() *
      (observation_h_ * covariance_ * observation_h_.transpose() +
       measurement_noise_vk_)
          .inverse();
  // 更新状态 x_k = x_k + K_k*(z_k - H*x_k)
  state_ = state_ + kalman_gain * (point - observation_h_ * state_);
  // 更新协方差 P_k = (I - K_k*H)*P_k
  covariance_ = (Eigen::Matrix3d::Identity() - kalman_gain * observation_h_) *
                covariance_;
  state_(2) = EstimateCurvature();
}

double KalmanFilter::EstimateCurvature() {
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
  double a = (p2 - p1).norm();
  double b = (p3 - p2).norm();
  double c = (p1 - p3).norm();
  double curvature = (4 * area) / (a * b * c + kEpsilon);
  return curvature;
}

void KalmanFilter::UpdateRecentPoints(const Eigen::Vector2d &point) {
  recent_points_.emplace_back(point);
  if (recent_points_.size() > window_size_) {
    recent_points_.erase(recent_points_.begin());
  }
}

void KalmanFilter::Process() {
  Init();
  if (data_.empty()) {
    return;
  }
  state_ = Eigen::Vector3d(data_[0](0), data_[0](1), 0.0);
  output_.emplace_back(state_.head<2>());
  UpdateRecentPoints(data_[0]);

  for (size_t idx = 1; idx < data_.size(); ++idx) {
    current_x_ = data_[idx](0);
    Predict();
    Update(data_[idx]);
    output_.emplace_back(state_.head<2>());
  }
}

} // namespace Practice