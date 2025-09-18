#pragma once
#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <vector>

namespace Practice {
class ExtendedKalmanFilter {
public:
  explicit ExtendedKalmanFilter(const std::vector<Eigen::Vector2d> &data)
      : data_(data) {};
  ~ExtendedKalmanFilter() = default;

  void Process();
  const std::vector<Eigen::Vector2d> &GetResult() const { return output_; }

private:
  void Init();
  void Predict();
  void Update(const Eigen::Vector2d &point);
  void UpdateRecentPoints(const Eigen::Vector2d &point);
  double EstimateCurvature();

private:
  // Extended Kalman Filter components
  // x_k = f(x_k-1, u_k) + w_k
  // z_k = h(x_k) + v_k
  std::vector<Eigen::Vector2d> data_;         // 存储数据
  Eigen::Vector3d state_;                     // 状态向量
  Eigen::Matrix3d covariance_;                // 协方差矩阵
  Eigen::Matrix3d transition_f_;              // 状态转移矩阵
  Eigen::Matrix<double, 2, 3> observation_h_; // 观测矩阵
  Eigen::Matrix3d process_noise_wk_;          // 过程噪声协方差
  Eigen::Matrix2d measurement_noise_vk_;      // 测量噪声协方差
  std::vector<Eigen::Vector2d> output_;       // 输出结果

  std::vector<Eigen::Vector2d> recent_points_;
  size_t window_size_;
  double current_x_;
};
} // namespace Practice

#endif