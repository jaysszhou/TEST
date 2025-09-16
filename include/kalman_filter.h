#pragma once
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include "base_type.h"
#include <Eigen/Dense>
#include <vector>

namespace Practice {

class KalmanFilter {
public:
  KalmanFilter(const std::vector<Eigen::VectorXd> &data,
               const Practice::RansacModelParams fit_result)
      : data_(data), fit_result_(fit_result) {};
  ~KalmanFilter() = default;

  void Process();
  const std::vector<Eigen::VectorXd> &GetResult() const { return output_; }

private:
  void Init();
  void Predict();
  void Update();

private:
  // Kalman Filter components
  // x_k = f(x_k-1, u_k) + w_k
  // z_k = h(x_k) + v_k
  std::vector<Eigen::VectorXd> data_;      // 存储数据
  Practice::RansacModelParams fit_result_; // RANSAC拟合结果
  Eigen::VectorXd state_;                  // 状态向量
  Eigen::VectorXd observe_state_;          // 观测向量
  Eigen::MatrixXd covariance_;             // 协方差矩阵
  Eigen::MatrixXd transition_f_;           // 状态转移矩阵
  Eigen::MatrixXd observation_h_;          // 观测矩阵
  Eigen::MatrixXd process_noise_wk_;       // 过程噪声协方差
  Eigen::MatrixXd measurement_noise_vk_;   // 测量噪声协方差
  Eigen::MatrixXd kalman_gain_;            // 卡尔曼增益
  std::vector<Eigen::VectorXd> output_;    // 输出结果
};

} // namespace Practice
#endif