#include "kalman_filter.h"

namespace Practice {

void KalmanFilter::Init() {}

void KalmanFilter::Predict() {
  // 状态预测 x_k = F*x_k-1 + B*u_k + w_k
  state_ = transition_f_ * state_ + process_noise_wk_;
  // 协方差预测 P_k = F*P_k-1*F' + Q
  covariance_ = transition_f_ * covariance_ * transition_f_.transpose() +
                process_noise_wk_;
}

void KalmanFilter::Update() {
  // 计算卡尔曼增益 K_k = P_k*H^T*(H*P_k*H^T + Q)^-1
  kalman_gain_ = covariance_ * observation_h_.transpose() *
                 (observation_h_ * covariance_ * observation_h_.transpose() +
                  measurement_noise_vk_)
                     .inverse();
  // 更新状态 x_k = x_k + K_k*(z_k - H*x_k)
  state_ = state_ + kalman_gain_ * (observe_state_ - observation_h_ * state_);
  // 更新协方差 P_k = (I - K_k*H)*P_k
  covariance_ = (Eigen::MatrixXd::Identity(state_.size(), state_.size()) -
                 kalman_gain_ * observation_h_) *
                covariance_;
}

void KalmanFilter::Process() {
  Init();
  if (data_.empty()) {
    return;
  }

  for (const Eigen::VectorXd &measurement : data_) {
    observe_state_ = measurement;
    Predict();
    Update();
    output_.push_back(state_);
  }
}

} // namespace Practice