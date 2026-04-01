#include "wbr/wbr_controller.hpp"

namespace wbr {

// 构造函数
WbrController::WbrController() {
  // 初始化 40 行，6列，预留空间（默认全为 0）
  k_coeffs_.resize(40, {0, 0, 0, 0, 0, 0});
}

// 供主程序写入 MATLAB 提取的 LQR 系数数据
void WbrController::SetLqrCoefficients(
    const std::vector<std::array<f32, 6>> &coeff_matrix) {
  if (coeff_matrix.size() == 40) {
    k_coeffs_ = coeff_matrix;
  }
}

// 根据腿长参数，对二维二次多项式函数求值
// p(l_l,l_r) = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
f32 WbrController::EvaluatePolynomial(const std::array<f32, 6> &p, f32 l_l,
                                      f32 l_r) {
  f32 p00 = p[0];
  f32 p10 = p[1];
  f32 p01 = p[2];
  f32 p20 = p[3];
  f32 p11 = p[4];
  f32 p02 = p[5];

  return p00 + p10 * l_l + p01 * l_r + p20 * l_l * l_l + p11 * l_l * l_r +
         p02 * l_r * l_r;
}

// 解算实时的 4x10 LQR 反馈系数矩阵
void WbrController::ComputeKMatrix(f32 l_l, f32 l_r,
                                   f32 k_matrix[4][10]) const {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 10; ++j) {
      // 系数在一维中按行主序平铺展开（与 MATLAB 提取一致）
      int index = i * 10 + j;
      k_matrix[i][j] = EvaluatePolynomial(k_coeffs_[index], l_l, l_r);
    }
  }
}

// 核心解算：计算控制输入
MotorTorque WbrController::ComputeControl(const CurrentState &current,
                                          const ExpectedState &expected) const {
  // 1. 调用腿长作为变量，实时解算系统对应的 LQR K 连续矩阵
  f32 k_matrix[4][10];
  ComputeKMatrix(current.l_l, current.l_r, k_matrix);

  // 2. 将当前机器人状态减去参考目标状态，获得偏差向量 X_err = X - X_ref
  f32 x_err[10];
  x_err[0] = current.s - expected.s;
  x_err[1] = current.s_dot - expected.s_dot;
  x_err[2] = current.phi - expected.phi;
  x_err[3] = current.phi_dot - expected.phi_dot;
  x_err[4] = current.theta_ll - expected.theta_ll;
  x_err[5] = current.theta_ll_dot - expected.theta_ll_dot;
  x_err[6] = current.theta_lr - expected.theta_lr;
  x_err[7] = current.theta_lr_dot - expected.theta_lr_dot;
  x_err[8] = current.theta_b - expected.theta_b;
  x_err[9] = current.theta_b_dot - expected.theta_b_dot;

  // 3. 求解输入向量 U = -K * x_err
  // U依次对应 t_wl, t_wr, t_bl, t_br
  f32 u_vec[4] = {0, 0, 0, 0};
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 10; ++j) {
      u_vec[i] -= k_matrix[i][j] * x_err[j];
    }
  }

  // 4. 将数学表示映射回输出接口结构体
  MotorTorque torque{};
  torque.t_wl = u_vec[0]; // 左驱动轮输出力矩
  torque.t_wr = u_vec[1]; // 右驱动轮输出力矩
  torque.t_bl = u_vec[2]; // 左侧髋关节输出力矩（自然坐标系下算得）
  torque.t_br = u_vec[3]; // 右腿髋关节输出力矩（自然坐标系下算得）

  return torque;
}

} // namespace wbr