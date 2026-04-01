#ifndef WBR_CONTROLLER_HPP_
#define WBR_CONTROLLER_HPP_

#include <librm.hpp>

using namespace rm;

namespace wbr {

// 当前机体状态：包含系统所有状态与腿长变量
struct CurrentState {
  f32 s;            // 直线位移
  f32 s_dot;        // 直线速度
  f32 phi;          // 偏航角
  f32 phi_dot;      // 偏航角速度
  f32 theta_ll;     // 左腿摆杆与竖直方向夹角
  f32 theta_ll_dot; // 左腿摆角速度
  f32 theta_lr;     // 右腿摆杆与竖直方向夹角
  f32 theta_lr_dot; // 右腿摆角速度
  f32 theta_b;      // 机体俯仰角
  f32 theta_b_dot;  // 机体俯仰角速度

  // 当前腿长数据（由五连杆机构正运动学计算得出）
  f32 l_l; // 左腿腿长
  f32 l_r; // 右腿腿长
};

// 状态向量期望
struct ExpectedState {
  f32 s;
  f32 s_dot;
  f32 phi;
  f32 phi_dot;
  f32 theta_ll;
  f32 theta_ll_dot;
  f32 theta_lr;
  f32 theta_lr_dot;
  f32 theta_b;
  f32 theta_b_dot;
};

// 输出控制扭矩
struct MotorTorque {
  f32 t_wl; // 左驱动轮输出力矩
  f32 t_wr; // 右驱动轮输出力矩
  f32 t_bl; // 左侧髋关节输出力矩（需要随后映射到五连杆关节电机）
  f32 t_br; // 右侧髋关节输出力矩（需要随后映射到五连杆关节电机）
};

// 轮腿机器人控制器核心类
class WbrController {
public:
  WbrController();
  ~WbrController() = default;

  // 设置拟合出的 LQR K矩阵系数，40行6列
  // 由 MATLAB 的拟合程序生成离线数据
  void SetLqrCoefficients(const std::vector<std::array<f32, 6>> &coeff_matrix);

  // 核心控制计算：在此带入当前机器人的机体状态和期望状态，解算出控制输出
  [[nodiscard]] MotorTorque ComputeControl(const CurrentState &current,
                                           const ExpectedState &expected) const;

private:
  std::vector<std::array<f32, 6>>
      k_coeffs_; // 40个系数序列，对应 4x10 的 K 矩阵

  // 求多项式的运行函数：p(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y +
  // p02*y^2
  static f32 EvaluatePolynomial(const std::array<f32, 6> &p, f32 l_l, f32 l_r);

  // 依据当前系统腿长，更新实时的 LQR 反馈矩阵 K (4行 x 10列)
  void ComputeKMatrix(f32 l_l, f32 l_r, f32 k_matrix[4][10]) const;
};

} // namespace wbr

#endif // WBR_CONTROLLER_HPP_