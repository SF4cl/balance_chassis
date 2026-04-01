#include "chassis.hpp"

using namespace rm;
using namespace rm::modules;
using namespace rm::device;
using namespace wbr;

Chassis::Chassis(f32 l1, f32 l2) : left_leg_(l1, l2), right_leg_(l1, l2) {}

void Chassis::Init() {
  // 初始化四个达妙关节电机
  lf_joint_ = std::make_shared<DmMotor<DmMotorControlMode::kMit>>();
  lb_joint_ = std::make_shared<DmMotor<DmMotorControlMode::kMit>>();
  rf_joint_ = std::make_shared<DmMotor<DmMotorControlMode::kMit>>();
  rb_joint_ = std::make_shared<DmMotor<DmMotorControlMode::kMit>>();

  // 初始化两个轮毂电机
  left_wheel_ = std::make_shared<M3508>();
  right_wheel_ = std::make_shared<M3508>();
}

wbr::MotorTorque Chassis::ComputeTorques(const wbr::ExpectedState &expected,
                                         f32 imu_pitch, f32 imu_pitch_dot,
                                         f32 imu_yaw, f32 imu_yaw_dot) {
  // 1. 更新双腿运动学
  if (lf_joint_ && lb_joint_) {
    left_leg_.SetPhi1(lf_joint_->GetPos());
    left_leg_.SetPhi4(lb_joint_->GetPos());
    left_leg_.SetWPhi1(lf_joint_->GetVel());
    left_leg_.SetWPhi4(lb_joint_->GetVel());
    left_leg_.Update();
  }

  if (rf_joint_ && rb_joint_) {
    right_leg_.SetPhi1(rf_joint_->GetPos());
    right_leg_.SetPhi4(rb_joint_->GetPos());
    right_leg_.SetWPhi1(rf_joint_->GetVel());
    right_leg_.SetWPhi4(rb_joint_->GetVel());
    right_leg_.Update();
  }

  // 2. 更新当前机体状态
  current_state_.theta_b = imu_pitch;
  current_state_.theta_b_dot = imu_pitch_dot;
  current_state_.phi = imu_yaw;
  current_state_.phi_dot = imu_yaw_dot;

  current_state_.l_l = left_leg_.l0();
  current_state_.l_r = right_leg_.l0();

  // 使用运动学解算出的摆角
  current_state_.theta_ll = left_leg_.beta();
  current_state_.theta_ll_dot = left_leg_.beta_dot();
  current_state_.theta_lr = right_leg_.beta();
  current_state_.theta_lr_dot = right_leg_.beta_dot();

  // 简单估计直线速度
  if (left_wheel_ && right_wheel_) {
    current_state_.s_dot =
        (left_wheel_->GetSpeed() + right_wheel_->GetSpeed()) / 2.0f;
  }

  // 3. 调用LQR/VMC解算器计算所需力矩
  wbr::MotorTorque base_torque =
      wbr_controller_.ComputeControl(current_state_, expected);

  // 返回所需力矩
  return base_torque;
}
