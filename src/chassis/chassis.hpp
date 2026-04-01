#ifndef BALANCE_CHASSIS_CHASSIS_HPP
#define BALANCE_CHASSIS_CHASSIS_HPP

#include "wbr/leg_kinematics.hpp"
#include "wbr/wbr_controller.hpp"
#include <librm.hpp>
#include <memory>

using namespace rm;
using namespace rm::modules;
using namespace rm::device;

class Chassis {
public:
  // 传入两个 LegKinematics 需要的连杆参数 l1 和 l2，这里给定默认值
  Chassis(f32 l1 = 0.15f, f32 l2 = 0.25f);
  ~Chassis() = default;

  // 初始化电机对象
  void Init();

  // 根据当前电机反馈、IMU数据和期望状态，综合进行运动学正解和控制解算，返回最终所需的电机力矩
  // 这个接口不执行下发动作，具体控制逻辑（例如下发CAN）由外部调用
  wbr::MotorTorque ComputeTorques(const wbr::ExpectedState &expected,
                                  f32 imu_pitch, f32 imu_pitch_dot, f32 imu_yaw,
                                  f32 imu_yaw_dot);

  // Getter for current internal state (useful for telemetry or debug)
  const wbr::CurrentState &GetCurrentState() const { return current_state_; }

  // 供外部获取电机指针以配置 CAN ID 等参数
  std::shared_ptr<DmMotor<DmMotorControlMode::kMit>> GetLfJoint() {
    return lf_joint_;
  }
  std::shared_ptr<DmMotor<DmMotorControlMode::kMit>> GetLbJoint() {
    return lb_joint_;
  }
  std::shared_ptr<DmMotor<DmMotorControlMode::kMit>> GetRfJoint() {
    return rf_joint_;
  }
  std::shared_ptr<DmMotor<DmMotorControlMode::kMit>> GetRbJoint() {
    return rb_joint_;
  }

  std::shared_ptr<M3508> GetLeftWheel() { return left_wheel_; }
  std::shared_ptr<M3508> GetRightWheel() { return right_wheel_; }

private:
  // 四个达妙关节电机智能指针
  std::shared_ptr<DmMotor<DmMotorControlMode::kMit>> lf_joint_; // 左前
  std::shared_ptr<DmMotor<DmMotorControlMode::kMit>> lb_joint_; // 左后
  std::shared_ptr<DmMotor<DmMotorControlMode::kMit>> rf_joint_; // 右前
  std::shared_ptr<DmMotor<DmMotorControlMode::kMit>> rb_joint_; // 右后

  // 两个轮毂电机智能指针
  std::shared_ptr<M3508> left_wheel_;  // 左轮
  std::shared_ptr<M3508> right_wheel_; // 右轮

  // 解算器与控制器
  wbr::WbrController wbr_controller_;
  wbr::LegKinematics left_leg_;
  wbr::LegKinematics right_leg_;

  // 当前机体的融合状态
  wbr::CurrentState current_state_{};
};

#endif // BALANCE_CHASSIS_CHASSIS_HPP
