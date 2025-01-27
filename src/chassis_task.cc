
#include "chassis_task.hpp"

#include <usart.h>
#include <cstdint>
#include "bsp_dwt.h"
#include "imu_task.h"

using namespace rm;
using namespace rm::device;
using namespace rm::hal;
using namespace rm::modules::algorithm;

static Chassis *chassis;

static Serial *left_joint_rs485;
static Serial *right_joint_rs485;
static Serial *wheel_rs485;

static UnitreeMotor *lf_joint;
static UnitreeMotor *lb_joint;
static UnitreeMotor *rf_joint;
static UnitreeMotor *rb_joint;

static Go8010Motor *left_wheel;
static Go8010Motor *right_wheel;

extern BMI088 *imu;

static Serial *remote_uart;
static DR16 *remote;

static FirstOrderFilter left_wheel_vel_filter(1.f / 500.f, 0.04f);
static FirstOrderFilter right_wheel_vel_filter(1.f / 500.f, 0.04f);

static PID *wheel_stand_pid;
static PID *wheel_speed_pid;

static int count = 0;

f32 x_target_pre = 0.f;

Chassis::Chassis()
    : left_leg_vmc_(0.12f, 0.25f, 0.15f),
      right_leg_vmc_(0.12f, 0.25f, 0.15f),
      left_l0_pid_(1500.f, 400.f, 20000.f, 360.f, 100.f),
      right_l0_pid_(1500.f, 400.f, 20000.f, 360.f, 100.f),
      left_turn_pid_(10.f, 0.0f, 6.f, 10.f, 0.f),
      right_turn_pid_(10.f, 0.0f, 6.f, 10.f, 0.f),
      lf_pos_filter_(1.f / 500.f, 0.007f),
      lb_pos_filter_(1.f / 500.f, 0.007f),
      rf_pos_filter_(1.f / 500.f, 0.007f),
      rb_pos_filter_(1.f / 500.f, 0.007f),
      lf_spd_filter_(1.f / 500.f, 0.005f),
      lb_spd_filter_(1.f / 500.f, 0.005f),
      rf_spd_filter_(1.f / 500.f, 0.005f),
      rb_spd_filter_(1.f / 500.f, 0.005f),
      pitch_filter_(1.f / 500.f, 0.002f),
      pitch_dot_filter_(1.f / 500.f, 0.02f),
      yaw_dot_filter_(1.f / 500.f, 0.06f) {
  x_ = 0.0f;
  x_dot_ = 0.0f;

  A1_init_flag_ = true;
}

void Chassis::LegPosTrans() {
  left_leg_phi1_ = lb_pos_filter_.value() - lb_init_angle_ + 3.14159f + 0.3996f;
  left_leg_phi4_ = lf_pos_filter_.value() - lf_init_angle_ - 0.3996f;
  right_leg_phi1_ = -rb_pos_filter_.value() + rb_init_angle_ + 3.14159f + 0.3996f;
  right_leg_phi4_ = -rf_pos_filter_.value() + rf_init_angle_ - 0.3996f;

  left_leg_w_phi1_ = lb_spd_filter_.value();
  left_leg_w_phi4_ = lf_spd_filter_.value();
  right_leg_w_phi1_ = -rb_spd_filter_.value();
  right_leg_w_phi4_ = -rf_spd_filter_.value();
  Chassis::Param2Vmc();
}

void Chassis::Param2Vmc() {
  left_leg_vmc_.SetPhi1(left_leg_phi1_);
  left_leg_vmc_.SetPhi4(left_leg_phi4_);
  right_leg_vmc_.SetPhi1(right_leg_phi1_);
  right_leg_vmc_.SetPhi4(right_leg_phi4_);

  left_leg_vmc_.SetWPhi1(left_leg_w_phi1_);
  left_leg_vmc_.SetWPhi4(left_leg_w_phi4_);
  right_leg_vmc_.SetWPhi1(right_leg_w_phi1_);
  right_leg_vmc_.SetWPhi4(right_leg_w_phi4_);
}

void Chassis::StateTrans() {
  left_theta_ = -1.5708f - (pitch_ - left_leg_vmc_.phi0());
  right_theta_ = -1.5708f - (pitch_ - right_leg_vmc_.phi0());

  left_theta_dot_ = -(left_leg_vmc_.xc_dot() * left_leg_vmc_.yc() - left_leg_vmc_.xc() * left_leg_vmc_.yc_dot()) /
                        (left_leg_vmc_.l0() * left_leg_vmc_.l0()) -
                    pitch_dot_;
  right_theta_dot_ = -(right_leg_vmc_.xc_dot() * right_leg_vmc_.yc() - right_leg_vmc_.xc() * right_leg_vmc_.yc_dot()) /
                         (right_leg_vmc_.l0() * right_leg_vmc_.l0()) -
                     pitch_dot_;
}

void Chassis::Visual2Real() {
  lf_tau_ = left_leg_vmc_.jacobi()(0, 0) * left_force_ + left_leg_vmc_.jacobi()(0, 1) * (-left_tau_(1));
  lb_tau_ = left_leg_vmc_.jacobi()(1, 0) * left_force_ + left_leg_vmc_.jacobi()(1, 1) * (-left_tau_(1));

  rf_tau_ = right_leg_vmc_.jacobi()(0, 0) * right_force_ + right_leg_vmc_.jacobi()(0, 1) * (-right_tau_(1));
  rb_tau_ = right_leg_vmc_.jacobi()(1, 0) * right_force_ + right_leg_vmc_.jacobi()(1, 1) * (-right_tau_(1));

  // lf_tau_ = left_leg_vmc_.jacobi()(0, 0) * left_force_;
  // lb_tau_ = left_leg_vmc_.jacobi()(1, 0) * left_force_;

  // rf_tau_ = right_leg_vmc_.jacobi()(0, 0) * right_force_;
  // rb_tau_ = right_leg_vmc_.jacobi()(1, 0) * right_force_;

  // lf_tau_ = left_leg_vmc_.jacobi()(0, 1) * (-left_tau_(1));
  // lb_tau_ = left_leg_vmc_.jacobi()(1, 1) * (-left_tau_(1));

  // rf_tau_ = right_leg_vmc_.jacobi()(0, 1) * (-right_tau_(1));
  // rb_tau_ = right_leg_vmc_.jacobi()(1, 1) * (-right_tau_(1));

  lf_tau_ = -lf_tau_;
  lb_tau_ = -lb_tau_;
}

void Chassis::CalcTau() {
  left_tau_(0) = -(CalcPoly(left_leg_vmc_.l0(), 0, 0) * left_theta_ + CalcPoly(left_leg_vmc_.l0(), 0, 1) * left_theta_dot_ +
                   CalcPoly(left_leg_vmc_.l0(), 0, 2) * (x_ - x_target_) + CalcPoly(left_leg_vmc_.l0(), 0, 3) * x_dot_ +
                   CalcPoly(left_leg_vmc_.l0(), 0, 4) * pitch_ + CalcPoly(left_leg_vmc_.l0(), 0, 5) * pitch_dot_);
  left_tau_(1) = -(CalcPoly(left_leg_vmc_.l0(), 1, 0) * left_theta_ + CalcPoly(left_leg_vmc_.l0(), 1, 1) * left_theta_dot_ +
                   CalcPoly(left_leg_vmc_.l0(), 1, 2) * (x_ - x_target_) + CalcPoly(left_leg_vmc_.l0(), 1, 3) * x_dot_ +
                   CalcPoly(left_leg_vmc_.l0(), 1, 4) * pitch_ + CalcPoly(left_leg_vmc_.l0(), 1, 5) * pitch_dot_) +
                 (right_theta_ - left_theta_) * theta_kp - (right_theta_dot_ - left_theta_dot_) * theta_kd;

  right_tau_(0) = -(CalcPoly(right_leg_vmc_.l0(), 0, 0) * right_theta_ + CalcPoly(right_leg_vmc_.l0(), 0, 1) * right_theta_dot_ +
                    CalcPoly(right_leg_vmc_.l0(), 0, 2) * (x_ - x_target_) + CalcPoly(right_leg_vmc_.l0(), 0, 3) * x_dot_ +
                    CalcPoly(right_leg_vmc_.l0(), 0, 4) * pitch_ + CalcPoly(right_leg_vmc_.l0(), 0, 5) * pitch_dot_);
  right_tau_(1) = -(CalcPoly(right_leg_vmc_.l0(), 1, 0) * right_theta_ + CalcPoly(right_leg_vmc_.l0(), 1, 1) * right_theta_dot_ +
                    CalcPoly(right_leg_vmc_.l0(), 1, 2) * (x_ - x_target_) + CalcPoly(right_leg_vmc_.l0(), 1, 3) * x_dot_ +
                    CalcPoly(right_leg_vmc_.l0(), 1, 4) * pitch_ + CalcPoly(right_leg_vmc_.l0(), 1, 5) * pitch_dot_) -
                  (right_theta_ - left_theta_) * theta_kp + (right_theta_dot_ - left_theta_dot_) * theta_kd;

  // left_tau_(0) = -(CalcPoly(left_leg_vmc_.l0(), 0, 0) * left_theta_ +
  //                  CalcPoly(left_leg_vmc_.l0(), 0, 2) * (x_ - x_target_) + CalcPoly(left_leg_vmc_.l0(), 0, 3) * x_dot_ +
  //                  CalcPoly(left_leg_vmc_.l0(), 0, 4) * pitch_ + CalcPoly(left_leg_vmc_.l0(), 0, 5) * pitch_dot_);
  // left_tau_(1) = -(CalcPoly(left_leg_vmc_.l0(), 1, 0) * left_theta_ +
  //                  CalcPoly(left_leg_vmc_.l0(), 1, 2) * (x_ - x_target_) + CalcPoly(left_leg_vmc_.l0(), 1, 3) * x_dot_ +
  //                  CalcPoly(left_leg_vmc_.l0(), 1, 4) * pitch_ + CalcPoly(left_leg_vmc_.l0(), 1, 5) * pitch_dot_) +
  //                (right_theta_ - left_theta_) * theta_kp - (right_theta_dot_ - left_theta_dot_) * theta_kd;

  // right_tau_(0) = -(CalcPoly(right_leg_vmc_.l0(), 0, 0) * right_theta_ +
  //                   CalcPoly(right_leg_vmc_.l0(), 0, 2) * (x_ - x_target_) + CalcPoly(right_leg_vmc_.l0(), 0, 3) * x_dot_ +
  //                   CalcPoly(right_leg_vmc_.l0(), 0, 4) * pitch_ + CalcPoly(right_leg_vmc_.l0(), 0, 5) * pitch_dot_);
  // right_tau_(1) = -(CalcPoly(right_leg_vmc_.l0(), 1, 0) * right_theta_ +
  //                   CalcPoly(right_leg_vmc_.l0(), 1, 2) * (x_ - x_target_) + CalcPoly(right_leg_vmc_.l0(), 1, 3) * x_dot_ +
  //                   CalcPoly(right_leg_vmc_.l0(), 1, 4) * pitch_ + CalcPoly(right_leg_vmc_.l0(), 1, 5) * pitch_dot_) -
  //                 (right_theta_ - left_theta_) * theta_kp + (right_theta_dot_ - left_theta_dot_) * theta_kd;
}

f32 Chassis::CalcPoly(f32 length, int j, int i) {
  return length * length * length * ctrl_kp[i][j][0] + length * length * ctrl_kp[i][j][1] + length * ctrl_kp[i][j][2] +
         ctrl_kp[i][j][3];
}

void Chassis::Update(const f32 left_leg_phi1, const f32 left_leg_phi4, const f32 right_leg_phi1,
                     const f32 right_leg_phi4, const f32 left_leg_w_phi1, const f32 left_leg_w_phi4,
                     const f32 right_leg_w_phi1, const f32 right_leg_w_phi4) {
  // 关节电机角度，速度滤波
  lf_pos_filter_.Update(left_leg_phi4);
  lb_pos_filter_.Update(left_leg_phi1);
  rf_pos_filter_.Update(right_leg_phi4);
  rb_pos_filter_.Update(right_leg_phi1);
  lf_spd_filter_.Update(left_leg_w_phi4);
  lb_spd_filter_.Update(left_leg_w_phi1);
  rf_spd_filter_.Update(right_leg_w_phi4);
  rb_spd_filter_.Update(right_leg_w_phi1);

  // pitch_filter_.Update(pitch_);
  pitch_dot_filter_.Update(pitch_dot_);
  yaw_dot_filter_.Update(yaw_dot_);

  // pitch_ = pitch_filter_.value();
  pitch_dot_ = pitch_dot_filter_.value();
  yaw_dot_ = yaw_dot_filter_.value();

  LegPosTrans();

  left_leg_vmc_.Update();
  right_leg_vmc_.Update();

  StateTrans();
  CalcTau();

  left_l0_pid_.SetIout(70.f);
  right_l0_pid_.SetIout(70.f);
  left_l0_pid_.Update(l0_target_, left_leg_vmc_.l0());
  right_l0_pid_.Update(l0_target_, right_leg_vmc_.l0());
  left_force_ = left_l0_pid_.value() - (INS.Roll - 1.f) * roll_kp - (imu->gyro_y() - 0.f) * roll_kd;
  right_force_ = right_l0_pid_.value() + (INS.Roll - 1.f) * roll_kp + (imu->gyro_y() - 0.f) * roll_kd;

  left_turn_pid_.Update(wz_, yaw_dot_);
  right_turn_pid_.Update(wz_, yaw_dot_);

  Visual2Real();

  left_wheel_tau_ = left_tau_(0) - left_turn_pid_.value();
  right_wheel_tau_ = -right_tau_(0) - right_turn_pid_.value();
}

extern "C" {
    void ChassisInit() {
        chassis = new Chassis();

        left_joint_rs485 = new Serial(huart2, 78, stm32::UartMode::kDma, stm32::UartMode::kDma);
        right_joint_rs485 = new Serial(huart3, 78, stm32::UartMode::kDma, stm32::UartMode::kDma);
        wheel_rs485 = new Serial(huart8, 16, stm32::UartMode::kDma, stm32::UartMode::kDma);

        lf_joint = new UnitreeMotor(*left_joint_rs485, 0x0);
        lb_joint = new UnitreeMotor(*left_joint_rs485, 0x1);        
        rf_joint = new UnitreeMotor(*right_joint_rs485, 0x0);
        rb_joint = new UnitreeMotor(*right_joint_rs485, 0x1);

        left_wheel = new Go8010Motor(*wheel_rs485, 0x0);
        right_wheel = new Go8010Motor(*wheel_rs485, 0x1);

        remote_uart = new Serial(huart5, 18, stm32::UartMode::kNormal, stm32::UartMode::kDma);
        remote = new DR16(*remote_uart);

        wheel_stand_pid = new PID(rm::modules::algorithm::PIDType::kPosition, 12.f, 0.f, 0.05f, 50.f, 30.f);
        wheel_speed_pid = new PID(rm::modules::algorithm::PIDType::kPosition, 0.04f, 0.f, 0.04f, 100.f, 30.f);

        remote->Begin();

        left_joint_rs485->Begin();
        right_joint_rs485->Begin();
        wheel_rs485->Begin();
    }

    void ChassisUpdate() {
        while (chassis->A1_init_flag_) {
            DWT_Delay(2);
            chassis->lf_init_angle_ = lf_joint->pos();
            chassis->lb_init_angle_ = lb_joint->pos();
            chassis->rf_init_angle_ = rf_joint->pos();
            chassis->rb_init_angle_ = rb_joint->pos();
            chassis->A1_init_flag_ = false;
        }

        chassis->SetPitch(-INS.Pitch * 3.14159 / 180.f); // 0.012f
        chassis->SetPitchDot(imu->gyro_x() - 0.003f);

        left_wheel_vel_filter.Update(left_wheel->vel());
        right_wheel_vel_filter.Update(right_wheel->vel());

        chassis->SetXDot((left_wheel_vel_filter.value() - right_wheel_vel_filter.value()) / 2.f * 0.077f);
        chassis->SetX(chassis->x() + chassis->x_dot() * 0.002f);

        chassis->SetYawDot(imu->gyro_z() - 0.00385f);

        x_target_pre += (f32)remote->left_y() / 330.f / 2.f * 0.002f;

        chassis->SetCtrlParam(x_target_pre, 0.f, 0.f, 0.18f);
        

        chassis->Update(lb_joint->pos(), lf_joint->pos(), rb_joint->pos(), rf_joint->pos(), lb_joint->vel(),
                        lf_joint->vel(), rb_joint->vel(), rf_joint->vel());

        wheel_speed_pid->Update(0.f, chassis->x_dot());
        wheel_stand_pid->Update(wheel_speed_pid->value(), chassis->pitch());        

        if (remote->switch_l() != RcSwitchState::kMid) {
            lf_joint->SetTau(0.f);
            lb_joint->SetTau(0.f);
            rf_joint->SetTau(0.f);
            rb_joint->SetTau(0.f);

            left_wheel->SetTau(0.f);
            right_wheel->SetTau(0.f);

            count = 0;
        }
        else {
            if (count == 500) {
              lf_joint->SetTau(chassis->lf_tau() / 9.1f);
              lb_joint->SetTau(chassis->lb_tau() / 9.1f);
              rf_joint->SetTau(chassis->rf_tau() / 9.1f);
              rb_joint->SetTau(chassis->rb_tau() / 9.1f);

              // lf_joint->SetTau(0.f);
              // lb_joint->SetTau(0.f);
              // rf_joint->SetTau(0.f);
              // rb_joint->SetTau(0.f);

              left_wheel->SetTau(chassis->left_wheel_tau() / 6.33f);
              right_wheel->SetTau(chassis->right_wheel_tau() / 6.33f);

              // left_wheel->SetTau(0.f);
              // right_wheel->SetTau(0.f);
            }
            else {
              lf_joint->SetTau(0.f);
              lb_joint->SetTau(0.f);
              rf_joint->SetTau(0.f);
              rb_joint->SetTau(0.f);

              // left_wheel->SetTau(wheel_stand_pid->value()/6.33f);
              // right_wheel->SetTau(-wheel_stand_pid->value()/6.33f);

              left_wheel->SetTau(chassis->left_wheel_tau() / 6.33f);
              right_wheel->SetTau(chassis->right_wheel_tau() / 6.33f);

              // left_wheel->SetTau(0.f);
              // right_wheel->SetTau(0.f);

              count++;
            }
        }

      lf_joint->SendCommend();
      rf_joint->SendCommend();
      left_wheel->SendCommend();

      DWT_Delay(0.0003f);

      lb_joint->SendCommend();
      rb_joint->SendCommend();
      right_wheel->SendCommend();
    }
}