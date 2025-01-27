
#include "imu_task.h"

#include "librm.hpp"

#include "spi.h"

using namespace rm;
using namespace rm::device;
using namespace rm::modules::algorithm;

BMI088 *imu;

static PID *temp_pid;

static uint32_t pwmout = 0;

MahonyAhrs *mahony_ahrs;

// ekf_ahrs
INS_t INS;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;

f32 PwmPidCalc() {
    temp_pid->Update(40.f, imu->temperature());
    if ((temp_pid->value()) < 0.f) {
        return 0.f;
    }
    else {
        return (temp_pid->value());
    }
}

extern "C" {

static void IMU_Param_Correction(IMU_Param_t* param, float gyro[3],
                                 float accel[3]);

static void BodyFrameToEarthFrame(const float* vecBF, float* vecEF, float* q);
static void EarthFrameToBodyFrame(const float* vecEF, float* vecBF, float* q);

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float* init_q4) {
  float acc_init[3] = {0};
  float gravity_norm[3] = {0, 0, 1};  // 导航系重力加速度矢量,归一化后为(0,0,1)
  float axis_rot[3] = {0};  // 旋转轴
  // 读取100次加速度计数据,取平均值作为初始值
  for (uint8_t i = 0; i < 100; ++i) {
    imu->Update();

    acc_init[0] += imu->accel_y();
    acc_init[1] -= imu->accel_x();
    acc_init[2] += imu->accel_z();
    DWT_Delay(0.001);
  }
  for (uint8_t i = 0; i < 3; ++i)
    acc_init[i] /= 100;
  Math::Norm3d(acc_init);
  // 计算原始加速度矢量和导航系重力加速度矢量的夹角
  float angle = acosf(Math::Dot3d(acc_init, gravity_norm));
  Math::Cross3d(acc_init, gravity_norm, axis_rot);
  Math::Norm3d(axis_rot);
  init_q4[0] = cosf(angle / 2.0f);
  for (uint8_t i = 0; i < 2; ++i)
    init_q4[i + 1] =
        axis_rot[i] * sinf(angle / 2.0f);  // 轴角公式,第三轴为0(没有z轴分量)
}

void INS_Init(void) {
  float init_quaternion[4] = {0};
  InitQuaternion(init_quaternion);
  IMU_QuaternionEKF_Init(init_quaternion, 10, 0.01, 10000000, 0.9996, 0);

  INS.AccelLPF = 0.0085f;
  INS.DGyroLPF = 0.008f;
}

void INS_Task(void) {
  static uint32_t count = 0;
  const float gravity[3] = {0, 0, 9.805f};
  dt = DWT_GetDeltaT(&INS_DWT_Count);
  t += dt;

  // ins update
  if ((count % 1) == 0) {

    INS.Accel[0] = imu->accel_y();
    INS.Accel[1] = -imu->accel_x();
    INS.Accel[2] = imu->accel_z();

    INS.Gyro[0] = imu->gyro_y();
    INS.Gyro[1] = -imu->gyro_x();
    INS.Gyro[2] = imu->gyro_z();

    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(INS.Gyro[0], INS.Gyro[1], INS.Gyro[2],
                             INS.Accel[0], INS.Accel[1], INS.Accel[2], dt);

    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(xb, INS.xn, INS.q);
    BodyFrameToEarthFrame(yb, INS.yn, INS.q);
    BodyFrameToEarthFrame(zb, INS.zn, INS.q);

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; i++)  // 同样过一个低通滤波
    {
      INS.MotionAccel_b[i] =
          (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) +
          INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
    }
    BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n,
                          INS.q);  // 转换回导航系n

    // 获取最终数据
    INS.Yaw = QEKF_INS.Yaw;
    INS.Pitch = QEKF_INS.Roll;
    INS.Roll = QEKF_INS.Pitch;
    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
  }

  count++;
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
static void BodyFrameToEarthFrame(const float* vecBF, float* vecEF, float* q) {
  vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                     (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                     (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

  vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                     (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                     (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

  vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                     (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                     (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
static void EarthFrameToBodyFrame(const float* vecEF, float* vecBF, float* q) {
  vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                     (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                     (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

  vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                     (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                     (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

  vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                     (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                     (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief
 * reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t* param, float gyro[3],
                                 float accel[3]) {
  static float lastYawOffset, lastPitchOffset, lastRollOffset;
  static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
  float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

  if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
      fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
      fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag) {
    cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
    cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
    cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
    sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
    sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
    sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

    // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
    c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
    c_12 = cosPitch * sinYaw;
    c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
    c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
    c_22 = cosYaw * cosPitch;
    c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
    c_31 = -cosPitch * sinRoll;
    c_32 = sinPitch;
    c_33 = cosPitch * cosRoll;
    param->flag = 0;
  }
  float gyro_temp[3];
  for (uint8_t i = 0; i < 3; i++)
    gyro_temp[i] = gyro[i] * param->scale[i];

  gyro[0] = c_11 * gyro_temp[0] + c_12 * gyro_temp[1] + c_13 * gyro_temp[2];
  gyro[1] = c_21 * gyro_temp[0] + c_22 * gyro_temp[1] + c_23 * gyro_temp[2];
  gyro[2] = c_31 * gyro_temp[0] + c_32 * gyro_temp[1] + c_33 * gyro_temp[2];

  float accel_temp[3];
  for (uint8_t i = 0; i < 3; i++)
    accel_temp[i] = accel[i];

  accel[0] = c_11 * accel_temp[0] + c_12 * accel_temp[1] + c_13 * accel_temp[2];
  accel[1] = c_21 * accel_temp[0] + c_22 * accel_temp[1] + c_23 * accel_temp[2];
  accel[2] = c_31 * accel_temp[0] + c_32 * accel_temp[1] + c_33 * accel_temp[2];

  lastYawOffset = param->Yaw;
  lastPitchOffset = param->Pitch;
  lastRollOffset = param->Roll;
}

    void ImuInit() {
        imu = new BMI088(hspi2, ACC_CS_GPIO_Port, ACC_CS_Pin, GYRO_CS_GPIO_Port, GYRO_CS_Pin);

        temp_pid = new PID(rm::modules::algorithm::PIDType::kPosition, 100.f, 20.f, 10.f, 500.f, 100.f);

        mahony_ahrs = new MahonyAhrs(500.f, 6.f, 0.f);

        INS_Init();
    }

    void ImuUpdate() {
        imu->Update();

        ImuData6Dof data{
            imu->gyro_y(),
            imu->gyro_x(),
            imu->gyro_z() - 0.00385f,
            imu->accel_y(),
            imu->accel_x(),
            imu->accel_z()
        };

        pwmout = PwmPidCalc();

        mahony_ahrs->Update(data);

        INS_Task();
    }

    uint32_t GetPwmOut() {
        return pwmout;
    }
}
