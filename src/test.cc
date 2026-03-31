#include "librm/modules/ahrs/quaternion_ekf.hpp"
#include "librm.hpp"
#include "spi.h"
#include "usart.h"

#include "test.h"

using namespace rm;
using namespace rm::device;
using namespace rm::modules;

extern "C" void TestQuaternionEkfAhrs_Init();
extern "C" void TestQuaternionEkfAhrs_Update();

std::unique_ptr<BMI088> imu;
std::unique_ptr<QuaternionEkfAhrs> my_ahrs;

// for debug
f32 accel_z;
f32 accel_y;
f32 accel_x;

f32 gyro_z;
f32 gyro_x;
f32 gyro_y;

f32 yaw_angle;
f32 pitch_angle;
f32 roll_angle;

void TestQuaternionEkfAhrs_Init() {
    my_ahrs = std::make_unique<QuaternionEkfAhrs>();
    imu = std::make_unique<BMI088>(hspi2, ACC_CS_GPIO_Port, ACC_CS_Pin, GYRO_CS_GPIO_Port, GYRO_CS_Pin);  // 传入 nullptr 因为我们不实际使用它

    // 初始四元数: w=1, x=0, y=0, z=0 (对应欧拉角 yaw=pitch=roll=0)
    rm::f32 init_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    // 初始化滤波器
    // 参数：初始四元数, 过程噪声(Q1), 过程噪声(Q2), 测量噪声(R), 渐消因子(lambda), 低通滤波系数(lpf)
    my_ahrs->Init(init_q, 10.0f, 0.001f, 1000000.0f, 0.9996f, 0.0f);
    my_ahrs->SetUpdatePeriod(0.001f); // 假设1kHz的更新频率 (dt = 0.001s)
}

void TestQuaternionEkfAhrs_Update() {
    if (!imu || !my_ahrs) return;

    imu->Update();

    accel_z = imu->accel_z();  // 记录加速度z轴数据以供调试

    ImuData6Dof imu_data{imu->gyro_x() - 0.005f, imu->gyro_y() - 0.005f, imu->gyro_z() - 0.005f, imu->accel_x(), imu->accel_y(), imu->accel_z()};

    my_ahrs->Update(imu_data);

    gyro_z = imu->gyro_z();
    gyro_x = imu->gyro_x();
    gyro_y = imu->gyro_y();
    accel_x = imu->accel_x();
    accel_y = imu->accel_y();
    accel_z = imu->accel_z();

    yaw_angle = my_ahrs->euler_angle().yaw;  // 获取当前的yaw角以供调试
    pitch_angle = my_ahrs->euler_angle().pitch;  // 获取当前的pitch角以供调试
    roll_angle = my_ahrs->euler_angle().roll;  // 获取当前的roll角以供调试
}
