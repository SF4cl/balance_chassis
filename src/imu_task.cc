
#include "librm.hpp"

#include "spi.h"

using namespace rm;
using namespace rm::device;
using namespace rm::modules::algorithm;

BMI088 *imu;

static PID *temp_pid;

static uint32_t pwmout = 0;

MahonyAhrs *mahony_ahrs;

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
    void ImuInit() {
        imu = new BMI088(hspi2, ACC_CS_GPIO_Port, ACC_CS_Pin, GYRO_CS_GPIO_Port, GYRO_CS_Pin);

        temp_pid = new PID(rm::modules::algorithm::PIDType::kPosition, 100.f, 20.f, 10.f, 500.f, 100.f);

        mahony_ahrs = new MahonyAhrs(500.f, 6.f, 0.f);
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
    }

    uint32_t GetPwmOut() {
        return pwmout;
    }
}