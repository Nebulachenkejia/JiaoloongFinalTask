//
// Created by Nebula on 2025/11/11.
//
#include "imu.h"
#define RAD_TO_DEG  (180.0f / 3.14159265358979323846)
#define DEG_TO_RAD  (3.14159265358979323846 / 180.0f)
#define BMI088_ACCEL_X_L  0x12
#define BMI088_ACCEL_X_H  0x13
#define BMI088_ACCEL_Y_H  0x15
#define BMI088_ACCEL_Z_L  0x16
#define BMI088_ACCEL_Z_H  0x17
#define BMI088_GYRO_X_L   0x02
#define BMI088_GYRO_X_H   0x03
#define BMI088_GYRO_Y_L   0x04
#define BMI088_GYRO_Y_H   0x05
#define BMI088_GYRO_Z_L   0x06
#define BMI088_GYRO_Z_H   0x07
#define BMI088_ACCEL_TEMP 0x22

IMU::IMU(const float& dt, const float& kg, const float& g_thres,
     const float R_imu[3][3], const float gyro_bias[3])
    : gyro_bias_{gyro_bias[0], gyro_bias[1], gyro_bias[2]},mahony_(dt, kg, g_thres)
{
    //赋值R_imu_
    for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j) {
            R_imu_[i][j] = R_imu[i][j];
        }
    }
    //赋值gyro_bias_（需要空指针检验）
    if (gyro_bias != nullptr) {
        for (int i=0; i<3; ++i) {
            gyro_bias_[i] = gyro_bias[i];
        }
    }
    else {
        gyro_bias_[0] = gyro_bias_[1] = gyro_bias_[2] = 0.0f;
    }
}

// IMU传感器初始化    需要初始化传感器并设置初始姿态角
void IMU::init(EulerAngle_t euler_deg_init) {
    //初始化传感器
    bmi088_init();

    //角度制欧拉角
    euler_deg_ = euler_deg_init;

    //弧度制欧拉角
    float yaw_rad = euler_deg_init.yaw * DEG_TO_RAD;
    float pitch_rad = euler_deg_init.pitch * DEG_TO_RAD;
    float roll_rad = euler_deg_init.roll * DEG_TO_RAD;
    euler_rad_.yaw = yaw_rad;
    euler_rad_.pitch = pitch_rad;
    euler_rad_.roll = roll_rad;

    //yaw-pitch-roll顺序转四元数
    float cy = cos(yaw_rad * 0.5f);
    float sy = sin(yaw_rad * 0.5f);
    float cp = cos(pitch_rad * 0.5f);
    float sp = sin(pitch_rad * 0.5f);
    float cr = cos(roll_rad * 0.5f);
    float sr = sin(roll_rad * 0.5f);
    q_[0] = cy * cp * cr + sy * sp * sr;  // w
    q_[1] = cy * cp * sr - sy * sp * cr;  // x
    q_[2] = sy * cp * sr + cy * sp * cr;  // y
    q_[3] = sy * cp * cr - cy * sp * sr;  // z
}
// 读取bmi088数据：加速度，角速度，温度
void IMU::readSensor() {
    uint8_t rx_buf[6] = {0}; //x,y,z高、低字节
    // 读取加速度计数据，量程为6g，转换为m/s^2
    bmi088_accel_read_reg(BMI088_ACCEL_X_L, rx_buf, 6);
    int16_t accel_x_raw = (rx_buf[1] << 8) | rx_buf[0];
    int16_t accel_y_raw = (rx_buf[3] << 8) | rx_buf[2];
    int16_t accel_z_raw = (rx_buf[5] << 8) | rx_buf[4];
    raw_data_.accel[0] = accel_x_raw * 6 * gravity_accel / 32768 ; // X轴
    raw_data_.accel[1] = accel_y_raw * 6 * gravity_accel / 32768 ; // Y轴
    raw_data_.accel[2] = accel_z_raw * 6 * gravity_accel / 32768 ; // Z轴

    // 读取陀螺仪数据，量程为2000dps，转换为rad/s
    bmi088_gyro_read_reg(BMI088_GYRO_X_L, rx_buf, 6);
    int16_t gyro_x_raw = (rx_buf[1] << 8) | rx_buf[0];
    int16_t gyro_y_raw = (rx_buf[3] << 8) | rx_buf[2];
    int16_t gyro_z_raw = (rx_buf[5] << 8) | rx_buf[4];
    raw_data_.gyro[0] = gyro_x_raw * 61.0f * DEG_TO_RAD - gyro_bias_[0]; // X轴
    raw_data_.gyro[1] = gyro_y_raw * 61.0f * DEG_TO_RAD - gyro_bias_[1]; // Y轴
    raw_data_.gyro[2] = gyro_z_raw * 61.0f * DEG_TO_RAD - gyro_bias_[2]; // Z轴

    // 读取温度传感器
    bmi088_accel_read_reg(BMI088_ACCEL_TEMP, rx_buf, 2);
    int16_t temp_raw = (rx_buf[1] << 8) | rx_buf[0] >> 5;
    if(temp_raw > 1023) {
        temp_raw = temp_raw - 2048;
    }
    raw_data_.temp[0] = 23.0f + temp_raw * 0.125f;

    //传感器坐标系
    for (int i=0;i<3;++i) {
        gyro_sensor_[i] = raw_data_.gyro[i];
        gyro_sensor_[i] = raw_data_.gyro[i];
        gyro_sensor_dps_[i] = raw_data_.gyro[i] * RAD_TO_DEG;
    }
}
// 利用线性互补滤波算法，上一时刻姿态角，加速度与角速度更新当前姿态角
void IMU::update(void) {
    readSensor();

    //转换到世界坐标系
    for (int i=0;i<3;++i) {
        accel_world_[i] = accel_sensor_[0] * R_imu_[i][0]
                        + accel_sensor_[1] * R_imu_[i][1]
                        + accel_sensor_[2] * R_imu_[i][2];
        gyro_world_[i] = gyro_sensor_[0] * R_imu_[i][0]
                       + gyro_sensor_[1] * R_imu_[i][1]
                       + gyro_sensor_[2] * R_imu_[i][2];
        gyro_world_dps_[i] = gyro_world_[i] * RAD_TO_DEG;
    }

    //mahony更新四元数
    mahony_.update(q_, gyro_world_, accel_world_);

    //四元数转欧拉角
    float w = q_[0], x = q_[1], y = q_[2], z = q_[3];
    euler_rad_.pitch = asin(2.0f * (w*y - x*z));    // 俯仰角（pitch）：-π/2 ~ π/2
    euler_rad_.roll = atan2(2.0f * (w*x + y*z), 1.0f - 2.0f * (x*x + y*y)); // 横滚角（roll）：-π ~ π
    euler_rad_.yaw = atan2(2.0f * (w*z + x*y), 1.0f - 2.0f * (y*y + z*z));   // 偏航角（yaw）：-π ~ π（需处理象限，确保连续）
    euler_deg_.pitch = euler_rad_.pitch * RAD_TO_DEG;
    euler_deg_.roll = euler_rad_.roll * RAD_TO_DEG;
    euler_deg_.yaw = euler_rad_.yaw * RAD_TO_DEG;
}