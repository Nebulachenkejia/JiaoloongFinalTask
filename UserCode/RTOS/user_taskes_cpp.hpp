#ifndef USER_TASKS_CPP_HPP
#define USER_TASKS_CPP_HPP

// 仅 C++ 文件包含本头
#include "../imu/imu.h"
#include "../Motor/GM6020.h"
#include "../RC/DT7_RC.h"
#include "../Motor/pid.h"

// 在 tasks.cpp 中定义这些全局对象，其他 C++ 文件若需访问可用 extern 声明
extern IMU     imu;
extern DT7_RC  rc;
extern GM6020  pitchMotor;
extern GM6020  yawMotor;

// 全局云台状态变量（在 tasks.cpp 中定义）
extern float pitch_zero_offset;
extern float yaw_zero_offset;
extern float desired_pitch;
extern float desired_yaw;
extern GimbalState gimbal_state;

#endif // USER_TASKS_CPP_HPP
