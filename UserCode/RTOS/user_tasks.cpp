//
// Created by Nebula on 2025/11/17.
//

#include "user_tasks.h"
#include "../RTOS/user_taskes_cpp.hpp"

// IMU
static const float R_imu_default[3][3] = {
    {1,0,0},
    {0,1,0},
    {0,0,1}
};
static const float gyro_bias_default[3] = {0,0,0};
IMU imu(0.004f, 0.01f, 1.0f, R_imu_default, gyro_bias_default);

// RC
DT7_RC rc;

// 电机
GM6020 pitchMotor(0.0f, 0.0f, 0.0f, 1000.0f, 1000.0f, 0.1f,
    0.0f, 0.0f, 0.0f, 1600.0f, 1800.0f, 0.1f, 1.0f, 4);
GM6020 yawMotor(4.0f, 0.1f, 90.0f, 10.0f, 1000.0f, 0.01f,
    0.01f, 0.0f, 0.2f, 10.0f, 1800.0f, 0.015f, 1.0f, 1);

// 消息队列
osMessageQueueId_t rcQueueHandle;
osMessageQueueId_t motorFeedbackQueueHandle;
osMessageQueueId_t canTxQueueHandle;

// 云台状态
float pitch_zero_offset = 0.0f;
float yaw_zero_offset   = 0.0f;
float desired_pitch = 0.0f;
float desired_yaw   = 0.0f;

GimbalState gimbal_state = GIMBAL_INIT;

// 限幅工具
inline void limit(float &val, float min_val, float max_val) {
    if (val > max_val) val = max_val;
    if (val < min_val) val = min_val;
}

// ---------------------- FreeRTOS 初始化 ----------------------


// ---------------------- controlTask ----------------------
extern IWDG_HandleTypeDef hiwdg;
float tortial_target_yaw_angle = 70;
float tortial_target_pitch_angle = 0;
float tortial_target_intensity = 0;
void controlTask(void *argument)
{
    const TickType_t control_period = 1; // 1ms
    TickType_t lastWake = osKernelGetTickCount();
    uint8_t rc_raw[32];
    rc.init();

    while (1)
    {
        // 获取遥控器数据
        if (osMessageQueueGet(rcQueueHandle, rc_raw, NULL, osWaitForever) == osOK)
            rc.handle(rc_raw, 18);
        // 增量控制
        const DT7_RC::DT7_RC_Data& data = rc.getData();

        //遥控器挡位设置
        if (data.s2 == DT7_RC::SWITCH_DOWN)
        {
            gimbal_state = GIMBAL_INIT;
        }
        if (data.s2 == DT7_RC::SWITCH_UP) {
            gimbal_state = GIMBAL_CONTROL;
        }
        if (data.s1 == DT7_RC::SWITCH_UP && data.s2 == DT7_RC::SWITCH_MID)
        {
            gimbal_state = GIMBAL_ANGLE_ATEP;
        }
        //左中右中档进入柔性档
        if (data.s2 == DT7_RC::SWITCH_MID && data.s1 == DT7_RC::SWITCH_MID)
        {
            gimbal_state = GIMBAL_SELF_CONTROL;
        }
        if (data.s1 == DT7_RC::SWITCH_DOWN && data.s2 == DT7_RC::SWITCH_MID)
        {
            gimbal_state = GIMBAL_GIVEN_INTENSITY;
        }

        // 零点设置
        if (gimbal_state == GIMBAL_INIT)
        {
            imu.update();
            pitch_zero_offset = imu.euler_deg_.pitch;
            yaw_zero_offset   = imu.euler_deg_.yaw;
            desired_pitch = 20.0f;
            desired_yaw   = 70.0f;
        }

        float dt = control_period * 0.001f;
        desired_pitch += data.ch[1] * 800.0f * dt;
        desired_yaw   += data.ch[0] * 800.0f * dt;
        limit(desired_pitch, -30.0f, 30.0f);
        limit(desired_yaw,   -180.0f, 170.0f);
        // PID计算
        float pitch_feedback = imu.euler_deg_.pitch - pitch_zero_offset;
        float yaw_feedback   = imu.euler_deg_.yaw   - yaw_zero_offset;

        //遥控器控制档
        if (gimbal_state == GIMBAL_CONTROL)
        {
            pitchMotor.SetPosition(desired_pitch, pitch_feedback, 0);
            yawMotor.SetPosition(desired_yaw, yaw_feedback, 0);
        }
        // 柔性档
        if (gimbal_state == GIMBAL_SELF_CONTROL)
        {
            pitchMotor.SetIntensity(pitchMotor.Calfeedforward_intensity(desired_pitch));
            yawMotor.SetIntensity(yawMotor.Calfeedforward_intensity(desired_yaw));
        }

        //角度阶跃档
        if (gimbal_state == GIMBAL_ANGLE_ATEP)
        {
            limit(tortial_target_pitch_angle, -12.0f, 48.0f);
            limit(tortial_target_yaw_angle,   -180.0f, 180.0f);
            pitchMotor.SetPosition(tortial_target_pitch_angle, pitch_feedback, 0);
            yawMotor.SetPosition(tortial_target_yaw_angle, yaw_feedback, 0);
        }

        if (gimbal_state == GIMBAL_GIVEN_INTENSITY)
        {
            pitchMotor.SetIntensity(tortial_target_intensity);
        }

        // 喂狗
        HAL_IWDG_Refresh(&hiwdg);

        // ----------周期控制----------
        TickType_t now = osKernelGetTickCount();
        if (now - lastWake < control_period)
            osDelay(control_period - (now - lastWake));
        lastWake = osKernelGetTickCount();
    }
}

// ---------------------- imuTask ----------------------
void imuTask(void *argument)
{
    const TickType_t imu_period = 1; // 1ms
    TickType_t lastWake = osKernelGetTickCount();

    static const EulerAngle_t zeroEuler{0.0f, 0.0f, 0.0f};
    imu.init(zeroEuler);

    while (1)
    {
        imu.update();

        // ----------周期控制----------
        TickType_t now = osKernelGetTickCount();
        if (now - lastWake < imu_period)
            osDelay(imu_period - (now - lastWake));
        lastWake = osKernelGetTickCount();
    }
}

// ---------------------- motorTask ----------------------
CanTxMsg motor_tx_msg;     //调试变量
void motorTask(void *argument)
{
    const TickType_t motor_period = 1; // 1ms
    TickType_t lastWake = osKernelGetTickCount();
    CanTxMsg msg;
    memset(msg.data, 0, sizeof(msg.data));
    msg.id = 0x1FE;
    while (1)
    {
        pitchMotor.handle();
        pitchMotor.SendTxCanMsg(msg.data);

        for (int i = 0; i < 8; i++) {
            motor_tx_msg.data[i] = msg.data[i];
        }
        osMessageQueuePut(canTxQueueHandle, &msg, 0, 0);

        yawMotor.handle();
        yawMotor.SendTxCanMsg(msg.data);
        for (int i = 0; i < 8; i++) {
            motor_tx_msg.data[i] = msg.data[i];
        }
        osMessageQueuePut(canTxQueueHandle, &msg, 0, 0);

        // ----------周期控制----------
        TickType_t now = osKernelGetTickCount();
        if (now - lastWake < motor_period)
            osDelay(motor_period - (now - lastWake));
        lastWake = osKernelGetTickCount();
    }
}


// ---------------------- canTxTask ----------------------
CanTxMsg tortial_motor_msg;     //调试变量
extern CAN_HandleTypeDef hcan1;
void canTxTask(void *argument)
{
    const TickType_t can_period = 1; // 1ms
    TickType_t lastWake = osKernelGetTickCount();

    CanTxMsg msg;
    CAN_TxHeaderTypeDef tx_header = {
        .StdId = 0x1FE,
        .ExtId = 0,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE,
      };

    while (1)
    {
        // 等待消息
  
        if (osMessageQueueGet(canTxQueueHandle, &msg, NULL, 100) == osOK)
        {
            tx_header.StdId = msg.id;
            //调试
            for (int i = 0; i < 8; i++) {
                tortial_motor_msg.data[i] = msg.data[i];
            }
            HAL_CAN_AddTxMessage(&hcan1, &tx_header, msg.data, NULL);
        }

        // ----------周期控制----------
        TickType_t now = osKernelGetTickCount();
        if (now - lastWake < can_period)
            osDelay(can_period - (now - lastWake));
        lastWake = osKernelGetTickCount();
    }
}
