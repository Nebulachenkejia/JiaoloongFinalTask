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
IMU imu(0.001f, 0.1f, 0.5f, R_imu_default, gyro_bias_default);

// RC
DT7_RC rc;

// 电机
GM6020 pitchMotor(5.0f, 0.0f, 0.0f, 1000.0f, 1000.0f,
    10.0f, 0.0f, 0.0f, 1600.0f, 1800.0f, 36.0f, 4);
GM6020 yawMotor(5.0f, 0.0f, 0.0f, 1000.0f, 1000.0f,
    10.0f, 0.0f, 0.0f, 1600.0f, 1800.0f, 36.0f, 1);

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
//extern IWDG_HandleTypeDef hiwdg;
void controlTask(void *argument)
{
    const TickType_t control_period = 5; // 5ms
    TickType_t lastWake = osKernelGetTickCount();
    uint8_t rc_raw[32];
    rc.init();

    while (1)
    {
        // 获取遥控器数据
        if (osMessageQueueGet(rcQueueHandle, rc_raw, NULL, osWaitForever) == osOK)
            rc.handle(rc_raw, 18);

        // 初始化零点
        if (gimbal_state == GIMBAL_INIT)
        {
            imu.update();
            pitch_zero_offset = imu.euler_deg_.pitch;
            yaw_zero_offset   = imu.euler_deg_.yaw;
            desired_pitch = 0.0f;
            desired_yaw   = 0.0f;
            gimbal_state = GIMBAL_CONTROL;
        }

        // 增量控制
        float dt = control_period * 0.001f;
        const DT7_RC::DT7_RC_Data& data = rc.getData();
        desired_pitch += data.ch[1] * 5.0f * dt;
        desired_yaw   += data.ch[0] * 5.0f * dt;
        limit(desired_pitch, -30.0f, 30.0f);
        limit(desired_yaw,   -180.0f, 180.0f);
        // PID计算
        float pitch_feedback = imu.euler_deg_.pitch - pitch_zero_offset;
        float yaw_feedback   = imu.euler_deg_.yaw   - yaw_zero_offset;
        if (gimbal_state == GIMBAL_CONTROL)
        {
            pitchMotor.SetPosition(desired_pitch, pitch_feedback, 0);
            yawMotor.SetPosition(desired_yaw, yaw_feedback, 0);
        }

        if (gimbal_state == GIMBAL_SELF_CONTROL)
        {
            pitchMotor.SetIntensity(pitchMotor.Calfeedforward_intensity(desired_pitch));
            yawMotor.SetIntensity(yawMotor.Calfeedforward_intensity(desired_yaw));
        }



        // 喂狗
        //HAL_IWDG_Refresh(&hiwdg);

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
void motorTask(void *argument)
{
    const TickType_t motor_period = 1; // 1ms
    TickType_t lastWake = osKernelGetTickCount();

    while (1)
    {
        CanTxMsg msg;

        pitchMotor.handle();
        pitchMotor.SendTxCanMsg(msg.data);
        msg.id = 0x1FF;
        osMessageQueuePut(canTxQueueHandle, &msg, 0, 0);

        yawMotor.handle();
        yawMotor.SendTxCanMsg(msg.data);
        msg.id = 0x1FF;
        osMessageQueuePut(canTxQueueHandle, &msg, 0, 0);

        // ----------周期控制----------
        TickType_t now = osKernelGetTickCount();
        if (now - lastWake < motor_period)
            osDelay(motor_period - (now - lastWake));
        lastWake = osKernelGetTickCount();
    }
}

// ---------------------- canTxTask ----------------------
extern CAN_HandleTypeDef hcan1;
void canTxTask(void *argument)
{
    const TickType_t can_period = 1; // 1ms
    TickType_t lastWake = osKernelGetTickCount();

    CanTxMsg msg;
    CAN_TxHeaderTypeDef tx_header;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    while (1)
    {
        // 等待消息
  
        if (osMessageQueueGet(canTxQueueHandle, &msg, NULL, 100) == osOK)
        {
            tx_header.StdId = msg.id;
            uint32_t mailbox;
						HAL_CAN_AddTxMessage(&hcan1, &tx_header, msg.data, &mailbox);
        }

        // ----------周期控制----------
        TickType_t now = osKernelGetTickCount();
        if (now - lastWake < can_period)
            osDelay(can_period - (now - lastWake));
        lastWake = osKernelGetTickCount();
    }
}
