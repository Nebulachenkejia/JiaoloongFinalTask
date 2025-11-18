//
// Created by Nebula on 2025/11/17.
//

#ifndef USER_TASKS_H
#define USER_TASKS_H

#ifdef __cplusplus
extern "C" {
    #endif

#include "main.h"
#include "freertos_mpool.h"
#include "cmsis_os2.h"

// CAN发送消息结构
typedef struct{
    uint32_t id;
    uint8_t data[8];
} CanTxMsg ;

// 电机反馈消息结构
typedef struct{
    uint32_t id;
    uint8_t data[8];
} MotorFeedbackMsg ;

// 云台状态
typedef enum{ GIMBAL_INIT, GIMBAL_CONTROL, GIMBAL_SELF_CONTROL} GimbalState ;

// 公开消息队列句柄（在 tasks.cpp 中定义）
extern osMessageQueueId_t rcQueueHandle;
extern osMessageQueueId_t motorFeedbackQueueHandle;
extern osMessageQueueId_t canTxQueueHandle;

// 任务函数
void controlTask(void *argument);
void imuTask(void *argument);
void motorTask(void *argument);
void canTxTask(void *argument);


    #ifdef __cplusplus
}
#endif

#endif //USER_TASKS_H
