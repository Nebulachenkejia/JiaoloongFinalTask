//
// Created by Nebula on 2025/11/5.
//
#include "user_tasks.h"
#include "cmsis_os2.h"

uint32_t send = 0;
uint32_t recv = 0;

osMessageQueueAttr_t test_queue_attributes = { .name = "test_queue"};
osMessageQueueId_t test_queue_handle;

osThreadId_t test1TaskHandle;
constexpr osThreadAttr_t testTask_attributes = {
    .name = "testTask",
    .stack_size = 256 * 4,
    .priority = osPriorityAboveNormal
};

void test1_tasks(void *){
    while(true){
        const auto tick = osKernelGetTickCount();
        ++send;
        osMessageQueuePut(test_queue_handle, &send, 0, 0);
        osDelayUntil(tick + 1);
    }
}

osThreadId_t test2TaskHandle;
void test2_tasks(void *){
    while(true){
        osMessageQueueGet(test_queue_handle, &recv, nullptr, osWaitForever);
    }
}



void user_tasks_init()
{
    test_queue_handle = osMessageQueueNew(10, sizeof(uint32_t), &test_queue_attributes);
    test1TaskHandle = osThreadNew(test1_tasks, nullptr, &testTask_attributes);
    test2TaskHandle = osThreadNew(test2_tasks, nullptr, &testTask_attributes);

}