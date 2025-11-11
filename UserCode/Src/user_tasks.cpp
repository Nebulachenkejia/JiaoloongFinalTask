//
// Created by Nebula on 2025/11/5.
//
#include "user_tasks.h"

uint32_t count = 0;

osThreadId_t testTaskHandler;
constexpr osThreadAttr_t testTask_attributes = {
    .name = "testTask",
    .stack_size = 256 * 4,
    .priority = osPriorityAboveNormal
};

void test_tasks(void *){
    while(true){
        const auto tick = osKernelGetTickCount();
        ++count;
        osDelayUntil(tick + 1);
    }
}


void user_tasks_init()
{
    testTaskHandler = osThreadNew(test_tasks, nullptr, &testTask_attributes);
}