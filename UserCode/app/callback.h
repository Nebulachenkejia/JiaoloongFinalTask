//
// Created by Nebula on 2025/11/12.
//

#ifndef JIAOLOONGFINALTASK_CALLBACK_H
#define JIAOLOONGFINALTASK_CALLBACK_H

#include "../imu/imu.h"
#include "tim.h"
#include "main.h"
#include <cstring>
#include "../RC/DT7_RC.h"
#include "cmsis_os2.h"

//头文件保护
#ifdef __cplusplus
extern "C" {
#endif


void user_tasks_init();

#ifdef __cplusplus
}
#endif

#endif //JIAOLOONGFINALTASK_CALLBACK_H
