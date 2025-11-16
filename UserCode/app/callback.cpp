//
// Created by Nebula on 2025/11/12.
//

#include "callback.h"

#define USART3_RX_BUF_SIZE 32
#define USART3_RX_DATA_SIZE 32
extern UART_HandleTypeDef huart3;
extern uint64_t msg_time;
extern uint8_t rx_buf[USART3_RX_BUF_SIZE];
extern uint8_t rx_data[USART3_RX_DATA_SIZE];


//float R_imu_[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
//float gyro_bias_[3] = {0.0f, 0.0f, 0.0f};
//IMU imu(0.001,0.1,0,R_imu_, gyro_bias_); //待优化
/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
        //待优化
}
*/
DT7_RC rc;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == huart3.Instance)
    {
        static bool rc_inited = false;
        if (!rc_inited)
        {
            rc.init();
            rc_inited = true;
        }
        rc.current_time = HAL_GetTick();
        rc.isConnected =  (rc.current_time - rc.last_time) < 100;
        memcpy(rx_data, rx_buf, Size);
        rc.handle(rx_data, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, USART3_RX_BUF_SIZE);
    }
}