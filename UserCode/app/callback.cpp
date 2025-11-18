//
// Created by Nebula on 2025/11/12.
//

#include "callback.h"


//外部队列句柄
extern osMessageQueueId_t rcQueueHandle;
extern osMessageQueueId_t motorFeedbackQueueHandle;

//电机反馈报文解包
extern CAN_RxHeaderTypeDef rx_header;
extern GM6020 pitchMotor;
extern GM6020 yawMotor;
extern uint8_t motor_msg_data[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, motor_msg_data);

        switch (rx_header.StdId)
        {
            case 0x208:
                pitchMotor.canRxMsgCallback(motor_msg_data);
		break;

            case 0x205:
                yawMotor.canRxMsgCallback(motor_msg_data);
                break;

            default:
                break;
			
        }
			
    }
}

//外部缓冲区
extern uint8_t rc_rx_buf[32];
extern uint8_t rc_rx_data[32];

//遥控器空闲中断（DMA接收完成）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART3) 
	{
	    memcpy(rc_rx_data, rc_rx_buf, Size);
            osMessageQueuePut(rcQueueHandle, rc_rx_data, 0, 0);
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rc_rx_buf, 32);
	}
}


