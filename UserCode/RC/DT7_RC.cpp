//
// Created by Nebula on 2025/11/11.
//

#include "DT7_RC.h"
void DT7_RC::init()
{
    current_time = last_time = HAL_GetTick();
}
void DT7_RC::handle(uint8_t* data, uint8_t size)
{
    if (data == nullptr || size != 18) return;

    last_time = current_time;
    dt7_rc_data_raw_.ch0 = ((int16_t)data[0]      | (int16_t)data[1] << 8) & 0x07FF;  //取11位
    dt7_rc_data_raw_.ch1 = ((int16_t)data[1] >> 3 | (int16_t)data[2] << 5) & 0x07FF;    //取11位
    dt7_rc_data_raw_.ch2 = ((int16_t)data[2] >> 6 | (int16_t)data[3] << 2 | (int16_t)data[4] << 10) & 0x07FF; //取11位
    dt7_rc_data_raw_.ch3 = ((int16_t)data[4] >> 1 | (int16_t)data[5] << 7) & 0x07FF;

    // 开关解析
    dt7_rc_data_raw_.s1 = ((data[5] >> 4) & 0x0C) >> 2;
    dt7_rc_data_raw_.s2 = (data[5] >> 4) & 0x03;

    // 鼠标与键盘解析
    dt7_rc_data_raw_.mouse_x = (int16_t)data[6] | ((int16_t)data[7] << 8);
    dt7_rc_data_raw_.mouse_y = (int16_t)data[8] | ((int16_t)data[9] << 8);
    dt7_rc_data_raw_.mouse_z = (int16_t)data[10] | ((int16_t)data[11] << 8);
    dt7_rc_data_raw_.mouse_left = data[12];
    dt7_rc_data_raw_.mouse_right = data[13];
    dt7_rc_data_raw_.key = (int16_t)data[14] | ((int16_t)data[15] << 8);
    dt7_rc_data_raw_.reserve = (int16_t)data[16] | ((int16_t)data[17] << 8);

    dt7_rc_data_.ch0 = linermapping(dt7_rc_data_raw_.ch0 ,364, 1684, -1.0f, 1.0f);
    dt7_rc_data_.ch1 = linermapping(dt7_rc_data_raw_.ch1 ,364, 1684, -1.0f, 1.0f);
    dt7_rc_data_.ch2 = linermapping(dt7_rc_data_raw_.ch2 ,364, 1684, -1.0f, 1.0f);
    dt7_rc_data_.ch3 = linermapping(dt7_rc_data_raw_.ch3 ,364, 1684, -1.0f, 1.0f);
    switch(dt7_rc_data_raw_.s1)
    {
    case 2:
        {
            dt7_rc_data_.s1 = switch_down;
            break;
        }
    case 1:
        {
            dt7_rc_data_.s1 = switch_up;
            break;
        }
    case 3:
        {
            dt7_rc_data_.s1 = switch_mid;
            break;
        }
    }
    switch(dt7_rc_data_raw_.s2)
    {
    case 2:
        {
            dt7_rc_data_.s2 = switch_down;
            break;
        }
    case 1:
        {
            dt7_rc_data_.s2 = switch_up;
            break;
        }
    case 3:
        {
            dt7_rc_data_.s2 = switch_mid;
            break;
        }
    }
}

float DT7_RC::linermapping(int16_t x, int16_t in_min, int16_t in_max, float out_min, float out_max)
{
    int16_t mid = (in_min + in_max) /2;
    if (x > mid * 0.98 && x < mid * 1.02) return 0.0f;
    return (x - mid)  * (out_max - out_min) / (in_max - in_min);
}