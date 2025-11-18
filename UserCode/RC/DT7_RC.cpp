//
// Created by Nebula on 2025/11/11.
//

#include "DT7_RC.h"
extern uint8_t stop_flag;
DT7_RC::DT7_RC() {

}

void DT7_RC::init()
{
    last_update_time = HAL_GetTick();
    isConnected = false;
}

void DT7_RC::handle(const uint8_t* data, uint8_t size)
{
    if (data == nullptr || size != 18)
        return;

    // 更新时间戳
    last_update_time = HAL_GetTick();
    isConnected = true;

    //原始数据解包
    raw_.ch[0] = ((int16_t)data[0]      | (int16_t)data[1] << 8) & 0x07FF;
    raw_.ch[1] = ((int16_t)data[1] >> 3 | (int16_t)data[2] << 5) & 0x07FF;
    raw_.ch[2] = ((int16_t)data[2] >> 6 | (int16_t)data[3] << 2 | (int16_t)data[4] << 10) & 0x07FF;
    raw_.ch[3] = ((int16_t)data[4] >> 1 | (int16_t)data[5] << 7) & 0x07FF;

    raw_.s1 = ((data[5] >> 4) & 0x0C) >> 2;
    raw_.s2 = (data[5] >> 4) & 0x03;

    raw_.mouse_x = (int16_t)data[6] | ((int16_t)data[7] << 8);
    raw_.mouse_y = (int16_t)data[8] | ((int16_t)data[9] << 8);
    raw_.mouse_z = (int16_t)data[10] | ((int16_t)data[11] << 8);

    raw_.mouse_left  = data[12];
    raw_.mouse_right = data[13];
    raw_.key = (int16_t)data[14] | ((int16_t)data[15] << 8);
    raw_.reserve = (int16_t)data[16] | ((int16_t)data[17] << 8);

    //映射
    for (int i = 0; i < 4; i++)
        dt7_.ch[i] = linearMapping(raw_.ch[i], 364, 1684, -1.0f, 1.0f);

    //开关解析
    dt7_.s1 = decodeSwitch(raw_.s1);
    dt7_.s2 = decodeSwitch(raw_.s2);
    if (dt7_.s2 == SWITCH_DOWN)
        stop_flag = 1;
    else
        stop_flag = 0;

    //失联更新
    if (HAL_GetTick() - last_update_time > 100)
        isConnected = false;
    //断联下三
        if (isConnected == false)
        {
           stop_flag = 1;
        }
}

DT7_RC::SwitchState DT7_RC::decodeSwitch(uint8_t raw)
{
    switch (raw)
    {
    case 1: return SWITCH_UP;
    case 3: return SWITCH_MID;
    case 2:
    default: return SWITCH_DOWN;
    }
}

float DT7_RC::linearMapping(int16_t x,
                            int16_t in_min, int16_t in_max,
                            float out_min, float out_max)
{
    const int16_t mid = (in_min + in_max) / 2;
    const int deadband = 20;
    // 死区
    if (x - mid < deadband && x - mid > -deadband)
        return 0.0f;
    return (float)(x - mid) / (float)(in_max - mid);
}
