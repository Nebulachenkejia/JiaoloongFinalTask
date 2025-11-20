//
// Created by Nebula on 2025/11/11.
//

#ifndef JIAOLOONGFINALTASK_DT7_RC_H
#define JIAOLOONGFINALTASK_DT7_RC_H
#include "main.h"

#ifdef __cplusplus
extern "C" {
    #endif
class DT7_RC {
private:


    // 原始数据
    struct __packed DT7_RC_Raw
    {
        uint16_t ch[4];

        uint8_t s1;
        uint8_t s2;

        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;

        uint8_t mouse_left;
        uint8_t mouse_right;

        uint16_t key;
        uint16_t reserve;

        DT7_RC_Raw()
        {
            for (int i = 0; i < 4; i++)
                ch[i] = 1024;

            s1 = s2 = 3;
            mouse_x = mouse_y = mouse_z = 0;
            mouse_left = mouse_right = 0;
            key = reserve = 0;
        }
    };

public:
    enum SwitchState : uint8_t
    {
        SWITCH_UP   = 0,
        SWITCH_MID  = 1,
        SWITCH_DOWN = 2
    };
    // 解析后的标准化数据
    struct __packed DT7_RC_Data
    {
        float ch[4];

        SwitchState s1;
        SwitchState s2;

        DT7_RC_Data()
        {
            for(int i = 0; i < 4; i++)
                ch[i] = 0.0f;

            s1 = s2 = SWITCH_DOWN;
        }
    };

    DT7_RC(){}
    void init();
    void handle(const uint8_t* data, uint8_t size);

    // 公共获取接口
    inline const DT7_RC_Data& getData() const { return dt7_; }
    //inline const DT7_RC_Raw&  getRaw()  const { return raw_; }

private:
    DT7_RC_Raw raw_;
    DT7_RC_Data dt7_;
    bool isConnected = false;
    uint32_t last_update_time = 0;
    static float linearMapping(int16_t x,
                        int16_t in_min, int16_t in_max,
                        float out_min, float out_max);
    static SwitchState decodeSwitch(uint8_t raw);
};

    #ifdef __cplusplus
}
#endif
#endif //JIAOLOONGFINALTASK_DT7_RC_H
