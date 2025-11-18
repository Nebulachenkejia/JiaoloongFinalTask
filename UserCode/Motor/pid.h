//
// Created by Nebula on 2025/11/11.
//

#ifndef JIAOLOONGFINALTASK_PID_H
#define JIAOLOONGFINALTASK_PID_H
#include "main.h"

#ifdef __cplusplus
extern "C" {
    #endif

class PID
{
public:
    PID(void) : PID(0.0f, 0.0f, 0.0f, 0.0f, 0.0f)
    {
    };
    PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k = 0.1, uint64_t freq = 1000);

    void reset(void); //重置积分项和误差项
    float calc(float ref, float fdb);

    float kp_, ki_, kd_, d_filter_k_;
    float i_max_, out_max_;
    float output_;
    const uint64_t freq_;

private:
    float ref_, fdb_;
    float err_, err_sum_, last_err_;
    float pout_, iout_, dout_, last_dout_;
};

    #ifdef __cplusplus
}
#endif
#endif //JIAOLOONGFINALTASK_PID_H
