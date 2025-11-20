#include "GM6020.h"

extern uint8_t stop_flag;

GM6020::GM6020(float angle_kp, float angle_ki, float angle_kd, float angle_i_max, float angle_out_max, float p_d_filter_k_,
               float speed_kp, float speed_ki, float speed_kd, float speed_i_max, float speed_out_max, float s_d_filter_k_,
               const float ratio, const uint8_t ID) :
        ratio_(ratio), ID_(ID),control_method_(TORQUE),
        ppid_(angle_kp, angle_ki, angle_kd, angle_i_max, angle_out_max, p_d_filter_k_),
        spid_(speed_kp, speed_ki, speed_kd, speed_i_max, speed_out_max, s_d_filter_k_)

{
}

//反馈解包
void GM6020::canRxMsgCallback(const uint8_t rx_data[8])
{
    // 解析编码器与速度、电流、温度
    ecd_ = (rx_data[0] << 8) | rx_data[1];
    fdb_angle_ = linearMapping(ecd_, 0, 8191, -180, 180);
    int16_t speed_raw = (rx_data[2] << 8) | rx_data[3];
    fdb_speed_ = static_cast<float>(speed_raw) / ratio_;
    uint16_t current_raw = (rx_data[4] << 8) | rx_data[5];
    current_ = float(current_raw);
    temp_ = rx_data[6];


    //限幅
    while (fdb_angle_ > 180.0f) fdb_angle_ -= 360.0f;
    while (fdb_angle_ < -180.0f) fdb_angle_ += 360.0f;
}



// SetPosition：保持签名（不重置PID）
void GM6020::SetPosition(float target_position, float imu_feedback_position, float feedforward_intensity)
{
    control_method_ = POSITION_SPEED;
    target_angle_ = target_position;
    imu_feedback_ = imu_feedback_position;
    feedforward_intensity_ = feedforward_intensity;

}

// SetSpeed：保持签名（不重置PID）
void GM6020::SetSpeed(float target_speed, float feedforward_intensity)
{
    control_method_ = SPEED;
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
}

// SetIntensity（TORQUE）
void GM6020::SetIntensity(float intensity)
{
    control_method_ = TORQUE;
    feedforward_intensity_ = intensity;
}

//电机主处理函数
void GM6020::handle()
{
    if (stop_flag == 1)
    {
        output_intensity_ = 0.0f;
        return;
    }

    switch (control_method_)
    {
    case TORQUE:
        output_intensity_ = feedforward_intensity_;
        break;

    case SPEED:
        output_intensity_ = spid_.calc(target_speed_, fdb_speed_) + feedforward_intensity_;
        break;

    case POSITION_SPEED:
    {
        // 电机角度控制
        target_speed_ = ppid_.calc(target_angle_, fdb_angle_);
        // imu控制
        //target_speed_ = ppid_.calc(target_angle_, imu_feedback_);
        // 内环：速度 -> 力矩（输出）
        output_intensity_ = spid_.calc(target_speed_, fdb_speed_) + feedforward_intensity_;
        break;
    }
    }

    // 限幅
    if (output_intensity_ > 1.0f) output_intensity_ = 1.0f;
    else if (output_intensity_ < -1.0f) output_intensity_ = -1.0f;
}

//电流转换工具函数
int16_t GM6020::getGivenCurrent() const
{
    return static_cast<uint16_t>(output_intensity_ * 16384.0f);
}

//获取发包数据接口函数
void GM6020::SendTxCanMsg(uint8_t (&data)[8]) const {
    //memset(data, 0, 8);
    uint16_t current = getGivenCurrent();
    data[2 * ID_ - 2]     = (current >> 8) & 0xFF;
    data[2 * ID_ - 1] = current        & 0xFF;
}

//前馈计算
float GM6020::Calfeedforward_intensity(float target_angle) {
    return 0.0f;
}

float GM6020::linearMapping(uint16_t x,
                        uint16_t in_min, uint16_t in_max,
                        float out_min, float out_max) {
    return out_min + x * (out_max - out_min) / (in_max - in_min);
}