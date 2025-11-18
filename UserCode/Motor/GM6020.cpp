#include "GM6020.h"

extern uint8_t stop_flag;

GM6020::GM6020(float angle_kp, float angle_ki, float angle_kd, float angle_i_max, float angle_out_max,
               float speed_kp, float speed_ki, float speed_kd, float speed_i_max, float speed_out_max,
               const float ratio, uint8_t ID) :
        ratio_(ratio), ID_(ID),control_method_(TORQUE),
        spid_(angle_kp, angle_ki, angle_kd, angle_i_max, angle_out_max),
        ppid_(speed_kp, speed_ki, speed_kd, speed_i_max, speed_out_max)

{
}

//反馈解包
void GM6020::canRxMsgCallback(const uint8_t rx_data[8])
{
    // 解析编码器与速度、电流、温度
    ecd_ = (rx_data[0] << 8) | rx_data[1];
    int16_t speed_raw = (rx_data[2] << 8) | rx_data[3];
    fdb_speed_ = float(speed_raw);
    int16_t current_raw = (rx_data[4] << 8) | rx_data[5];
    current_ = current_raw * 1.62 / 16384.0f;
    temp_ = rx_data[6];

    // 多圈角度解算
    int16_t diff = ecd_ - last_ecd_;
    if (diff > 4096) diff -= 8192;
    else if (diff < -4096) diff += 8192;

    float delta_angle = diff * (360.0f / 8192.0f) / ratio_;
    last_ecd_ = ecd_;

    if (angle_flag == 1) {
        fdb_angle_ += delta_angle;
    } else {
        fdb_angle_ = 0.0f;
        angle_flag = 1;
    }

    // 保持在 -180..180（可选）
    while (fdb_angle_ > 180.0f) fdb_angle_ -= 360.0f;
    while (fdb_angle_ < -180.0f) fdb_angle_ += 360.0f;
}


// ---------------------------
// SetPosition：保持签名（不重置PID）
//  注意：不要把 IMU 角度作为第二个参数传入，
// 如果要用IMU角度，请先调用 SetIMUFeedback()
// ---------------------------
void GM6020::SetPosition(float target_position, float feedback_position, float feedforward_intensity)
{
    control_method_ = POSITION_SPEED;
    target_angle_ = target_position;
    imu_feedback_ = feedback_position;
    feedforward_intensity_ = feedforward_intensity;

}

// ---------------------------
// SetSpeed：保持签名（不重置PID）
// ---------------------------
void GM6020::SetSpeed(float target_speed, float feedforward_intensity)
{
    control_method_ = SPEED;
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
}

// ---------------------------
// SetIntensity（TORQUE）
// ---------------------------
void GM6020::SetIntensity(float intensity)
{
    control_method_ = TORQUE;
    feedforward_intensity_ = intensity;
}

// ---------------------------
// handle：位置环优先使用 IMU（若启用），否则使用编码器累计角度
// ---------------------------
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
        // 外环：角度 -> 速度
        target_speed_ = ppid_.calc(target_angle_, imu_feedback_);

        // 内环：速度 -> 力矩（输出）
        output_intensity_ = spid_.calc(target_speed_, fdb_speed_) + feedforward_intensity_;
        break;
    }
    }

    // 限幅
    if (output_intensity_ > 1.0f) output_intensity_ = 1.0f;
    else if (output_intensity_ < -1.0f) output_intensity_ = -1.0f;
}


int16_t GM6020::getGivenCurrent() const
{
    return static_cast<int16_t>(output_intensity_ * 16384.0f);
}

void GM6020::SendTxCanMsg(uint8_t (&data)[8]) const {
    memset(data, 0, 8);
    int16_t current = getGivenCurrent();
    data[2 * ID_]     = (current >> 8) & 0xFF;
    data[2 * ID_ + 1] = current        & 0xFF;
}

float GM6020::Calfeedforward_intensity(float target_angle) {
    return 0.0f;
}