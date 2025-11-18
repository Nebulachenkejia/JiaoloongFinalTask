#ifndef GM6020_H
#define GM6020_H

#include "main.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
    #endif
#include "string.h"

class GM6020 {
public:
    GM6020(float angle_kp, float angle_ki, float angle_kd, float angle_i_max, float angle_out_max,
               float speed_kp, float speed_ki, float speed_kd, float speed_i_max, float speed_out_max,
               const float ratio, uint8_t ID);

    // CAN 解包
    void canRxMsgCallback(const uint8_t rx_data[8]);


    void SetPosition(float target_position,
                     float feedforward_speed = 0,
                     float feedforward_intensity = 0);

    void SetSpeed(float target_speed,
                  float feedforward_intensity = 0);

    void SetIntensity(float intensity);

    // 核心计算
    void handle();

    // 获取输出电流值（用于 CAN 发送）
    int16_t getGivenCurrent() const;


public:
    // 对外暴露方便调试
    float fdb_angle_ = 0;    // 来自编码器的累计角度（deg，多圈）
    float fdb_speed_ = 0;    // 速度反馈（raw）
    void SendTxCanMsg(uint8_t (&data)[8]) const; // 填充 CAN 发送数据
    float Calfeedforward_intensity(float target_angle);

private:
    // 参数
    float ratio_;
    uint8_t ID_;
    uint8_t temp_ = 0;
    float current_= 0;
    uint8_t angle_flag = 0; // 编码器角度初始化标志
    // 控制方式
    enum { TORQUE, SPEED, POSITION_SPEED } control_method_;

    // 目标与前馈
    float target_speed_ = 0;
    float target_angle_ = 0;
    float feedforward_speed_ = 0;
    float feedforward_intensity_ = 0;

    // 输出
    float output_intensity_ = 0;

    // 编码器多圈计算用
    uint16_t ecd_ = 0;
    uint16_t last_ecd_ = 0;

    // 新增的 IMU 角度缓存（deg）
    float imu_angle_ = 0.0f;
    bool imu_feedback_ = true; // 默认使用 IMU 作为外环反馈
    // PID
    PID spid_;   // 速度环
    PID ppid_;   // 位置环
};

    #ifdef __cplusplus
}
#endif
#endif // GM6020_H
