#ifndef MAHONY_H
#define MAHONY_H

const float gravity_accel = 9.8f;

class Mahony {
 public:
  // Initialize & config parameter(time step & fusion coefficient)
  // 初始化，配置参数（时间步长，数据融合系数）
  Mahony(float dt, float kg, float g_thres)
      : dt_(dt),
        kg_(kg),
        g_thres_(g_thres){}

  // Mahony algorithm, sensor data fusion, update quaternion
  // Mahony算法，传感器数据融合，四元数更新
  void update(float q[4], float ws[3], float as[3]);

 public:
  float q_[4];                   // quaternion
  float ws_[3], as_[3];          // gyro/accleration(sensor)
  float R_[3][3];                // rotate matrix
  float ww_[3], aw_[3];          // gyro/acceleration(no g)(world)

 private:
  float dq_[4];          // quaternion increment
  float RT_[3][3];       // rotate matrix transpose
  float _gs_[3];         // minus gravity acceleration (sensor)
  float ea_[3];          // error vector, ea=cross(-a,g)
  float wu_[3], we_[3];  // wu(update)=ws+kg*ea

  float dt_;       // update time step;
  float kg_, km_;  // data fusion coefficient

  const float _gw_[3] = {0, 0, gravity_accel};  // -g(world)
  const float g_thres_;                         // threshold of g offset
};

#endif  // MAHONY_H