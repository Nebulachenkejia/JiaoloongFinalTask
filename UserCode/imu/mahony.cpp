#include "mahony.h"
#include "matrix.h"

// Mahony algorithm, sensor data fusion, update quaternion
// Mahony算法，传感器数据融合，四元数更新
void Mahony::update(float q[4], float ws[3], float as[3]) {
    // Update sensor data
    memcpy(q_, q, 4 * sizeof(float));
    memcpy(ws_, ws, 3 * sizeof(float));
    memcpy(as_, as, 3 * sizeof(float));

    // Calculate rotate matrix
    R_[0][0] = 1 - 2.f * (q_[2] * q_[2] + q_[3] * q_[3]);
    R_[0][1] = 2.f * (q_[1] * q_[2] - q_[0] * q_[3]);
    R_[0][2] = 2.f * (q_[1] * q_[3] + q_[0] * q_[2]);
    R_[1][0] = 2.f * (q_[1] * q_[2] + q_[0] * q_[3]);
    R_[1][1] = 1 - 2.f * (q_[1] * q_[1] + q_[3] * q_[3]);
    R_[1][2] = 2.f * (q_[2] * q_[3] - q_[0] * q_[1]);
    R_[2][0] = 2.f * (q_[1] * q_[3] - q_[0] * q_[2]);
    R_[2][1] = 2.f * (q_[2] * q_[3] + q_[0] * q_[1]);
    R_[2][2] = 1 - 2.f * (q_[1] * q_[1] + q_[2] * q_[2]);
    Matrix33fTrans(R_, RT_);

    // Calculate acceleration(g) field reference &
    // centripetal acceleration reference
    Matrix33fMultVector3f(RT_, _gw_, _gs_);  // -gs=R^T*(-gw)

    // calculate error between measure & reference of acceleration
    Vector3fCross(as_, _gs_, ea_);

    // wu(update)=ws+kg*eg
    if (fabs(Vector3fNorm(as_) - gravity_accel) < g_thres_) {
        for (int i = 0; i < 3; i++) {
            we_[i] = kg_ * ea_[i];
        }
    }

    Vector3fAdd(ws_, we_, wu_);

    // ww=R*ws
    Matrix33fMultVector3f(R_, ws_, ww_);
    // ap_w=R*as+g (as~-gs)
    Matrix33fMultVector3f(R_, as_, aw_);
    Vector3fSub(aw_, _gw_, aw_);

    // update quaternion
    dq_[0] = 0.5f * (-q_[1] * wu_[0] - q_[2] * wu_[1] - q_[3] * wu_[2]) * dt_;
    dq_[1] = 0.5f * (q_[0] * wu_[0] - q_[3] * wu_[1] + q_[2] * wu_[2]) * dt_;
    dq_[2] = 0.5f * (q_[3] * wu_[0] + q_[0] * wu_[1] - q_[1] * wu_[2]) * dt_;
    dq_[3] = 0.5f * (-q_[2] * wu_[0] + q_[1] * wu_[1] + q_[0] * wu_[2]) * dt_;
    for (int i = 0; i < 4; i++) {
        q_[i] += dq_[i];
    }
    Vector4fUnit(q_, q_);
    memcpy(q, q_, 4 * sizeof(float));
}
