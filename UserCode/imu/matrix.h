//
// Created by Nebula on 2025/11/17.
//
#ifndef MATRIX_H
#define MATRIX_H
#include <cmath>

// 3x3 矩阵与向量的乘法
void Matrix33fMultVector3f(const float M[3][3], const float v[3], float result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = 0;
        for (int j = 0; j < 3; j++) {
            result[i] += M[i][j] * v[j];
        }
    }
}

// 3x3 矩阵转置
void Matrix33fTrans(const float M[3][3], float result[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = M[j][i];
        }
    }
}

// 向量叉乘
void Vector3fCross(const float v1[3], const float v2[3], float result[3]) {
    result[0] = v1[1] * v2[2] - v1[2] * v2[1];
    result[1] = v1[2] * v2[0] - v1[0] * v2[2];
    result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 向量加法
void Vector3fAdd(const float v1[3], const float v2[3], float result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = v1[i] + v2[i];
    }
}

// 向量减法
void Vector3fSub(const float v1[3], const float v2[3], float result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = v1[i] - v2[i];
    }
}

// 计算向量的模
float Vector3fNorm(const float v[3]) {
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 向量单位化
void Vector3fUnit(const float v[3], float result[3]) {
    float norm = Vector3fNorm(v);
    if (norm > 0.0f) {
        for (int i = 0; i < 3; i++) {
            result[i] = v[i] / norm;
        }
    } else {
        result[0] = result[1] = result[2] = 0.0f;
    }
}

// 4D 向量单位化
void Vector4fUnit(const float v[4], float result[4]) {
    float norm = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
    if (norm > 0.0f) {
        for (int i = 0; i < 4; i++) {
            result[i] = v[i] / norm;
        }
    } else {
        result[0] = result[1] = result[2] = result[3] = 0.0f;
    }
}

#endif  // MATRIX_H
