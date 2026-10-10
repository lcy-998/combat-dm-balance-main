#ifndef VEL_MEAS_H
#define VEL_MEAS_H

/* 这个模块实现了一个简单的卡尔曼滤波器，用于轮腿速度观测以防止打滑。
   你可以根据需要修改滤波器的状态维度、观测维度、系统矩阵等。 */

#include <math.h>
#include <stdint.h>
#include <arm_math.h>
#include "leg_const.hpp"

static inline void mat2_mul(const float A[2][2], const float B[2][2], float C[2][2]);
static inline void mat2_add(const float A[2][2], const float B[2][2], float C[2][2]);
static inline void mat2_transpose(const float A[2][2], float AT[2][2]);
static inline void mat2_vec2_mul(const float A[2][2], const float x[2], float y[2]);
static inline void mat2_inv(const float A[2][2], float Ainv[2][2]);
static void eul2R_bw_deg(float roll_deg, float pitch_deg, float yaw_deg, float R[3][3]);
float WorldXSpeedKF(float dt,
                    const float acc_b[3],
                    const float eul_deg[3],
                    float vx_meas,
                    float *ax_out);

#endif // VEL_MEAS_HPP