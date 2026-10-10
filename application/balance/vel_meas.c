#include "vel_meas.h"

#ifndef DEG2RAD
#define DEG2RAD (0.01745329251994329577f)
#endif

/* ---------- 2x2/2x1 矩阵工具 ---------- */
static inline void mat2_mul(const float A[2][2], const float B[2][2], float C[2][2])
{
    C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0];
    C[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1];
    C[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0];
    C[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1];
}
static inline void mat2_add(const float A[2][2], const float B[2][2], float C[2][2])
{
    C[0][0] = A[0][0] + B[0][0];
    C[0][1] = A[0][1] + B[0][1];
    C[1][0] = A[1][0] + B[1][0];
    C[1][1] = A[1][1] + B[1][1];
}
static inline void mat2_transpose(const float A[2][2], float AT[2][2])
{
    AT[0][0] = A[0][0];
    AT[0][1] = A[1][0];
    AT[1][0] = A[0][1];
    AT[1][1] = A[1][1];
}
static inline void mat2_vec2_mul(const float A[2][2], const float x[2], float y[2])
{
    y[0] = A[0][0] * x[0] + A[0][1] * x[1];
    y[1] = A[1][0] * x[0] + A[1][1] * x[1];
}
static inline void mat2_inv(const float A[2][2], float Ainv[2][2])
{
    float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    if (fabsf(det) < 1e-12f) {
        Ainv[0][0] = 1.0f;
        Ainv[0][1] = 0.0f;
        Ainv[1][0] = 0.0f;
        Ainv[1][1] = 1.0f;
        return;
    }
    float invdet = 1.0f / det;
    Ainv[0][0]   = A[1][1] * invdet;
    Ainv[0][1]   = -A[0][1] * invdet;
    Ainv[1][0]   = -A[1][0] * invdet;
    Ainv[1][1]   = A[0][0] * invdet;
}

/* ---------- 姿态：欧拉角 -> R_bw(body->world) ----------
   你说“顺序你会处理”，所以这里给一个常见实现；你按需要替换即可。 */
static void eul2R_bw_deg(float roll_deg, float pitch_deg, float yaw_deg, float R[3][3])
{
    float cr = arm_cos_f32(roll_deg * DEG2RAD);
    float sr = arm_sin_f32(roll_deg * DEG2RAD);
    float cp = arm_cos_f32(pitch_deg * DEG2RAD);
    float sp = arm_sin_f32(pitch_deg * DEG2RAD);
    float cy = arm_cos_f32(yaw_deg * DEG2RAD);
    float sy = arm_sin_f32(yaw_deg * DEG2RAD);

    /* R = Rz(yaw)*Ry(pitch)*Rx(roll) */
    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;

    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;

    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}

/*
  一个函数内完成：
  1) 原始加速度计输出 acc_b（包含重力） -> 去重力 -> 世界系X方向线加速度 a_wx
     - 做法：先 body->world 得到 acc_w，再减去重力向量 g_w 得到线加速度 linacc_w
  2) Kalman Filter：匀加速运动模型，状态转移F按 [v,a]
     - 状态 x = [v_x, a_x]^T
     - F = [[1, dt],
            [0,  1]]
     - 量测模型 H = I（单位阵）
     - 量测 z = [v_meas, a_wx]^T  （速度来自外部；加速度来自IMU推到世界系X）

  输入：
    dt        : s
    acc_b[3]  : 加速度计输出（m/s^2，包含重力）
    eul_deg[3]: 欧拉角（度），顺序你可自行保证
    vx_meas   : 世界系X方向速度量测（m/s）
  输出：
    返回滤波后的 v_x
    ax_out（可选）返回滤波后的 a_x
*/
float WorldXSpeedKF(float dt,
                    const float acc_b[3],
                    const float eul_deg[3],
                    float vx_meas,
                    float *ax_out)
{
    /* ---------- 1) acc_b -> acc_w ---------- */
    float Rbw[3][3];
    eul2R_bw_deg(eul_deg[0], eul_deg[1], eul_deg[2], Rbw);

    float acc_w[3];
    acc_w[0] = Rbw[0][0] * acc_b[0] + Rbw[0][1] * acc_b[1] + Rbw[0][2] * acc_b[2];
    acc_w[1] = Rbw[1][0] * acc_b[0] + Rbw[1][1] * acc_b[1] + Rbw[1][2] * acc_b[2];
    acc_w[2] = Rbw[2][0] * acc_b[0] + Rbw[2][1] * acc_b[1] + Rbw[2][2] * acc_b[2];

    /* 世界系重力向量（+Z向上时，重力是负Z） */
    const float g_w[3] = {0.0f, 0.0f, -G};

    /* 线加速度（世界系）= 测得加速度（世界系） - 重力（世界系） */
    float linacc_w[3];
    linacc_w[0] = acc_w[0] - g_w[0];
    linacc_w[1] = acc_w[1] - g_w[1];
    linacc_w[2] = acc_w[2] - g_w[2];

    float a_wx = linacc_w[0];

    /* ---------- 2) 2-state KF ---------- */
    static uint8_t inited = 0;
    static float x[2]; /* [v, a] */
    static float P[2][2];

    if (!inited) {
        x[0]    = vx_meas;
        x[1]    = a_wx;
        P[0][0] = 10.0f;
        P[0][1] = 0.0f;
        P[1][0] = 0.0f;
        P[1][1] = 10.0f;
        inited  = 1;
    }

    /* 状态转移 F */
    const float F[2][2] = {{1.0f, dt},
                           {0.0f, 1.0f}};

    /* 过程噪声 Q（建议你后续按数据标定/调参）
       这里用对角阵 + dt 缩放，简单可用 */
    const float q_v = 0.5f; /* (m/s)^2 / s */
    const float q_a = 8.0f; /* (m/s^2)^2 / s */
    float Q[2][2]   = {{q_v * dt, 0.0f},
                       {0.0f, q_a * dt}};

    /* 量测噪声 R：速度量测与IMU推算加速度的可信度 */
    const float r_v = 1.0f; /* 速度方差 */
    const float r_a = 6.0f; /* 加速度方差（姿态误差/震动会让它偏大） */
    float R[2][2]   = {{r_v, 0.0f},
                       {0.0f, r_a}};

    /* ---- Predict ---- */
    float x_prior[2];
    mat2_vec2_mul(F, x, x_prior);

    float FP[2][2], FT[2][2], FPFt[2][2], P_prior[2][2];
    mat2_mul(F, P, FP);
    mat2_transpose(F, FT);
    mat2_mul(FP, FT, FPFt);
    mat2_add(FPFt, Q, P_prior);

    /* ---- Update (H=I) ---- */
    float z[2] = {vx_meas, a_wx};

    float y[2] = {z[0] - x_prior[0],
                  z[1] - x_prior[1]};

    float S[2][2];
    mat2_add(P_prior, R, S);

    float Sinv[2][2], K[2][2];
    mat2_inv(S, Sinv);
    mat2_mul(P_prior, Sinv, K);

    x[0] = x_prior[0] + K[0][0] * y[0] + K[0][1] * y[1];
    x[1] = x_prior[1] + K[1][0] * y[0] + K[1][1] * y[1];

    float IminusK[2][2] = {{1.0f - K[0][0], -K[0][1]},
                           {-K[1][0], 1.0f - K[1][1]}};
    mat2_mul(IminusK, P_prior, P);

    if (ax_out) *ax_out = x[1];
    return x[0];
}