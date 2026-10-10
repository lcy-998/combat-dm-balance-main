#include "arm_math.h"
#include <math.h>
#include <stdio.h>
#include "user_lib.h"
#include "legalgorithm.h"
#include "ins_task.h"
#include "leg_const.hpp"

// 正运动学解算
void forward_kinematics(LEGInstance *leg, float phi1, float dphi1, float phi4, float dphi4)
{
    // 占位变量，实际并未使用
    float phi2_l, phi3_l, phi2_r, phi3_r;

    // 计算 phi2, phi3, L0, phi0
    getPhi(phi1, phi4, L_1, L_2, L_3, L_4, L_5,
           &phi2_l, &phi3_l, &(leg->result->L0), &(leg->result->theta0));

    // 计算 L0_dot 和 phi0_dot
    spd(dphi1, dphi4, L_1, L_2, L_3, L_4, L_5, phi1, phi4,
        &(leg->result->dl),
        &(leg->result->w),
        &(leg->result->ddl),
        &(leg->result->dw));
}
// 逆解用于位置控制
void Inverse_solution(float L0_set, float theta0_set, float *phi1_set, float *phi4_set)
{
    float phi11, phi21;
    phi11 = acosf((L0_set * L0_set + L_1 * L_1 - L_2 * L_2) / (2 * L0_set * L_1));
    phi21 = acosf((L0_set * L0_set + L_4 * L_4 - L_3 * L_3) / (2 * L0_set * L_4));
    *phi1_set = theta0_set + phi11;
    *phi4_set = theta0_set - phi21;
}

// getPhi: 计算 phi2, phi3, L0, phi0
void getPhi(float phi1, float phi4,
            float l1, float l2, float l3, float l4, float l5,
            float *phi2, float *phi3, float *L0, float *phi0)
{
    float x_B = -l5 / 2.0 + arm_cos_f32(phi1) * l1;
    float y_B = arm_sin_f32(phi1) * l1;
    float x_D = l5 / 2.0 + arm_cos_f32(phi4) * l4;
    float y_D = arm_sin_f32(phi4) * l4;

    float A_0 = 2.0 * l2 * (x_D - x_B);
    float B_0 = 2.0 * l2 * (y_D - y_B);
    float l_BD = sqrtf((x_D - x_B) * (x_D - x_B) + (y_D - y_B) * (y_D - y_B));
    float C_0 = l2 * l2 + l_BD * l_BD - l3 * l3;

    *phi2 = 2.0 * atan2f(B_0 + sqrtf(A_0 * A_0 + B_0 * B_0 - C_0 * C_0), A_0 + C_0);

    float x_C = -l5 / 2.0 + l1 * arm_cos_f32(phi1) + l2 * arm_cos_f32(*phi2);
    float y_C = 0.0 + l1 * arm_sin_f32(phi1) + l2 * arm_sin_f32(*phi2);

    *phi3 = atan2f(y_C - y_D, x_C - x_D);

    *L0 = sqrtf(x_C * x_C + y_C * y_C);
    *phi0 = atan2f(y_C, x_C);
}

// spd: 计算 L0_dot 和 phi0_dot
void spd(float dphi1, float dphi4,
         float l1, float l2, float l3, float l4, float l5,
         float phi1, float phi4,
         float *spd1, float *spd2, float *spd3, float *spd4)
{
    float t8 = l2 * l2;
    float t10 = l1 * arm_cos_f32(phi1);
    float t11 = l4 * arm_cos_f32(phi4);
    float t12 = l1 * arm_sin_f32(phi1);
    float t13 = l4 * arm_sin_f32(phi4);
    float t19 = 2.0 * l2 * t10;
    float t20 = 2.0 * l2 * t11;
    float t33 = t12 - t13;
    float t34 = l5 + t11 - t10;
    float t35 = t33 * t33;
    float t36 = t34 * t34;
    float t37 = 2.0 * l2 * t33;
    float t50 = 2.0 * (t10 * t33 + t12 * t34);
    float t51 = 2.0 * (t11 * t33 + t13 * t34);
    float t52 = t8 - l3 * l3 + t35 + t36;
    float t54 = 2.0 * l2 * t12 + t50;
    float t55 = 2.0 * l2 * t13 + t51;
    float t60 = 2.0 * t50 * t52;
    float t61 = 2.0 * t51 * t52;
    float t58 = 1.0 / (2.0 * l2 * t34 + t52);
    float t59 = t58 * t58;
    float t64 = 4.0 * t8 * t35 + 4.0 * t8 * t36 - t52 * t52;
    float t69 = 8.0 * t8 * t10 * t33 + 8.0 * t8 * t12 * t34 - t60;
    float t70 = 8.0 * t8 * t11 * t33 + 8.0 * t8 * t13 * t34 - t61;
    float t65 = sqrtf(t64);
    float t90 = -t54 * t59 * (t37 - t65);
    float t91 = -t55 * t59 * (t37 - t65);
    float t72 = atanf(-(t37 - t65) * t58);
    float t94 = 0.5 * (t69 / t65);
    float t95 = 0.5 * (t70 / t65);
    float t80 = 1.0 / (t59 * (t37 - t65) * (t37 - t65) + 1.0);
    float t78 = l2 * arm_cos_f32(2.0 * t72);
    float t79 = l2 * arm_sin_f32(2.0 * t72);
    float t86 = t12 + t79;
    float t88 = t10 - l5 / 2.0 + t78;
    float t112 = -t78 * t80 * (t90 + t58 * (t19 - t94));
    float t113 = -t78 * t80 * (t91 + t58 * (t20 - t95));
    float t114 = -t79 * t80 * (t90 + t58 * (t19 - t94));
    float t115 = -t79 * t80 * (t91 + t58 * (t20 - t95));
    float t100 = t12 + t79;
    float t103 = l5 / 2.0 - t10 - t78;
    float t104 = 1.0 / sqrtf(t86 * t86 + t88 * t88);
    float t105 = t103 * t103;
    float t106 = 1.0 / t103;
    float t107 = 1.0 / t105;
    float t111 = 1.0 / (t100 * t100 + t105);
    static uint32_t cnt;
    static float dt;

    *spd3 = *spd1, *spd4 = *spd2;

    *spd1 = 0.5 * dphi1 * t104 * (2.0 * t86 * (t10 - t78 * t80 * (t90 + t58 * (t19 - t94)) * 2.0) - 2.0 * t88 * (t12 - t79 * t80 * (t90 + t58 * (t19 - t94)) * 2.0)) + 0.5 * dphi4 * t104 * (4.0 * t78 * t80 * t86 * (t91 + t58 * (t20 - t95)) - 4.0 * t79 * t80 * t88 * (t91 + t58 * (t20 - t95)));

    *spd2 = dphi4 * t105 * t111 * (-2.0 * t106 * t113 + 2.0 * t100 * t107 * t115) + dphi1 * t105 * t111 * (2.0 * t106 * (t10 + 2.0 * t112) - 2.0 * t100 * t107 * (t12 + 2.0 * t114));
    dt = DWT_GetDeltaT(&cnt);
    *spd3 = (*spd1 - *spd3) / dt, *spd4 = (*spd2 - *spd4) / dt;
}

LEGInstance *FK_Result_init(leg_Init_Config_s *config)
{
    LEGInstance *leg_instance = (LEGInstance *)zmalloc(sizeof(LEGInstance)); // 动态分配内存

    // 初始化 result 的成员
    leg_instance->result = (FK_Result *)zmalloc(sizeof(FK_Result));
    leg_instance->result->L0 = 0;
    leg_instance->result->theta0 = 0;
    leg_instance->result->w = 0;
    leg_instance->result->dl = 0;
    leg_instance->result->dw = 0;
    leg_instance->result->ddl = 0;

    // 初始化PID控制器
    PIDInit(&leg_instance->leglCtrl, &config->lpid_config);
    PIDInit(&leg_instance->legvCtrl, &config->vpid_config);

    return leg_instance;
}

ctrl_pair vmc_get_tau(float F_bl, float T_bl, float l1, float phi1, float phi2, float l, float theta)
{
    ctrl_pair result = {0, 0};

    float xe = l * arm_sin_f32(theta), ye = l * arm_cos_f32(theta);
    float x1 = -l1 * arm_cos_f32(phi1), y1 = l1 * arm_sin_f32(phi1);
    float x2 = -l1 * arm_cos_f32(phi2), y2 = l1 * arm_sin_f32(phi2);

    float phi1_diff_l = ((xe + x1) * arm_sin_f32(theta) + (ye - y1) * arm_cos_f32(theta)) / (l * (-(xe + x1) * arm_sin_f32(phi1) + (ye - y1) * arm_cos_f32(phi1)));
    float phi1_diff_theta = (l * ((xe + x1) * arm_cos_f32(theta) - (ye - y1) * arm_sin_f32(theta))) / (l1 * (-(xe + x1) * arm_sin_f32(phi1) + (ye - y1) * arm_cos_f32(phi1)));
    float phi2_diff_l = ((xe - x2) * arm_sin_f32(theta) + (ye - y2) * arm_cos_f32(theta)) / (l * ((xe - x2) * arm_sin_f32(phi2) + (ye - y2) * arm_cos_f32(phi2)));
    float phi2_diff_theta = (l * ((xe - x2) * arm_cos_f32(theta) - (ye - y2) * arm_sin_f32(theta))) / (l1 * ((xe - x2) * arm_sin_f32(phi2) + (ye - y2) * arm_cos_f32(phi2)));

    float det = phi1_diff_l * phi2_diff_theta - phi2_diff_l * phi1_diff_theta;
    if (det == 0)
        return result;

    /*
    J:[a b
       c d]
    */
    float a = phi2_diff_theta / det;
    float b = -phi2_diff_l / det;
    float c = -phi1_diff_theta / det;
    float d = phi1_diff_l / det;

    result.tau1 = F_bl * a + T_bl * b;
    result.tau2 = F_bl * c + T_bl * d;
    return result;
}

void TorqueCalculation(float L_set, float v_set, LEGInstance *leg_instance)
{
    leg_instance->tor = PIDCalculate(&leg_instance->leglCtrl, leg_instance->result->L0, L_set);
    leg_instance->tor += PIDCalculate(&leg_instance->legvCtrl, leg_instance->result->dl, v_set);
}
void WheelKfInit(Kalman *kf)
{
    kf->dt = 0.001;

    kf->x[0] = 0.0;
    kf->x[1] = 0.0;

    kf->P[0][0] = 1.0;
    kf->P[0][1] = 0.0;
    kf->P[1][0] = 0.0;
    kf->P[1][1] = 1.0;

    kf->Q[0][0] = 1e-4;
    kf->Q[0][1] = 0.0;
    kf->Q[1][0] = 0.0;
    kf->Q[1][1] = 1e-4;

    kf->R[0][0] = 1e-5;
    kf->R[0][1] = 0.0;
    kf->R[1][0] = 0.0;
    kf->R[1][1] = 1e-5;
}
void kalman_predict(Kalman *kf, float a)
{
    double dt = kf->dt;

    /* x = A x + B u */
    kf->x[0] += kf->x[1] * dt + 0.5 * dt * dt * a;
    kf->x[1] += a * dt;

    /* P = A P A^T + Q */
    double P00 = kf->P[0][0] + dt * (kf->P[1][0] + kf->P[0][1]) + dt * dt * kf->P[1][1] + kf->Q[0][0];
    double P01 = kf->P[0][1] + dt * kf->P[1][1];
    double P10 = kf->P[1][0] + dt * kf->P[1][1];
    double P11 = kf->P[1][1] + kf->Q[1][1];

    kf->P[0][0] = P00;
    kf->P[0][1] = P01;
    kf->P[1][0] = P10;
    kf->P[1][1] = P11;
}
void kalman_update(Kalman *kf, float zx, float zv) // 测量
{
    double S00 = kf->P[0][0] + kf->R[0][0];
    double S01 = kf->P[0][1];
    double S10 = kf->P[1][0];
    double S11 = kf->P[1][1] + kf->R[1][1];

    double det = S00 * S11 - S01 * S10;
    if (det < 1e-12)
        return;

    double invS00 = S11 / det;
    double invS01 = -S01 / det;
    double invS10 = -S10 / det;
    double invS11 = S00 / det;

    /* K = P * inv(S) */
    double K00 = kf->P[0][0] * invS00 + kf->P[0][1] * invS10;
    double K01 = kf->P[0][0] * invS01 + kf->P[0][1] * invS11;
    double K10 = kf->P[1][0] * invS00 + kf->P[1][1] * invS10;
    double K11 = kf->P[1][0] * invS01 + kf->P[1][1] * invS11;

    double y0 = zx - kf->x[0];
    double y1 = zv - kf->x[1];

    kf->x[0] += K00 * y0 + K01 * y1;
    kf->x[1] += K10 * y0 + K11 * y1;

    /* P = (I - K) P */
    double P00 = (1 - K00) * kf->P[0][0] - K01 * kf->P[1][0];
    double P01 = (1 - K00) * kf->P[0][1] - K01 * kf->P[1][1];
    double P10 = -K10 * kf->P[0][0] + (1 - K11) * kf->P[1][0];
    double P11 = -K10 * kf->P[0][1] + (1 - K11) * kf->P[1][1];

    kf->P[0][0] = P00;
    kf->P[0][1] = P01;
    kf->P[1][0] = P10;
    kf->P[1][1] = P11;
}

void KFtask(Kalman *kf, float s, float ds, float dds)
{
    static uint32_t feed_cnt;
    kf->dt = DWT_GetDeltaT(&feed_cnt);
    if (!kf->dt)
        kf->dt = 0.00001;
    kalman_predict(kf, dds);
    kalman_update(kf, s, ds);
}
