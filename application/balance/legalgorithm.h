#ifndef LEG_ALGORITHM_H
#define LEG_ALGORITHM_H
#include "controller.h"
// 结构体用于存储正向运动学计算结果
typedef struct {
    float L0;     // 末端位置的距离
    float theta0; // 末端执行器的角度
    float w;      // 末端角速度
    float dl;     // 末端线速度
    float dw;     // 摆角二阶导
    float ddl;    // 腿长二阶导
} FK_Result;

typedef struct
{
    FK_Result *result;
    PIDInstance leglCtrl;
    PIDInstance legvCtrl;
    float tor;
    // leg_Working_Type_e stop_flag; // 启停标志

} LEGInstance;

typedef struct
{
    PID_Init_Config_s lpid_config;
    PID_Init_Config_s vpid_config;
} leg_Init_Config_s;

typedef struct {
    float tau1, tau2;
} ctrl_pair;

void FK(float theta1, float theta2, float L1, float L2, float dtheta1, float dtheta2, LEGInstance *leg_instance);
LEGInstance *FK_Result_init(leg_Init_Config_s *config);
void TorqueCalculation(float L_set, float v_set, LEGInstance *leg_instance);
void getPhi(float phi1, float phi4,
            float l1, float l2, float l3, float l4, float l5,
            float *phi2, float *phi3, float *L0, float *phi0);
void spd(float dphi1, float dphi4,
         float l1, float l2, float l3, float l4, float l5,
         float phi1, float phi4,
         float *spd1, float *spd2, float *spd3, float *spd4);
void forward_kinematics(LEGInstance *leg, float phi1, float dphi1, float phi4, float dphi4);
void Inverse_solution(float L0_set, float theta0_set, float *phi1_set, float *phi4_set);
ctrl_pair vmc_get_tau(float F_bl, float T_bl, float l1, float phi1, float phi2, float l, float theta);
typedef struct {
    float x[2];    // 状态 [x, v]
    float P[2][2]; // 协方差
    float Q[2][2]; // 过程噪声
    float R[2][2]; // 测量噪声
    float A[2][2]; // 状态转移
    float B[2];    // 控制矩阵
    float dt;      // 采样周期

} Kalman;

typedef struct {
    uint8_t fly_flag;
    float force;
    float zw_ddot;
} LegState;

void WheelKfInit(Kalman *kf);
void KFtask(Kalman *kf, float s, float ds, float dds);

#endif // !LEG_ALGORITHM_H