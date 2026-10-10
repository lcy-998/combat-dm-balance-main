#ifndef DMmotor_H
#define DMmotor_H

#include "stdint.h"
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"
#define DM_MOTOR_MX_CNT 4 // 最多允许4个DM电机使用多电机指令,挂载在一条总线上
#define DMP_MIN -12.5f
#define DMP_MAX 12.5f
#define DMV_MIN -45.0f
#define DMV_MAX 45.0f
#define DMKP_MIN 0.0f
#define DMKP_MAX 500.0f
#define DMKD_MIN 0.0f
#define DMKD_MAX 5.0f
#define DMT_MIN -54.0f
#define DMT_MAX 54.0f

typedef struct
{
    uint8_t mode;
    float pos_set;
    float vel_set;
    float tor_set;
    float kp_set;
    float kd_set;
} motor_ctrl_t;
typedef struct
{
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
} motor_fbpara_t;

typedef struct
{
    Motor_Type_e motor_type; // 电机类型

    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器

    CANInstance *motor_can_instance; // 电机CAN实例
    MotorSet motor_set;
    motor_ctrl_t ctrl;

    motor_fbpara_t measure;         // 电机测量值
    Motor_Working_Type_e stop_flag; // 启停标志
    DaemonInstance *daemon;
    uint32_t feed_cnt;
    float dt;
    uint8_t online;
} DMMotorInstance;

// 电机回传信息结构体

/**
 * @brief 初始化DM电机
 *
 * @param config 电机配置
 * @return DMMotorInstance* 返回实例指针
 */
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);
void DMMotorControl();
void DMMotorSetRef(DMMotorInstance *motor, float ref);
void DMMotorEnableMode(DMMotorInstance *motor);
void DMMotorSetTorque(DMMotorInstance *motor, float torque);
void DMMotorStop(DMMotorInstance *motor);
void DMMotorStart(DMMotorInstance *motor);
void DMMotorSetPos(DMMotorInstance *motor, float pos, float vel);
void DMMotorEnable1(DMMotorInstance *motor);
#endif // DMmotor_H