/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn，ssjjzero01@gmail.com
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "super_cap.h"
#include "message_center.h"
#include "LQR.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "referee_task.h"
#include "rm_referee.h"
#include "arm_math.h"
#include "power_calc.h"
#include "tool.h"
#include "legalgorithm.h"
#include "leg_const.hpp"
#include "controller.h"
#include "ins_task.h"
#include "chassis_can_link.h"
#include "chassis_cmd_watchdog.h"
#include "rl_shadow.h"
#include "rl_control.h"
#include "wheel_friction.h"
#include "buzzer.h"
#include "FreeRTOS.h"
#include "task.h"
#ifdef CHASSIS_BOARD
static CANInstance *chassis_can;
static ChassisCanCtrlRxCtx chassis_ctrl_rx_ctx;
static Chassis_Ctrl_Cmd_s chassis_can_cmd;
static ChassisCmdWatchdog chassis_cmd_watchdog;
#endif
#include "vel_meas.h"
/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 一些安装和机械引入的值 */
#define REDUCE_RATIO (268.0f / 17.0f)
// #define LF_ENCODE_OFFSET 5.54346179038468985769f - 5.4739542f + PI / 2
// #define LB_ENCODE_OFFSET 0.17422222338468985769f - 4.38848686f + PI / 2
// #define RF_ENCODE_OFFSET 10.23450102679489661923f + 1.07225978f - 9.21969128f - 1.0722599 + 1.0774368
// #define RB_ENCODE_OFFSET 4.95883163379489661923f - 0.496213317 - 3.14923954f
#define LF_ENCODE_OFFSET 1.640323924f  // (0.0695075904 + 1.570816333)
#define LB_ENCODE_OFFSET -2.643448303f // (-4.2142646366 + 1.570816333)
#define RF_ENCODE_OFFSET 1.092286447f - 0.618772388 + PI / 2 + 0.03721809
#define RB_ENCODE_OFFSET 1.313378777f - 1.5654273 + PI / 2 - 0.0021917
#define LF 0
#define RF 1
#define RB 2
#define LB 3
#define SPRING_TOR_SHORT 21.4f
#define SPRING_TOR_LONG 16.4f
#define SPRING_TOR_SWITCH_L0 0.251771474f
#define ROTATE_WZ_START 220.0f
#define ROTATE_WZ_TARGET 1100.0f
#define ROTATE_WZ_STEP 0.80f
#define ROTATE_WZ_DOWN_STEP 1.60f
#define LEG_SWING_CUTOFF_RAD (PI / 2.0f)
#define UPSTAIR_ACCEL_CUTOFF -3.5f
#define STANDUP_GIMBAL_RESET_ERR_DEG 8.0f
#define STANDUP_GIMBAL_RESET_CONFIRM_CNT 20u
#define STEP2_RETRACT_CONFIRM_CNT 100u
#define FOLLOW_GIMBAL_DEADBAND_DEG 1.0f
#define FOLLOW_GIMBAL_KP 8.5f
#define FOLLOW_GIMBAL_WZ_MAX 320.0f
#define CHASSIS_WZ_LIMIT 420.0f
#define LEG_RETRACT_TORQUE_SOFTSTART_RC 0.05f
#define LEG_RETRACT_TORQUE_RELEASE_RC 0.01f
#define V_CMD_MAX_ACCEL 100.0f
#define CHASSIS_CMD_TIMEOUT_MS 50u

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "bsp_can.h"
#include "ins_task.h"
static Subscriber_t *chassis_sub; // 用于订阅底盘的控制命令
// Ch040UsartInstance *imu;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
;                                     // 保留空语句,兼容现有条件编译结构
static Subscriber_t *chassis_sub;     // 用于订阅底盘的控制命令
static Subscriber_t *gimbal_feed_sub; // 云台反馈信息订阅者
static Publisher_t *chassis_pub;
#endif                                                                                 // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;                                            // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data;                                    // 底盘回传的反馈数据
SuperCapInstance *cap;                                                                 // 超级电容
static DJIMotorInstance *motor_l, *motor_r;                                            // 左右轮电机（前后驱动）
static DMMotorInstance *jiont_motorlf, *jiont_motorlb, *jiont_motorrf, *jiont_motorrb; // 底盘关节电机
static LEGInstance *Rleg, *Lleg;

float yaw, pitch, roll, yaw_dot, pitch_dot, roll_dot, yawtotlangle, yawlast = 0;
int16_t yawround = 0;
float s, s_dot, phi, phi_dot, theta_l, theta_l_dot, theta_r, theta_r_dot, theta_b, theta_b_dot;
Kalman kf;
static float Rwheel, Lwheel;
static float chassis_wz_cmd;
float lfangle, rfangle, lbangle, rbangle, lmangle, rmangle, lfw, lbw, rfw, rbw;
float lbt, rbt, lwt, rwt, lwi, rwi, lfjt, lbjt, rfjt, rbjt;
static float wheel_send_current[2];
static float wheel_drive_snapshot[2];
static uint8_t wheel_rl_snapshot;
float wheel_friction_torque_nm[2]; /* physical left/right, forward positive */
float l_tar, r_tar, sol_alpha, sol_beta;
double ChangeL;
float v_cmd, w_cmd, phi_tar = 0, s_tar, theta_bal = 0.129f, miu = 0.8f; // 4.0f;
float F_roll_vmc, F_l_vmc, F_g_vmc, F_in_vmc, T_vmc;
ctrl_pair tau_l, tau_r;
float l_xx = 0.16, ttl = 1, ttr = 0;
float a_xb, a_yb, a_zb, a_w_l, a_w_r, a_w;
static float rl_heading_vx_raw;
static uint32_t rl_snapshot_sequence;
static RLControlState rl_control_state;
static uint32_t rl_activation_sequence;
static RLControlLegRamp rl_leg_ramp;
static BuzzzerInstance *rl_switch_buzzer;
static uint32_t rl_buzzer_off_tick;
static uint32_t rl_buzzer_gap_until_tick;
static uint8_t rl_buzzer_on;
static uint8_t rl_buzzer_pending;
static octave_e rl_buzzer_pending_note;
static uint8_t rl_joint_mode;
float F_l, F_r, T_l, T_r, P_l, P_r, F_n, MG;
float setLangle, setRangle, setLangle_tar, setRangle_tar;
float rotate;
static float rotate_wz_soft = ROTATE_WZ_START;
static float rotate_wz_exit = 0.0f;
static uint8_t rotate_wz_softstop_active = 0;
static int done = 0;
static uint16_t upstair_flag0_cnt = 0;
static uint8_t upstair_flag0_latch = 0;
LegState leg_state_l, leg_state_r;
uint8_t clockcent = 0, jumpingflag = 0, upstair_flag = 0, upstair_step = 0, last_mode = 0, switch_flag = 0, upstair_flag0 = 0;
uint8_t step_0 = 0, step_1 = 0, step_2 = 0, step_3 = 0;
uint8_t chassis_status = 1; // 0正常 1复位中
float v_set = 0, radior, radiol;
uint64_t cent = 0;
static float del_t;
static uint32_t balance_dwt_cnt;
float phi1, phi4, Lphi1, Lphi4, Rphi1, Rphi4;
float LFtor, LBtor, RFtor, RBtor;
float absds;
float thetar = 0.04, thetal = 0.04;
float RFW, LFW;
static attitude_t *IMU_data;       // 云台IMU数据
int reset_time = 0, save_time = 0; // 起身时间计数,离地检测豁免
static uint8_t standup_gimbal_reset_flag = 0;
static uint16_t standup_gimbal_reset_cnt = 0;
static uint16_t step2_retract_confirm_cnt = 0;
static float leg_retract_torque_scale = 0.0f;
static float v_cmd_last = 0.0f;
static uint8_t position_hold_latched = 0u;
static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static float UpdateLegRetractTorqueScale(bool enable)
{
    float dt = del_t;
    float target = enable ? 1.0f : 0.0f;
    float rc = enable ? LEG_RETRACT_TORQUE_SOFTSTART_RC : LEG_RETRACT_TORQUE_RELEASE_RC;

    if (dt <= 0.0f)
    {
        dt = 0.001f;
    }

    leg_retract_torque_scale += (dt / (rc + dt)) * (target - leg_retract_torque_scale);
    if (leg_retract_torque_scale < 0.0f)
        leg_retract_torque_scale = 0.0f;
    if (leg_retract_torque_scale > 1.0f)
        leg_retract_torque_scale = 1.0f;

    return leg_retract_torque_scale;
}

static void GetLegRetractTorquePair(bool enable, float *left_torque, float *right_torque)
{
    float scale = UpdateLegRetractTorqueScale(enable);
    *left_torque = scale * Lleg->tor;
    *right_torque = scale * Rleg->tor;
}

static float GetSpringTorqueByLegLength(float leg_length)
{
    float spring_tor = (-83.121f * leg_length + 40.242f) * 0.8f;
    if (spring_tor < 16.5f)
        spring_tor = 16.5f;
    else if (spring_tor > 27.0f)
        spring_tor = 27.0f;
    return spring_tor;
}

static PIDInstance Pitch = {
    .Kp = 12.0f,
    .Ki = 0.0f,
    .Kd = 0.5f,
    .DeadBand = 0.06f,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .IntegralLimit = 2,
    .MaxOut = 5.0};
static PIDInstance Rtheta = {
    .Kp = 8.0f,
    .Ki = 0.0f,
    .Kd = 0.5f,
    .DeadBand = 0.03f,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .Last_Dout = 0.01,
    .IntegralLimit = 2,
    .MaxOut = 6.5};
static PIDInstance Ltheta = {
    .Kp = 8.0f,
    .Ki = 0.0f,
    .Kd = 0.5f,
    .Last_Dout = 0.01,
    .DeadBand = 0.03f,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .IntegralLimit = 2,
    .MaxOut = 6.5};
static PIDInstance F_roll_vmc_pid_instance =
    {
        .Kp = 150.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .DeadBand = 0.0f,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 2,
        .MaxOut = 20.0};
static PIDInstance Langle_pid_instance = {
    .Kp = -30.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .DeadBand = 0.0f,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .IntegralLimit = 2,
    .MaxOut = 20.0};
static PIDInstance Rangle_pid_instance = {
    .Kp = 30.0f,
    .Ki = 0.0f,
    .Kd = 0.0f,
    .DeadBand = 0.0f,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    .IntegralLimit = 2,
    .MaxOut = 20.0};
void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hfdcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 0.0, // 4.5
                .Ki = 0.0, // 0
                .Kd = 0.0, // 0
                .IntegralLimit = 800,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = WHEEL_CURRENT_LIMIT,
            }},
        .controller_setting_init_config = {
            .feedforward_flag = CURRENT_FEEDFORWARD,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = OPEN_LOOP,
            .close_loop_type = OPEN_LOOP,
        },
        .motor_type = M3508,
    };
    Motor_Init_Config_s jiontmotor = {
        .can_init_config.can_handle = &hfdcan1,
        .motor_contro_type = TORQUE_LOOP_CONTRO, // 控制类型,电流
        .motor_set = {
            .PosMXSet = 6.283f,
            .VelMXSet = 45.0f,
            .KpMXSet = 500.0f,
            .KdMXSet = 5.0f,
            .TorMXSet = 40.0f,
            .flag = 1},
    };
    leg_Init_Config_s config = {
        .lpid_config = {
            .Kp = 250.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .DeadBand = 0.0001f,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .IntegralLimit = 3,
            .MaxOut = 15.0f},
        .vpid_config = {.Kp = 25.0f, .DeadBand = 0.4f, .Improve = PID_Integral_Limit, .IntegralLimit = 5000, .MaxOut = 6}};

    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.can_handle = &hfdcan1;
    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_r = DJIMotorInit(&chassis_motor_config);
    chassis_motor_config.can_init_config.can_handle = &hfdcan2;
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_l = DJIMotorInit(&chassis_motor_config);
    /* The motor task may run before the first non-emergency chassis cycle. */
    motor_l->motor_controller.current_feedforward_ptr = &wheel_send_current[0];
    motor_r->motor_controller.current_feedforward_ptr = &wheel_send_current[1];
    jiontmotor.can_init_config.can_handle = &hfdcan1;
    jiontmotor.can_init_config.tx_id = 0x01;
    jiontmotor.can_init_config.rx_id = 0x11; // l电机的接收id
    jiont_motorrf = DMMotorInit(&jiontmotor);
    jiontmotor.can_init_config.tx_id = 0x02;
    jiontmotor.can_init_config.rx_id = 0x12; // l电机的接收id
    jiont_motorrb = DMMotorInit(&jiontmotor);
    jiontmotor.can_init_config.can_handle = &hfdcan2;
    jiontmotor.can_init_config.tx_id = 0x04;
    jiontmotor.can_init_config.rx_id = 0x14; // r电机的接收id
    jiont_motorlb = DMMotorInit(&jiontmotor);
    jiontmotor.can_init_config.tx_id = 0x03;
    jiontmotor.can_init_config.rx_id = 0x13; // r电机的接收id
    jiont_motorlf = DMMotorInit(&jiontmotor);
    // SuperCap_Init_Config_s cap_conf  = {
    //      .can_config = {
    //          .can_handle = &hfdcan2,
    //          .tx_id      = 0X255, // 超级电容默认接收id
    //          .rx_id      = 0x100, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
    //     }};
    // cap    = SuperCapInit(&cap_conf); // 超级电容初始化
    Rleg = FK_Result_init(&config);
    Lleg = FK_Result_init(&config);
    Rwheel = motor_r->measure.total_angle;
    Lwheel = motor_l->measure.total_angle;
    WheelKfInit(&kf);
    IMU_data = INS_Init();
    Buzzer_config_s rl_buzzer_config = {
        .alarm_level = ALARM_LEVEL_LOW,
        .octave = OCTAVE_5,
        .loudness = 0.4f,
    };
    rl_switch_buzzer = BuzzerRegister(&rl_buzzer_config);
    // imu             = Ch040_USART_Init();
    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    CAN_Init_Config_s can_config = {
        .can_handle = &hfdcan3,
        .tx_id = 0x11,
        .rx_id = 0x22,
        .can_module_callback = ChassisCanCtrlRxCallback,
        .id = &chassis_ctrl_rx_ctx,
    };
    chassis_ctrl_rx_ctx.ctrl_dst = &chassis_can_cmd;
    chassis_can = CANRegister(&can_config);
    CANSetDLC(chassis_can, sizeof(Chassis_Upload_Data_CAN_s));
    referee_data = UITaskInit(&huart1, &ui_data); // 裁判系统初始化,会同时初始化UI

#endif // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));

    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
#endif // ONE_BOARD
}

float acc, dscov, acclast, ds, yabs, zabs, MotionAccel_b[3];

float map_linear(float x, float in_min, float in_max, float out_min, float out_max)
{
    if (fabs(x) < 4000)
        x = 0;
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// float SpeedEstimation(float dt, imu_data_t imudata, float dsm)
// {
//     const float g[3] = {0, 0, 1.0f};
//     float g_b[3];
//     EarthFrameToBodyFrame(g, g_b, imudata.quat);
//     for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
//     {
//         MotionAccel_b[i] = (imudata.acc[i] - g_b[i]);
//     }
//     static float vel_prior, vel_measure, vel_cov; // 先验估计、测量、先验协方差
//     yabs = MotionAccel_b[0];
//     zabs = MotionAccel_b[2];
//     acc = imudata.acc[0] * arm_cos_f32(-imudata.eul[0] * DEGREE_2_RAD) - imudata.acc[2] * arm_sin_f32(-imudata.eul[0] * DEGREE_2_RAD);
//     // if (fabs(acc) < 0.01) acc = 0.0f;
//     static float u, k;
//     // 预测
//     u = (acc + acclast) / 2;
//     vel_prior = ds + u * dt;
//     vel_cov = dscov + 10 * dt;
//     // 校正
//     vel_measure = dsm;
//     k = vel_cov / (vel_cov + 1000.0f);
//     ds = vel_prior + k * (vel_measure - vel_prior);
//     dscov = (1.0f - k) * vel_cov;
//     acclast = acc;
//     VAL_LIMIT(dscov, 0.01f, 100.0f);
//     return ds;
// }

void GetAngleOffset(float p1, float p4, float a1, float a4, float *offset1, float *offset4)
{
    float d1 = a1 - p1;
    float d4 = a4 - p4;

    while (d1 >= PI)
        d1 -= PI2;
    while (d1 < -PI)
        d1 += PI2;

    while (d4 >= PI)
        d4 -= PI2;
    while (d4 < -PI)
        d4 += PI2;

    *offset1 = d1;
    *offset4 = d4;
}

void getLegData()
{
    // 获得四个关节的角度和速度和气弹簧力臂与力夹角,并进行一些预处理
    lfangle = -jiont_motorlf->measure.pos + LF_ENCODE_OFFSET;
    lbangle = -jiont_motorlb->measure.pos + LB_ENCODE_OFFSET;
    rfangle = jiont_motorrf->measure.pos + RF_ENCODE_OFFSET;
    rbangle = jiont_motorrb->measure.pos + RB_ENCODE_OFFSET;
    // 夹角
    lmangle = lfangle - lbangle, rmangle = rfangle - rbangle;

    lfw = -jiont_motorlf->measure.vel;
    lbw = -jiont_motorlb->measure.vel;
    rfw = jiont_motorrf->measure.vel;
    rbw = jiont_motorrb->measure.vel;
    // 对3508电机的速度进行死区处理,防止小速度下的抖动,这个值可以根据实际情况调整
    if (fabsf(lfw) < 0.022f)
        lfw = 0;
    if (fabsf(lbw) < 0.022f)
        lbw = 0;
    if (fabsf(rfw) < 0.022f)
        rfw = 0;
    if (fabsf(rbw) < 0.022f)
        rbw = 0;

    // 正运动学解算，包含了角度和速度的解算，得到末端执行器的位置、角度、线速度和角速度等信息
    forward_kinematics(Lleg, lfangle, lfw, lbangle, lbw);
    forward_kinematics(Rleg, rfangle, rfw, rbangle, rbw);
}

void getStateData()
{
    // 获取机体上陀螺仪的数据
    /*
    pitch     = -imu->ch040_data.eul[0] * DEGREE_2_RAD;
    pitch_dot = -imu->ch040_data.gyr[1] * DEGREE_2_RAD;
    roll      = -imu->ch040_data.eul[1] * DEGREE_2_RAD;
    roll_dot  = -imu->ch040_data.gyr[0] * DEGREE_2_RAD;
    yaw       = imu->ch040_data.eul[2] * DEGREE_2_RAD;
    yaw_dot   = imu->ch040_data.gyr[2] * DEGREE_2_RAD;
    */

    // 获取c板陀螺仪数据 （由于安装正方向与官方推荐不一致）
    yaw = IMU_data->Yaw * DEGREE_2_RAD;
    yaw_dot = IMU_data->Gyro[2];
    pitch = IMU_data->Pitch * DEGREE_2_RAD;
    pitch_dot = IMU_data->Gyro[0];
    roll = -IMU_data->Roll * DEGREE_2_RAD;
    roll_dot = -IMU_data->Gyro[1];

    // yaw的处理,进行360度连续化,并且计算总的旋转角度,这个值对于后续的控制很重要,需要保证其正确性
    if ((yawlast - yaw) > PI)
        yawround++;
    if ((yawlast - yaw) < -PI)
        yawround--;
    yawtotlangle = yawround * PI * 2 + yaw;
    yawlast = yaw;

    // 摆角转动应该会带动轮子编码器的值改变，这里没考虑，后续验证后加入
    // 机体倾角对摆角影响已经消除
    s = R_W * (-motor_l->measure.total_angle / REDUCE_RATIO + Lwheel / REDUCE_RATIO + motor_r->measure.total_angle / REDUCE_RATIO - Rwheel / REDUCE_RATIO) / 2 * DEGREE_2_RAD,
    s_dot = R_W * ((-motor_l->measure.speed_aps + motor_r->measure.speed_aps) / REDUCE_RATIO * DEGREE_2_RAD + Lleg->result->w + Rleg->result->w) / 2,
    phi = yaw,
    phi_dot = yaw_dot,
    theta_l = (PI / 2 - Lleg->result->theta0) + pitch,
    theta_l_dot = Lleg->result->w + pitch_dot,
    theta_r = (PI / 2 - Rleg->result->theta0) + pitch,
    theta_r_dot = Rleg->result->w + pitch_dot;

    theta_b = pitch;

    theta_b_dot = pitch_dot;
    // 加入死区避免过大的thetab权重导致后退
    // 速度需要滤波以防打滑
    float acc_b[3] = {-IMU_data->Accel[1], -IMU_data->Accel[0], IMU_data->Accel[2]};
    float eul_deg[3] = {roll * RAD_2_DEG, pitch * RAD_2_DEG, yaw * RAD_2_DEG};
    rl_heading_vx_raw = s_dot;
    s_dot = WorldXSpeedKF(del_t, acc_b, eul_deg, s_dot, NULL);
}

static void PublishRLShadowSnapshot(void)
{
    RLShadowSnapshot sample = {0};
    sample.sequence = ++rl_snapshot_sequence;
    sample.tick_ms = xTaskGetTickCount();
    sample.chassis_mode = chassis_cmd_recv.chassis_mode;
    sample.chassis_ready = (chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE && !chassis_status);
    sample.rl_active = rl_joint_mode;
    sample.rl_activation_sequence = rl_activation_sequence;
    sample.odom_s_m = s;
    sample.target_s_m = s_tar;
    sample.remote_vx_m_s = v_cmd;
    sample.roll_rad = roll;
    sample.pitch_rad = pitch;
    sample.roll_rate_rad_s = roll_dot;
    sample.pitch_rate_rad_s = pitch_dot;
    sample.yaw_rate_rad_s = yaw_dot;
    sample.target_yaw_rad_s = w_cmd;
    sample.heading_vx_m_s = rl_heading_vx_raw;
    sample.lqr_speed_m_s = s_dot;
    sample.joint_angle_rad[0] = lfangle;
    sample.joint_angle_rad[1] = lbangle;
    sample.joint_angle_rad[2] = rfangle;
    sample.joint_angle_rad[3] = rbangle;
    sample.joint_rate_rad_s[0] = lfw;
    sample.joint_rate_rad_s[1] = lbw;
    sample.joint_rate_rad_s[2] = rfw;
    sample.joint_rate_rad_s[3] = rbw;
    /* Both model wheel joints have negative velocity during forward rolling. */
    sample.wheel_rate_rad_s[0] = motor_l->measure.speed_aps / REDUCE_RATIO * DEGREE_2_RAD;
    sample.wheel_rate_rad_s[1] = -motor_r->measure.speed_aps / REDUCE_RATIO * DEGREE_2_RAD;
    sample.lqr_torque_nm[0] = lwt;
    sample.lqr_torque_nm[1] = rwt;
    sample.lqr_torque_nm[2] = lbt;
    sample.lqr_torque_nm[3] = rbt;
    /* Match the positive active/hip action axes used by the RL observation. */
    sample.motor_torque_nm[0] = -jiont_motorlf->measure.tor;
    sample.motor_torque_nm[1] = -jiont_motorlb->measure.tor;
    sample.motor_torque_nm[2] = jiont_motorrf->measure.tor;
    sample.motor_torque_nm[3] = -jiont_motorrb->measure.tor;
    memcpy(sample.previous_action, rl_control_state.previous_action,
           sizeof(sample.previous_action));
    if (sample.chassis_ready) {
        sample.wheel_current_cmd[0] = lwi;
        sample.wheel_current_cmd[1] = rwi;
    }
    RLShadowPublish(&sample);
}

static int ChassisNormalLQRState(void)
{
    return chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE &&
           !chassis_status && !(jumpingflag && clockcent < 200u) &&
           !upstair_flag && !upstair_flag0 &&
           fabs(theta_l) <= LEG_SWING_CUTOFF_RAD &&
           fabs(theta_r) <= LEG_SWING_CUTOFF_RAD;
}

void legLengthCalc()
{
    // 复位时腿长压低
    if (chassis_status)
        l_xx = 0.15;

    // 简单状态机通过拨杆调整腿长，数值是控制周期乘步长
    switch (chassis_cmd_recv.changeL)
    {
    case 1:

        l_xx -= 0.002f * 0.3f;
        break;
    case -1:
        // jumpingflag = 1;
        // clockcent = 0;
        l_xx += 0.002f * 0.3f;
        break;
    case 2:
        upstair_flag0 = 1;
        upstair_flag0_latch = 0;
        break;
    case -2:
        // upstair_flag = 1;
        clockcent = 0;
        break;
    default:
        l_xx += 0.0;
        break;
    }

    // 腿长限位
    l_xx = fmax(0.16f, l_xx);
    l_xx = fmin(0.36f, l_xx);

    /* Extend the target over the first five seconds of normal LQR operation. */
    const int can_extend = ChassisNormalLQRState() &&
                           !(reset_time >= 5000 && F_n < 80.0f);
    l_xx = RLControlLegRampStep(&rl_leg_ramp, xTaskGetTickCount(),
                                can_extend, l_xx);

    // 腿长差补偿拟通过roll补偿解决，当然之前的计算方法也可以保留，后续验证后再决定是否加入
    l_tar = l_xx;
    //+(Lleg->result->L0 - Rleg->result->L0) / 2;
    r_tar = l_xx;
    //   -(Lleg->result->L0 - Rleg->result->L0) / 2;

    // 计算力矩
    TorqueCalculation(l_tar, 0, Lleg);
    TorqueCalculation(r_tar, 0, Rleg);
    lbt = get_lb_torque();
    rbt = get_rb_torque();
    rwt = get_rw_torque();
    lwt = get_lw_torque();
    if (upstair_flag0)
    {
        lbt *= 0.7;
        rbt *= 0.7;
        if (s_dot > 0.8)
        {
            lbt *= 0.4;
            rbt *= 0.4;
            thetal = 0.09f;
            thetar = 0.09f;
        }
    }
    else
    {
        thetal = 0.04f;
        thetar = 0.04f;
    }
}

void vmcCalc()
{
    // 懒得改为中文注释，这里选择英文是怕编码问题，但是其他地方为了统一性妥协了，后人自有后人福把
    // 1. 重力补偿
    F_g_vmc = (M_B / 2 + M_L * 0.2) * G;

    // 2. 横滚平衡
    F_roll_vmc = -PIDCalculate(&F_roll_vmc_pid_instance, roll, 0);

    // 3. 腿长平衡
    F_l_vmc = 0;
    // PIDCalculate(&F_l_vmc_pid_instance, (Lleg->result->L0 + Rleg->result->L0) / 2, l_xx);

    // 4. 惯性补偿
    F_in_vmc = (M_L * 0.2 + M_B / 2) * l_xx / 2 / R_L * phi_dot * s_dot; //* absds;

    // 5. 雅可比转置映射
    float F_L_vmc = -F_roll_vmc + F_l_vmc + F_g_vmc - F_in_vmc;
    float F_R_vmc = +F_roll_vmc + F_l_vmc + F_g_vmc + F_in_vmc;
    tau_l = vmc_get_tau(F_L_vmc, 0.0f, 0.210f, lfangle, PI - lbangle, Lleg->result->L0, PI / 2 - Lleg->result->theta0);
    tau_r = vmc_get_tau(F_R_vmc, 0.0f, 0.210f, rfangle, PI - rbangle, Rleg->result->L0, PI / 2 - Rleg->result->theta0);

    // 支持力解算，详见whx文章（增加奇异位保护，避免 tau1 过小导致数值爆炸）
    const float tau1_eps = 1e-3f;
    float tau1_l = tau_l.tau1;
    float tau1_r = tau_r.tau1;
    if (fabs(tau1_l) < tau1_eps)
        tau1_l = (tau1_l >= 0.0f) ? tau1_eps : -tau1_eps;
    if (fabs(tau1_r) < tau1_eps)
        tau1_r = (tau1_r >= 0.0f) ? tau1_eps : -tau1_eps;
    F_l = F_L_vmc * (-tau1_l + Lleg->tor) / (-tau1_l);
    F_r = F_R_vmc * (-tau1_r + Rleg->tor) / (-tau1_r);
    T_l = lbt;
    T_r = rbt;

    float a_x = -IMU_data->Accel[1],
          a_y = -IMU_data->Accel[0],
          a_z = IMU_data->Accel[2];
    a_xb = a_x * arm_cos_f32(pitch) + a_y * arm_sin_f32(roll) * arm_sin_f32(pitch) + a_z * arm_cos_f32(roll) * arm_sin_f32(pitch);
    a_yb = a_y * arm_cos_f32(roll) - a_z * arm_sin_f32(roll);
    a_zb = a_x * arm_sin_f32(pitch) - a_y * arm_sin_f32(roll) * arm_cos_f32(pitch) + a_z * arm_cos_f32(roll) * arm_cos_f32(pitch) - G;
    a_w_l = a_zb - arm_cos_f32(theta_l) * Lleg->result->ddl + 2 * Lleg->result->dl * Lleg->result->w * arm_sin_f32(theta_l) + Lleg->result->L0 * Lleg->result->dw * arm_sin_f32(theta_l) + Lleg->result->L0 * (Lleg->result->w * Lleg->result->w) * arm_cos_f32(theta_l);
    a_w_r = a_zb - arm_cos_f32(theta_r) * Rleg->result->ddl + 2 * Rleg->result->dl * Rleg->result->w * arm_sin_f32(theta_r) + Rleg->result->L0 * Rleg->result->dw * arm_sin_f32(theta_r) + Rleg->result->L0 * (Rleg->result->w * Rleg->result->w) * arm_cos_f32(theta_r);

    P_l = F_l * arm_cos_f32(theta_l) + T_l * arm_sin_f32(theta_l) / Lleg->result->L0;
    P_r = F_r * arm_cos_f32(theta_r) + T_r * arm_sin_f32(theta_r) / Rleg->result->L0;
    MG = 2 * M_W * (a_zb + G);
    F_n = P_l + P_r + MG;
}

void setTargetData()
{
    // 由摇杆控制的的速度指令值
    float v_cmd_raw = chassis_cmd_recv.vx / 660.0f * 2.2f;
    float v_cmd_dt = del_t;
    if (v_cmd_dt <= 0.0f)
        v_cmd_dt = 0.001f;
    float v_cmd_step = V_CMD_MAX_ACCEL * v_cmd_dt;
    float v_cmd_delta = v_cmd_raw - v_cmd_last;
    if (v_cmd_delta > v_cmd_step)
        v_cmd_delta = v_cmd_step;
    else if (v_cmd_delta < -v_cmd_step)
        v_cmd_delta = -v_cmd_step;
    v_cmd = v_cmd_last + v_cmd_delta;
    v_cmd_last = v_cmd;
    w_cmd = chassis_wz_cmd / 660.0f * 8;

    // 每次复位仅执行一次的代码，机器人死亡时的数据需要更新防止抽风
    if (!done)
    {
        ++done, phi_tar = phi, s_tar = s;
        position_hold_latched = 0u;
    }
    if (w_cmd != 0)
        phi_tar = phi + w_cmd * 0.002f;
    else
    {
        phi_tar = phi;
    }
    // absds = SpeedEstimation(del_t, imu->ch040_data, s_zdot);//要更改
    if (v_cmd == 0.0f)
    {
        if (!position_hold_latched)
        {
            s_tar = s;
            position_hold_latched = 1u;
        }
    }
    else
    {
        position_hold_latched = 0u;
        s_tar = s + v_cmd * 0.003f;
    }

    del_t = DWT_GetDeltaT(&balance_dwt_cnt);
    // if(fabs(absds)<0.0001) s_tar =s;

    // 线性拟合得到的摆角 2.019448601894652* l_xx^2-1.3926649689170425*l_xx+0.25703709140597736
    theta_bal = 2.019448601894652 * l_xx * l_xx - 1.3926649689170425 * l_xx + 0.25703709140597736;

    set_lqr_data(s,
                 s_dot,
                 phi,
                 phi_dot,
                 theta_l,
                 theta_l_dot,
                 theta_r,
                 theta_r_dot,
                 theta_b,
                 theta_b_dot,
                 Lleg->result->L0,
                 Rleg->result->L0,
                 -1);
    set_lqr_target(s_tar,
                   v_cmd,
                   phi_tar,
                   w_cmd,
                   thetal, //+ map_linear(motor_l->measure.real_current, -8192, 8192, -PI / 36, PI / 36), // theta_l,
                   0,      // theta_l_dot,
                   thetar, //+ map_linear(motor_r->measure.real_current, -8192, 8192, PI / 36, -PI / 36), // theta_r,
                   0,      // theta_r_dot,
                   0,
                   0);
    // 调用C接口执行LQR计算
    calc_lqr();
}

void flyDetect()
{
    if (reset_time < 5000)
        return; // 如果处于起身状态，则不进行离地检测，防止起身时误判离地

    // 离地检测实现比较暴力，直接把支持力写进set函数并通过分支语句来控制
    // 如果离地，就会将新的值赋给力矩输出，同时此函数也获得所有力矩
    set_lqr_data(s,
                 s_dot,
                 phi,
                 phi_dot,
                 theta_l,
                 theta_l_dot,
                 theta_r,
                 theta_r_dot,
                 theta_b,
                 theta_b_dot,
                 Lleg->result->L0,
                 Rleg->result->L0,
                 F_n);
    calc_lqr();
    if (F_n < 80)
    {
        Lleg->leglCtrl.Kp = 125;
        Lleg->legvCtrl.Kd = 35;
        Rleg->leglCtrl.Kp = 125;
        Rleg->legvCtrl.Kd = 35;
        l_xx = 0.16;
    }
    else
    {
        Lleg->leglCtrl.Kp = 250;
        Lleg->legvCtrl.Kd = 45;
        Rleg->leglCtrl.Kp = 250;
        Rleg->legvCtrl.Kd = 45;
    }

    // LQR力矩
    lbt = get_lb_torque();
    rbt = get_rb_torque();
    rwt = get_rw_torque();
    lwt = get_lw_torque();
}

void SetLegSpeed(bool clockwise)
{
    jiont_motorlf->ctrl.kd_set = 5.5;
    jiont_motorlb->ctrl.kd_set = 5.5;
    jiont_motorrf->ctrl.kd_set = 5.5;
    jiont_motorrb->ctrl.kd_set = 5.5;

    DMMotorSetPos(jiont_motorlf, 0, clockwise * PI * 1.2);
    DMMotorSetPos(jiont_motorlb, 0, clockwise * PI * 1.2);
    DMMotorSetPos(jiont_motorrf, 0, -clockwise * PI * 1.2);
    DMMotorSetPos(jiont_motorrb, 0, -clockwise * PI * 1.2);
}

void UnsetLegSpeed()
{
    jiont_motorlf->ctrl.kd_set = 0;
    jiont_motorlb->ctrl.kd_set = 0;
    jiont_motorrf->ctrl.kd_set = 0;
    jiont_motorrb->ctrl.kd_set = 0;
}

void SetBodySpeed(bool clockwise)
{
    jiont_motorlf->ctrl.kd_set = 5.8;
    jiont_motorlb->ctrl.kd_set = 5.8;
    jiont_motorrf->ctrl.kd_set = 5.8;
    jiont_motorrb->ctrl.kd_set = 5.8;

    DMMotorSetPos(jiont_motorlf, 0, clockwise * PI * 1.2);
    DMMotorSetPos(jiont_motorlb, 0, clockwise * PI * 1.2);
    DMMotorSetPos(jiont_motorrf, 0, -clockwise * PI * 1.2);
    DMMotorSetPos(jiont_motorrb, 0, -clockwise * PI * 1.2);
}

void UnsetBodySpeed()
{
    jiont_motorlf->ctrl.kd_set = 0;
    jiont_motorlb->ctrl.kd_set = 0;
    jiont_motorrf->ctrl.kd_set = 0;
    jiont_motorrb->ctrl.kd_set = 0;
}

void setLEGkd(float kd)
{
    jiont_motorlf->ctrl.kd_set = kd;
    jiont_motorlb->ctrl.kd_set = kd;
    jiont_motorrf->ctrl.kd_set = kd;
    jiont_motorrb->ctrl.kd_set = kd;
}

void SetLegW(float w)
{
    jiont_motorlf->ctrl.kd_set = 2.5;
    jiont_motorlb->ctrl.kd_set = 2.5;
    jiont_motorrf->ctrl.kd_set = 2.5;
    jiont_motorrb->ctrl.kd_set = 2.5;
    DMMotorSetPos(jiont_motorlf, 0, w);
    DMMotorSetPos(jiont_motorlb, 0, w);
    DMMotorSetPos(jiont_motorrf, 0, -w);
    DMMotorSetPos(jiont_motorrb, 0, -w);
}

void Setlegkd()
{
    jiont_motorlf->ctrl.kd_set = 2.5;
    jiont_motorlb->ctrl.kd_set = 2.5;
    jiont_motorrf->ctrl.kd_set = 2.5;
    jiont_motorrb->ctrl.kd_set = 2.5;
    DMMotorSetPos(jiont_motorlf, 0, -1.3);
    DMMotorSetPos(jiont_motorlb, 0, -1.3);
    DMMotorSetPos(jiont_motorrf, 0, 1.3);
    DMMotorSetPos(jiont_motorrb, 0, 1.3);
}

void Unsetlegkd()
{
    jiont_motorlf->ctrl.kd_set = 0;
    jiont_motorlb->ctrl.kd_set = 0;
    jiont_motorrf->ctrl.kd_set = 0;
    jiont_motorrb->ctrl.kd_set = 0;
}

static float WrapToPiLocal(float x)
{
    while (x >= PI)
        x -= PI2;
    while (x < -PI)
        x += PI2;
    return x;
}

// 把目标角映射到“离当前编码器位置最近”的等效角
static float MapToNearestEncoder(float raw_target, float encoder_now, float pos_limit)
{
    float mapped = encoder_now + WrapToPiLocal(raw_target - encoder_now);

    // 若跨圈后仍越界，尝试挪一圈
    if (mapped > pos_limit)
        mapped -= PI2;
    else if (mapped < -pos_limit)
        mapped += PI2;

    // 最终保险限幅
    if (mapped > pos_limit)
        mapped = pos_limit;
    else if (mapped < -pos_limit)
        mapped = -pos_limit;

    return mapped;
}

static void ResetRLJointMode(void)
{
    if (!rl_joint_mode)
        return;
    DMMotorInstance *joint[4] = {jiont_motorlf, jiont_motorlb,
                                 jiont_motorrf, jiont_motorrb};
    taskENTER_CRITICAL();
    for (unsigned i = 0; i < 4u; ++i)
    {
        joint[i]->ctrl.kp_set = 0.0f;
        joint[i]->ctrl.host_position_pd = 0u;
        joint[i]->ctrl.kd_set = 0.0f;
        DMMotorSetPos(joint[i], 0.0f, 0.0f);
    }
    rl_joint_mode = 0u;
    taskEXIT_CRITICAL();
}

static void ApplyRLJointOutput(const float target[4], float blend)
{
    if (!rl_joint_mode)
        ++rl_activation_sequence;
    DMMotorInstance *joint[4] = {jiont_motorlf, jiont_motorlb,
                                 jiont_motorrf, jiont_motorrb};
    for (unsigned i = 0; i < 4u; ++i)
    {
        float position, torque, kp, kd;
        RLControlBlendJoint(joint[i]->measure.pos, joint[i]->ctrl.tor_set,
                            target[i], blend, &position, &torque, &kp, &kd);
        DMMotorSetPos(joint[i], position, 0.0f);
        DMMotorSetTorque(joint[i], torque);
        joint[i]->ctrl.kp_set = kp;
        joint[i]->ctrl.kd_set = kd;
        joint[i]->ctrl.host_position_pd = 1u;
    }
    rl_joint_mode = 1u;
}

static void StartRLBeep(uint32_t now_tick, octave_e note)
{
    rl_switch_buzzer->octave = note;
    AlarmSetStatus(rl_switch_buzzer, ALARM_ON);
    rl_buzzer_off_tick = now_tick + pdMS_TO_TICKS(100u);
    rl_buzzer_on = 1u;
}

static void RequestRLBeep(uint32_t now_tick, octave_e note)
{
    if (rl_switch_buzzer == NULL)
        return;
    if (rl_buzzer_on || (int32_t)(now_tick - rl_buzzer_gap_until_tick) < 0)
    {
        rl_buzzer_pending = 1u;
        rl_buzzer_pending_note = note;
    }
    else
        StartRLBeep(now_tick, note);
}

static void ServiceRLBeep(uint32_t now_tick)
{
    if (rl_buzzer_on && (int32_t)(now_tick - rl_buzzer_off_tick) >= 0)
    {
        AlarmSetStatus(rl_switch_buzzer, ALARM_OFF);
        rl_buzzer_on = 0u;
        rl_buzzer_gap_until_tick = now_tick + pdMS_TO_TICKS(50u);
    }
    if (!rl_buzzer_on && rl_buzzer_pending &&
        (int32_t)(now_tick - rl_buzzer_gap_until_tick) >= 0)
    {
        rl_buzzer_pending = 0u;
        StartRLBeep(now_tick, rl_buzzer_pending_note);
    }
}

void outputTorque()
{
    // 内部是一个状态机，处理复位中，起跳，和正常运行三个状态
    // 该函数的实现比较丑陋，但是我不打算修改了，正常工作的行为树不会很漂亮

    // 先累加得到关节电机力矩
    if (upstair_flag0 == 1)
    {
        lbt = lbt * 0.4;
        rbt = rbt * 0.4;
    }
    float spring_tor_l = GetSpringTorqueByLegLength(Lleg->result->L0);
    float spring_tor_r = GetSpringTorqueByLegLength(Rleg->result->L0);
    lfjt = +Lleg->tor - tau_l.tau1 + lbt / 2 - spring_tor_l * arm_sin_f32(lmangle);
    lbjt = -Lleg->tor + tau_l.tau2 + lbt / 2 + spring_tor_l * arm_sin_f32(lmangle);
    rfjt = -Rleg->tor + tau_r.tau1 - rbt / 2 + spring_tor_r * arm_sin_f32(rmangle);
    rbjt = +Rleg->tor - tau_r.tau2 - rbt / 2 - spring_tor_r * arm_sin_f32(rmangle);
    float leg_retract_tor_l = 0.0f;
    float leg_retract_tor_r = 0.0f;
    wheel_friction_torque_nm[0] = 0.0f;
    wheel_friction_torque_nm[1] = 0.0f;

    const uint32_t now_tick = xTaskGetTickCount();
    ServiceRLBeep(now_tick);
    RLShadowCommand rl_command;
    const RLShadowCommand *rl_command_ptr =
        (RLShadowReadCommand(&rl_command) == 0) ? &rl_command : NULL;
    const int rl_normal_lqr = ChassisNormalLQRState();
    RLControlDecision rl_decision = RLControlStep(&rl_control_state, now_tick,
                                                 rl_normal_lqr, rl_command_ptr);
    if (rl_decision.exit_beep && rl_joint_mode)
    {
        /* Do: emergency; Re: stand-up/special-motion protection; Mi: invalid RL command. */
        const octave_e exit_note = chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE
            ? OCTAVE_1 : (rl_normal_lqr ? OCTAVE_3 : OCTAVE_2);
        RequestRLBeep(now_tick, exit_note);
    }
    if (!rl_decision.active)
        ResetRLJointMode();

    // 非双下
    if (chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE)
    {
        // 如果复位中且非上台阶
        if (chassis_status && !upstair_flag)
        {
            lwi = 0;
            rwi = 0;
            // 该数字是控制起身前摆角的位置，考虑到不太重要因此不作为宏
            RFW = PIDCalculate(&Rtheta, theta_r + (theta_r < 0) * PI * 3, 1.25f);
            LFW = PIDCalculate(&Ltheta, theta_l + (theta_l < 0) * PI * 3, 1.25f);

            if (fabs(pitch) > 0.8f)
            {
                UpdateLegRetractTorqueScale(false);
                RFW = fabs(PIDCalculate(&Pitch, pitch, 0.0f));
                LFW = fabs(PIDCalculate(&Pitch, pitch, 0.0f));
                if (pitch < 0)
                    RFW = -RFW * 1.6, LFW = -LFW * 1.6;
                SetBodySpeed(pitch > 0);
                DMMotorSetTorque(jiont_motorlf, LFW);
                DMMotorSetTorque(jiont_motorlb, LFW);
                DMMotorSetTorque(jiont_motorrf, -RFW);
                DMMotorSetTorque(jiont_motorrb, -RFW);
            }
            else if (fabs(pitch) > 0.2f && save_time < 800)
            {
                UpdateLegRetractTorqueScale(false);
                ++save_time; // 自救成功后等待平衡
                UnsetBodySpeed();
                DMMotorSetTorque(jiont_motorlf, -spring_tor_l * arm_sin_f32(lmangle));
                DMMotorSetTorque(jiont_motorlb, +spring_tor_l * arm_sin_f32(lmangle));
                DMMotorSetTorque(jiont_motorrf, +spring_tor_r * arm_sin_f32(rmangle));
                DMMotorSetTorque(jiont_motorrb, -spring_tor_r * arm_sin_f32(rmangle));
            }
            else
            {
                save_time = 0; // 重置自救成功后时间
                if (!step_0)
                {
                    UpdateLegRetractTorqueScale(false);
                    if (!standup_gimbal_reset_flag)
                    {
                        Unsetlegkd();
                        UnsetBodySpeed();
                        UnsetLegSpeed();
                    }
                    else
                    {
                        if (Lleg->result->L0 > 0.16f && Rleg->result->L0 > 0.16f)
                            step_0 = true;
                        SetBodySpeed(true);
                        if (Lleg->result->L0 < 0.16f)
                        {
                            DMMotorSetTorque(jiont_motorlf, 3.0);
                            DMMotorSetTorque(jiont_motorlb, 3.0);
                        }
                        if (Rleg->result->L0 < 0.16f)
                        {
                            DMMotorSetTorque(jiont_motorrf, -3.0);
                            DMMotorSetTorque(jiont_motorrb, -3.0);
                        }
                    }
                }
                else if (!step_1)
                {
                    UpdateLegRetractTorqueScale(false);

                    SetLegSpeed(false);
                    UnsetBodySpeed();
                    setLEGkd(0.5);
                    DMMotorSetTorque(jiont_motorlf, LFW - spring_tor_l * arm_sin_f32(lmangle));
                    DMMotorSetTorque(jiont_motorlb, LFW + spring_tor_l * arm_sin_f32(lmangle));
                    DMMotorSetTorque(jiont_motorrf, -RFW + spring_tor_r * arm_sin_f32(rmangle));
                    DMMotorSetTorque(jiont_motorrb, -RFW - spring_tor_r * arm_sin_f32(rmangle));
                    if (fabs(theta_r - 1.29f) < 0.20f && fabs(theta_l - 1.29f) < 0.20f)
                    {
                        step_1 = true;
                        UnsetLegSpeed();
                    }
                }
                else if (!step_2)
                {

                    // 逆运动学解算
                    Inverse_solution(0.145, 0.7f, &phi1, &phi4);
                    Inverse_solution(Lleg->result->L0, Lleg->result->theta0, &Lphi1, &Lphi4);
                    Inverse_solution(Rleg->result->L0, Rleg->result->theta0, &Rphi1, &Rphi4);
                    Setlegkd();
                    LFtor = PIDCalculate(&Langle_pid_instance, Lphi1, phi1);
                    LBtor = PIDCalculate(&Langle_pid_instance, Lphi4, phi4);
                    RFtor = PIDCalculate(&Rangle_pid_instance, Rphi1, phi1);
                    RBtor = PIDCalculate(&Rangle_pid_instance, Rphi4, phi4);
                    DMMotorSetTorque(jiont_motorlf, -spring_tor_l * arm_sin_f32(lmangle) + LFtor);
                    DMMotorSetTorque(jiont_motorlb, +spring_tor_l * arm_sin_f32(lmangle) + LBtor);
                    DMMotorSetTorque(jiont_motorrf, +spring_tor_r * arm_sin_f32(rmangle) + RFtor);
                    DMMotorSetTorque(jiont_motorrb, -spring_tor_r * arm_sin_f32(rmangle) + RBtor);
                    if (fabs(Lleg->result->L0 - 0.150) < 0.05f && fabs(Rleg->result->L0 - 0.150) < 0.05f)
                        step_2 = true;
                }
                else if (!step_3)
                {
                    Inverse_solution(0.160, 1.45f, &phi1, &phi4);
                    Inverse_solution(Lleg->result->L0, Lleg->result->theta0, &Lphi1, &Lphi4);
                    Inverse_solution(Rleg->result->L0, Rleg->result->theta0, &Rphi1, &Rphi4);
                    Setlegkd();
                    lwi = -3400;
                    rwi = 3400;
                    LFtor = PIDCalculate(&Langle_pid_instance, Lphi1, phi1);
                    LBtor = PIDCalculate(&Langle_pid_instance, Lphi4, phi4);
                    RFtor = PIDCalculate(&Rangle_pid_instance, Rphi1, phi1);
                    RBtor = PIDCalculate(&Rangle_pid_instance, Rphi4, phi4);
                    DMMotorSetTorque(jiont_motorlf, -spring_tor_l * arm_sin_f32(lmangle) + LFtor);
                    DMMotorSetTorque(jiont_motorlb, +spring_tor_l * arm_sin_f32(lmangle) + LBtor);
                    DMMotorSetTorque(jiont_motorrf, +spring_tor_r * arm_sin_f32(rmangle) + RFtor);
                    DMMotorSetTorque(jiont_motorrb, -spring_tor_r * arm_sin_f32(rmangle) + RBtor);
                    if (theta_l < 0.50f && theta_r < 0.50f && fabs(theta_b) < 0.01)
                    {
                        step_3 = true, Rwheel = motor_r->measure.total_angle, Lwheel = motor_l->measure.total_angle; // 复位完成，重置轮子编码器的初始值,新增复为完成后将轮子的初始位置重置为当前值，防止复位过程中轮子转动过多导致的编码器值过大影响后续位置估计，这是应该用卡尔曼滤波修复的东西
                        Unsetlegkd();
                        s_tar = s;
                        w_cmd = 0;
                        v_cmd = 0;
                        phi_tar = phi;
                        upstair_flag0 = 0;
                    }
                }
                else
                {
                    UpdateLegRetractTorqueScale(false);
                    chassis_status = 0;
                    reset_time = 0;
                }
            }
        }
        // 如果跳跃
        else if (jumpingflag && clockcent < 80)
        {
            UpdateLegRetractTorqueScale(false);
            clockcent++;
            DMMotorSetTorque(jiont_motorlf, 10.0f);
            DMMotorSetTorque(jiont_motorlb, -10.0f);
            DMMotorSetTorque(jiont_motorrf, -10.0f);
            DMMotorSetTorque(jiont_motorrb, +10.0f);
        }
        else if (jumpingflag && clockcent < 200)
        {
            UpdateLegRetractTorqueScale(false);
            clockcent++;
            DMMotorSetTorque(jiont_motorlf, -spring_tor_l * arm_sin_f32(lmangle) * 0.5f);
            DMMotorSetTorque(jiont_motorlb, +spring_tor_l * arm_sin_f32(lmangle) * 0.5f);
            DMMotorSetTorque(jiont_motorrf, +spring_tor_r * arm_sin_f32(rmangle) * 0.5f);
            DMMotorSetTorque(jiont_motorrb, -spring_tor_r * arm_sin_f32(rmangle) * 0.5f);
        }
        // 如果在上台阶
        else if (upstair_flag && upstair_step == 0)
        {
            UpdateLegRetractTorqueScale(false);
            SetLegSpeed(true);
            RFW = 1.5 * PIDCalculate(&Rtheta, theta_r, 1.55f);
            LFW = 1.5 * PIDCalculate(&Ltheta, theta_l, 1.55f);
            DMMotorSetTorque(jiont_motorlf, LFW - spring_tor_l * arm_sin_f32(lmangle));
            DMMotorSetTorque(jiont_motorlb, LFW + spring_tor_l * arm_sin_f32(lmangle));
            DMMotorSetTorque(jiont_motorrf, -RFW + spring_tor_r * arm_sin_f32(rmangle));
            DMMotorSetTorque(jiont_motorrb, -RFW - spring_tor_r * arm_sin_f32(rmangle));
            if (fabs(theta_l - 1.5f) < 0.05f && fabs(theta_r - 1.47f) < 0.05f)
                upstair_step++;
        }
        else if (upstair_flag && upstair_step == 1)
        {
            UpdateLegRetractTorqueScale(false);
            UnsetLegSpeed();
            DMMotorSetTorque(jiont_motorlf, -spring_tor_l * arm_sin_f32(lmangle) + Lleg->tor);
            DMMotorSetTorque(jiont_motorlb, +spring_tor_l * arm_sin_f32(lmangle) - Lleg->tor);
            DMMotorSetTorque(jiont_motorrf, +spring_tor_r * arm_sin_f32(rmangle) - Rleg->tor);
            DMMotorSetTorque(jiont_motorrb, -spring_tor_r * arm_sin_f32(rmangle) + Rleg->tor);
            if (Lleg->result->L0 < 0.17f && Rleg->result->L0 < 0.17f)
                upstair_step++;
        }
        else if (upstair_flag && upstair_step == 2)
        {
            UpdateLegRetractTorqueScale(false);
            RFW = 2 * PIDCalculate(&Rtheta, theta_r + (theta_r < 0) * PI * 2, 0.25f);
            LFW = 2 * PIDCalculate(&Ltheta, theta_l + (theta_l < 0) * PI * 2, 0.25f);
            DMMotorSetTorque(jiont_motorlf, LFW - spring_tor_l * arm_sin_f32(lmangle) + Lleg->tor);
            DMMotorSetTorque(jiont_motorlb, LFW + spring_tor_l * arm_sin_f32(lmangle) - Lleg->tor);
            DMMotorSetTorque(jiont_motorrf, -RFW + spring_tor_r * arm_sin_f32(rmangle) - Rleg->tor);
            DMMotorSetTorque(jiont_motorrb, -RFW - spring_tor_r * arm_sin_f32(rmangle) + Rleg->tor);
            if (theta_l < 0.75f && theta_r < 0.75f)
                upstair_flag = 0, upstair_step = 0, reset_time = 0, chassis_status = 0, upstair_flag0_cnt = 0;
            ;
        }
        // 如果在正常运行
        else
        {
            // 复位时间计时
            ++reset_time;

            // LQR摆角过大时停止腿部输出，防止立腿过程中过冲
            // if (upstair_flag0 && (fabs(theta_l) > LEG_SWING_CUTOFF_RAD || fabs(theta_r) > LEG_SWING_CUTOFF_RAD || a_zb < UPSTAIR_ACCEL_CUTOFF))
            // {
            //     upstair_flag0_latch = 1;
            // }

            if (fabs(theta_l) > LEG_SWING_CUTOFF_RAD || fabs(theta_r) > LEG_SWING_CUTOFF_RAD || (upstair_flag0 && (a_zb < UPSTAIR_ACCEL_CUTOFF) || (fabs(theta_l) > (PI / 4) || fabs(theta_r) > (PI / 4))))
            {
                UpdateLegRetractTorqueScale(false);
                lwi = 0;
                rwi = 0;
                DMMotorSetTorque(jiont_motorlf, 0.0f);
                DMMotorSetTorque(jiont_motorlb, 0.0f);
                DMMotorSetTorque(jiont_motorrf, 0.0f);
                DMMotorSetTorque(jiont_motorrb, 0.0f);
                if (upstair_flag0)
                {
                    if (upstair_flag0_cnt < 30)
                    {
                        ++upstair_flag0_cnt;
                        DMMotorSetTorque(jiont_motorlf, 4.0f);
                        DMMotorSetTorque(jiont_motorlb, 4.0f);
                        DMMotorSetTorque(jiont_motorrf, -4.0f);
                        DMMotorSetTorque(jiont_motorrb, -4.0f);
                    }
                    else
                    {
                        upstair_flag0 = 0;
                        upstair_flag0_cnt = 0;
                        upstair_flag0_latch = 0;
                        upstair_flag = 0;
                        upstair_step = 0;
                        clockcent = 0;
                        reset_time = 0;
                        save_time = 0;
                        chassis_status = 1;
                        cent = 0;
                        done = 0;
                        step_0 = 0;
                        step_1 = 0;
                        step_2 = 0;
                        step2_retract_confirm_cnt = 0;
                        step_3 = 0;
                        standup_gimbal_reset_flag = 0;
                        standup_gimbal_reset_cnt = 0;
                    }
                }
            }
            else
            {

                // 前面的数字是3508转矩常数复合电流离散值,由于电流环代码有锅，需要直接输入离散值，修改电机时一定会处理这部分，因此亦不作为宏
                UpdateLegRetractTorqueScale(false);
                rwi = rwt / 0.00036 * 19.0f / REDUCE_RATIO;
                lwi = -lwt / 0.00036 * 19.0f / REDUCE_RATIO; // negative
                /* Publish the LQR/RL joint command as one update. The motor
                   task must not send LQR torque with the previous RL gains. */
                taskENTER_CRITICAL();
                DMMotorSetTorque(jiont_motorlf, lfjt);
                DMMotorSetTorque(jiont_motorlb, lbjt);
                DMMotorSetTorque(jiont_motorrf, rfjt);
                DMMotorSetTorque(jiont_motorrb, rbjt);

                // 电流限位
                if (lwi > WHEEL_CURRENT_LIMIT)
                    lwi = WHEEL_CURRENT_LIMIT;
                if (lwi < -WHEEL_CURRENT_LIMIT)
                    lwi = -WHEEL_CURRENT_LIMIT;
                if (rwi > WHEEL_CURRENT_LIMIT)
                    rwi = WHEEL_CURRENT_LIMIT;
                if (rwi < -WHEEL_CURRENT_LIMIT)
                    rwi = -WHEEL_CURRENT_LIMIT;

                if (rl_decision.active)
                {
                    const float encoder_now[4] = {
                        jiont_motorlf->measure.pos, jiont_motorlb->measure.pos,
                        jiont_motorrf->measure.pos, jiont_motorrb->measure.pos};
                    const float encoder_offset[4] = {
                        LF_ENCODE_OFFSET, LB_ENCODE_OFFSET,
                        RF_ENCODE_OFFSET, RB_ENCODE_OFFSET};
                    float encoder_target[4], rl_wheel_current[2];
                    if (RLControlMapOutput(&rl_command.output, encoder_now,
                                           encoder_offset, encoder_target,
                                           rl_wheel_current) == 0)
                    {
                        ApplyRLJointOutput(encoder_target, rl_decision.blend);
                        lwi = RLControlBlendCurrent(lwi, rl_wheel_current[0],
                                                    rl_decision.blend);
                        rwi = RLControlBlendCurrent(rwi, rl_wheel_current[1],
                                                    rl_decision.blend);
                        if (rl_decision.beep)
                            RequestRLBeep(now_tick, OCTAVE_5);
                    }
                    else
                    {
                        if (rl_decision.beep)
                            rl_control_state.beeped = 0u;
                        RLControlDecision lost =
                            RLControlStep(&rl_control_state, now_tick, 1, NULL);
                        if (lost.exit_beep && rl_joint_mode)
                            RequestRLBeep(now_tick, OCTAVE_3);
                        ResetRLJointMode();
                    }
                }

                taskEXIT_CRITICAL();

                if (ChassisNormalLQRState() && !rl_joint_mode)
                {
                    const float left_speed_rad_s =
                        -motor_l->measure.speed_aps / REDUCE_RATIO * DEGREE_2_RAD;
                    const float right_speed_rad_s =
                        motor_r->measure.speed_aps / REDUCE_RATIO * DEGREE_2_RAD;
                    float compensated_current;
                    if (WheelFrictionCompensate(WHEEL_FRICTION_LEFT, left_speed_rad_s,
                                                lwi, &wheel_friction_torque_nm[0],
                                                &compensated_current) == 0)
                        lwi = compensated_current;
                    if (WheelFrictionCompensate(WHEEL_FRICTION_RIGHT, right_speed_rad_s,
                                                rwi, &wheel_friction_torque_nm[1],
                                                &compensated_current) == 0)
                        rwi = compensated_current;
                }

                if (!upstair_flag0)
                {
                    upstair_flag0_cnt = 0;
                    upstair_flag0_latch = 0;
                }
            }
        }
        // 直接写入离散值
        motor_l->motor_controller.current_feedforward_ptr = &wheel_send_current[0];
        motor_r->motor_controller.current_feedforward_ptr = &wheel_send_current[1];
    }
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    Chassis_Ctrl_Cmd_s received_cmd;
    uint32_t rx_sequence, rx_tick;
    taskENTER_CRITICAL();
    rx_sequence = chassis_ctrl_rx_ctx.rx_sequence;
    rx_tick = chassis_ctrl_rx_ctx.last_rx_tick_ms;
    received_cmd = chassis_can_cmd;
    taskEXIT_CRITICAL();
    if (ChassisCmdWatchdogFresh(&chassis_cmd_watchdog, rx_sequence, rx_tick,
                                HAL_GetTick(), CHASSIS_CMD_TIMEOUT_MS))
        chassis_cmd_recv = received_cmd;
    else
        memset(&chassis_cmd_recv, 0, sizeof(chassis_cmd_recv));
#endif

    chassis_wz_cmd = chassis_cmd_recv.wz;

    if (chassis_status && !upstair_flag && chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE &&
        fabs(chassis_cmd_recv.gimbal_error_angle) < STANDUP_GIMBAL_RESET_ERR_DEG)
    {
        if (standup_gimbal_reset_cnt < STANDUP_GIMBAL_RESET_CONFIRM_CNT)
        {
            ++standup_gimbal_reset_cnt;
        }
    }
    else
    {
        standup_gimbal_reset_cnt = 0;
    }
    standup_gimbal_reset_flag = (standup_gimbal_reset_cnt >= STANDUP_GIMBAL_RESET_CONFIRM_CNT);
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        lwi = 0.0f;
        rwi = 0.0f;
        DJIMotorStop(motor_l);
        DJIMotorStop(motor_r);
        DMMotorStop(jiont_motorlb);
        DMMotorStop(jiont_motorlf);
        DMMotorStop(jiont_motorrf);
        DMMotorStop(jiont_motorrb);
        chassis_status = 1;
        cent = 0;
        done = 0;
        step_0 = 0;
        step_1 = 0;
        step_2 = 0;
        step2_retract_confirm_cnt = 0;
        step_3 = 0;
        standup_gimbal_reset_flag = 0;
        standup_gimbal_reset_cnt = 0;
        leg_retract_torque_scale = 0.0f;
    }
    else
    { // 正常工作

        DJIMotorEnable(motor_l);
        DJIMotorEnable(motor_r);
        DMMotorEnable1(jiont_motorlb);
        DMMotorEnable1(jiont_motorlf);
        DMMotorEnable1(jiont_motorrf);
        DMMotorEnable1(jiont_motorrb);
    }

    // 根据控制模式设定旋转速度
    if (last_mode == CHASSIS_ROTATE && chassis_cmd_recv.chassis_mode != CHASSIS_ROTATE && chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE)
    {
        rotate_wz_softstop_active = 1;
        rotate_wz_exit = rotate_wz_soft;
    }

    // 退出自旋时做软刹车/状态复位
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    {
        rotate_wz_soft = ROTATE_WZ_START;
        rotate_wz_exit = 0.0f;
        rotate_wz_softstop_active = 0;
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_ZERO_FORCE:
        chassis_status = 1;
        cent = 0;
        done = 0;
    case CHASSIS_NO_FOLLOW:
        // 底盘不旋转,但维持全向机动,一般用于调整云台姿态

        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台
    {
        float follow_wz = 0.0f;
        if (fabs(chassis_cmd_recv.gimbal_error_angle) > FOLLOW_GIMBAL_DEADBAND_DEG)
        {
            follow_wz = chassis_cmd_recv.gimbal_error_angle * FOLLOW_GIMBAL_KP;
            if (follow_wz > FOLLOW_GIMBAL_WZ_MAX)
                follow_wz = FOLLOW_GIMBAL_WZ_MAX;
            if (follow_wz < -FOLLOW_GIMBAL_WZ_MAX)
                follow_wz = -FOLLOW_GIMBAL_WZ_MAX;
        }
        chassis_wz_cmd = follow_wz;
        break;
    }
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        rotate_wz_softstop_active = 0;
        if (rotate_wz_soft < ROTATE_WZ_TARGET)
        {
            rotate_wz_soft += ROTATE_WZ_STEP;
            if (rotate_wz_soft > ROTATE_WZ_TARGET)
                rotate_wz_soft = ROTATE_WZ_TARGET;
        }
        chassis_wz_cmd = rotate_wz_soft;
        chassis_cmd_recv.vx = 0;
        break;
    default:
        break;
    }

    // 退出自旋时,将wz渐变至0
    if (chassis_cmd_recv.chassis_mode != CHASSIS_ROTATE && rotate_wz_softstop_active)
    {
        rotate_wz_exit -= ROTATE_WZ_DOWN_STEP;
        if (rotate_wz_exit <= 0.0f)
        {
            rotate_wz_exit = 0.0f;
            rotate_wz_soft = ROTATE_WZ_START;
            rotate_wz_softstop_active = 0;
        }
        chassis_wz_cmd = rotate_wz_exit;
    }

    last_mode = chassis_cmd_recv.chassis_mode;

    getLegData();
    getStateData();
    setTargetData();
    legLengthCalc();
    vmcCalc();
    flyDetect();
    outputTorque();
    taskENTER_CRITICAL();
    wheel_drive_snapshot[0] = lwi;
    wheel_drive_snapshot[1] = rwi;
    wheel_rl_snapshot = rl_joint_mode;
    taskEXIT_CRITICAL();
    PublishRLShadowSnapshot();

    chassis_feedback_data.standup_flag = (uint8_t)(chassis_status && !upstair_flag); // 1: 起身中, 0: 非起身
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    Chassis_Upload_Data_CAN_s tx_data;

    ChassisCanPackUpload(&tx_data, &chassis_feedback_data);
    memcpy(chassis_can->tx_buff, &tx_data, sizeof(tx_data));

    CANTransmit(chassis_can, 0.01);
#endif // CHASSIS_BOARD
}

/* Called immediately before DJI send at1kHz. Targets and RL mode are read as
 * one snapshot. Wheel encoder source/filter matches PublishRL observation.
 * Leg PD remains in DMMotorControl at2ms. */
void ChassisWheelControlTick(void)
{
    if (motor_l == NULL || motor_r == NULL) return;
    taskENTER_CRITICAL();
    float left = wheel_drive_snapshot[0], right = wheel_drive_snapshot[1];
    const uint8_t use_rl = wheel_rl_snapshot;
    const float wl = -motor_l->measure.speed_aps / REDUCE_RATIO * DEGREE_2_RAD;
    const float wr = motor_r->measure.speed_aps / REDUCE_RATIO * DEGREE_2_RAD;
    taskEXIT_CRITICAL();
    if (use_rl) {
        if (WheelRLCompensate(WHEEL_FRICTION_LEFT, wl, left,
                             &wheel_friction_torque_nm[0], &left) != 0) left = 0;
        if (WheelRLCompensate(WHEEL_FRICTION_RIGHT, wr, right,
                             &wheel_friction_torque_nm[1], &right) != 0) right = 0;
    }
    wheel_send_current[0] = isfinite(left) ? fminf(fmaxf(left, -WHEEL_CURRENT_LIMIT), WHEEL_CURRENT_LIMIT) : 0;
    wheel_send_current[1] = isfinite(right) ? fminf(fmaxf(right, -WHEEL_CURRENT_LIMIT), WHEEL_CURRENT_LIMIT) : 0;
}
