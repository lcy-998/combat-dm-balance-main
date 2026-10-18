#include "DMmotor.h"
#include "stdlib.h"
#include "general_def.h"
#include "daemon.h"
#include "bsp_dwt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dm_position_pd.h"

uint8_t idm = 0;
static DMMotorInstance *dmmotor_instance[DM_MOTOR_MX_CNT] = {NULL};
void DMMotorEnableMode(DMMotorInstance *motor);
void DMMotorErrorDetection(DMMotorInstance *motor);
void mit_ctrl(DMMotorInstance *motor);
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
/**
 * @brief 电机反馈报文解析
 *
 * @param _instance 发生中断的caninstance
 */
void DMMotorDecode(CANInstance *_instance)
{
    DMMotorInstance *motor = (DMMotorInstance *)_instance->id; // 通过caninstance保存的father id获取对应的motorinstance
    motor_fbpara_t *measure = &motor->measure;
    uint8_t *rx_buff = _instance->rx_buff;

    DaemonReload(motor->daemon); // 喂狗
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    measure->id = (rx_buff[0]) & 0x0F;
    measure->state = (rx_buff[0]) >> 4;
    measure->p_int = (rx_buff[1] << 8) | rx_buff[2];
    measure->v_int = (rx_buff[3] << 4) | (rx_buff[4] >> 4);
    measure->t_int = ((rx_buff[4] & 0xF) << 8) | rx_buff[5];
    if (motor->motor_set.flag == 1)
    {
        measure->pos = uint_to_float(measure->p_int, -motor->motor_set.PosMXSet, motor->motor_set.PosMXSet, 16); // (-12.5,12.5)
        measure->vel = uint_to_float(measure->v_int, -motor->motor_set.VelMXSet, motor->motor_set.VelMXSet, 12); // (-45.0,45.0)
        measure->tor = uint_to_float(measure->t_int, -motor->motor_set.TorMXSet, motor->motor_set.TorMXSet, 12); // (-18.0,18.0)
    }
    else
    {
        measure->pos = uint_to_float(measure->p_int, DMP_MIN, DMP_MAX, 16); // (-12.5,12.5)
        measure->vel = uint_to_float(measure->v_int, DMV_MIN, DMV_MAX, 12); // (-45.0,45.0)
        measure->tor = uint_to_float(measure->t_int, DMT_MIN, DMT_MAX, 12); // (-18.0,18.0)
    }
    measure->Tmos = (float)(rx_buff[6]);
    measure->Tcoil = (float)(rx_buff[7]);
    if (motor->measure.state == 1)
        motor->online = 1;
    DMMotorErrorDetection(motor);
}

static void DMMotorLostCallback(void *motor_ptr)
{

    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    motor->online = 0;
    LOGWARNING("[DMMotor] motor lost, id: %d", (motor->motor_can_instance->tx_id & (0x1f << 5)) >> 5);
}

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    // motor = (DRMotorInstance *)malloc(sizeof(DRMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    // motor basic setting 电机基本设置
    motor->ctrl.kp_set = 0;
    motor->ctrl.kd_set = 0; // config->controller_param_init_config.speed_PID.Kd;
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    // motor->motor_controller.current_feedforward_ptr  = config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    motor->ctrl.mode = config->motor_contro_type; // 控制模式
    motor->motor_type = config->motor_type;
    config->can_init_config.id = motor;
    config->can_init_config.can_module_callback = DMMotorDecode;
    motor->motor_set = config->motor_set;
    motor->motor_can_instance = CANRegister(&config->can_init_config);
    motor->online = 1;
    for (size_t i = 0; i < 100; i++)
    {

        DMMotorEnableMode(motor); // 使能电机模式,发送使能指令
        CANTransmit(motor->motor_can_instance, 0.2);
    }

    motor->ctrl.vel_set = 0.0;
    motor->ctrl.tor_set = 0.0; //   默认关闭，避免出问题

    dmmotor_instance[idm++] = motor;

    Daemon_Init_Config_s daemon_config = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 500, // 20ms
    };
    motor->daemon = DaemonRegister(&daemon_config);
    motor->stop_flag = MOTOR_STOP;
    return motor;
}
void pos_ctrl(DMMotorInstance *motor)
{

    uint8_t *pbuf, *vbuf;
    uint8_t data[8];

    pbuf = (uint8_t *)&motor->ctrl.pos_set;
    vbuf = (uint8_t *)&motor->ctrl.vel_set;

    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);

    data[4] = *vbuf;
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);

    memset(motor->motor_can_instance->tx_buff, 0, sizeof(motor->motor_can_instance->tx_buff));
    memcpy(motor->motor_can_instance->tx_buff, &data, sizeof(data));
}
void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}
void DMMotorControl()
{

    DMMotorInstance *motor;

    for (size_t i = 0; i < idm; ++i)
    {
        motor = dmmotor_instance[i];
        if (motor->stop_flag == MOTOR_STOP)
        {
            motor->ctrl.host_position_pd = 0u;
            motor->ctrl.vel_set = 0.0;
            motor->ctrl.tor_set = 0.0;
            motor->ctrl.kp_set = 0.0;
            motor->ctrl.kd_set = 0.0;
        }

        // 发送
        // memset(motor->motor_can_instance->tx_buff, 0, sizeof(motor->motor_can_instance->tx_buff));
        // memcpy(motor->motor_can_instance->tx_buff, &motor_controller->output_current, sizeof(float));
        // motor->motor_can_instance->tx_buff[6] = 0x01;
        if (motor->online && motor->measure.state)
        {
            switch (motor->ctrl.mode) // 控制模式
            {
            case 0:
                mit_ctrl(motor);
                break;
            case 1:
                pos_ctrl(motor);
            default:
                break;
            }
        }
        else
        {
            DMMotorEnableMode(motor);
        }

        CANTransmit(motor->motor_can_instance, 0.1);
    }
}

void DMMotorEnableMode(DMMotorInstance *motor) //
{
    uint8_t data[8];

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;
    memset(motor->motor_can_instance->tx_buff, 0, sizeof(motor->motor_can_instance->tx_buff));
    memcpy(motor->motor_can_instance->tx_buff, &data, sizeof(data));
}

void mit_ctrl(DMMotorInstance *motor) //
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    motor_ctrl_t command;
    float position, velocity;
    taskENTER_CRITICAL();
    command = motor->ctrl;
    position = motor->measure.pos;
    velocity = motor->measure.vel;
    taskEXIT_CRITICAL();

    motor->host_position_error_rad = 0.0f;
    if (command.host_position_pd) {
        const float torque_limit = motor->motor_set.flag
            ? motor->motor_set.TorMXSet : DMT_MAX;
        command.tor_set = DMPositionPDTorque(command.pos_set, command.vel_set,
                                             position, velocity,
                                             command.kp_set, command.kd_set,
                                             command.tor_set, torque_limit,
                                             &motor->host_position_error_rad);
        command.pos_set = 0.0f;
        command.vel_set = 0.0f;
        command.kp_set = 0.0f;
        command.kd_set = 0.0f;
    }
    motor->sent_torque_nm = command.tor_set;

    if (motor->motor_set.flag)
    {
        pos_tmp = float_to_uint(command.pos_set, -motor->motor_set.PosMXSet, motor->motor_set.PosMXSet, 16);
        vel_tmp = float_to_uint(command.vel_set, -motor->motor_set.VelMXSet, motor->motor_set.VelMXSet, 12);
        kp_tmp = float_to_uint(command.kp_set, 0.0f, motor->motor_set.KpMXSet, 12);
        kd_tmp = float_to_uint(command.kd_set, 0.0f, motor->motor_set.KdMXSet, 12);
        tor_tmp = float_to_uint(command.tor_set, -motor->motor_set.TorMXSet, motor->motor_set.TorMXSet, 12);
    }
    else
    {
        pos_tmp = float_to_uint(command.pos_set, DMP_MIN, DMP_MAX, 16);
        vel_tmp = float_to_uint(command.vel_set, DMV_MIN, DMV_MAX, 12);
        kp_tmp = float_to_uint(command.kp_set, DMKP_MIN, DMKP_MAX, 12);
        kd_tmp = float_to_uint(command.kd_set, DMKD_MIN, DMKD_MAX, 12);
        tor_tmp = float_to_uint(command.tor_set, DMT_MIN, DMT_MAX, 12);
    }
    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;

    memset(motor->motor_can_instance->tx_buff, 0, sizeof(motor->motor_can_instance->tx_buff));
    memcpy(motor->motor_can_instance->tx_buff, &data, sizeof(data));
}

uint8_t DMMotorIsOnline(DMMotorInstance *motor)
{

    return DaemonIsOnline(motor->daemon);
}

// 异常检测
void DMMotorErrorDetection(DMMotorInstance *motor)
{
}

// DM电机软重启
//  void DMMotorReset(DMMotorInstance* motor){
//     DMMotorEnableMode(motor); // 使能电机模式,发送使能指令
//      CANTransmit_once(motor->motor_can_instance->can_handle,
//                           (motor->motor_can_instance->tx_id) + 0x0000,
//                           motor->motor_can_instance->tx_buff, 2);
//  }

void DMMotorStop(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorEnable1(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorSetTorque(DMMotorInstance *motor, float torque)
{
    motor->ctrl.tor_set = torque;
}
void DMMotorSetPos(DMMotorInstance *motor, float pos, float vel)
{
    motor->ctrl.pos_set = pos;
    motor->ctrl.vel_set = vel;
}
