#pragma once

#include <string.h>

#include "bsp_can.h"
#include "robot_def.h"

typedef struct
{
    Chassis_Ctrl_Cmd_s *ctrl_dst;
} ChassisCanCtrlRxCtx;

typedef struct
{
    Chassis_Upload_Data_s *upload_dst;
} ChassisCanUploadRxCtx;

static inline int16_t ChassisCanClampToInt16(float value)
{
    if (value > 32767.0f)
        return 32767;
    if (value < -32768.0f)
        return -32768;
    return (int16_t)value;
}

static inline void ChassisCanPackCtrl(Chassis_Ctrl_Cmd_CAN_s *dst, const Chassis_Ctrl_Cmd_s *src)
{
    dst->vx = ChassisCanClampToInt16(src->vx);
    dst->wz = ChassisCanClampToInt16(src->wz);
    dst->gimbal_error_angle_cdeg = ChassisCanClampToInt16(src->gimbal_error_angle * 100.0f);
    dst->changeL = (int8_t)src->changeL;
    dst->chassis_mode = (uint8_t)src->chassis_mode;
}

static inline void ChassisCanUnpackCtrl(Chassis_Ctrl_Cmd_s *dst, const Chassis_Ctrl_Cmd_CAN_s *src)
{
    dst->vx = (float)src->vx;
    dst->wz = (float)src->wz;
    dst->chassis_mode = (chassis_mode_e)src->chassis_mode;
    dst->changeL = src->changeL;
    dst->gimbal_error_angle = (float)src->gimbal_error_angle_cdeg * 0.01f;
}

static inline void ChassisCanPackUpload(Chassis_Upload_Data_CAN_s *dst, const Chassis_Upload_Data_s *src)
{
    dst->rest_heat = src->rest_heat;
    dst->bullet_speed = (uint8_t)src->bullet_speed;
    dst->enemy_color = (uint8_t)src->enemy_color;
    dst->standup_flag = src->standup_flag;
    dst->reserved = 0;
}

static inline void ChassisCanUnpackUpload(Chassis_Upload_Data_s *dst, const Chassis_Upload_Data_CAN_s *src)
{
    dst->rest_heat = src->rest_heat;
    dst->bullet_speed = (Bullet_Speed_e)src->bullet_speed;
    dst->enemy_color = (Enemy_Color_e)src->enemy_color;
    dst->standup_flag = src->standup_flag;
}

static void ChassisCanCtrlRxCallback(CANInstance *_instance)
{
    ChassisCanCtrlRxCtx *ctx = (ChassisCanCtrlRxCtx *)_instance->id;
    Chassis_Ctrl_Cmd_CAN_s rx_data;

    memcpy(&rx_data, _instance->rx_buff, sizeof(rx_data));
    ChassisCanUnpackCtrl(ctx->ctrl_dst, &rx_data);
}

static void ChassisCanUploadRxCallback(CANInstance *_instance)
{
    ChassisCanUploadRxCtx *ctx = (ChassisCanUploadRxCtx *)_instance->id;
    Chassis_Upload_Data_CAN_s rx_data;

    memcpy(&rx_data, _instance->rx_buff, sizeof(rx_data));
    ChassisCanUnpackUpload(ctx->upload_dst, &rx_data);
}
