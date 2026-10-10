#include "robot_def.h"
#include "robot_cmd.h"

#include <string.h>

#ifdef GIMBAL_BOARD
#include "chassis_can_link.h"
#endif

#include "message_center.h"

#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;
static Subscriber_t *chassis_feed_sub;
#endif

static Chassis_Ctrl_Cmd_s chassis_cmd_send;
static Chassis_Upload_Data_s chassis_fetch_data;

#ifdef GIMBAL_BOARD
static CANInstance *chassis_can;
static ChassisCanUploadRxCtx chassis_upload_rx_ctx;
#endif

static void ResetChassisCmd(void)
{
    memset(&chassis_cmd_send, 0, sizeof(chassis_cmd_send));
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    chassis_cmd_send.gimbal_error_angle = 0.0f;
    chassis_cmd_send.changeL = 0;
}

void RobotCMDInit()
{
#ifdef ONE_BOARD
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif

#ifdef GIMBAL_BOARD
    CAN_Init_Config_s can_config = {
        .can_handle = &hfdcan3,
        .tx_id = 0x22,
        .rx_id = 0x11,
        .can_module_callback = ChassisCanUploadRxCallback,
        .id = &chassis_upload_rx_ctx,
    };

    chassis_upload_rx_ctx.upload_dst = &chassis_fetch_data;
    chassis_can = CANRegister(&can_config);
    CANSetDLC(chassis_can, sizeof(Chassis_Ctrl_Cmd_CAN_s));
#endif

    memset(&chassis_fetch_data, 0, sizeof(chassis_fetch_data));
    ResetChassisCmd();
}

void RobotCMDTask()
{
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif

#ifdef GIMBAL_BOARD
    Chassis_Ctrl_Cmd_CAN_s tx_data;

    ChassisCanPackCtrl(&tx_data, &chassis_cmd_send);
    memcpy(chassis_can->tx_buff, &tx_data, sizeof(tx_data));
    CANTransmit(chassis_can, 1);
#endif
}
