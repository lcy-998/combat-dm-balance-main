#ifndef RL_CAPTURE_H
#define RL_CAPTURE_H
#include "rl_control.h"

#define RL_CAPTURE_VERSION 1u
#ifndef RL_CAPTURE_CAPACITY
#define RL_CAPTURE_CAPACITY 128u
#endif

/* Plain 32-bit fields for a reproducible binary export. No packing/unaligned floats.
   Float arrays use model ch0..5 except the explicitly named motor-order arrays.
   Driver commands are not measured motor torque/current. */
typedef struct {
    uint32_t frame_begin, sequence, tick_ms, owner, fault, control_epoch;
    uint32_t policy_sequence, policy_snapshot_tick_ms, policy_output_tick_ms;
    uint32_t adopted_sequence, adopted_tick_ms, previous_source, previous_sequence;
    uint32_t valid_mask; /* bit0=policy candidate, bit1=RL PD/compensation, bit2=local final commands */
    uint32_t feedback_valid_mask, feedback_receive_tick_ms[6], imu_update_tick_ms;
    float observation[31], mean_action[6], requested_target[6], adopted_target[6];
    float pd_q[4], pd_dq[6], pd_request_nm[6], pd_limited_nm[6];
    float spring_motor_nm[4], final_motor_command_nm[4], wheel_command_raw[2];
    float driver_reported_leg_torque_motor_nm[4], driver_reported_wheel_current_raw[2];
    uint32_t frame_end;
} RLCaptureRecord;

typedef struct {
    uint32_t schema_version, record_bytes, capacity, count, write_index;
    uint32_t armed, triggered, frozen, remaining, post_records;
    uint32_t records_written, owner_previous;
    RLCaptureRecord records[RL_CAPTURE_CAPACITY];
} RLCaptureBuffer;

extern volatile RLCaptureBuffer g_rl_capture;
/* Control owner is sole producer. Arm only from its setup/safe configuration. */
void RLCaptureArm(uint32_t post_records);
void RLCapturePush(const RLShadowSnapshot *feedback, const RLShadowCommand *command,
                    const RLControlFrame *control);
#endif
