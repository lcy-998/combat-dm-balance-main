#include "rl_capture.h"
#include <stddef.h>
#include <string.h>

#if defined(__arm__) || defined(__thumb__)
/* CPU-only trace ring; explicitly initialized by RLCaptureArm before tasks. */
__attribute__((section(".rl_capture"),aligned(32)))
#endif
volatile RLCaptureBuffer g_rl_capture;
_Static_assert(sizeof(RLCaptureRecord)%4u==0u,"capture must be 32-bit aligned");

void RLCaptureArm(uint32_t post_records)
{
    memset((void *)&g_rl_capture,0,sizeof(g_rl_capture));
    g_rl_capture.schema_version=RL_CAPTURE_VERSION;
    g_rl_capture.record_bytes=sizeof(RLCaptureRecord);
    g_rl_capture.capacity=RL_CAPTURE_CAPACITY;
    g_rl_capture.post_records=post_records<RL_CAPTURE_CAPACITY?post_records:RL_CAPTURE_CAPACITY-1u;
    g_rl_capture.armed=1u;
}

void RLCapturePush(const RLShadowSnapshot *feedback, const RLShadowCommand *command,
                    const RLControlFrame *control)
{
    if (!g_rl_capture.armed || g_rl_capture.frozen || feedback==NULL || control==NULL) return;
    RLCaptureRecord r={0};
    r.sequence=++g_rl_capture.records_written;
    r.frame_begin=r.sequence*2u;r.frame_end=r.frame_begin;
    r.tick_ms=control->tick_ms;r.owner=control->owner;r.fault=control->fault;
    r.control_epoch=control->control_epoch;r.adopted_sequence=control->adopted_sequence;
    r.adopted_tick_ms=control->adopted_tick_ms;r.previous_source=feedback->previous_source;
    r.valid_mask=4u|(control->owner==RL_OWNER_RL?2u:0u);
    r.feedback_valid_mask=feedback->feedback_valid_mask;
    memcpy(r.feedback_receive_tick_ms,feedback->feedback_receive_tick_ms,sizeof(r.feedback_receive_tick_ms));
    r.imu_update_tick_ms=feedback->imu_update_tick_ms;
    memcpy(r.pd_q,feedback->motion.joint_position_rad,sizeof(r.pd_q));
    memcpy(r.pd_dq,feedback->motion.joint_velocity_rad_s,sizeof(r.pd_dq));
    memcpy(r.driver_reported_leg_torque_motor_nm,feedback->driver_reported_leg_torque_motor_nm,sizeof(r.driver_reported_leg_torque_motor_nm));
    memcpy(r.driver_reported_wheel_current_raw,feedback->driver_reported_wheel_current_raw,sizeof(r.driver_reported_wheel_current_raw));
    if (command!=NULL && command->valid) {
        r.valid_mask|=1u;r.previous_source=command->previous_source;r.previous_sequence=command->previous_sequence;
        r.actor_input_tick_ms=command->actor_input_tick_ms;
        r.recovery_mode=command->recovery_mode;r.motion_candidate_valid=command->motion_valid;
        r.policy_sequence=command->snapshot_sequence;
        r.policy_snapshot_tick_ms=command->snapshot_tick_ms;r.policy_output_tick_ms=command->output_tick_ms;
        memcpy(r.observation,command->preview.observation,sizeof(r.observation));
        memcpy(r.mean_action,command->action,sizeof(r.mean_action));
        memcpy(r.requested_target,command->preview.target,sizeof(r.requested_target));
    }
    memcpy(r.adopted_target,control->adopted_target,sizeof(r.adopted_target));
    memcpy(r.pd_request_nm,control->tau_pd_nm,sizeof(r.pd_request_nm));
    memcpy(r.pd_limited_nm,control->tau_limited_nm,sizeof(r.pd_limited_nm));
    memcpy(r.spring_motor_nm,control->spring_motor_nm,sizeof(r.spring_motor_nm));
    memcpy(r.final_motor_command_nm,control->final_motor_nm,sizeof(r.final_motor_command_nm));
    memcpy(r.wheel_command_raw,control->wheel_current_raw,sizeof(r.wheel_command_raw));
    volatile RLCaptureRecord *slot=&g_rl_capture.records[g_rl_capture.write_index];
    slot->frame_begin=r.frame_begin|1u;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    r.frame_begin|=1u;*slot=r;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    slot->frame_begin=r.frame_end;
    g_rl_capture.write_index=(g_rl_capture.write_index+1u)%RL_CAPTURE_CAPACITY;
    if (g_rl_capture.count<RL_CAPTURE_CAPACITY) g_rl_capture.count++;
    if (!g_rl_capture.triggered &&
        ((control->owner==RL_OWNER_RL && g_rl_capture.owner_previous!=RL_OWNER_RL) || control->fault)) {
        g_rl_capture.triggered=1u;g_rl_capture.remaining=g_rl_capture.post_records;
    } else if (g_rl_capture.triggered && g_rl_capture.remaining) g_rl_capture.remaining--;
    if (g_rl_capture.triggered && !g_rl_capture.remaining) g_rl_capture.frozen=1u;
    g_rl_capture.owner_previous=control->owner;
}
