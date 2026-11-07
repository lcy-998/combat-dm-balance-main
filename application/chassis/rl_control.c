#include "rl_control.h"

#include <math.h>
#include <string.h>

#define RL_CONTROL_TWO_PI 6.2831853071795864769f

static float clampf(float value, float lower, float upper)
{
    return fminf(fmaxf(value, lower), upper);
}

float RLControlCommandHeight(const RLControlState *state)
{
    return state!=NULL && state->height.initialized ? state->height.command_m : RL_MOTION31_HEIGHT_M;
}

static int command_usable(const RLShadowCommand *command, uint32_t now_tick)
{
    if (command == NULL || !command->valid || command->snapshot_sequence == 0u ||
        (uint32_t)(now_tick - command->output_tick_ms) > RL_CONTROL_COMMAND_MAX_AGE_MS ||
        (uint32_t)(now_tick - command->snapshot_tick_ms) > RL_CONTROL_COMMAND_MAX_AGE_MS) {
        return 0;
    }
    for (unsigned i = 0; i < RL_MOTION31_ACTION_SIZE; ++i) {
        if (!isfinite(command->action[i]) || !isfinite(command->preview.target[i])) return 0;
    }
    for (unsigned i = 0; i < RL_MOTION31_OBSERVATION_SIZE; ++i) {
        if (!isfinite(command->preview.observation[i])) return 0;
    }
    /* Decode using this packet's original Actor input height, not the newer
       live height. Transport legitimately overlaps a changed height command;
       already computed absolute targets must never be recentered in flight. */
    float decoded[6];
    if (RLMotion31DecodeOutput(command->action,
            command->preview.observation[24] * 0.5f, decoded) != 0) return 0;
    for (unsigned i = 0; i < 6u; ++i) {
        if (fabsf(command->action[i]) > 1.00001f ||
            fabsf(decoded[i] - command->preview.target[i]) > 0.0001f) return 0;
    }
    return 1;
}

float RLControlLegRampStep(RLControlState *state, uint32_t now_tick,
                           int normal_lqr, float current_target_m)
{
    if (state == NULL || !isfinite(current_target_m)) return current_target_m;
    if (!normal_lqr) {
        state->ramp_started = state->ramp_completed = 0u;
        return current_target_m;
    }
    if (!state->ramp_started) {
        state->ramp_started = 1u;
        state->ramp_start_tick = now_tick;
        state->ramp_start_length_m = current_target_m;
    }
    const uint32_t elapsed = now_tick - state->ramp_start_tick;
    if (state->ramp_completed || elapsed >= RL_CONTROL_LEG_RAMP_MS) {
        state->ramp_completed = 1u;
        return RL_CONTROL_LEG_TARGET_M;
    }
    return state->ramp_start_length_m + (RL_CONTROL_LEG_TARGET_M -
           state->ramp_start_length_m) * ((float)elapsed / RL_CONTROL_LEG_RAMP_MS);
}

RLControlDecision RLControlStep(RLControlState *state,
                                uint32_t now_tick, int normal_lqr,
                                const RLShadowCommand *command)
{
    RLControlDecision decision = {0};
    if (state == NULL) return decision;
    if (!normal_lqr) {
        decision.exited = state->active;
        const uint32_t next = state->epoch + (state->timing || state->active || state->fault ? 1u : 0u);
        const uint8_t fault=state->fault;
        memset(state, 0, sizeof(*state)); state->epoch = next;state->fault=fault;
        return decision;
    }
    if (state->fault || !state->ramp_completed) return decision;
    if (!state->timing) {
        state->timing = 1u; state->epoch++;
        state->wait_start_tick = now_tick;
        return decision;
    }
    /* Readiness remains latched until mode exit/reset, even after clock wrap. */
    if (!state->ready)
        state->ready = ((uint32_t)(now_tick-state->wait_start_tick) >= RL_CONTROL_INFERENCE_DELAY_MS);
    /* This is eligibility only; RLControlExecute commits actual ownership. */
    decision.active = state->ready && command_usable(command, now_tick) && command->control_epoch == state->epoch;
    return decision;
}

void RLControlReset(RLControlState *state)
{
    if(state==NULL)return;
    const uint32_t epoch=state->epoch+1u;
    memset(state,0,sizeof(*state));state->epoch=epoch;
}

void RLControlFillPrevious(const RLControlState *state, RLShadowSnapshot *s)
{
    if (state == NULL || s == NULL) return;
    s->control_epoch = state->epoch;
    s->controller_owner = state->active ? RL_OWNER_RL : RL_OWNER_LQR;
    s->adopted_sequence = state->active ? state->adopted_sequence : 0u;
    s->adopted_tick_ms = state->active ? state->adopted_tick_ms : 0u;
    s->previous_source = state->active ? RL_PREVIOUS_ADOPTED : RL_PREVIOUS_INIT_PROXY;
    if (state->active) {
        memcpy(s->motion.previous_target, state->adopted_target, sizeof(state->adopted_target));
    } else {
        /* Declared takeover proxy v1. Does not claim LQR executed these targets. */
        const unsigned legs[4] = {0u,1u,3u,4u};
        for (unsigned i=0;i<4u;++i) s->motion.previous_target[legs[i]] = s->motion.joint_position_rad[i];
        s->motion.previous_target[2] = s->motion.joint_velocity_rad_s[2];
        s->motion.previous_target[5] = s->motion.joint_velocity_rad_s[5];
    }
    s->motion.previous_target_valid = 1u;
}

static int execution_fault(RLControlState *state, RLControlFrame *out, unsigned reason)
{
    state->fault = (uint8_t)reason;
    state->active = 0u; state->ready = 0u; state->epoch++;
    out->owner = RL_OWNER_LQR; out->fault = reason; out->control_epoch = state->epoch;
    return -1;
}

int RLControlFeedbackFresh(const RLShadowSnapshot *s,uint32_t now)
{
    if (s==NULL || (uint32_t)(now-s->tick_ms)>RL_CONTROL_COMMAND_MAX_AGE_MS ||
        s->feedback_valid_mask!=0x3fu || s->imu_update_sequence==0u ||
        (uint32_t)(now-s->imu_update_tick_ms)>RL_CONTROL_COMMAND_MAX_AGE_MS) return 0;
    for (unsigned i=0;i<6u;++i)
        if ((uint32_t)(now-s->feedback_receive_tick_ms[i])>RL_CONTROL_COMMAND_MAX_AGE_MS) return 0;
    return 1;
}

int RLControlSetCommandHeight(RLControlState *state,float command_height_m)
{
    if(state==NULL || !isfinite(command_height_m) ||
       command_height_m<RL_MOTION31_HEIGHT_MIN_M-1e-7f ||
       command_height_m>RL_MOTION31_HEIGHT_MAX_M+1e-7f) return -1;
    state->height.initialized=1;state->height.command_m=command_height_m;
    return 0;
}

int RLControlExecute(RLControlState *state,
                     uint32_t now_ms, int normal_lqr,
                     const RLShadowSnapshot *current, const RLShadowCommand *command,
                     RLControlFrame *frame)
{
    if (state == NULL || frame == NULL) return -1;
    memset(frame,0,sizeof(*frame)); frame->tick_ms=now_ms;
    const RLControlDecision eligible=RLControlStep(state,now_ms,normal_lqr,command);
    frame->control_epoch=state->epoch;frame->fault=state->fault;
    if (!normal_lqr || !state->ready || state->fault) return 1;
    if (!RLControlFeedbackFresh(current,now_ms) ||
        RLMotion31ValidateSnapshot(&current->motion)!=0)
        return execution_fault(state,frame,RL_FAULT_INPUT);
    RLControlState pending=*state;
    int new_target=0;
    if (eligible.active && (!state->active ||
        (int32_t)(command->snapshot_sequence-state->adopted_sequence)>0)) {
        const int causal = state->active ?
            (command->previous_source==RL_PREVIOUS_ADOPTED && command->previous_sequence==state->adopted_sequence) :
            (command->previous_source==RL_PREVIOUS_INIT_PROXY && command->previous_sequence==0u);
        if (causal) {
            memcpy(pending.adopted_target,command->preview.target,sizeof(pending.adopted_target));
            pending.adopted_sequence=command->snapshot_sequence;
            pending.adopted_tick_ms=now_ms;new_target=1;
            pending.adopted_height_m=command->preview.observation[24]*.5f;
        }
    }
    if (!state->active && !new_target) return 1; /* keep LQR until the first valid policy command */
    if (!new_target && (uint32_t)(now_ms-state->adopted_tick_ms)>RL_CONTROL_COMMAND_MAX_AGE_MS)
        return execution_fault(state,frame,RL_FAULT_COMMAND);
    float motor[4],wheel[2];
    if (RLMotion31ComputePdTorque(&current->motion,pending.adopted_target,
                                  frame->tau_pd_nm,frame->tau_limited_nm)!=0 ||
        RLMotion31MapPolicyLegTorqueToMotorOrder(frame->tau_limited_nm,motor)!=0 ||
        RLMotion31MapPolicyWheelTorqueToForwardOrder(frame->tau_limited_nm,wheel)!=0)
        return execution_fault(state,frame,RL_FAULT_PD);
    const float zero[4]={0};
    if (RLControlCompensateSpring(zero,current->leg_length_m,current->leg_spread_rad,frame->spring_motor_nm)!=0 ||
        RLControlCompensateSpring(motor,current->leg_length_m,current->leg_spread_rad,frame->final_motor_nm)!=0)
        return execution_fault(state,frame,RL_FAULT_COMPENSATION);
    frame->wheel_current_raw[0]=clampf(-wheel[0]*RL_MOTION31_WHEEL_CURRENT_PER_NM,-RL_CONTROL_WHEEL_RAW_LIMIT,RL_CONTROL_WHEEL_RAW_LIMIT);
    frame->wheel_current_raw[1]=clampf( wheel[1]*RL_MOTION31_WHEEL_CURRENT_PER_NM,-RL_CONTROL_WHEEL_RAW_LIMIT,RL_CONTROL_WHEEL_RAW_LIMIT);
    pending.active=1u; *state=pending;
    frame->owner=RL_OWNER_RL; frame->adopted_sequence=state->adopted_sequence;
    frame->adopted_tick_ms=state->adopted_tick_ms;frame->snapshot_sequence=current->sequence;
    memcpy(frame->adopted_target,state->adopted_target,sizeof(state->adopted_target));
    return 0;
}

int RLControlCompensateSpring(const float policy_torque_nm[4],
                              const float leg_length_m[2],
                              const float leg_spread_rad[2],
                              float motor_torque_nm[4])
{
    if (policy_torque_nm == NULL || leg_length_m == NULL ||
        leg_spread_rad == NULL || motor_torque_nm == NULL) return -1;
    for (unsigned i = 0; i < 4u; ++i) {
        if (!isfinite(policy_torque_nm[i])) return -1;
    }
    float spring[2];
    for (unsigned i = 0; i < 2u; ++i) {
        if (!isfinite(leg_length_m[i]) || leg_length_m[i] <= 0.0f ||
            !isfinite(leg_spread_rad[i])) return -1;
        spring[i] = clampf((-83.121f * leg_length_m[i] + 40.242f) * 0.8f,
                            16.5f, 27.0f) * sinf(leg_spread_rad[i]);
    }
    motor_torque_nm[0] = clampf(policy_torque_nm[0] - spring[0], -40.0f, 40.0f);
    motor_torque_nm[1] = clampf(policy_torque_nm[1] + spring[0], -40.0f, 40.0f);
    motor_torque_nm[2] = clampf(policy_torque_nm[2] + spring[1], -40.0f, 40.0f);
    motor_torque_nm[3] = clampf(policy_torque_nm[3] - spring[1], -40.0f, 40.0f);
    return 0;
}

int RLControlMapOutput(const RLMotion31Preview *preview,
                       const float wheel_forward_torque_nm[2],
                       const float encoder_now[4], const float encoder_offset[4],
                       float encoder_target[4], float wheel_current[2])
{
    if (preview == NULL || wheel_forward_torque_nm == NULL ||
        encoder_now == NULL || encoder_offset == NULL ||
        encoder_target == NULL || wheel_current == NULL) return -1;

    for (unsigned i = 0; i < 4u; ++i) {
        const float target_angle = preview->hardware_joint_target_rad[i];
        if (!isfinite(target_angle) || !isfinite(encoder_now[i]) ||
            !isfinite(encoder_offset[i]) || fabsf(encoder_now[i]) > RL_CONTROL_TWO_PI) {
            return -1;
        }
        const float raw = (i < 2u) ? encoder_offset[i] - target_angle
                                   : target_angle - encoder_offset[i];
        encoder_target[i] = remainderf(raw, RL_CONTROL_TWO_PI);
        if (!isfinite(encoder_target[i]) ||
            fabsf(encoder_target[i]) > RL_CONTROL_TWO_PI) return -1;
    }
    for (unsigned i = 0; i < 2u; ++i) {
        if (!isfinite(preview->motor_wheel_target_rad_s[i]) ||
            !isfinite(wheel_forward_torque_nm[i])) return -1;
    }
    wheel_current[0] = clampf(-wheel_forward_torque_nm[0] * RL_MOTION31_WHEEL_CURRENT_PER_NM,
                              -8192.0f, 8192.0f);
    wheel_current[1] = clampf(wheel_forward_torque_nm[1] * RL_MOTION31_WHEEL_CURRENT_PER_NM,
                              -8192.0f, 8192.0f);
    return 0;
}
