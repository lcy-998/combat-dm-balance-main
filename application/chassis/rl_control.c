#include "rl_control.h"

#include <math.h>
#include <string.h>

#define RL_CONTROL_TWO_PI 6.2831853071795864769f

static float clampf(float value, float lower, float upper)
{
    return fminf(fmaxf(value, lower), upper);
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
    for (unsigned i = 0; i < 4u; ++i) {
        if (!isfinite(command->preview.hardware_joint_target_rad[i])) return 0;
    }
    for (unsigned i = 0; i < 2u; ++i) {
        if (!isfinite(command->wheel_forward_torque_nm[i])) return 0;
    }
    return 1;
}

float RLControlLegRampStep(RLControlLegRamp *ramp, uint32_t now_tick,
                           int normal_lqr, float current_target_m)
{
    if (ramp == NULL || !isfinite(current_target_m)) return current_target_m;
    if (!normal_lqr) {
        memset(ramp, 0, sizeof(*ramp));
        return current_target_m;
    }
    if (!ramp->started) {
        ramp->started = 1u;
        ramp->start_tick = now_tick;
        ramp->start_length_m = current_target_m;
        return current_target_m;
    }
    if (ramp->completed) return current_target_m;

    const uint32_t elapsed = (uint32_t)(now_tick - ramp->start_tick);
    if (elapsed >= RL_CONTROL_LEG_RAMP_MS) {
        ramp->completed = 1u;
        return RL_CONTROL_LEG_TARGET_M;
    }
    const float fraction = (float)elapsed / (float)RL_CONTROL_LEG_RAMP_MS;
    return ramp->start_length_m +
           fraction * (RL_CONTROL_LEG_TARGET_M - ramp->start_length_m);
}

RLControlDecision RLControlStep(RLControlState *state, RLControlLegRamp *ramp,
                                uint32_t now_tick, int normal_lqr,
                                const RLShadowCommand *command)
{
    RLControlDecision decision = {0};
    if (state == NULL || ramp == NULL) return decision;
    if (!normal_lqr) {
        decision.exited = state->active;
        memset(state, 0, sizeof(*state));
        memset(ramp, 0, sizeof(*ramp));
        return decision;
    }
    if (!ramp->completed) {
        state->timing = 0u;
        state->active = 0u;
        return decision;
    }
    if (!state->timing) {
        state->timing = 1u;
        state->wait_start_tick = now_tick;
        return decision;
    }
    if ((uint32_t)(now_tick - state->wait_start_tick) <
        RL_CONTROL_LQR_HOLD_AFTER_RAMP_MS) {
        return decision;
    }
    if (!command_usable(command, now_tick)) {
        decision.exited = state->active;
        state->active = 0u;
        state->timing = 0u;
        memset(ramp, 0, sizeof(*ramp));
        return decision;
    }
    state->active = 1u;
    decision.active = 1u;
    return decision;
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
