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
        if (!isfinite(command->preview.hardware_joint_target_rad[i]) ||
            !isfinite(command->leg_torque_motor_nm[i])) return 0;
    }
    for (unsigned i = 0; i < 2u; ++i) {
        if (!isfinite(command->wheel_forward_torque_nm[i])) return 0;
    }
    return 1;
}

RLControlDecision RLControlStep(RLControlState *state, uint32_t now_tick,
                                int enabled,
                                const RLShadowCommand *command)
{
    RLControlDecision decision = {0};
    if (state == NULL) return decision;
    const uint8_t was_active = state->active;
    if (!enabled) {
        memset(state, 0, sizeof(*state));
        decision.exited = was_active;
        return decision;
    }
    if (!state->enabled) {
        state->enabled = 1u;
        state->recovering = 1u;
        state->enable_tick = now_tick;
    }
    /* Reject pre-enable outputs, including outputs from the same scheduler tick. */
    state->active = command_usable(command, now_tick) &&
        (int32_t)(command->snapshot_tick_ms - state->enable_tick) > 0 &&
        command->recovery_mode == state->recovering;
    decision.active = state->active;
    decision.exited = was_active && !state->active;
    decision.recovering = state->recovering;
    return decision;
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
        /* Original measured leg-length fit and compensation direction. */
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
