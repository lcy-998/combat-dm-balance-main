#include "rl_control.h"

#include <math.h>
#include <string.h>

#define RL_TWO_PI 6.2831853071795864769f
#define RL_MOTOR_POS_LIMIT 6.283f
#define RL_WHEEL_CURRENT_PER_NM (19.0f / (0.00036f * (268.0f / 17.0f)))

static float clampf(float x, float lo, float hi)
{
    return fminf(fmaxf(x, lo), hi);
}

static int command_usable(const RLShadowCommand *command, uint32_t now)
{
    if (command == NULL || !command->valid || command->snapshot_sequence == 0u ||
        (uint32_t)(now - command->output_tick_ms) > RL_CONTROL_COMMAND_MAX_AGE_MS ||
        (uint32_t)(now - command->snapshot_tick_ms) > RL_CONTROL_COMMAND_MAX_AGE_MS) {
        return 0;
    }
    for (unsigned i = 0; i < RL_SHADOW_ACTION_SIZE; ++i) {
        if (!isfinite(command->raw_action[i])) return 0;
    }
    for (unsigned i = 0; i < 4u; ++i) {
        if (!isfinite(command->output.joint_target_angle_rad[i])) return 0;
    }
    for (unsigned i = 0; i < 2u; ++i) {
        if (!isfinite(command->output.wheel_forward_torque_nm[i])) return 0;
    }
    return 1;
}

RLControlDecision RLControlStep(RLControlState *state, uint32_t now_tick,
                                int normal_lqr, const RLShadowCommand *command)
{
    RLControlDecision decision = {0};
    if (state == NULL) return decision;
    if (!normal_lqr) {
        decision.exit_beep = state->active;
        memset(state, 0, sizeof(*state));
        return decision;
    }
    if (!state->timing) {
        state->timing = 1u;
        state->lqr_since_tick = now_tick;
        return decision;
    }
    if ((uint32_t)(now_tick - state->lqr_since_tick) < RL_CONTROL_LQR_WAIT_MS) {
        return decision;
    }
    if (!command_usable(command, now_tick)) {
        decision.exit_beep = state->active;
        state->active = 0u;
        memset(state->previous_action, 0, sizeof(state->previous_action));
        return decision;
    }
    if (!state->active) {
        state->active = 1u;
        if (!state->beeped) {
            state->beeped = 1u;
            decision.beep = 1u;
        }
    }
    decision.active = 1u;
    /* The first valid RL cycle takes full control, including re-entry. */
    decision.blend = 1.0f;
    for (unsigned i = 0; i < RL_SHADOW_ACTION_SIZE; ++i) {
        state->previous_action[i] = decision.blend * clampf(command->raw_action[i], -1.0f, 1.0f);
    }
    return decision;
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
    if (elapsed >= RL_CONTROL_LEG_EXTEND_MS) {
        ramp->completed = 1u;
        return RL_CONTROL_LEG_TARGET_M;
    }
    const float fraction = (float)elapsed / (float)RL_CONTROL_LEG_EXTEND_MS;
    return ramp->start_length_m +
           fraction * (RL_CONTROL_LEG_TARGET_M - ramp->start_length_m);
}

int RLControlMapOutput(const RLShadowOutputPreview *output,
                       const float encoder_now[4], const float encoder_offset[4],
                       float encoder_target[4], float wheel_current[2])
{
    if (output == NULL || encoder_now == NULL || encoder_offset == NULL ||
        encoder_target == NULL || wheel_current == NULL) return -1;
    for (unsigned i = 0; i < 4u; ++i) {
        if (!isfinite(output->joint_target_angle_rad[i]) ||
            !isfinite(encoder_now[i]) || !isfinite(encoder_offset[i]) ||
            fabsf(encoder_now[i]) > RL_MOTOR_POS_LIMIT) return -1;
        const float raw = (i < 2u) ? encoder_offset[i] - output->joint_target_angle_rad[i]
                                   : output->joint_target_angle_rad[i] - encoder_offset[i];
        /* The motor task uses MCU shortest-angle PD, so keep the reference
           canonical and recompute its error from fresh feedback at each send. */
        float mapped = remainderf(raw, RL_TWO_PI);
        if (!isfinite(mapped) || mapped < -RL_MOTOR_POS_LIMIT ||
            mapped > RL_MOTOR_POS_LIMIT) return -1;
        encoder_target[i] = mapped;
    }
    for (unsigned i = 0; i < 2u; ++i) {
        if (!isfinite(output->wheel_forward_torque_nm[i])) return -1;
    }
    wheel_current[0] = clampf(-output->wheel_forward_torque_nm[0] * RL_WHEEL_CURRENT_PER_NM,
                              -8192.0f, 8192.0f);
    wheel_current[1] = clampf(output->wheel_forward_torque_nm[1] * RL_WHEEL_CURRENT_PER_NM,
                              -8192.0f, 8192.0f);
    return 0;
}

void RLControlBlendJoint(float encoder_now, float lqr_torque,
                         float encoder_target, float blend,
                         float *position, float *torque, float *kp, float *kd)
{
    blend = clampf(blend, 0.0f, 1.0f);
    *position = encoder_now + blend * remainderf(encoder_target - encoder_now, RL_TWO_PI);
    *torque = (1.0f - blend) * lqr_torque;
    *kp = 60.0f * blend;
    *kd = 2.0f * blend;
}

float RLControlBlendCurrent(float lqr_current, float rl_current, float blend)
{
    return clampf((1.0f - blend) * lqr_current + blend * rl_current,
                  -8192.0f, 8192.0f);
}
