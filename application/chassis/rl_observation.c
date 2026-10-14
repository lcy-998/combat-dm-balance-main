#include "rl_shadow.h"

#include <math.h>
#include <string.h>

#define RL_PI 3.14159265358979323846f

static float clipf(float x, float lo, float hi)
{
    return fminf(fmaxf(x, lo), hi);
}

static float wrap_pi(float angle)
{
    return remainderf(angle, 2.0f * RL_PI);
}

int RLShadowUpdatePositionTarget(RLShadowPositionTarget *state,
                                 const RLShadowSnapshot *sample,
                                 uint32_t now_tick_ms, float *target_s_m)
{
    if (state == NULL || sample == NULL || target_s_m == NULL) {
        return -1;
    }
    if (!isfinite(sample->odom_s_m) || !isfinite(sample->target_s_m) ||
        !isfinite(sample->remote_vx_m_s)) {
        state->active = 0u;
        state->zero_latched = 0u;
        return -1;
    }
    if (!sample->rl_active) {
        state->active = 0u;
        state->zero_latched = 0u;
        *target_s_m = sample->target_s_m;
        return 0;
    }

    const uint32_t elapsed_ms = (uint32_t)(now_tick_ms - state->last_tick_ms);
    if (!state->active ||
        state->activation_sequence != sample->rl_activation_sequence ||
        elapsed_ms > RL_SHADOW_POSITION_MAX_DT_MS) {
        state->target_s_m = sample->odom_s_m;
        state->active = 1u;
        state->zero_latched = (sample->remote_vx_m_s == 0.0f);
    } else if (sample->remote_vx_m_s == 0.0f) {
        if (!state->zero_latched) {
            state->target_s_m = sample->odom_s_m;
            state->zero_latched = 1u;
        }
    } else {
        state->zero_latched = 0u;
        const float next_target = state->target_s_m +
                                  sample->remote_vx_m_s * (0.001f * (float)elapsed_ms);
        if (!isfinite(next_target)) {
            state->active = 0u;
            return -1;
        }
        state->target_s_m = next_target;
    }
    state->last_tick_ms = now_tick_ms;
    state->activation_sequence = sample->rl_activation_sequence;
    *target_s_m = state->target_s_m;
    return 0;
}

int RLShadowPositionPID(float odom_s_m, float target_s_m,
                        float *target_vx_m_s)
{
    if (target_vx_m_s == NULL || !isfinite(odom_s_m) || !isfinite(target_s_m)) {
        return -1;
    }
    const float error_m = target_s_m - odom_s_m;
    if (!isfinite(error_m)) {
        return -1;
    }
    /* Position PID: Kp=1, Ki=0, Kd=0 for the initial tuning. */
    *target_vx_m_s = clipf(RL_SHADOW_POSITION_KP * error_m,
                            -RL_SHADOW_TARGET_SPEED_LIMIT_M_S,
                            RL_SHADOW_TARGET_SPEED_LIMIT_M_S);
    return 0;
}

int RLShadowBuildObservation(const RLShadowSnapshot *s,
                             float o[RL_SHADOW_OBSERVATION_SIZE])
{
    if (s == NULL || o == NULL) {
        return -1;
    }
    const float cp = cosf(s->pitch_rad);
    const float sr = sinf(s->roll_rad);
    const float cr = cosf(s->roll_rad);
    const float yaw_rate = s->yaw_rate_rad_s;

    memset(o, 0, sizeof(float) * RL_SHADOW_OBSERVATION_SIZE);
    o[0] = 0.25f * s->roll_rate_rad_s;
    o[1] = -0.25f * s->pitch_rate_rad_s;
    o[2] = 0.25f * yaw_rate;
    o[3] = sinf(s->pitch_rad);          /* projected gravity: nose down + */
    o[4] = sr * cp;                     /* projected gravity: right lean + */
    o[5] = -cr * cp;
    o[6] = clipf(2.10f * (s->target_vx_m_s - s->heading_vx_m_s), -2.0f, 2.0f);
    o[7] = clipf(0.50f * s->target_vx_m_s, -2.0f, 2.0f);
    o[8] = clipf(s->target_yaw_rad_s - yaw_rate, -2.0f, 2.0f);

    /* Model right = physical left. Slot 4 (model left_hip) has a reversed axis. */
    o[9] = wrap_pi(s->joint_angle_rad[0] - RL_SHADOW_ACTIVE_ZERO_RAD);
    o[10] = wrap_pi(s->joint_angle_rad[1] - RL_SHADOW_HIP_ZERO_RAD);
    o[11] = wrap_pi(s->joint_angle_rad[2] - RL_SHADOW_ACTIVE_ZERO_RAD);
    o[12] = -wrap_pi(s->joint_angle_rad[3] - RL_SHADOW_HIP_ZERO_RAD);
    o[13] = 0.05f * s->joint_rate_rad_s[0];
    o[14] = 0.05f * s->joint_rate_rad_s[1];
    o[15] = 0.05f * s->wheel_rate_rad_s[0];
    o[16] = 0.05f * s->joint_rate_rad_s[2];
    o[17] = -0.05f * s->joint_rate_rad_s[3];
    o[18] = 0.05f * s->wheel_rate_rad_s[1];
    for (unsigned i = 0; i < RL_SHADOW_ACTION_SIZE; ++i) {
        o[19u + i] = s->previous_action[i];
    }

    for (unsigned i = 0; i < RL_SHADOW_OBSERVATION_SIZE; ++i) {
        if (!isfinite(o[i])) {
            return -1;
        }
    }
    return 0;
}
