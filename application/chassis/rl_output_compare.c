#include "rl_shadow.h"

#include <math.h>
#include <stddef.h>

typedef struct {
    float speed_m_s;
    float scale_rad_per_action;
} LegScalePoint;

static const LegScalePoint leg_scale_table[] = {
    {0.0f, 0.180f}, {1.0f, 0.220f}, {1.2f, 0.240f},
    {1.35f, 0.240f}, {1.5f, 0.240f}, {1.6f, 0.242f},
    {1.75f, 0.245f}, {2.0f, 0.250f}, {2.2f, 0.255f},
    {2.4f, 0.262f}, {2.5f, 0.265f}, {2.6f, 0.268f},
    {2.7f, 0.270f}, {2.8f, 0.273f}, {2.9f, 0.276f},
    {3.0f, 0.280f}, {3.1f, 0.283f}, {3.2f, 0.286f}
};

static float clipf(float value, float lower, float upper)
{
    return fminf(fmaxf(value, lower), upper);
}

static float leg_scale_at(float target_vx_m_s)
{
    const float speed = fabsf(target_vx_m_s);
    const size_t count = sizeof(leg_scale_table) / sizeof(leg_scale_table[0]);
    if (speed <= leg_scale_table[0].speed_m_s) {
        return leg_scale_table[0].scale_rad_per_action;
    }
    if (speed >= leg_scale_table[count - 1].speed_m_s) {
        return leg_scale_table[count - 1].scale_rad_per_action;
    }
    for (size_t i = 1; i < count; ++i) {
        if (speed <= leg_scale_table[i].speed_m_s) {
            const LegScalePoint *lo = &leg_scale_table[i - 1];
            const LegScalePoint *hi = &leg_scale_table[i];
            const float t = (speed - lo->speed_m_s) / (hi->speed_m_s - lo->speed_m_s);
            return lo->scale_rad_per_action + t * (hi->scale_rad_per_action - lo->scale_rad_per_action);
        }
    }
    return leg_scale_table[count - 1].scale_rad_per_action;
}

int RLShadowConvertOutput(float target_vx_m_s,
                          const float raw_action[RL_SHADOW_ACTION_SIZE],
                          RLShadowOutputPreview *preview)
{
    if (raw_action == NULL || preview == NULL || !isfinite(target_vx_m_s)) {
        return -1;
    }
    for (unsigned i = 0; i < RL_SHADOW_ACTION_SIZE; ++i) {
        if (!isfinite(raw_action[i])) {
            return -1;
        }
    }

    const float scale = leg_scale_at(target_vx_m_s);
    const float left_hip_action = clipf(raw_action[1], -1.0f, 1.0f);
    const float right_hip_action = clipf(raw_action[4], -1.0f, 1.0f);
    preview->leg_scale_rad_per_action = scale;
    preview->hip_model_target_rad[0] = clipf(scale * left_hip_action, -3.08f, 3.08f);
    preview->hip_model_target_rad[1] = clipf(scale * right_hip_action, -3.08f, 3.08f);
    preview->hip_physical_target_rad[0] = RL_SHADOW_HIP_ZERO_RAD + preview->hip_model_target_rad[0];
    preview->hip_physical_target_rad[1] = RL_SHADOW_HIP_ZERO_RAD - preview->hip_model_target_rad[1];
    preview->wheel_forward_torque_nm[0] = 5.5f * clipf(raw_action[2], -1.0f, 1.0f);
    preview->wheel_forward_torque_nm[1] = 5.5f * clipf(raw_action[5], -1.0f, 1.0f);
    return 0;
}
