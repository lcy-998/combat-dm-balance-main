#include "rl_shadow.h"

#include <math.h>
#include <stddef.h>

typedef struct {
    float speed_m_s;
    float scale_rad_per_action;
} LegScalePoint;

/* D5 teacher_leg_scale_profile.json; abs(command vx), linear interpolation. */
static const LegScalePoint leg_scale_table[] = {
    {0.000000f, 0.190000f},
    {0.050000f, 0.190000f},
    {0.100000f, 0.190000f},
    {0.150000f, 0.190000f},
    {0.225000f, 0.190000f},
    {0.250000f, 0.190000f},
    {0.275000f, 0.191000f},
    {0.350000f, 0.194000f},
    {0.500000f, 0.200000f},
    {0.750000f, 0.210000f},
    {1.000000f, 0.220000f},
    {1.200000f, 0.240000f},
    {1.250000f, 0.240000f},
    {1.350000f, 0.240000f},
    {1.500000f, 0.240000f},
    {2.000000f, 0.250000f},
    {2.500000f, 0.265000f},
    {2.600000f, 0.268000f},
    {2.700000f, 0.270000f},
    {2.800000f, 0.273000f},
    {2.900000f, 0.276000f},
    {3.000000f, 0.280000f},
    {3.100000f, 0.283000f},
    {3.200000f, 0.286000f},
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
    const float lf_q = clipf(scale * clipf(raw_action[0], -1.0f, 1.0f), -2.58f, 2.58f);
    const float lb_q = clipf(scale * clipf(raw_action[1], -1.0f, 1.0f), -3.08f, 3.08f);
    const float rf_q = clipf(scale * clipf(raw_action[3], -1.0f, 1.0f), -2.58f, 2.58f);
    const float rb_q = clipf(scale * clipf(raw_action[4], -1.0f, 1.0f), -3.08f, 3.08f);
    preview->leg_scale_rad_per_action = scale;
    preview->joint_target_angle_rad[0] = RL_SHADOW_ACTIVE_ZERO_RAD + lf_q;
    preview->joint_target_angle_rad[1] = RL_SHADOW_HIP_ZERO_RAD + lb_q;
    preview->joint_target_angle_rad[2] = RL_SHADOW_ACTIVE_ZERO_RAD + rf_q;
    preview->joint_target_angle_rad[3] = RL_SHADOW_HIP_ZERO_RAD - rb_q;
    preview->wheel_forward_torque_nm[0] = 5.5f * clipf(raw_action[2], -1.0f, 1.0f);
    preview->wheel_forward_torque_nm[1] = 5.5f * clipf(raw_action[5], -1.0f, 1.0f);
    return 0;
}
