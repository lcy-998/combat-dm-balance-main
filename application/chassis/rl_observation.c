#include "rl_shadow.h"

#include <math.h>
#include <string.h>

#define RL_PI 3.14159265358979323846f
#define RL_DEG_TO_RAD (RL_PI / 180.0f)
#define RL_ACTIVE_ZERO_RAD ((180.0f - 42.8919f) * RL_DEG_TO_RAD)

static float clipf(float x, float lo, float hi)
{
    return fminf(fmaxf(x, lo), hi);
}

static float wrap_pi(float angle)
{
    return remainderf(angle, 2.0f * RL_PI);
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
    o[9] = wrap_pi(s->joint_angle_rad[0] - RL_ACTIVE_ZERO_RAD);
    o[10] = wrap_pi(s->joint_angle_rad[1] - RL_SHADOW_HIP_ZERO_RAD);
    o[11] = wrap_pi(s->joint_angle_rad[2] - RL_ACTIVE_ZERO_RAD);
    o[12] = -wrap_pi(s->joint_angle_rad[3] - RL_SHADOW_HIP_ZERO_RAD);
    o[13] = 0.05f * s->joint_rate_rad_s[0];
    o[14] = 0.05f * s->joint_rate_rad_s[1];
    o[15] = 0.05f * s->wheel_rate_rad_s[0];
    o[16] = 0.05f * s->joint_rate_rad_s[2];
    o[17] = -0.05f * s->joint_rate_rad_s[3];
    o[18] = 0.05f * s->wheel_rate_rad_s[1];
    /* 19..24 are zero: no policy action has actually been executed. */

    for (unsigned i = 0; i < RL_SHADOW_OBSERVATION_SIZE; ++i) {
        if (!isfinite(o[i])) {
            return -1;
        }
    }
    return 0;
}
