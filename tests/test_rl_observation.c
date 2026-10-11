#include "rl_shadow.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define DEG_TO_RAD (3.14159265358979323846f / 180.0f)

static void near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

int main(void)
{
    RLShadowSnapshot s = {0};
    float o[RL_SHADOW_OBSERVATION_SIZE];
    const float active_zero = 137.1081f * DEG_TO_RAD;
    const float hip_zero = 42.8966f * DEG_TO_RAD;
    s.joint_angle_rad[0] = active_zero;
    s.joint_angle_rad[1] = hip_zero;
    s.joint_angle_rad[2] = active_zero;
    s.joint_angle_rad[3] = hip_zero;
    assert(RLShadowBuildObservation(&s, o) == 0);
    near(o[3], 0.0f);
    near(o[4], 0.0f);
    near(o[5], -1.0f);
    for (unsigned i = 9; i <= 12; ++i) near(o[i], 0.0f);
    for (unsigned i = 19; i < 25; ++i) near(o[i], 0.0f);

    s.pitch_rad = 0.1f;          /* nose down */
    s.roll_rad = 0.2f;           /* lean right */
    s.roll_rate_rad_s = 2.0f;
    s.pitch_rate_rad_s = 3.0f;  /* nose down rate => policy pitch negative */
    s.yaw_rate_rad_s = 0.5f;    /* left turn */
    s.target_vx_m_s = 1.0f;
    s.heading_vx_m_s = 0.5f;
    s.target_yaw_rad_s = 1.0f;
    s.joint_angle_rad[0] += 0.1f;
    s.joint_angle_rad[1] += 0.2f;
    s.joint_angle_rad[2] += 0.3f;
    s.joint_angle_rad[3] += 0.4f;
    s.joint_rate_rad_s[0] = 1.0f;
    s.joint_rate_rad_s[1] = 2.0f;
    s.joint_rate_rad_s[2] = 3.0f;
    s.joint_rate_rad_s[3] = 4.0f;
    s.wheel_rate_rad_s[0] = -5.0f; /* forward rolling */
    s.wheel_rate_rad_s[1] = -6.0f;
    assert(RLShadowBuildObservation(&s, o) == 0);
    near(o[0], 0.5f);
    near(o[1], -0.75f);
    near(o[2], 0.125f);
    near(o[3], sinf(0.1f));
    near(o[4], sinf(0.2f) * cosf(0.1f));
    near(o[6], 1.05f);
    near(o[7], 0.5f);
    near(o[8], 0.5f); /* unscaled yaw rate */
    near(o[9], 0.1f);
    near(o[10], 0.2f);
    near(o[11], 0.3f);
    near(o[12], -0.4f);
    near(o[13], 0.05f);
    near(o[14], 0.1f);
    near(o[15], -0.25f);
    near(o[16], 0.15f);
    near(o[17], -0.2f);
    near(o[18], -0.3f);

    s.target_vx_m_s = 10.0f;
    s.target_yaw_rad_s = 10.0f;
    assert(RLShadowBuildObservation(&s, o) == 0);
    near(o[6], 2.0f);
    near(o[7], 2.0f);
    near(o[8], 2.0f);
    s.joint_rate_rad_s[0] = NAN;
    assert(RLShadowBuildObservation(&s, o) != 0);
    puts("rl observation tests passed");
    return 0;
}
