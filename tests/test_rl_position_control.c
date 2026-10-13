#include "rl_shadow.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

int main(void)
{
    float target_vx = 0.0f;
    assert(RLShadowPositionPID(1.0f, 1.0f, &target_vx) == 0);
    near(target_vx, 0.0f);
    assert(RLShadowPositionPID(1.0f, 1.8f, &target_vx) == 0);
    near(target_vx, 0.8f);
    assert(RLShadowPositionPID(1.0f, 0.25f, &target_vx) == 0);
    near(target_vx, -0.75f);
    assert(RLShadowPositionPID(0.0f, 10.0f, &target_vx) == 0);
    near(target_vx, 2.2f);
    assert(RLShadowPositionPID(10.0f, 0.0f, &target_vx) == 0);
    near(target_vx, -2.2f);
    assert(RLShadowPositionPID(NAN, 1.0f, &target_vx) != 0);
    assert(RLShadowPositionPID(1.0f, INFINITY, &target_vx) != 0);
    assert(RLShadowPositionPID(-3.0e38f, 3.0e38f, &target_vx) != 0);

    RLShadowSnapshot sample = {0};
    sample.odom_s_m = 0.0f;
    sample.target_s_m = 1.2f;
    sample.remote_vx_m_s = 0.1f;
    sample.heading_vx_m_s = 0.4f;
    assert(RLShadowPositionPID(sample.odom_s_m, sample.target_s_m,
                               &sample.target_vx_m_s) == 0);
    float observation[RL_SHADOW_OBSERVATION_SIZE];
    assert(RLShadowBuildObservation(&sample, observation) == 0);
    near(observation[6], 1.68f);
    near(observation[7], 0.6f);
    near(observation[8], 0.0f);

    const float action[RL_SHADOW_ACTION_SIZE] = {0};
    RLShadowOutputPreview converted;
    assert(RLShadowConvertOutput(sample.target_vx_m_s, action, &converted) == 0);
    near(converted.leg_scale_rad_per_action, 0.240f);
    puts("rl position control tests passed");
    return 0;
}
