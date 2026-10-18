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

    RLShadowSnapshot sample = {0};
    sample.odom_s_m = 0.0f;
    sample.target_s_m = 1.2f;
    sample.remote_vx_m_s = 0.1f;
    sample.heading_vx_m_s = 0.4f;
    RLShadowPositionTarget position_target = {0};
    float target_s_m;
    assert(RLShadowUpdatePositionTarget(&position_target, &sample, 100u,
                                        &target_s_m) == 0);
    near(target_s_m, 1.2f); /* Legacy target before RL takeover. */
    assert(RLShadowRemoteVelocity(sample.remote_vx_m_s,
                                  &sample.target_vx_m_s) == 0);
    float observation[RL_SHADOW_OBSERVATION_SIZE];
    assert(RLShadowBuildObservation(&sample, observation) == 0);
    near(observation[6], -0.63f);
    near(observation[7], 0.05f);
    near(observation[8], 0.0f);

    const float action[RL_SHADOW_ACTION_SIZE] = {0};
    RLShadowOutputPreview converted;
    assert(RLShadowConvertOutput(sample.target_vx_m_s, action, &converted) == 0);
    near(converted.leg_scale_rad_per_action, 0.186f);

    sample.rl_active = 1u;
    sample.rl_activation_sequence = 1u;
    sample.odom_s_m = 2.0f;
    sample.target_s_m = 2.0f;
    sample.remote_vx_m_s = 1.0f;
    assert(RLShadowUpdatePositionTarget(&position_target, &sample, 200u,
                                        &target_s_m) == 0);
    near(target_s_m, 2.0f); /* Entry resets the RL target to measured s. */
    assert(RLShadowUpdatePositionTarget(&position_target, &sample, 208u,
                                        &target_s_m) == 0);
    near(target_s_m, 2.008f);
    assert(RLShadowUpdatePositionTarget(&position_target, &sample, 220u,
                                        &target_s_m) == 0);
    near(target_s_m, 2.020f); /* Actual 12 ms period, not fixed 8 ms. */
    assert(RLShadowRemoteVelocity(sample.remote_vx_m_s, &target_vx) == 0);
    near(target_vx, 1.0f);
    sample.remote_vx_m_s = 0.0f;
    sample.odom_s_m = 2.03f;
    assert(RLShadowUpdatePositionTarget(&position_target, &sample, 228u,
                                        &target_s_m) == 0);
    near(target_s_m, 2.03f); /* Zero command latches the measured position. */
    sample.odom_s_m = 2.04f;
    assert(RLShadowUpdatePositionTarget(&position_target, &sample, 232u,
                                        &target_s_m) == 0);
    near(target_s_m, 2.03f); /* Drift does not move the held target. */
    sample.remote_vx_m_s = -1.0f;
    assert(RLShadowUpdatePositionTarget(&position_target, &sample, 236u,
                                        &target_s_m) == 0);
    near(target_s_m, 2.026f);
    sample.rl_activation_sequence = 2u;
    sample.odom_s_m = 2.5f;
    assert(RLShadowUpdatePositionTarget(&position_target, &sample, 240u,
                                        &target_s_m) == 0);
    near(target_s_m, 2.5f); /* A brief exit/reentry may be missed by this task. */

    sample.rl_active = 0u;
    sample.target_s_m = 3.0f;
    assert(RLShadowUpdatePositionTarget(&position_target, &sample, 244u,
                                        &target_s_m) == 0);
    near(target_s_m, 3.0f);
    assert(!position_target.active);
    sample.rl_active = 1u;
    sample.odom_s_m = 4.0f;
    assert(RLShadowUpdatePositionTarget(&position_target, &sample, 252u,
                                        &target_s_m) == 0);
    near(target_s_m, 4.0f);
    sample.odom_s_m = 4.1f;
    assert(RLShadowUpdatePositionTarget(&position_target, &sample, 300u,
                                        &target_s_m) == 0);
    near(target_s_m, 4.1f); /* Long task gap reanchors instead of jumping. */
    sample.remote_vx_m_s = NAN;
    assert(RLShadowUpdatePositionTarget(&position_target, &sample, 308u,
                                        &target_s_m) != 0);
    assert(!position_target.active);
    puts("rl position control tests passed");
    return 0;
}
