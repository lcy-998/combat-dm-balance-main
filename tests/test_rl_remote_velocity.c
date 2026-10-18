#include "rl_shadow.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

int main(void)
{
    float target_vx = 0.0f;
    const float speeds[] = {0.0f, 0.8f, -0.75f, 10.0f, -10.0f};
    const float expected[] = {0.0f, 0.8f, -0.75f, 2.2f, -2.2f};
    for (unsigned i = 0; i < sizeof(speeds) / sizeof(speeds[0]); ++i) {
        assert(RLShadowRemoteVelocity(speeds[i], &target_vx) == 0);
        near(target_vx, expected[i]);
    }
    assert(RLShadowRemoteVelocity(NAN, &target_vx) != 0);
    assert(RLShadowRemoteVelocity(INFINITY, &target_vx) != 0);
    assert(RLShadowRemoteVelocity(-INFINITY, &target_vx) != 0);
    assert(RLShadowRemoteVelocity(0.0f, NULL) != 0);

    RLShadowSnapshot sample = {0};
    sample.target_s_m = 1.2f;
    sample.remote_vx_m_s = -0.8f;
    sample.heading_vx_m_s = -0.4f;
    assert(RLShadowRemoteVelocity(sample.remote_vx_m_s,
                                  &sample.target_vx_m_s) == 0);
    float observation[RL_SHADOW_OBSERVATION_SIZE];
    assert(RLShadowBuildObservation(&sample, observation) == 0);
    near(observation[6], -0.84f);
    near(observation[7], -0.4f);

    /* A large position error must not change any policy input. */
    float original[RL_SHADOW_OBSERVATION_SIZE];
    memcpy(original, observation, sizeof(original));
    sample.odom_s_m = -100.0f;
    sample.target_s_m = 100.0f;
    assert(RLShadowRemoteVelocity(sample.remote_vx_m_s,
                                  &sample.target_vx_m_s) == 0);
    assert(RLShadowBuildObservation(&sample, observation) == 0);
    for (unsigned i = 0; i < RL_SHADOW_OBSERVATION_SIZE; ++i)
        near(observation[i], original[i]);

    /* Releasing the stick requests zero speed even away from the LQR target. */
    sample.remote_vx_m_s = 0.0f;
    assert(RLShadowRemoteVelocity(sample.remote_vx_m_s,
                                  &sample.target_vx_m_s) == 0);
    near(sample.target_vx_m_s, 0.0f);
    const float action[RL_SHADOW_ACTION_SIZE] = {0};
    RLShadowOutputPreview converted;
    assert(RLShadowConvertOutput(sample.target_vx_m_s, action, &converted) == 0);
    near(converted.leg_scale_rad_per_action, 0.180f);
    puts("rl remote velocity tests passed");
    return 0;
}
