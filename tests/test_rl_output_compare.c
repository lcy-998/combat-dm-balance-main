#include "rl_shadow.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.00001f);
}

int main(void)
{
    static const float speeds[] = {
        0.0f, 1.0f, 1.2f, 1.35f, 1.5f, 1.6f, 1.75f, 2.0f,
        2.2f, 2.4f, 2.5f, 2.6f, 2.7f, 2.8f, 2.9f, 3.0f, 3.1f, 3.2f
    };
    static const float scales[] = {
        0.180f, 0.220f, 0.240f, 0.240f, 0.240f, 0.242f,
        0.245f, 0.250f, 0.255f, 0.262f, 0.265f, 0.268f,
        0.270f, 0.273f, 0.276f, 0.280f, 0.283f, 0.286f
    };
    float action[RL_SHADOW_ACTION_SIZE] = {0};
    RLShadowOutputPreview preview;
    for (unsigned i = 0; i < sizeof(speeds) / sizeof(speeds[0]); ++i) {
        assert(RLShadowConvertOutput(speeds[i], action, &preview) == 0);
        near(preview.leg_scale_rad_per_action, scales[i]);
        assert(RLShadowConvertOutput(-speeds[i], action, &preview) == 0);
        near(preview.leg_scale_rad_per_action, scales[i]);
        if (i > 0) {
            const float midpoint = (speeds[i - 1] + speeds[i]) / 2.0f;
            assert(RLShadowConvertOutput(midpoint, action, &preview) == 0);
            near(preview.leg_scale_rad_per_action, (scales[i - 1] + scales[i]) / 2.0f);
        }
    }

    action[1] = 0.5f;
    action[4] = 0.5f;
    action[2] = 0.6f;
    action[5] = -0.5f;
    assert(RLShadowConvertOutput(1.0f, action, &preview) == 0);
    near(preview.hip_model_target_rad[0], 0.11f);
    near(preview.hip_model_target_rad[1], 0.11f);
    near(preview.hip_physical_target_rad[0], RL_SHADOW_HIP_ZERO_RAD + 0.11f);
    near(preview.hip_physical_target_rad[1], RL_SHADOW_HIP_ZERO_RAD - 0.11f);
    near(preview.wheel_forward_torque_nm[0], 3.3f);
    near(preview.wheel_forward_torque_nm[1], -2.75f);

    action[1] = 2.0f;
    action[4] = -2.0f;
    action[2] = 2.0f;
    action[5] = -2.0f;
    assert(RLShadowConvertOutput(10.0f, action, &preview) == 0);
    near(preview.leg_scale_rad_per_action, 0.286f);
    near(preview.hip_model_target_rad[0], 0.286f);
    near(preview.hip_model_target_rad[1], -0.286f);
    near(preview.wheel_forward_torque_nm[0], 5.5f);
    near(preview.wheel_forward_torque_nm[1], -5.5f);
    action[1] = NAN;
    assert(RLShadowConvertOutput(1.0f, action, &preview) != 0);
    action[1] = 0.0f;
    assert(RLShadowConvertOutput(NAN, action, &preview) != 0);
    puts("rl output comparison tests passed");
    return 0;
}
