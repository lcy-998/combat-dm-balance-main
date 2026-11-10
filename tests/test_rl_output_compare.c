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
    static const float speeds[] = {0.000000f, 0.050000f, 0.100000f, 0.150000f, 0.225000f, 0.250000f, 0.275000f, 0.350000f, 0.500000f, 0.750000f, 1.000000f, 1.200000f, 1.250000f, 1.350000f, 1.500000f, 2.000000f, 2.500000f, 2.600000f, 2.700000f, 2.800000f, 2.900000f, 3.000000f, 3.100000f, 3.200000f};
    static const float scales[] = {0.190000f, 0.190000f, 0.190000f, 0.190000f, 0.190000f, 0.190000f, 0.191000f, 0.194000f, 0.200000f, 0.210000f, 0.220000f, 0.240000f, 0.240000f, 0.240000f, 0.240000f, 0.250000f, 0.265000f, 0.268000f, 0.270000f, 0.273000f, 0.276000f, 0.280000f, 0.283000f, 0.286000f};
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

    action[0] = 0.25f;
    action[1] = 0.5f;
    action[3] = -0.75f;
    action[4] = 0.5f;
    action[2] = 0.6f;
    action[5] = -0.5f;
    assert(RLShadowConvertOutput(1.0f, action, &preview) == 0);
    near(preview.joint_target_angle_rad[0], RL_SHADOW_ACTIVE_ZERO_RAD + 0.055f);
    near(preview.joint_target_angle_rad[1], RL_SHADOW_HIP_ZERO_RAD + 0.11f);
    near(preview.joint_target_angle_rad[2], RL_SHADOW_ACTIVE_ZERO_RAD - 0.165f);
    near(preview.joint_target_angle_rad[3], RL_SHADOW_HIP_ZERO_RAD - 0.11f);
    near(preview.wheel_forward_torque_nm[0], 3.3f);
    near(preview.wheel_forward_torque_nm[1], -2.75f);

    action[0] = 2.0f;
    action[1] = 2.0f;
    action[3] = -2.0f;
    action[4] = -2.0f;
    action[2] = 2.0f;
    action[5] = -2.0f;
    assert(RLShadowConvertOutput(10.0f, action, &preview) == 0);
    near(preview.leg_scale_rad_per_action, 0.286f);
    near(preview.joint_target_angle_rad[0], RL_SHADOW_ACTIVE_ZERO_RAD + 0.286f);
    near(preview.joint_target_angle_rad[1], RL_SHADOW_HIP_ZERO_RAD + 0.286f);
    near(preview.joint_target_angle_rad[2], RL_SHADOW_ACTIVE_ZERO_RAD - 0.286f);
    near(preview.joint_target_angle_rad[3], RL_SHADOW_HIP_ZERO_RAD + 0.286f);
    near(preview.wheel_forward_torque_nm[0], 5.5f);
    near(preview.wheel_forward_torque_nm[1], -5.5f);
    action[1] = NAN;
    assert(RLShadowConvertOutput(1.0f, action, &preview) != 0);
    action[1] = 0.0f;
    assert(RLShadowConvertOutput(NAN, action, &preview) != 0);
    puts("rl output comparison tests passed");
    return 0;
}
