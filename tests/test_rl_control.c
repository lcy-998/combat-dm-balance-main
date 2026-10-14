#include "rl_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

int main(void)
{
    RLControlState state = {0};
    RLShadowCommand command = {0};
    command.valid = 1u;
    command.snapshot_sequence = 1u;
    command.raw_action[0] = 0.5f;
    command.output.joint_target_angle_rad[0] = 2.4f;
    RLControlDecision d = RLControlStep(&state, 100u, 1, &command);
    assert(!d.active && !d.beep);
    d = RLControlStep(&state, 10099u, 1, &command);
    assert(!d.active);
    command.snapshot_tick_ms = 10100u;
    command.output_tick_ms = 10100u;
    d = RLControlStep(&state, 10100u, 1, &command);
    assert(d.active && d.beep);
    near(d.blend, 1.0f);
    near(state.previous_action[0], 0.5f);
    float switch_position, switch_torque, switch_kp, switch_kd;
    RLControlBlendJoint(0.0f, 4.0f, 1.0f, d.blend,
                        &switch_position, &switch_torque, &switch_kp, &switch_kd);
    near(switch_position, 1.0f);
    near(switch_torque, 0.0f);
    near(switch_kp, 60.0f);
    near(switch_kd, 2.0f);
    near(RLControlBlendCurrent(100.0f, -100.0f, d.blend), -100.0f);
    command.snapshot_tick_ms = 10200u;
    command.output_tick_ms = 10200u;
    d = RLControlStep(&state, 10200u, 1, &command);
    near(d.blend, 1.0f);
    near(state.previous_action[0], 0.5f);
    command.snapshot_tick_ms = 10300u;
    command.output_tick_ms = 10300u;
    d = RLControlStep(&state, 10300u, 1, &command);
    near(d.blend, 1.0f);

    d = RLControlStep(&state, 10325u, 1, &command);
    assert(!d.active && d.exit_beep);
    near(state.previous_action[0], 0.0f);
    d = RLControlStep(&state, 10326u, 1, &command);
    assert(!d.exit_beep);
    command.snapshot_tick_ms = 10330u;
    command.output_tick_ms = 10330u;
    d = RLControlStep(&state, 10330u, 1, &command);
    assert(d.active && !d.beep);
    near(d.blend, 1.0f);

    command.raw_action[1] = NAN;
    d = RLControlStep(&state, 10331u, 1, &command);
    assert(!d.active && d.exit_beep);
    command.raw_action[1] = 0.0f;

    d = RLControlStep(&state, 10335u, 0, &command);
    assert(!d.active && !d.exit_beep && !state.timing && !state.beeped);
    d = RLControlStep(&state, 10340u, 1, &command);
    assert(!d.active);
    command.snapshot_tick_ms = 20339u;
    command.output_tick_ms = 20339u;
    d = RLControlStep(&state, 20339u, 1, &command);
    assert(!d.active);
    command.snapshot_tick_ms = 20340u;
    command.output_tick_ms = 20340u;
    d = RLControlStep(&state, 20340u, 1, &command);
    assert(d.active && d.beep);
    d = RLControlStep(&state, 20341u, 0, &command);
    assert(!d.active && d.exit_beep);
    d = RLControlStep(&state, 20342u, 0, &command);
    assert(!d.exit_beep);

    RLControlLegRamp ramp = {0};
    near(RLControlLegRampStep(&ramp, 100u, 1, 0.16f), 0.16f);
    near(RLControlLegRampStep(&ramp, 2600u, 1, 0.16f), 0.255f);
    near(RLControlLegRampStep(&ramp, 5100u, 1, 0.16f), 0.35f);
    assert(ramp.completed);
    near(RLControlLegRampStep(&ramp, 5200u, 1, 0.34f), 0.34f);
    near(RLControlLegRampStep(&ramp, 5300u, 0, 0.34f), 0.34f);
    assert(!ramp.started && !ramp.completed);
    near(RLControlLegRampStep(&ramp, 5400u, 1, 0.18f), 0.18f);
    near(RLControlLegRampStep(&ramp, 7900u, 1, 0.18f), 0.265f);

    const float encoder_now[4] = {0.0f, -3.0f, 0.0f, 0.0f};
    const float encoder_offset[4] = {1.6f, -2.6f, 1.1f, 1.3f};
    RLShadowOutputPreview output = {0};
    output.joint_target_angle_rad[0] = 2.4f;
    output.joint_target_angle_rad[1] = 0.7f;
    output.joint_target_angle_rad[2] = 2.3f;
    output.joint_target_angle_rad[3] = 0.8f;
    output.wheel_forward_torque_nm[0] = 1.0f;
    output.wheel_forward_torque_nm[1] = 1.0f;
    float target[4], current[2];
    assert(RLControlMapOutput(&output, encoder_now, encoder_offset,
                              target, current) == 0);
    near(target[0], -0.8f);
    near(target[1], -3.3f + 6.283185307f);
    near(target[2], 1.2f);
    near(target[3], -0.5f);
    assert(current[0] < 0.0f && current[1] > 0.0f);
    output.joint_target_angle_rad[2] = NAN;
    assert(RLControlMapOutput(&output, encoder_now, encoder_offset,
                              target, current) != 0);
    output.joint_target_angle_rad[2] = 2.3f;
    output.wheel_forward_torque_nm[0] = 5.5f;
    assert(RLControlMapOutput(&output, encoder_now, encoder_offset,
                              target, current) == 0);
    near(current[0], -8192.0f);

    /* Canonical targets remain valid across encoder boundaries because the
       motor task now computes shortest-angle PD on the MCU. */
    float boundary_now[4] = {6.2f, -6.2f, 6.2f, -6.2f};
    output.joint_target_angle_rad[0] = encoder_offset[0] - 0.1f;
    assert(RLControlMapOutput(&output, boundary_now, encoder_offset,
                              target, current) == 0);
    output.joint_target_angle_rad[0] = encoder_offset[0] + 0.1f;
    output.joint_target_angle_rad[1] = encoder_offset[1] + 0.1f;
    assert(RLControlMapOutput(&output, boundary_now, encoder_offset,
                              target, current) == 0);

    /* Both sides of the encoder range give small periodic position errors. */
    output.joint_target_angle_rad[1] = encoder_offset[1] - 0.1f;
    output.joint_target_angle_rad[2] = encoder_offset[2] - 0.1f;
    output.joint_target_angle_rad[3] = encoder_offset[3] + 0.1f;
    assert(RLControlMapOutput(&output, boundary_now, encoder_offset,
                              target, current) == 0);
    for (unsigned i = 0; i < 4u; ++i) {
        assert(fabsf(remainderf(target[i] - boundary_now[i], 6.283185307f)) < 0.1f);
        assert(fabsf(target[i]) <= 6.283f);
    }

    float position, torque, kp, kd;
    RLControlBlendJoint(0.0f, 4.0f, 1.0f, 0.5f,
                        &position, &torque, &kp, &kd);
    near(position, 0.5f);
    near(torque, 2.0f);
    near(kp, 30.0f);
    near(kd, 1.0f);
    near(RLControlBlendCurrent(100.0f, -100.0f, 0.5f), 0.0f);
    puts("rl control tests passed");
    return 0;
}
