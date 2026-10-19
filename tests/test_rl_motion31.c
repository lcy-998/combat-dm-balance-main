#include "rl_motion31.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void near(float actual, float expected)
{
    if (fabsf(actual - expected) >= 0.00001f) {
        fprintf(stderr, "expected %.8f, got %.8f\n", expected, actual);
    }
    assert(fabsf(actual - expected) < 0.00001f);
}

int main(void)
{
    RLMotion31Snapshot s = {0};
    const float pi = 3.14159265358979323846f;
    const float degrees_to_radians = pi / 180.0f;
    float axis_observation[RL_MOTION31_OBSERVATION_SIZE];
    s.command_height_m = 0.34f;
    /* LQR theta_b is nose-down positive (+FLU Y). */
    RLMotion31MapLqrAttitude(&s, 0.0f, 0.1f, 0.0f, 1.0f, 0.0f);
    assert(RLMotion31BuildObservation(&s, axis_observation) == 0);
    near(axis_observation[0], -0.25f);
    near(axis_observation[1], 0.0f);
    near(axis_observation[2], 0.0f);
    near(axis_observation[3], 0.0f);
    near(axis_observation[4], -cosf(0.1f));
    near(axis_observation[5], -sinf(0.1f));
    RLMotion31MapLqrAttitude(&s, 0.1f, 0.0f, 1.0f, 0.0f, 0.0f);
    assert(RLMotion31BuildObservation(&s, axis_observation) == 0);
    near(axis_observation[0], 0.0f);
    near(axis_observation[1], 0.0f);
    near(axis_observation[2], -0.25f);
    near(axis_observation[3], sinf(0.1f));
    near(axis_observation[4], -cosf(0.1f));
    near(axis_observation[5], 0.0f);
    RLMotion31MapLqrAttitude(&s, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    assert(RLMotion31BuildObservation(&s, axis_observation) == 0);
    near(axis_observation[0], 0.0f);
    near(axis_observation[1], 0.25f);
    near(axis_observation[2], 0.0f);
    /* FLU X is forward, Y is left, Z is up. Each positive body-axis rate
       must appear in the correct model-axis observation channel. */
    const float down_flu[3] = {0.0f, 0.0f, -1.0f};
    const float roll_flu[3] = {1.0f, 0.0f, 0.0f};
    const float pitch_flu[3] = {0.0f, 1.0f, 0.0f};
    const float yaw_flu[3] = {0.0f, 0.0f, 1.0f};
    RLMotion31MapBodyImu(&s, roll_flu, down_flu);
    assert(RLMotion31BuildObservation(&s, axis_observation) == 0);
    near(axis_observation[0], 0.0f);
    near(axis_observation[1], 0.0f);
    near(axis_observation[2], -0.25f);
    near(axis_observation[4], -1.0f);
    RLMotion31MapBodyImu(&s, pitch_flu, down_flu);
    assert(RLMotion31BuildObservation(&s, axis_observation) == 0);
    near(axis_observation[0], -0.25f);
    near(axis_observation[1], 0.0f);
    near(axis_observation[2], 0.0f);
    RLMotion31MapBodyImu(&s, yaw_flu, down_flu);
    assert(RLMotion31BuildObservation(&s, axis_observation) == 0);
    near(axis_observation[0], 0.0f);
    near(axis_observation[1], 0.25f);
    near(axis_observation[2], 0.0f);
    /* Nose down and left side up produce positive FLU pitch/roll. */
    const float nose_down_gravity_flu[3] = {1.0f, 0.0f, 0.0f};
    RLMotion31MapBodyImu(&s, yaw_flu, nose_down_gravity_flu);
    assert(RLMotion31BuildObservation(&s, axis_observation) == 0);
    near(axis_observation[3], 0.0f);
    near(axis_observation[4], 0.0f);
    near(axis_observation[5], -1.0f);
    const float left_up_gravity_flu[3] = {0.0f, -1.0f, 0.0f};
    RLMotion31MapBodyImu(&s, yaw_flu, left_up_gravity_flu);
    assert(RLMotion31BuildObservation(&s, axis_observation) == 0);
    near(axis_observation[3], 1.0f);
    near(axis_observation[4], 0.0f);
    near(axis_observation[5], 0.0f);

    const float hip_zero = (90.0f - 47.1033888f) * degrees_to_radians;
    const float active_zero = (90.0f + 47.1080837f) * degrees_to_radians;
    RLMotion31MapJointAngles(&s, hip_zero, active_zero,
                             hip_zero, active_zero,
                             1.0f, 2.0f, 3.0f, 4.0f);
    near(s.joint_position_rad[0], 0.0f);
    near(s.joint_position_rad[1], 0.0f);
    near(s.joint_position_rad[2], 0.0f);
    near(s.joint_position_rad[3], 0.0f);
    near(s.joint_velocity_rad_s[0], -1.0f);
    near(s.joint_velocity_rad_s[1], 2.0f);
    near(s.joint_velocity_rad_s[3], 3.0f);
    near(s.joint_velocity_rad_s[4], 4.0f);
    /* All angle/rate measurements are CCW from backward horizontal, viewed
       from physical right toward left. Model left is physical right. */
    const float ccw = 0.1f;
    RLMotion31MapJointAngles(&s,
                             hip_zero + ccw, active_zero + ccw,
                             hip_zero + ccw, active_zero + ccw,
                             1.0f, 1.0f, 1.0f, 1.0f);
    near(s.joint_position_rad[0], -ccw);
    near(s.joint_position_rad[1], ccw);
    near(s.joint_position_rad[2], ccw);
    near(s.joint_position_rad[3], ccw);
    near(s.joint_velocity_rad_s[0], -1.0f);
    near(s.joint_velocity_rad_s[1], 1.0f);
    near(s.joint_velocity_rad_s[3], 1.0f);
    near(s.joint_velocity_rad_s[4], 1.0f);
    assert(RLMotion31BuildObservation(&s, axis_observation) == 0);
    near(axis_observation[6], -ccw / 1.5f);
    near(axis_observation[7], ccw / 1.5f);
    near(axis_observation[8], ccw / 1.5f);
    near(axis_observation[9], ccw / 1.5f);
    near(axis_observation[10], -0.05f);
    near(axis_observation[11], 0.05f);
    near(axis_observation[13], 0.05f);
    near(axis_observation[14], 0.05f);
    const float expected_q[4][4] = {
        {-ccw, 0.0f, 0.0f, 0.0f},
        {0.0f, ccw, 0.0f, 0.0f},
        {0.0f, 0.0f, ccw, 0.0f},
        {0.0f, 0.0f, 0.0f, ccw}};
    const float expected_dq[4][4] = {
        {-1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}};
    const size_t joint_velocity_index[4] = {0u, 1u, 3u, 4u};
    for (size_t moved = 0; moved < 4u; ++moved) {
        float joint_angle[4] = {hip_zero, active_zero, hip_zero, active_zero};
        float joint_rate[4] = {0};
        joint_angle[moved] += ccw;
        joint_rate[moved] = 1.0f;
        RLMotion31MapJointAngles(&s, joint_angle[0], joint_angle[1],
                                 joint_angle[2], joint_angle[3],
                                 joint_rate[0], joint_rate[1],
                                 joint_rate[2], joint_rate[3]);
        for (size_t i = 0; i < 4u; ++i) {
            near(s.joint_position_rad[i], expected_q[moved][i]);
            near(s.joint_velocity_rad_s[joint_velocity_index[i]], expected_dq[moved][i]);
        }
    }
    /* Both motor axes have opposite signs for forward travel. The model
       wheel axes are backward-positive and are ordered physical right/left. */
    RLMotion31MapWheelMotorSpeeds(&s, -2.0f, 3.0f);
    near(s.joint_velocity_rad_s[2], -3.0f);
    near(s.joint_velocity_rad_s[5], -2.0f);
    assert(RLMotion31BuildObservation(&s, axis_observation) == 0);
    near(axis_observation[12], -0.15f);
    near(axis_observation[15], -0.10f);
    s.gravity_model[1] = -1.0f;
    s.gyro_model_rad_s[0] = 4.0f;
    s.gyro_model_rad_s[1] = -8.0f;
    s.gyro_model_rad_s[2] = 12.0f;
    s.command_height_m = 0.34f;
    s.joint_position_rad[0] = 0.75f;
    s.joint_position_rad[1] = -0.75f;
    s.joint_position_rad[2] = 1.5f;
    s.joint_position_rad[3] = -1.5f;
    s.joint_velocity_rad_s[0] = 2.0f;
    s.joint_velocity_rad_s[5] = -4.0f;
    s.previous_target[0] = 0.3f;
    s.previous_target[2] = 50.0f;
    s.previous_target[3] = -0.6f;
    s.previous_target[5] = -25.0f;
    s.command_vx_m_s = 1.0f;
    s.command_yaw_rad_s = 0.5f;

    float observation[RL_MOTION31_OBSERVATION_SIZE];
    assert(RLMotion31BuildObservation(&s, observation) == 0);
    near(observation[0], 1.0f);
    near(observation[1], -2.0f);
    near(observation[2], 3.0f);
    near(observation[4], -1.0f);
    near(observation[6], 0.5f);
    near(observation[7], -0.5f);
    near(observation[8], 1.0f);
    near(observation[9], -1.0f);
    near(observation[10], 0.1f);
    near(observation[15], -0.2f);
    near(observation[16], 0.2f);
    near(observation[17], 0.0f);
    near(observation[18], -0.4f);
    near(observation[19], 0.0f);
    near(observation[20], 0.5f);
    near(observation[21], -0.25f);
    near(observation[22], 0.25f);
    near(observation[23], 0.125f);
    near(observation[24], 0.68f);
    near(observation[25], 1.0f);
    near(observation[26], 0.0f);
    near(observation[27], 0.0f);
    near(observation[28], 1.0f);
    near(observation[29], 0.0f);
    near(observation[30], 0.0f);

    const float action[RL_MOTION31_ACTION_SIZE] = {-1.0f, -0.5f, 0.25f, 0.5f, 1.0f, -0.25f};
    float target[RL_MOTION31_ACTION_SIZE];
    assert(RLMotion31DecodeOutput(action, target) == 0);
    near(target[0], -1.5f);
    near(target[1], -0.75f);
    near(target[2], 25.0f);
    near(target[3], 0.75f);
    near(target[4], 1.5f);
    near(target[5], -25.0f);

    const float out_of_range[RL_MOTION31_ACTION_SIZE] = {2.0f, -2.0f, 2.0f, 0.0f, 0.0f, 0.0f};
    assert(RLMotion31DecodeOutput(out_of_range, target) == 0);
    near(target[0], 1.5f);
    near(target[1], -1.5f);
    near(target[2], 100.0f);

    float hardware_joint_target[4];
    float motor_wheel_target[2];
    const float targets_for_mapping[RL_MOTION31_ACTION_SIZE] = {0.1f, 0.2f, 3.0f, -0.3f, -0.4f, -5.0f};
    assert(RLMotion31MapPolicyTargets(targets_for_mapping, hardware_joint_target,
                                      motor_wheel_target) == 0);
    near(hardware_joint_target[0], active_zero - 0.4f);
    near(hardware_joint_target[1], hip_zero - 0.3f);
    near(hardware_joint_target[2], active_zero + 0.2f);
    near(hardware_joint_target[3], hip_zero - 0.1f);
    near(motor_wheel_target[0], -5.0f);
    near(motor_wheel_target[1], -3.0f);
    RLMotion31MapJointAngles(&s, hardware_joint_target[3], hardware_joint_target[2],
                             hardware_joint_target[1], hardware_joint_target[0],
                             0.0f, 0.0f, 0.0f, 0.0f);
    near(s.joint_position_rad[0], targets_for_mapping[0]);
    near(s.joint_position_rad[1], targets_for_mapping[1]);
    near(s.joint_position_rad[2], targets_for_mapping[3]);
    near(s.joint_position_rad[3], targets_for_mapping[4]);
    const float ccw_policy_targets[RL_MOTION31_ACTION_SIZE] =
        {-ccw, ccw, 0.0f, ccw, ccw, 0.0f};
    assert(RLMotion31MapPolicyTargets(ccw_policy_targets, hardware_joint_target,
                                      motor_wheel_target) == 0);
    near(hardware_joint_target[0], active_zero + ccw);
    near(hardware_joint_target[1], hip_zero + ccw);
    near(hardware_joint_target[2], active_zero + ccw);
    near(hardware_joint_target[3], hip_zero + ccw);

    const float policy_torque[RL_MOTION31_ACTION_SIZE] = {1, 2, 3, 4, 5, 6};
    float motor_joint_torque[4];
    assert(RLMotion31MapPolicyLegTorqueToMotorOrder(policy_torque,
                                                    motor_joint_torque) == 0);
    near(motor_joint_torque[0], -5.0f);
    near(motor_joint_torque[1], -4.0f);
    near(motor_joint_torque[2], 2.0f);
    near(motor_joint_torque[3], -1.0f);
    float forward_wheel_torque[2];
    assert(RLMotion31MapPolicyWheelTorqueToForwardOrder(policy_torque,
                                                        forward_wheel_torque) == 0);
    near(forward_wheel_torque[0], -6.0f);
    near(forward_wheel_torque[1], -3.0f);

    s.joint_position_rad[0] = 0.1f;
    s.joint_position_rad[1] = -0.2f;
    s.joint_position_rad[2] = 0.3f;
    s.joint_position_rad[3] = 0.4f;
    s.joint_velocity_rad_s[0] = 0.01f;
    s.joint_velocity_rad_s[1] = -0.02f;
    s.joint_velocity_rad_s[2] = 10.0f;
    s.joint_velocity_rad_s[3] = 0.03f;
    s.joint_velocity_rad_s[4] = 0.04f;
    s.joint_velocity_rad_s[5] = -20.0f;
    const float zero_target[RL_MOTION31_ACTION_SIZE] = {0};
    float requested[RL_MOTION31_ACTION_SIZE];
    float limited[RL_MOTION31_ACTION_SIZE];
    assert(RLMotion31ComputePdTorque(&s, zero_target, requested, limited) == 0);
    near(requested[0], -6.02f);
    near(requested[1], 12.04f);
    near(requested[2], -10.0f);
    near(requested[3], -18.06f);
    near(requested[4], -24.08f);
    near(requested[5], 20.0f);
    near(limited[2], -5.5f);
    near(limited[5], 5.5f);

    s.joint_position_rad[0] = 3.11f;
    assert(RLMotion31BuildObservation(&s, observation) == 0);
    near(observation[6], 3.11f / 1.5f);
    s.joint_position_rad[0] = 0.1f;
    s.joint_position_rad[1] = 2.61f;
    assert(RLMotion31BuildObservation(&s, observation) == 0);
    near(observation[7], 2.61f / 1.5f);
    s.joint_position_rad[1] = -0.2f;

    s.joint_velocity_rad_s[2] = NAN;
    assert(RLMotion31BuildObservation(&s, observation) != 0);
    assert(RLMotion31DecodeOutput(NULL, target) != 0);
    const float bad_action[RL_MOTION31_ACTION_SIZE] = {0, 0, 0, 0, INFINITY, 0};
    assert(RLMotion31DecodeOutput(bad_action, target) != 0);
    puts("Motion31 observation and decode tests passed");
    return 0;
}
