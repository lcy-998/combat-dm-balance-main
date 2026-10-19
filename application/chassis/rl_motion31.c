#include "rl_motion31.h"

#include <math.h>
#include <stddef.h>

#define RL_MOTION31_PI 3.14159265358979323846f
#define RL_MOTION31_TWO_PI (2.0f * RL_MOTION31_PI)
/* Hardware angles start at backward horizontal. The model q=0 links are
   47.1033888 deg backward / 47.1080837 deg forward from downward vertical. */
#define RL_MOTION31_HIP_ZERO_RAD ((90.0f - 47.1033888f) * (RL_MOTION31_PI / 180.0f))
#define RL_MOTION31_ACTIVE_ZERO_RAD ((90.0f + 47.1080837f) * (RL_MOTION31_PI / 180.0f))

static int finite_array(const float *values, size_t count)
{
    for (size_t i = 0; i < count; ++i) {
        if (!isfinite(values[i])) {
            return 0;
        }
    }
    return 1;
}

static float wrap_joint_angle(float angle_rad)
{
    return remainderf(angle_rad, RL_MOTION31_TWO_PI);
}

void RLMotion31MapBodyImu(RLMotion31Snapshot *s,
                          const float gyro_flu_rad_s[3],
                          const float gravity_flu[3])
{
    if (s == NULL || gyro_flu_rad_s == NULL || gravity_flu == NULL) return;
    s->gyro_model_rad_s[0] = -gyro_flu_rad_s[1];
    s->gyro_model_rad_s[1] = gyro_flu_rad_s[2];
    s->gyro_model_rad_s[2] = -gyro_flu_rad_s[0];
    s->gravity_model[0] = -gravity_flu[1];
    s->gravity_model[1] = gravity_flu[2];
    s->gravity_model[2] = -gravity_flu[0];
}

void RLMotion31MapLqrAttitude(RLMotion31Snapshot *s,
                              float roll_rad, float pitch_rad,
                              float roll_rate_rad_s, float pitch_rate_rad_s,
                              float yaw_rate_rad_s)
{
    /* LQR theta_b=pitch is nose-down positive, and phi=yaw is left-turn
       positive. These are +FLU Y and +FLU Z rotations, respectively. */
    const float gyro_flu[3] = {roll_rate_rad_s, pitch_rate_rad_s, yaw_rate_rad_s};
    const float gravity_flu[3] = {
        sinf(pitch_rad),
        -sinf(roll_rad) * cosf(pitch_rad),
        -cosf(roll_rad) * cosf(pitch_rad)};
    RLMotion31MapBodyImu(s, gyro_flu, gravity_flu);
}

void RLMotion31MapJointAngles(RLMotion31Snapshot *s,
                              float right_hip_from_back_rad, float right_active_from_back_rad,
                              float left_hip_from_back_rad, float left_active_from_back_rad,
                              float right_hip_rate_rad_s, float right_active_rate_rad_s,
                              float left_hip_rate_rad_s, float left_active_rate_rad_s)
{
    if (s == NULL) return;
    /* Physical L/R are viewed from behind. The model's left channels 0/1
       are on the physical right; its right channels 3/4 are on the left.
       All measured angles start at backward horizontal and increase CCW
       when viewed from the physical right toward the left. */
    /* Hip axes are periodic. Active links keep their calibrated mechanism branch. */
    s->joint_position_rad[0] = wrap_joint_angle(
        RL_MOTION31_HIP_ZERO_RAD - right_hip_from_back_rad);
    s->joint_position_rad[1] =
        right_active_from_back_rad - RL_MOTION31_ACTIVE_ZERO_RAD;
    s->joint_position_rad[2] = wrap_joint_angle(
        left_hip_from_back_rad - RL_MOTION31_HIP_ZERO_RAD);
    s->joint_position_rad[3] =
        left_active_from_back_rad - RL_MOTION31_ACTIVE_ZERO_RAD;
    s->joint_velocity_rad_s[0] = -right_hip_rate_rad_s;
    s->joint_velocity_rad_s[1] = right_active_rate_rad_s;
    s->joint_velocity_rad_s[3] = left_hip_rate_rad_s;
    s->joint_velocity_rad_s[4] = left_active_rate_rad_s;
}

void RLMotion31MapWheelMotorSpeeds(RLMotion31Snapshot *s,
                                   float left_motor_wheel_rad_s,
                                   float right_motor_wheel_rad_s)
{
    if (s == NULL) return;
    s->joint_velocity_rad_s[2] = -right_motor_wheel_rad_s;
    s->joint_velocity_rad_s[5] = left_motor_wheel_rad_s;
}

int RLMotion31BuildObservation(const RLMotion31Snapshot *s,
                               float o[RL_MOTION31_OBSERVATION_SIZE])
{
    if (s == NULL || o == NULL ||
        !finite_array(s->gyro_model_rad_s, 3u) ||
        !finite_array(s->gravity_model, 3u) ||
        !finite_array(s->joint_position_rad, 4u) ||
        !finite_array(s->joint_velocity_rad_s, 6u) ||
        !finite_array(s->previous_target, 6u) ||
        !isfinite(s->command_vx_m_s) || !isfinite(s->command_yaw_rad_s) ||
        !isfinite(s->command_height_m)) {
        return -1;
    }
    o[0] = 0.25f * s->gyro_model_rad_s[0];
    o[1] = 0.25f * s->gyro_model_rad_s[1];
    o[2] = 0.25f * s->gyro_model_rad_s[2];
    o[3] = s->gravity_model[0];
    o[4] = s->gravity_model[1];
    o[5] = s->gravity_model[2];
    o[6] = s->joint_position_rad[0] / 1.5f;
    o[7] = s->joint_position_rad[1] / 1.5f;
    o[8] = s->joint_position_rad[2] / 1.5f;
    o[9] = s->joint_position_rad[3] / 1.5f;
    for (size_t i = 0; i < 6u; ++i) {
        o[10u + i] = 0.05f * s->joint_velocity_rad_s[i];
    }
    o[16] = s->previous_target[0] / 1.5f;
    o[17] = s->previous_target[1] / 1.5f;
    o[18] = s->previous_target[3] / 1.5f;
    o[19] = s->previous_target[4] / 1.5f;
    o[20] = s->previous_target[2] / 100.0f;
    o[21] = s->previous_target[5] / 100.0f;
    o[22] = s->command_vx_m_s / 4.0f;
    o[23] = s->command_yaw_rad_s / 4.0f;
    o[24] = s->command_height_m / 0.5f;
    o[25] = 1.0f; /* Motion */
    o[26] = 0.0f; /* Recovery */
    o[27] = 0.0f; /* Jump */
    o[28] = 1.0f; /* Jump NONE */
    o[29] = 0.0f; /* Jump SMALL */
    o[30] = 0.0f; /* Jump BIG */
    return finite_array(o, RL_MOTION31_OBSERVATION_SIZE) ? 0 : -1;
}

int RLMotion31DecodeOutput(const float action[RL_MOTION31_ACTION_SIZE],
                           float target[RL_MOTION31_ACTION_SIZE])
{
    if (action == NULL || target == NULL ||
        !finite_array(action, RL_MOTION31_ACTION_SIZE)) {
        return -1;
    }
    for (size_t i = 0; i < RL_MOTION31_ACTION_SIZE; ++i) {
        float clipped = fminf(fmaxf(action[i], -1.0f), 1.0f);
        target[i] = (i == 2u || i == 5u) ? 100.0f * clipped : 1.5f * clipped;
    }
    return finite_array(target, RL_MOTION31_ACTION_SIZE) ? 0 : -1;
}

int RLMotion31MapPolicyTargets(const float target[RL_MOTION31_ACTION_SIZE],
                               float hardware_joint_target_rad[4],
                               float motor_wheel_target_rad_s[2])
{
    if (target == NULL || hardware_joint_target_rad == NULL || motor_wheel_target_rad_s == NULL ||
        !finite_array(target, RL_MOTION31_ACTION_SIZE)) {
        return -1;
    }
    hardware_joint_target_rad[0] = RL_MOTION31_ACTIVE_ZERO_RAD + target[4]; /* lfangle */
    hardware_joint_target_rad[1] = RL_MOTION31_HIP_ZERO_RAD + target[3];    /* lbangle */
    hardware_joint_target_rad[2] = RL_MOTION31_ACTIVE_ZERO_RAD + target[1]; /* rfangle */
    hardware_joint_target_rad[3] = RL_MOTION31_HIP_ZERO_RAD - target[0];    /* rbangle */
    motor_wheel_target_rad_s[0] = target[5];  /* left motor: positive is backward */
    motor_wheel_target_rad_s[1] = -target[2]; /* right motor: positive is forward */
    return finite_array(hardware_joint_target_rad, 4u) &&
           finite_array(motor_wheel_target_rad_s, 2u) ? 0 : -1;
}

int RLMotion31ComputePdTorque(const RLMotion31Snapshot *s,
                              const float target[RL_MOTION31_ACTION_SIZE],
                              float requested[RL_MOTION31_ACTION_SIZE],
                              float limited[RL_MOTION31_ACTION_SIZE])
{
    if (s == NULL || target == NULL || requested == NULL || limited == NULL ||
        !finite_array(s->joint_position_rad, 4u) ||
        !finite_array(s->joint_velocity_rad_s, RL_MOTION31_ACTION_SIZE) ||
        !finite_array(target, RL_MOTION31_ACTION_SIZE)) {
        return -1;
    }
    for (size_t i = 0; i < RL_MOTION31_ACTION_SIZE; ++i) {
        if (i == 2u || i == 5u) {
            requested[i] = target[i] - s->joint_velocity_rad_s[i];
        } else {
            const size_t joint_index = (i < 2u) ? i : i - 1u;
            requested[i] = 60.0f * (target[i] - s->joint_position_rad[joint_index])
                           - 2.0f * s->joint_velocity_rad_s[i];
        }
        if (!isfinite(requested[i])) return -1;
        const float limit = (i == 2u || i == 5u) ? 5.5f : 40.0f;
        limited[i] = fminf(fmaxf(requested[i], -limit), limit);
    }
    return finite_array(limited, RL_MOTION31_ACTION_SIZE) ? 0 : -1;
}

int RLMotion31MapPolicyLegTorqueToMotorOrder(const float policy_torque_nm[RL_MOTION31_ACTION_SIZE],
                                            float leg_torque_motor_nm[4])
{
    if (policy_torque_nm == NULL || leg_torque_motor_nm == NULL ||
        !finite_array(policy_torque_nm, RL_MOTION31_ACTION_SIZE)) {
        return -1;
    }
    /* Existing DMMotor order/sign: lf, lb, rf, rb. */
    leg_torque_motor_nm[0] = -policy_torque_nm[4];
    leg_torque_motor_nm[1] = -policy_torque_nm[3];
    leg_torque_motor_nm[2] = policy_torque_nm[1];
    leg_torque_motor_nm[3] = -policy_torque_nm[0];
    return finite_array(leg_torque_motor_nm, 4u) ? 0 : -1;
}

int RLMotion31MapPolicyWheelTorqueToForwardOrder(const float policy_torque_nm[RL_MOTION31_ACTION_SIZE],
                                                 float wheel_torque_forward_nm[2])
{
    if (policy_torque_nm == NULL || wheel_torque_forward_nm == NULL ||
        !finite_array(policy_torque_nm, RL_MOTION31_ACTION_SIZE)) {
        return -1;
    }
    wheel_torque_forward_nm[0] = -policy_torque_nm[5]; /* physical left */
    wheel_torque_forward_nm[1] = -policy_torque_nm[2]; /* physical right */
    return finite_array(wheel_torque_forward_nm, 2u) ? 0 : -1;
}

int RLMotion31BuildPreview(const RLMotion31Snapshot *sample,
                           const float action[RL_MOTION31_ACTION_SIZE],
                           RLMotion31Preview *preview)
{
    if (preview == NULL ||
        RLMotion31BuildObservation(sample, preview->observation) != 0 ||
        RLMotion31DecodeOutput(action, preview->target) != 0 ||
        RLMotion31MapPolicyTargets(preview->target, preview->hardware_joint_target_rad,
                                   preview->motor_wheel_target_rad_s) != 0) {
        return -1;
    }
    return 0;
}
