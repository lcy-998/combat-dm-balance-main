#ifndef RL_MOTION31_H
#define RL_MOTION31_H

#include <stdint.h>

#define RL_MOTION31_OBSERVATION_SIZE 31u
#define RL_MOTION31_ACTION_SIZE 6u
#define RL_MOTION31_WHEEL_CURRENT_PER_NM (19.0f / 0.00036f / (268.0f / 17.0f))

#define RL_MOTION31_OUTPUT_LEGACY 1u
#define RL_MOTION31_OUTPUT_SUPPORTED015 2u
/* Select together with matching weights. The checked-in network is legacy. */
#ifndef RL_MOTION31_OUTPUT_VERSION
#define RL_MOTION31_OUTPUT_VERSION RL_MOTION31_OUTPUT_LEGACY
#endif
#if RL_MOTION31_OUTPUT_VERSION != RL_MOTION31_OUTPUT_LEGACY && \
    RL_MOTION31_OUTPUT_VERSION != RL_MOTION31_OUTPUT_SUPPORTED015
#error "Unknown Motion31 output contract"
#endif

typedef struct {
    float gyro_model_rad_s[3];
    float gravity_model[3];
    float joint_position_rad[4]; /* model order: physical RH, RA, LH, LA */
    float joint_velocity_rad_s[6]; /* physical RH, RA, RW, LH, LA, LW */
    float previous_target[6]; /* locally adopted target or explicitly marked initialization proxy */
    uint8_t previous_target_valid;
    float lqr_leg_torque_motor_nm[4]; /* existing DMMotor order: lf, lb, rf, rb */
    float lqr_wheel_torque_forward_nm[2]; /* LQR wheel torque, forward-positive: left, right */
    uint8_t lqr_torque_valid;
    float command_vx_m_s;
    float command_yaw_rad_s;
    float command_height_m;
} RLMotion31Snapshot;

typedef struct {
    float observation[RL_MOTION31_OBSERVATION_SIZE];
    float target[RL_MOTION31_ACTION_SIZE]; /* physical RH, RA, RW, LH, LA, LW */
    float hardware_joint_target_rad[4]; /* lfangle, lbangle, rfangle, rbangle conventions */
    float motor_wheel_target_rad_s[2]; /* left/right wheel-output rad/s in hardware motor-axis sign */
} RLMotion31Preview;

/* Vehicle FLU: X forward, Y left, Z up. Model: X=-FLU Y, Y=FLU Z, Z=-FLU X. */
void RLMotion31MapBodyImu(RLMotion31Snapshot *sample,
                          const float gyro_flu_rad_s[3],
                          const float gravity_flu[3]);
/* LQR theta_b=pitch is positive nose-down (+FLU Y); phi=yaw is left-turn (+FLU Z). */
void RLMotion31MapLqrAttitude(RLMotion31Snapshot *sample,
                              float roll_rad, float pitch_rad,
                              float roll_rate_rad_s, float pitch_rate_rad_s,
                              float yaw_rate_rad_s);
void RLMotion31MapJointAngles(RLMotion31Snapshot *sample,
                              float right_hip_from_back_rad, float right_active_from_back_rad,
                              float left_hip_from_back_rad, float left_active_from_back_rad,
                              float right_hip_rate_rad_s, float right_active_rate_rad_s,
                              float left_hip_rate_rad_s, float left_active_rate_rad_s);
void RLMotion31MapWheelMotorSpeeds(RLMotion31Snapshot *sample,
                                   float left_motor_wheel_rad_s,
                                   float right_motor_wheel_rad_s);
int RLMotion31BuildObservation(const RLMotion31Snapshot *sample,
                               float observation[RL_MOTION31_OBSERVATION_SIZE]);
/* Unit gravity, finite fields and calibrated mechanical input branch. No input clipping. */
int RLMotion31ValidateSnapshot(const RLMotion31Snapshot *sample);
int RLMotion31DecodeOutput(const float action[RL_MOTION31_ACTION_SIZE],
                           float target[RL_MOTION31_ACTION_SIZE]);
/* Current supported015 training binds its center to fixed .34m, not measured
   height. Other height commands are rejected until a matching recipe exists. */
int RLMotion31DecodeOutputAtHeight(const float action[RL_MOTION31_ACTION_SIZE],
                                  float command_height_m,
                                  float target[RL_MOTION31_ACTION_SIZE]);
int RLMotion31MapPolicyTargets(const float target[RL_MOTION31_ACTION_SIZE],
                               float hardware_joint_target_rad[4],
                               float motor_wheel_target_rad_s[2]);
int RLMotion31ComputePdTorque(const RLMotion31Snapshot *sample,
                              const float target[RL_MOTION31_ACTION_SIZE],
                              float requested_torque_nm[RL_MOTION31_ACTION_SIZE],
                              float limited_torque_nm[RL_MOTION31_ACTION_SIZE]);
int RLMotion31MapPolicyLegTorqueToMotorOrder(const float policy_torque_nm[RL_MOTION31_ACTION_SIZE],
                                            float leg_torque_motor_nm[4]);
int RLMotion31MapPolicyWheelTorqueToForwardOrder(const float policy_torque_nm[RL_MOTION31_ACTION_SIZE],
                                                 float wheel_torque_forward_nm[2]);
int RLMotion31BuildPreview(const RLMotion31Snapshot *sample,
                           const float action[RL_MOTION31_ACTION_SIZE],
                           RLMotion31Preview *preview);

#endif
