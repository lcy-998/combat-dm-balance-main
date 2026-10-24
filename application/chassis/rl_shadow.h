#ifndef RL_SHADOW_H
#define RL_SHADOW_H

#include "rl_motion31.h"

#include <stdint.h>

typedef struct {
    uint32_t sequence;
    uint32_t tick_ms;
    float lqr_pitch_rad; /* body pitch used by theta_l/theta_r; not an Actor input */
    uint8_t chassis_ready;
    uint8_t rl_active;
    float leg_length_m[2]; /* left, right; spring compensation input */
    float leg_spread_rad[2]; /* left, right; spring compensation input */
    RLMotion31Snapshot motion;
} RLShadowSnapshot;

typedef struct {
    uint32_t snapshot_sequence;
    uint32_t snapshot_tick_ms;
    uint32_t output_tick_ms;
    uint8_t valid;
    float action[RL_MOTION31_ACTION_SIZE];
    RLMotion31Preview preview;
    float leg_torque_motor_nm[4];
    float wheel_forward_torque_nm[2];
} RLShadowCommand;

typedef struct {
    uint32_t sequence;
    uint32_t stage; /* 0=no sample, 1=fresh, 2=obs, 3=inference, 4=decode, 5=PD */
    uint32_t failure; /* 0=none, 1=init, 2=no sample, 3=stale, 4=input, 5=network,
                         6=decode, 7=target mapping, 8=PD, 9=torque mapping,
                         10=model golden vector mismatch */
    uint32_t snapshot_age_ms;
    uint32_t input_error_mask; /* bits 0..3=nonfinite q0,q1,q3,q4; 4=gyro; 5=gravity;
                                  6=joint velocities; 7=previous target; 8=commands */
    uint8_t observation_valid;
    float input_joint_position_rad[4]; /* model RH, RA, LH, LA before validation */
    uint8_t previous_target_valid;
    uint8_t previous_target_is_virtual; /* 1=shadow cache; no target was sent to hardware */
    uint8_t golden_checked;
    uint8_t rl_target_theta_valid; /* both five-bar target poses have a finite solution */
    uint8_t rl_wheel_target_valid;
    uint8_t rl_target_valid; /* decoded_target and mapped wheel speed are from a successful output */
    uint8_t rl_active;
    float golden_max_abs_error;
    float observation[RL_MOTION31_OBSERVATION_SIZE];
    float mean_action[RL_MOTION31_ACTION_SIZE];
    float decoded_target[RL_MOTION31_ACTION_SIZE];
    float rl_wheel_target_rad_s[2]; /* left/right, wheel-output rad/s, motor-axis sign */
    float rl_wheel_current_rad_s[2]; /* measured left/right, same units and motor-axis signs */
    float rl_target_theta_l_rad; /* physical left, same zero/sign as LQR theta_l */
    float rl_target_theta_r_rad; /* physical right, same zero/sign as LQR theta_r */
    float lqr_leg_torque_nm[4]; /* existing motor order: lf, lb, rf, rb */
    float lqr_wheel_torque_nm[2]; /* forward-positive: left, right */
    float rl_leg_torque_nm[4]; /* spring-compensated and clamped, motor order: lf, lb, rf, rb */
    float rl_wheel_torque_nm[2]; /* forward-positive: left, right */
} RLShadowDebug;

extern volatile RLShadowDebug g_rl_shadow_debug;
void RLShadowQueueInit(void);
void RLShadowPublish(const RLShadowSnapshot *sample);
int RLShadowReadCommand(RLShadowCommand *command);
void RLShadowTask(void const *argument);

#endif
