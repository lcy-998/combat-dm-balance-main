#ifndef RL_CONTROL_H
#define RL_CONTROL_H

#include "rl_shadow.h"

#define RL_CONTROL_LEG_RAMP_MS 5000u
#define RL_CONTROL_LQR_HOLD_AFTER_RAMP_MS 5000u
#define RL_CONTROL_COMMAND_MAX_AGE_MS 24u
#define RL_CONTROL_LEG_TARGET_M 0.28f
/* Switch to policy torque only after the ramp, hold and fresh-command checks. */
#define RL_CONTROL_OUTPUT_ENABLED 1u

typedef struct {
    uint32_t wait_start_tick;
    uint8_t timing;
    uint8_t active;
} RLControlState;
typedef struct {
    uint32_t start_tick;
    float start_length_m;
    uint8_t started;
    uint8_t completed;
} RLControlLegRamp;

typedef struct {
    uint8_t active;
    uint8_t exited;
} RLControlDecision;

/* The leg target rises over five seconds. RL then waits five more seconds. */
float RLControlLegRampStep(RLControlLegRamp *ramp, uint32_t now_tick,
                           int normal_lqr, float current_target_m);
RLControlDecision RLControlStep(RLControlState *state, RLControlLegRamp *ramp,
                                uint32_t now_tick, int normal_lqr,
                                const RLShadowCommand *command);
/* Apply the measured spring fit in lf/lb/rf/rb order and limit final torque. */
int RLControlCompensateSpring(const float policy_torque_nm[4],
                              const float leg_length_m[2],
                              const float leg_spread_rad[2],
                              float motor_torque_nm[4]);
int RLControlMapOutput(const RLMotion31Preview *preview,
                       const float wheel_forward_torque_nm[2],
                       const float encoder_now[4], const float encoder_offset[4],
                       float encoder_target[4], float wheel_current[2]);

#endif
