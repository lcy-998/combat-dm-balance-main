#ifndef RL_CONTROL_H
#define RL_CONTROL_H

#include "rl_shadow.h"

#define RL_CONTROL_COMMAND_MAX_AGE_MS 24u

typedef struct {
    uint32_t enable_tick;
    uint8_t enabled;
    uint8_t recovering;
    uint8_t active;
} RLControlState;

typedef struct {
    uint8_t active;
    uint8_t exited;
    uint8_t recovering;
} RLControlDecision;

/* Every enable runs recovery RL continuously until emergency stop. */
RLControlDecision RLControlStep(RLControlState *state, uint32_t now_tick,
                                int enabled,
                                const RLShadowCommand *command);
/* Add legacy spring compensation in lf/lb/rf/rb motor order; clamp to +/-40 Nm. */
int RLControlCompensateSpring(const float policy_torque_nm[4],
                              const float leg_length_m[2],
                              const float leg_spread_rad[2],
                              float motor_torque_nm[4]);
int RLControlMapOutput(const RLMotion31Preview *preview,
                       const float wheel_forward_torque_nm[2],
                       const float encoder_now[4], const float encoder_offset[4],
                       float encoder_target[4], float wheel_current[2]);

#endif
