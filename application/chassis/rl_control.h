#ifndef RL_CONTROL_H
#define RL_CONTROL_H

#include "rl_shadow.h"

#define RL_CONTROL_LQR_WAIT_MS 10000u
#define RL_CONTROL_BLEND_MS 200u
#define RL_CONTROL_COMMAND_MAX_AGE_MS 24u
#define RL_CONTROL_LEG_EXTEND_MS 5000u
#define RL_CONTROL_LEG_TARGET_M 0.35f

typedef struct {
    uint32_t lqr_since_tick;
    uint32_t blend_since_tick;
    uint8_t timing;
    uint8_t active;
    uint8_t beeped;
    float previous_action[RL_SHADOW_ACTION_SIZE];
} RLControlState;

typedef struct {
    uint8_t active;
    uint8_t beep;
    uint8_t exit_beep;
    float blend;
} RLControlDecision;

typedef struct {
    uint32_t start_tick;
    float start_length_m;
    uint8_t started;
    uint8_t completed;
} RLControlLegRamp;

RLControlDecision RLControlStep(RLControlState *state, uint32_t now_tick,
                                int normal_lqr, const RLShadowCommand *command);
float RLControlLegRampStep(RLControlLegRamp *ramp, uint32_t now_tick,
                           int normal_lqr, float current_target_m);
int RLControlMapOutput(const RLShadowOutputPreview *output,
                       const float encoder_now[4], const float encoder_offset[4],
                       float encoder_target[4], float wheel_current[2]);
void RLControlBlendJoint(float encoder_now, float lqr_torque,
                         float encoder_target, float blend,
                         float *position, float *torque, float *kp, float *kd);
float RLControlBlendCurrent(float lqr_current, float rl_current, float blend);

#endif
