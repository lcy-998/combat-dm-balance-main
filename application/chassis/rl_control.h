#ifndef RL_CONTROL_H
#define RL_CONTROL_H

#include "rl_shadow.h"

#define RL_CONTROL_LEG_RAMP_MS 5000u
#define RL_CONTROL_LEG_TARGET_M 0.28f
#define RL_CONTROL_INFERENCE_DELAY_MS 5000u
#define RL_CONTROL_COMMAND_MAX_AGE_MS 24u
/* Adopt policy output after the five-second ramp and five-second hold. */
#define RL_CONTROL_OUTPUT_ENABLED 1u
#define RL_CONTROL_PD_PERIOD_MS 5u
#define RL_CONTROL_WHEEL_RAW_LIMIT 16384.0f /* raw command units; physical calibration is still UNKNOWN */
#define RL_PREVIOUS_INIT_PROXY 1u
#define RL_PREVIOUS_ADOPTED 2u
#define RL_OWNER_LQR 0u
#define RL_OWNER_RL 1u
#define RL_OWNER_SAFE 2u /* explicit Recovery fault/wait: zero six-channel output */

enum { RL_FAULT_NONE=0u, RL_FAULT_INPUT=1u, RL_FAULT_COMMAND=2u,
       RL_FAULT_PD=3u, RL_FAULT_COMPENSATION=4u };
typedef struct {
    float command_m;
    uint8_t initialized;
} RLControlHeightState;

typedef struct {
    uint32_t ramp_start_tick;
    float ramp_start_length_m;
    uint8_t ramp_started, ramp_completed;
    uint32_t wait_start_tick;
    uint8_t timing;
    uint8_t active;
    uint8_t ready;
    uint8_t fault;
    uint32_t epoch;
    uint32_t adopted_sequence;
    uint32_t adopted_tick_ms;
    float adopted_target[6]; /* local PD adoption, not a claim of remote driver acceptance */
    float adopted_height_m; /* identity of the actually adopted Actor request */
    RLControlHeightState height;
    uint8_t recovery_session, recovering, recovery_initialized, recovery_pd_valid;
    uint32_t recovery_last_ms, recovery_output_ms, recovery_request_sequence;
    float recovery_request[6], recovery_targets[32][6], recovery_wheels[32][2];
    float recovery_pd[6], recovery_raw_pd[6];
    uint32_t recovery_target_sequence[32];
    RLHandoffState handoff;
} RLControlState;
typedef struct {
    uint8_t active;
    uint8_t exited;
} RLControlDecision;

typedef struct {
    uint32_t tick_ms, control_epoch, adopted_sequence, owner, fault;
    uint32_t adopted_tick_ms, snapshot_sequence;
    float adopted_target[6];
    float tau_pd_nm[6], tau_limited_nm[6];
    float spring_motor_nm[4], final_motor_nm[4]; /* lf/lb/rf/rb */
    float wheel_current_raw[2]; /* physical L/R, signed driver units, not measured A */
} RLControlFrame;

extern const RLHandoffConfig RL_HANDOFF_CONFIG;
/* Explicitly start only from a safe/idle state; does not auto-enable hardware. */
int RLControlStartRecovery(RLControlState *state);
/* Ramp the leg target for five seconds, then hold five seconds before inference. */
float RLControlLegRampStep(RLControlState *state, uint32_t now_tick,
                           int normal_lqr, float current_target_m);
RLControlDecision RLControlStep(RLControlState *state,
                                uint32_t now_tick, int normal_lqr,
                                const RLShadowCommand *command);
/* Called by the real 200Hz control owner. Recomputes PD on every call, even
   when no new policy target arrives. State commits only after all six outputs validate.
   Nonzero return keeps the existing LQR path and latches RL fault until mode reset. */
int RLControlExecute(RLControlState *state,
                     uint32_t now_ms, int normal_lqr,
                     const RLShadowSnapshot *current, const RLShadowCommand *command,
                     RLControlFrame *frame);
/* Before actual adoption LQR has no unique position target. The q/dq proxy
   is explicit metadata, never a shadow prediction or falsely marked executed. */
void RLControlFillPrevious(const RLControlState *state, RLShadowSnapshot *snapshot);
float RLControlCommandHeight(const RLControlState *state);
/* Explicit height request, no stop/dwell/slew/adoption gate. Does not rewrite
   actual previous or an in-flight request's physical target. */
int RLControlSetCommandHeight(RLControlState *state,float command_height_m);
int RLControlFeedbackFresh(const RLShadowSnapshot *snapshot, uint32_t now_ms);
/* Only explicit disabled/safe mode acknowledges a latched fault. */
void RLControlReset(RLControlState *state);
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
