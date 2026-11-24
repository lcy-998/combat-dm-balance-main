#ifndef RL_STAIR_H
#define RL_STAIR_H

#include "rl_shadow.h"

/* One explicitly armed 20 cm trial. These are external handoff gates. */
#define RL_STAIR_ARM_CHANGE_L 2
#define RL_STAIR_APPROACH_M 0.40f
#define RL_STAIR_GOAL_M 0.25f
#define RL_STAIR_LINK_MAX_AGE_MS 50u
#define RL_STAIR_IMPACT_MS 20u
#define RL_STAIR_CONFIRM_MS 500u
#define RL_STAIR_TIMEOUT_MS 3000u
#define RL_STAIR_BLEND_PACKETS 3u /* 3 x 10 ms; begin with the first valid packet */

typedef enum {
    RL_STAIR_IDLE, RL_STAIR_ACTIVE, RL_STAIR_RETURN, RL_STAIR_WAIT_RELEASE, RL_STAIR_STOPPED
} RLStairPhase;

typedef enum {
    RL_STAIR_OK, RL_STAIR_STALE, RL_STAIR_POSE, RL_STAIR_TIMEOUT, RL_STAIR_POLICY
} RLStairFault;

typedef struct {
    uint32_t tick_ms, imu_sequence;
    int armed, link_fresh, feedback_fresh, can_enter;
    float vx_cmd, vx_estimate, pitch, roll, pitch_rate, omega_squared, accel_up;
    float swing[2], wheel_to_root_m[2], wheel_up_estimate_n[2], wheel_forward_mps[2]; /* physical L/R */
} RLStairSample;

typedef struct {
    RLStairPhase phase;
    RLStairFault fault;
    uint32_t impact_start_tick, impact_tick, stair_adopted_tick, supported_tick, motion_adopted_tick;
    uint32_t last_sample_tick, imu_sequence, anchor_tick, condition_tick;
    float root_m, root_vz, bias, accel_last, support_travel_m, height_last;
    float wheel_speed_last[2], swing_last[2], swing_at_shock[2];
    uint32_t shock_tick[2];
    uint8_t have_sample, have_anchor, condition, stair_adopted, wheel_shock_mask, last_bilateral;
    float tread_estimate_m[2];
} RLStairHandoff;

typedef struct {
    uint32_t tick_ms, phase, fault, owner, requested_actor;
    uint32_t impact_start_tick, impact_tick, stair_adopted_tick, supported_tick, motion_adopted_tick;
    float wheel_up_estimate_n[2], tread_estimate_m[2], wheel_to_root_m[2];
    float pitch, roll, vx_estimate;
} RLStairDebug;

extern volatile RLStairDebug g_rl_stair_debug;

static inline int RLStairEngaged(const RLStairHandoff *state)
{
    return state->phase == RL_STAIR_ACTIVE || state->phase == RL_STAIR_RETURN;
}

/* The detector does not write targets. Actual adoption is reported separately. */
void RLStairUpdate(RLStairHandoff *state, const RLStairSample *sample);
void RLStairOnAdopt(RLStairHandoff *state, uint8_t actor, uint32_t now_ms);
void RLStairStop(RLStairHandoff *state, RLStairFault fault);
void RLStairReset(RLStairHandoff *state, int armed);

int RLStairBuildObservation(const RLMotion31Snapshot *sample, float observation[31]);
/* Quasi-static five-bar estimate. Torques use the two kinematic angle signs
 * with the 050 spring compensation removed. NaN means unobservable. */
float RLStairWheelLoad(float phi1, float phi4, float phi2, float phi3,
                       float tau1, float tau4, float pitch, float roll);

#endif
