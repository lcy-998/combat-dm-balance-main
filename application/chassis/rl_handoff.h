#ifndef RL_HANDOFF_H
#define RL_HANDOFF_H
#include <stdint.h>
/* Gate is sampled from measured state, never commanded posture or a fixed timer. */
typedef struct {
    uint32_t tick_ms;
    uint8_t valid; /* geometry/IMU and support evidence are fresh */
    float height_m, pitch_rad, roll_rad, angular_speed_rad_s;
    float normal_force_N[2]; /* physical left/right, measured or separately qualified estimator */
    float maximum_leg_speed_rad_s;
} RLHandoffSample;
typedef struct {
    uint32_t dwell_ms;
    float height_min_m, height_max_m, command_height_m, height_error_m;
    float tilt_rad, angular_speed_rad_s, leg_speed_rad_s, support_min_N;
    float maximum_leg_target_jump_rad, maximum_wheel_target_jump_rad_s;
} RLHandoffConfig;
typedef struct {
    uint32_t since_ms, last_ms, request_tick_ms;
    uint8_t dwelling, observed, requested, committed;
} RLHandoffState;
void RLHandoffReset(RLHandoffState *state);
/* Returns1 only after uninterrupted quality dwell, pending resets on lost quality; committed remains latched until reset. */
int RLHandoffObserve(RLHandoffState *state, const RLHandoffConfig *config,
                     const RLHandoffSample *sample);
/* New Motion output must use the SAME adopted previous; this only validates a switch. */
int RLHandoffCanCommit(const RLHandoffState *state, const RLHandoffConfig *config,
                       uint32_t snapshot_tick_ms, const float adopted[6],
                       const float motion_requested[6]);
/* Encoder/IMU readiness only: does NOT assert terrain height or wheel contact.
   q is model leg order0,1,3,4. Gravity/gyro are model frame; dq is six channels.
   Caller supplies current Motion center and tolerance; no normal force is invented. */
int RLHandoffObservePosture(RLHandoffState *state,const RLHandoffConfig *config,
                            uint32_t tick_ms,int feedback_valid,
                            const float q[4],const float dq[6],const float gyro[3],
                            const float gravity[3],const float center[4],
                            float maximum_center_error_rad);
#endif
