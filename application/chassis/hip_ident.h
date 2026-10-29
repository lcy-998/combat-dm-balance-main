#ifndef HIP_IDENT_H
#define HIP_IDENT_H
#include "rl_control.h"
#include "hip_ident_config.h"

enum { HIP_WAIT=0, HIP_RAMP, HIP_SETTLE, HIP_SWEEP, HIP_GAP, HIP_DONE, HIP_FAULT };
enum { HIP_OK=0, HIP_BAD_CONFIG, HIP_BAD_FEEDBACK, HIP_BAD_POSE, HIP_BAD_PD,
       HIP_COMMAND_LOST };

typedef struct {
    uint32_t stage, start_ms, center, direction, armed, fault;
    float ramp_from[6], previous_target[6], center_length_m;
} HipIdentState;

/* Identification joint arrays use model joint order RH, RA, LH, LA.
   observation values are the exact normalized network inputs, not rad/rad/s.
   pre_spring_torque_nm is signed, limited PD torque before spring compensation.
   Read status.sequence before/after copying; accept the same even value. */
typedef struct {
    float observation_position[4]; /* observation[6,7,8,9] = q/1.5 */
    float observation_velocity[4]; /* observation[10,11,13,14] = dq*0.05 */
    float pre_spring_torque_nm[4]; /* model axis; AFTER +/-40 Nm PD limit */
    float mean_actor[4];           /* raw normalized action[0,1,3,4] */
    float feedback_torque_nm[4];   /* DM feedback minus nominal spring command, in pre_spring model axes */
    float leg_length_m[2];        /* measured FK length in metres: [0]=left, [1]=right */
} HipIdentDebug;

/* Experiment metadata is separate so the signal watch window stays small. */
typedef struct {
    uint32_t sequence, tick_ms, stage, fault, output_enabled, observation_valid;
    uint32_t side, center, direction;
    uint32_t feedback_torque_valid_mask; /* bits RH,RA,LH,LA; finite feedback within timeout */
    float frequency_hz, sweep_elapsed_s, envelope;
    float target_leg_length_m, measured_leg_length_m, center_length_m;
} HipIdentStatus;

/* Internal result; not a debug export. Motor output order is lf/lb/rf/rb. */
typedef struct {
    HipIdentDebug debug;
    HipIdentStatus status;
    float motor_request_nm[4];
} HipIdentFrame;

extern volatile HipIdentDebug g_hip_ident_debug;
extern volatile HipIdentStatus g_hip_ident_status;
/* Center solve preserves the selected reference leg angle and uses the exact
   existing zero/sign mapping. Returns 0 on success, -1 on invalid geometry. */
int HipIdentCenterAction(uint32_t center, float action[6]);
/* Commit motor outputs only on return 1. Emergency acknowledges faults;
   link loss latches a fault and does not acknowledge it. */
int HipIdentStep(HipIdentState *state, uint32_t now_ms, int emergency,
                 int command_link_ok, const RLShadowSnapshot *feedback,
                 HipIdentFrame *out);
#endif
