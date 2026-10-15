#ifndef RL_SHADOW_H
#define RL_SHADOW_H

#include <stdint.h>

#define RL_SHADOW_OBSERVATION_SIZE 25u
#define RL_SHADOW_ACTION_SIZE 6u
#define RL_SHADOW_POSITION_KP -1.0f
#define RL_SHADOW_TARGET_SPEED_LIMIT_M_S 2.2f
#define RL_SHADOW_POSITION_MAX_DT_MS 24u
#define RL_SHADOW_ACTIVE_ZERO_RAD ((180.0f - 42.8919f) * 0.01745329251994329577f)
#define RL_SHADOW_HIP_ZERO_RAD (42.8966f * 0.01745329251994329577f)

/* One complete chassis sample. The publisher owns no memory used by the reader. */
typedef struct {
    uint32_t sequence;
    uint32_t tick_ms;
    int32_t chassis_mode;
    uint8_t chassis_ready;
    uint8_t rl_active;
    uint32_t rl_activation_sequence;
    float odom_s_m;
    float target_s_m;
    float remote_vx_m_s;
    float roll_rad;
    float pitch_rad;
    float roll_rate_rad_s;
    float pitch_rate_rad_s;
    float yaw_rate_rad_s;
    float target_vx_m_s;        /* position PID result, filled by RL task */
    float target_yaw_rad_s;
    float heading_vx_m_s;
    float lqr_speed_m_s;
    float joint_angle_rad[4];   /* physical lf, lb, rf, rb */
    float joint_rate_rad_s[4];  /* physical lf, lb, rf, rb */
    float wheel_rate_rad_s[2];  /* model right_wheel (physical left), left_wheel */
    float lqr_torque_nm[4];     /* physical left/right wheel, left/right leg */
    float motor_torque_nm[4];   /* DM feedback in RL active/hip-positive axes: lf, lb, rf, rb; stale if offline */
    float wheel_current_cmd[2];/* physical left, right; driver command units */
    float previous_action[RL_SHADOW_ACTION_SIZE]; /* last action applied by chassis */
} RLShadowSnapshot;

typedef struct {
    uint32_t last_tick_ms;
    uint32_t activation_sequence;
    float target_s_m;
    uint8_t active;
    uint8_t zero_latched;
} RLShadowPositionTarget;

/* Converted policy output in chassis joint and physical wheel coordinates. */
typedef struct {
    float leg_scale_rad_per_action;
    float joint_target_angle_rad[4];  /* lfangle, lbangle, rfangle, rbangle coordinates */
    float wheel_forward_torque_nm[2];/* physical left/right, forward positive */
} RLShadowOutputPreview;

/* Published atomically through a length-one FreeRTOS queue. */
typedef struct {
    uint32_t snapshot_sequence;
    uint32_t snapshot_tick_ms;
    uint32_t output_tick_ms;
    uint8_t valid;
    float raw_action[RL_SHADOW_ACTION_SIZE];
    RLShadowOutputPreview output;
} RLShadowCommand;

enum {
    RL_SHADOW_OK = 0,
    RL_SHADOW_NO_SAMPLE = 1,
    RL_SHADOW_STALE_SAMPLE = 2,
    RL_SHADOW_INVALID_INPUT = 3,
    RL_SHADOW_AI_INIT_FAILED = 4,
    RL_SHADOW_AI_RUN_FAILED = 5,
    RL_SHADOW_INVALID_OUTPUT = 6,
    RL_SHADOW_TASK_CREATE_FAILED = 7
};

/* Single writer: the shadow task. Ozone may watch g_rl_shadow_debug. */
typedef struct {
    uint32_t publish_seq;      /* odd while updating, even when coherent */
    uint32_t snapshot_seq;
    uint32_t snapshot_tick_ms;
    uint32_t inference_tick_ms;
    uint32_t missed_periods;
    uint32_t stack_free_min_words;
    uint32_t heap_free_min_bytes;
    int32_t status;
    int32_t chassis_mode;
    uint8_t chassis_ready;
    uint8_t rl_active;
    uint32_t rl_activation_sequence;
    float odom_s_m;
    float target_s_m;
    float position_error_m;
    float remote_vx_m_s;
    float observation[RL_SHADOW_OBSERVATION_SIZE]; /* scaled, dimensionless */
    float raw_action[RL_SHADOW_ACTION_SIZE];       /* dimensionless policy output */
    float target_vx_m_s;       /* position PID output to policy */
    float leg_scale_rad_per_action;
    float lqr_wheel_torque_nm[2];       /* physical left/right, forward positive */
    float policy_wheel_torque_nm[2];    /* converted action */
    float measured_joint_angle_rad[4]; /* lfangle, lbangle, rfangle, rbangle */
    float policy_joint_target_angle_rad[4]; /* same units, signs and zero positions */
    float lqr_torque_nm[4];
    float motor_torque_nm[4];  /* DM feedback, positive along each RL action angle; stale if offline */
    float wheel_current_cmd[2];
    float heading_vx_m_s;
    float lqr_speed_m_s;
} RLShadowDebug;

extern volatile RLShadowDebug g_rl_shadow_debug;

void RLShadowQueueInit(void);
void RLShadowPublish(const RLShadowSnapshot *sample);
int RLShadowReadCommand(RLShadowCommand *command);
int RLShadowUpdatePositionTarget(RLShadowPositionTarget *state,
                                 const RLShadowSnapshot *sample,
                                 uint32_t now_tick_ms, float *target_s_m);
int RLShadowPositionPID(float odom_s_m, float target_s_m,
                        float *target_vx_m_s);
int RLShadowBuildObservation(const RLShadowSnapshot *sample,
                             float observation[RL_SHADOW_OBSERVATION_SIZE]);
int RLShadowConvertOutput(float target_vx_m_s,
                          const float raw_action[RL_SHADOW_ACTION_SIZE],
                          RLShadowOutputPreview *preview);
void RLShadowTask(void const *argument);
void RLShadowTaskCreateFailed(void);

#endif
