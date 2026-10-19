#include "rl_shadow.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "tjb_network.h"
#include "tjb_network_data.h"
#include "legalgorithm.h"
#include "leg_const.hpp"

#include <math.h>
#include <string.h>

#define RL_SHADOW_PERIOD pdMS_TO_TICKS(10u)
#define RL_SHADOW_MAX_AGE pdMS_TO_TICKS(24u)

_Static_assert(AI_TJB_NETWORK_IN_1_SIZE == RL_MOTION31_OBSERVATION_SIZE,
               "Motion31 input size mismatch");
_Static_assert(AI_TJB_NETWORK_OUT_1_SIZE == RL_MOTION31_ACTION_SIZE,
               "Motion31 output size mismatch");

volatile RLShadowDebug g_rl_shadow_debug;
static StaticQueue_t snapshot_queue_struct;
static uint8_t snapshot_queue_storage[sizeof(RLShadowSnapshot)];
static QueueHandle_t snapshot_queue;
static StaticQueue_t command_queue_struct;
static uint8_t command_queue_storage[sizeof(RLShadowCommand)];
static QueueHandle_t command_queue;
AI_ALIGNED(32) static uint8_t network_activations[AI_TJB_NETWORK_DATA_ACTIVATION_1_SIZE];
static ai_handle network_handle = AI_HANDLE_NULL;
static ai_buffer *network_input;
static ai_buffer *network_output;

static uint32_t input_error_mask(const RLMotion31Snapshot *motion)
{
    uint32_t mask = 0u;
    for (size_t i = 0; i < 4u; ++i) {
        if (!isfinite(motion->joint_position_rad[i])) mask |= 1u << i;
    }
    for (size_t i = 0; i < 3u; ++i) {
        if (!isfinite(motion->gyro_model_rad_s[i])) mask |= 1u << 4;
        if (!isfinite(motion->gravity_model[i])) mask |= 1u << 5;
    }
    for (size_t i = 0; i < 6u; ++i) {
        if (!isfinite(motion->joint_velocity_rad_s[i])) mask |= 1u << 6;
        if (!isfinite(motion->previous_target[i])) mask |= 1u << 7;
    }
    if (!isfinite(motion->command_vx_m_s) ||
        !isfinite(motion->command_yaw_rad_s) ||
        !isfinite(motion->command_height_m)) mask |= 1u << 8;
    return mask;
}

void RLShadowQueueInit(void)
{
    snapshot_queue = xQueueCreateStatic(1u, sizeof(RLShadowSnapshot),
                                         snapshot_queue_storage, &snapshot_queue_struct);
    command_queue = xQueueCreateStatic(1u, sizeof(RLShadowCommand),
                                        command_queue_storage, &command_queue_struct);
}

int RLShadowReadCommand(RLShadowCommand *command)
{
    return (command != NULL && command_queue != NULL &&
            xQueuePeek(command_queue, command, 0u) == pdPASS) ? 0 : -1;
}

void RLShadowPublish(const RLShadowSnapshot *sample)
{
    if (sample != NULL && snapshot_queue != NULL) {
        (void)xQueueOverwrite(snapshot_queue, sample);
    }
}

static void publish_command(const RLShadowSnapshot *sample,
                            const float *action,
                            const RLMotion31Preview *preview,
                            const float *wheel_torque_forward_nm,
                            int valid)
{
    if (command_queue == NULL) return;
    RLShadowCommand command = {0};
    command.output_tick_ms = xTaskGetTickCount();
    if (sample != NULL) {
        command.snapshot_sequence = sample->sequence;
        command.snapshot_tick_ms = sample->tick_ms;
    }
    if (valid && sample != NULL && sample->chassis_ready &&
        sample->motion.lqr_torque_valid && action != NULL &&
        preview != NULL && wheel_torque_forward_nm != NULL) {
        command.valid = 1u;
        memcpy(command.action, action, sizeof(command.action));
        command.preview = *preview;
        memcpy(command.wheel_forward_torque_nm, wheel_torque_forward_nm,
               sizeof(command.wheel_forward_torque_nm));
    }
    (void)xQueueOverwrite(command_queue, &command);
}

static int network_init(void)
{
    ai_handle activations[] = {network_activations};
    ai_error error = ai_tjb_network_create_and_init(&network_handle, activations, NULL);
    if (error.type != AI_ERROR_NONE) {
        return -1;
    }
    network_input = ai_tjb_network_inputs_get(network_handle, NULL);
    network_output = ai_tjb_network_outputs_get(network_handle, NULL);
    return (network_input != NULL && network_output != NULL &&
            network_input[0].data != AI_HANDLE_NULL &&
            network_output[0].data != AI_HANDLE_NULL) ? 0 : -1;
}

static int network_run(const float observation[RL_MOTION31_OBSERVATION_SIZE],
                       float action[RL_MOTION31_ACTION_SIZE])
{
    memcpy(network_input[0].data, observation, AI_TJB_NETWORK_IN_1_SIZE_BYTES);
    if (ai_tjb_network_run(network_handle, network_input, network_output) != 1) {
        return -1;
    }
    memcpy(action, network_output[0].data, AI_TJB_NETWORK_OUT_1_SIZE_BYTES);
    return 0;
}

static void check_target_network_golden(void)
{
    /* model_golden_vectors.json id 144: a numerical fixture, not a motion test. */
    static const float observation[RL_MOTION31_OBSERVATION_SIZE] = {
        [4] = -1.0f, [24] = 0.68f, [25] = 1.0f, [28] = 1.0f};
    static const float expected[RL_MOTION31_ACTION_SIZE] = {
        0.010617729f, 0.071231335f, -0.019865362f,
        -0.004225575f, 0.032680389f, -0.023813765f};
    float actual[RL_MOTION31_ACTION_SIZE];
    if (network_run(observation, actual) != 0) {
        g_rl_shadow_debug.golden_checked = 0u;
        g_rl_shadow_debug.golden_max_abs_error = INFINITY;
        return;
    }
    float max_error = 0.0f;
    for (size_t i = 0; i < RL_MOTION31_ACTION_SIZE; ++i) {
        if (!isfinite(actual[i])) {
            max_error = INFINITY;
            break;
        }
        const float error = fabsf(actual[i] - expected[i]);
        if (error > max_error) max_error = error;
    }
    g_rl_shadow_debug.golden_checked = 1u;
    g_rl_shadow_debug.golden_max_abs_error = max_error;
}

static void publish_debug(const RLShadowSnapshot *sample,
                          const float *rl_torque_limited)
{
    volatile RLShadowDebug *d = &g_rl_shadow_debug;
    const int lqr_valid = sample != NULL && sample->motion.lqr_torque_valid;
    float rl_leg_torque[4] = {0};
    float rl_wheel_torque[2] = {0};
    if (sample != NULL) {
        d->sequence = sample->sequence;
        d->rl_active = sample->rl_active;
        /* Snapshot wheel velocities use model signs: left raw, right inverted. */
        d->rl_wheel_current_rad_s[0] = sample->motion.joint_velocity_rad_s[5];
        d->rl_wheel_current_rad_s[1] = -sample->motion.joint_velocity_rad_s[2];
    } else {
        d->sequence = 0u;
        d->rl_active = 0u;
        d->rl_wheel_current_rad_s[0] = 0.0f;
        d->rl_wheel_current_rad_s[1] = 0.0f;
        memset((void *)d->observation, 0, sizeof(d->observation));
    }
    if (rl_torque_limited != NULL) {
        (void)RLMotion31MapPolicyLegTorqueToMotorOrder(rl_torque_limited, rl_leg_torque);
        (void)RLMotion31MapPolicyWheelTorqueToForwardOrder(rl_torque_limited,
                                                            rl_wheel_torque);
    }
    for (size_t i = 0; i < 4u; ++i) {
        d->lqr_leg_torque_nm[i] = lqr_valid ? sample->motion.lqr_leg_torque_motor_nm[i] : 0.0f;
        d->rl_leg_torque_nm[i] = rl_leg_torque[i];
    }
    for (size_t i = 0; i < 2u; ++i) {
        d->lqr_wheel_torque_nm[i] = lqr_valid ? sample->motion.lqr_wheel_torque_forward_nm[i] : 0.0f;
        d->rl_wheel_torque_nm[i] = rl_wheel_torque[i];
    }
}

static void publish_target_swing(const RLShadowSnapshot *sample,
                                 const float target[RL_MOTION31_ACTION_SIZE])
{
    float joint_target[4]; /* lfangle, lbangle, rfangle, rbangle */
    float wheel_target[2];
    if (!isfinite(sample->lqr_pitch_rad) ||
        RLMotion31MapPolicyTargets(target, joint_target, wheel_target) != 0) {
        return;
    }

    /* Use exactly the position solution used by forward_kinematics().
       LQR defines theta_l/r as PI/2 - theta0 + current body pitch. */
    float phi2, phi3, leg_length_l, leg_length_r, theta0_l, theta0_r;
    getPhi(joint_target[0], joint_target[1], L_1, L_2, L_3, L_4, L_5,
           &phi2, &phi3, &leg_length_l, &theta0_l);
    getPhi(joint_target[2], joint_target[3], L_1, L_2, L_3, L_4, L_5,
           &phi2, &phi3, &leg_length_r, &theta0_r);
    if (!isfinite(leg_length_l) || !isfinite(leg_length_r) ||
        leg_length_l <= 0.0f || leg_length_r <= 0.0f ||
        !isfinite(theta0_l) || !isfinite(theta0_r)) {
        return;
    }
    const float theta_l_target = (PI / 2.0f - theta0_l) + sample->lqr_pitch_rad;
    const float theta_r_target = (PI / 2.0f - theta0_r) + sample->lqr_pitch_rad;
    if (!isfinite(theta_l_target) || !isfinite(theta_r_target)) return;
    g_rl_shadow_debug.rl_target_theta_l_rad = theta_l_target;
    g_rl_shadow_debug.rl_target_theta_r_rad = theta_r_target;
    g_rl_shadow_debug.rl_target_theta_valid = 1u;
}

void RLShadowTask(void const *argument)
{
    (void)argument;
    /* Shadow-only recurrence: these are the preceding policy's decoded
       physical targets, in model LH/LA/LW/RH/RA/RW order. They are virtual and
       have no hardware mechanical limit, since LQR drives the robot. */
    float previous_shadow_target[RL_MOTION31_ACTION_SIZE] = {0};
    uint8_t previous_shadow_target_valid = 0u;
    if (snapshot_queue == NULL || network_init() != 0) {
        g_rl_shadow_debug.failure = 1u;
        publish_debug(NULL, NULL);
        publish_command(NULL, NULL, NULL, NULL, 0);
        vTaskSuspend(NULL);
    }
    check_target_network_golden();
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, RL_SHADOW_PERIOD);
        g_rl_shadow_debug.observation_valid = 0u;
        g_rl_shadow_debug.input_error_mask = 0u;
        g_rl_shadow_debug.rl_wheel_target_valid = 0u;
        g_rl_shadow_debug.rl_target_valid = 0u;
        g_rl_shadow_debug.rl_target_theta_valid = 0u;
        RLShadowSnapshot sample;
        if (xQueuePeek(snapshot_queue, &sample, 0u) != pdPASS) {
            memset(previous_shadow_target, 0, sizeof(previous_shadow_target));
            previous_shadow_target_valid = 0u;
            g_rl_shadow_debug.stage = 0u;
            g_rl_shadow_debug.failure = 2u;
            publish_debug(NULL, NULL);
            publish_command(NULL, NULL, NULL, NULL, 0);
            continue;
        }
        g_rl_shadow_debug.sequence = sample.sequence;
        TickType_t now = xTaskGetTickCount();
        g_rl_shadow_debug.snapshot_age_ms = (uint32_t)(now - sample.tick_ms);
        for (size_t i = 0; i < 4u; ++i) {
            g_rl_shadow_debug.input_joint_position_rad[i] = sample.motion.joint_position_rad[i];
        }
        if ((TickType_t)(now - sample.tick_ms) > RL_SHADOW_MAX_AGE) {
            memset(previous_shadow_target, 0, sizeof(previous_shadow_target));
            previous_shadow_target_valid = 0u;
            g_rl_shadow_debug.stage = 0u;
            g_rl_shadow_debug.failure = 3u;
            publish_debug(&sample, NULL);
            publish_command(&sample, NULL, NULL, NULL, 0);
            continue;
        }
        g_rl_shadow_debug.stage = 1u;
        if (previous_shadow_target_valid) {
            memcpy(sample.motion.previous_target, previous_shadow_target,
                   sizeof(previous_shadow_target));
        } else {
            memset(sample.motion.previous_target, 0,
                   sizeof(sample.motion.previous_target));
        }
        sample.motion.previous_target_valid = previous_shadow_target_valid;
        g_rl_shadow_debug.previous_target_valid = previous_shadow_target_valid;
        g_rl_shadow_debug.previous_target_is_virtual = 1u;
        RLMotion31Preview preview;
        float action[RL_MOTION31_ACTION_SIZE];
        if (RLMotion31BuildObservation(&sample.motion, preview.observation) != 0) {
            g_rl_shadow_debug.failure = 4u;
            g_rl_shadow_debug.input_error_mask = input_error_mask(&sample.motion);
            previous_shadow_target_valid = 0u;
            publish_debug(&sample, NULL);
            publish_command(&sample, NULL, NULL, NULL, 0);
            continue;
        }
        for (size_t i = 0; i < RL_MOTION31_OBSERVATION_SIZE; ++i) {
            g_rl_shadow_debug.observation[i] = preview.observation[i];
        }
        g_rl_shadow_debug.observation_valid = 1u;
        g_rl_shadow_debug.stage = 2u;
        if (network_run(preview.observation, action) != 0) {
            g_rl_shadow_debug.failure = 5u;
            previous_shadow_target_valid = 0u;
            publish_debug(&sample, NULL);
            publish_command(&sample, NULL, NULL, NULL, 0);
            continue;
        }
        for (size_t i = 0; i < RL_MOTION31_ACTION_SIZE; ++i) {
            g_rl_shadow_debug.mean_action[i] = action[i];
        }
        g_rl_shadow_debug.stage = 3u;
        if (RLMotion31DecodeOutput(action, preview.target) != 0) {
            g_rl_shadow_debug.failure = 6u;
            previous_shadow_target_valid = 0u;
            publish_debug(&sample, NULL);
            publish_command(&sample, NULL, NULL, NULL, 0);
            continue;
        }
        for (size_t i = 0; i < RL_MOTION31_ACTION_SIZE; ++i) {
            g_rl_shadow_debug.decoded_target[i] = preview.target[i];
        }
        if (RLMotion31MapPolicyTargets(preview.target,
                                       preview.hardware_joint_target_rad,
                                       preview.motor_wheel_target_rad_s) != 0) {
            g_rl_shadow_debug.failure = 7u;
            previous_shadow_target_valid = 0u;
            publish_debug(&sample, NULL);
            publish_command(&sample, NULL, NULL, NULL, 0);
            continue;
        }
        g_rl_shadow_debug.rl_wheel_target_rad_s[0] = preview.motor_wheel_target_rad_s[0];
        g_rl_shadow_debug.rl_wheel_target_rad_s[1] = preview.motor_wheel_target_rad_s[1];
        g_rl_shadow_debug.rl_wheel_target_valid = 1u;
        g_rl_shadow_debug.rl_target_valid = 1u;
        publish_target_swing(&sample, preview.target);
        g_rl_shadow_debug.stage = 4u;
        /* README gives no hardware lower/upper limits; preview only. */
        float rl_torque_request[RL_MOTION31_ACTION_SIZE];
        float rl_torque_limited[RL_MOTION31_ACTION_SIZE];
        if (RLMotion31ComputePdTorque(&sample.motion, preview.target,
                                      rl_torque_request, rl_torque_limited) != 0) {
            g_rl_shadow_debug.failure = 8u;
            previous_shadow_target_valid = 0u;
            publish_debug(&sample, NULL);
            publish_command(&sample, NULL, NULL, NULL, 0);
            continue;
        }
        float rl_wheel_torque_forward_nm[2];
        if (RLMotion31MapPolicyWheelTorqueToForwardOrder(rl_torque_limited,
                                                        rl_wheel_torque_forward_nm) != 0) {
            g_rl_shadow_debug.failure = 9u;
            previous_shadow_target_valid = 0u;
            publish_debug(&sample, NULL);
            publish_command(&sample, NULL, NULL, NULL, 0);
            continue;
        }
        memcpy(previous_shadow_target, preview.target, sizeof(previous_shadow_target));
        previous_shadow_target_valid = 1u;
        publish_debug(&sample, rl_torque_limited);
        g_rl_shadow_debug.stage = 5u;
        g_rl_shadow_debug.failure = 0u;
        publish_command(&sample, action, &preview, rl_wheel_torque_forward_nm, 1);
    }
}
