#include "rl_shadow.h"
#include "rl_control.h"
#include "recovery2100/recovery_actor.h"

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
static RLShadowDebug working_debug;
static uint32_t debug_generation;
static uint32_t actor_input_tick;
static uint8_t inference_pending_timing;

static void commit_debug(void)
{
    const uint32_t generation=(debug_generation+=2u);
    g_rl_shadow_debug.publish_seq=generation|1u;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    working_debug.publish_seq=generation|1u;
    working_debug.frame_end=generation;
    g_rl_shadow_debug=working_debug;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    g_rl_shadow_debug.publish_seq=generation;
}
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

static int network_run(const float observation[31], float action[6]);

static void publish_command(const RLShadowSnapshot *sample,
                            const float *action,
                            const RLMotion31Preview *preview,
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
        action != NULL && preview != NULL) {
        command.valid = 1u;
        memcpy(command.action, action, sizeof(command.action));
        command.preview = *preview;
        command.control_epoch=sample->control_epoch;
        command.previous_source=sample->previous_source;
        command.previous_sequence=sample->adopted_sequence;
        command.recovery_mode=sample->motion.recovery_mode;
        command.actor_input_tick_ms=sample->recovery_session ? actor_input_tick : sample->tick_ms;
        if(sample->motion_probe) {
            RLMotion31Snapshot motion=sample->motion;
            motion.recovery_mode=0u;
            if(RLMotion31BuildObservation(&motion,command.motion_preview.observation)==0 &&
               network_run(command.motion_preview.observation,command.motion_action)==0 &&
               RLMotion31BuildPreview(&motion,command.motion_action,&command.motion_preview)==0)
                command.motion_valid=1u;
        }
    }
    command.output_tick_ms=xTaskGetTickCount();
    if(sample && (uint32_t)(command.output_tick_ms-sample->tick_ms)>RL_CONTROL_COMMAND_MAX_AGE_MS)
        command.valid=command.motion_valid=0u;
    if(inference_pending_timing) {
        inference_pending_timing=0u;
        working_debug.inference_end_tick_ms=command.output_tick_ms;
        if((uint32_t)(command.output_tick_ms-working_debug.inference_start_tick_ms)>10u)
            working_debug.inference_deadline_miss_count++;
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

static void publish_debug(const RLShadowSnapshot *sample)
{
    volatile RLShadowDebug *d = &working_debug;
    const int lqr_valid = sample != NULL && sample->motion.lqr_torque_valid;
    float rl_leg_torque[4] = {0};
    float rl_wheel_torque[2] = {0};
    if (sample != NULL) {
        d->snapshot_seq = sample->sequence;
        d->rl_active = sample->rl_active;
        /* Snapshot wheel velocities use model signs: left raw, right inverted. */
        d->rl_wheel_current_rad_s[0] = sample->motion.joint_velocity_rad_s[5];
        d->rl_wheel_current_rad_s[1] = -sample->motion.joint_velocity_rad_s[2];
    } else {
        d->snapshot_seq = 0u;
        d->rl_active = 0u;
        d->rl_wheel_current_rad_s[0] = 0.0f;
        d->rl_wheel_current_rad_s[1] = 0.0f;
        memset((void *)d->observation, 0, sizeof(d->observation));
    }
    if (sample != NULL) {
        d->recovery_mode=sample->motion.recovery_mode;
        d->handoff_source=sample->handoff_source;
        d->handoff_requested=sample->handoff_requested;
        d->handoff_committed=sample->handoff_committed;
        d->controller_owner=sample->controller_owner;
        d->control_epoch=sample->control_epoch;
        d->adopted_sequence=sample->adopted_sequence;
        d->previous_source=sample->previous_source;
        if (sample->controller_owner==RL_OWNER_RL) {
            memcpy(rl_leg_torque,sample->applied_leg_torque_motor_nm,sizeof(rl_leg_torque));
            rl_wheel_torque[0]=-sample->applied_wheel_current_raw[0]/RL_MOTION31_WHEEL_CURRENT_PER_NM;
            rl_wheel_torque[1]= sample->applied_wheel_current_raw[1]/RL_MOTION31_WHEEL_CURRENT_PER_NM;
        }
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

/* Diagnostic equivalent of the motor-output torque calculation. */
static unsigned publish_preview_torque(const RLShadowSnapshot *sample,
                                       const float target[RL_MOTION31_ACTION_SIZE])
{
    float requested[RL_MOTION31_ACTION_SIZE], limited[RL_MOTION31_ACTION_SIZE];
    float motor[4], wheel[2], final_motor[4];
    if (RLMotion31ComputePdTorque(&sample->motion, target, requested, limited) != 0)
        return 8u;
    if (RLMotion31MapPolicyLegTorqueToMotorOrder(limited, motor) != 0 ||
        RLMotion31MapPolicyWheelTorqueToForwardOrder(limited, wheel) != 0 ||
        RLControlCompensateSpring(motor, sample->leg_length_m,
                                  sample->leg_spread_rad, final_motor) != 0)
        return 9u;
    memcpy(working_debug.rl_leg_torque_nm, final_motor, sizeof(final_motor));
    const float wheel_limit = RL_CONTROL_WHEEL_RAW_LIMIT / RL_MOTION31_WHEEL_CURRENT_PER_NM;
    for (unsigned i = 0; i < 2u; ++i)
        working_debug.rl_wheel_torque_nm[i] = fminf(fmaxf(wheel[i], -wheel_limit), wheel_limit);
    working_debug.rl_torque_valid = 1u;
    return 0u;
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
    working_debug.rl_target_theta_l_rad = theta_l_target;
    working_debug.rl_target_theta_r_rad = theta_r_target;
    working_debug.rl_target_theta_valid = 1u;
}

void RLShadowTask(void const *argument)
{
    (void)argument;
    uint8_t network_ready = 0u;
    static RLMotion31Snapshot recovery_packets[3];
    static uint32_t recovery_ticks[3];
    unsigned cursor=0u;uint8_t packet_initialized=0u,packet_mode=0u;
    uint32_t packet_epoch=0u;
    if (snapshot_queue == NULL) {
        working_debug.failure = 1u;
        publish_debug(NULL);
        publish_command(NULL, NULL, NULL, 0);
        commit_debug();
        vTaskSuspend(NULL);
    }
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, RL_SHADOW_PERIOD);
        inference_pending_timing=0u;
        working_debug.rl_torque_valid = 0u;
        working_debug.observation_valid = 0u;
        working_debug.input_error_mask = 0u;
        working_debug.rl_wheel_target_valid = 0u;
        working_debug.rl_target_valid = 0u;
        working_debug.rl_target_theta_valid = 0u;
        RLShadowSnapshot sample;
        if (xQueuePeek(snapshot_queue, &sample, 0u) != pdPASS) {
            working_debug.stage = 0u;
            working_debug.failure = 2u;
            publish_debug(NULL);
            publish_command(NULL, NULL, NULL, 0);
            commit_debug();
            continue;
        }
        working_debug.snapshot_seq = sample.sequence;
        TickType_t now = xTaskGetTickCount();
        working_debug.snapshot_age_ms = (uint32_t)(now - sample.tick_ms);
        for (size_t i = 0; i < 4u; ++i) {
            working_debug.input_joint_position_rad[i] = sample.motion.joint_position_rad[i];
        }
        if ((TickType_t)(now - sample.tick_ms) > RL_SHADOW_MAX_AGE ||
            !RLControlFeedbackFresh(&sample,now)) {
            working_debug.stage = 0u;
            working_debug.failure = 3u;
            publish_debug(&sample);
            publish_command(&sample, NULL, NULL, 0);
            commit_debug();
            continue;
        }
        if(sample.recovery_session) {
            const int handoff_epoch=packet_initialized && packet_mode &&
                !sample.motion.recovery_mode && sample.handoff_committed &&
                sample.control_epoch==packet_epoch+1u;
            if(!packet_initialized || (sample.control_epoch!=packet_epoch && !handoff_epoch)) {
                for(unsigned i=0;i<3u;i++){recovery_packets[i]=sample.motion;recovery_ticks[i]=sample.tick_ms;}
                cursor=0u;packet_initialized=1u;
            }
            packet_epoch=sample.control_epoch;packet_mode=sample.motion.recovery_mode;
            recovery_packets[cursor%3u]=sample.motion;recovery_ticks[cursor%3u]=sample.tick_ms;
            RLMotion31Snapshot received=recovery_packets[(cursor+1u)%3u];
            /* The received state is old; previous is current adopted physical target.
               Command/mode metadata are current and never taken from shadow output. */
            memcpy(received.previous_target,sample.motion.previous_target,sizeof(received.previous_target));
            received.previous_target_valid=sample.motion.previous_target_valid;
            received.recovery_mode=sample.motion.recovery_mode;
            received.command_vx_m_s=sample.motion.command_vx_m_s;
            received.command_yaw_rad_s=sample.motion.command_yaw_rad_s;
            received.command_height_m=sample.motion.command_height_m;
            sample.motion=received;
            actor_input_tick=recovery_ticks[(cursor+1u)%3u];cursor++;
            /* tick_ms retains latest snapshot freshness; input age is separate below. */
        } else packet_initialized=0u;
        working_debug.stage = 1u;
        working_debug.previous_target_valid = sample.motion.previous_target_valid;
        working_debug.previous_target_is_virtual = (sample.previous_source != RL_PREVIOUS_ADOPTED);
        RLMotion31Preview preview;
        float action[RL_MOTION31_ACTION_SIZE];
        if (RLMotion31BuildObservation(&sample.motion, preview.observation) != 0) {
            working_debug.failure = 4u;
            working_debug.input_error_mask = input_error_mask(&sample.motion);
            publish_debug(&sample);
            publish_command(&sample, NULL, NULL, 0);
            commit_debug();
            continue;
        }
        for (size_t i = 0; i < RL_MOTION31_OBSERVATION_SIZE; ++i) {
            working_debug.observation[i] = preview.observation[i];
        }
        working_debug.observation_valid = 1u;
        working_debug.stage = 2u;
        if (!sample.rl_active) {
            working_debug.stage = 2u;
            publish_debug(&sample);
            publish_command(&sample, NULL, NULL, 0);
            commit_debug();
            continue;
        }
        if (!network_ready) {
            if (network_init() != 0) {
                working_debug.failure = 1u;
                publish_debug(&sample);
                publish_command(&sample, NULL, NULL, 0);
                commit_debug();
                vTaskSuspend(NULL);
            }
            network_ready = 1u;
        }
        inference_pending_timing=1u;
        working_debug.inference_start_tick_ms=xTaskGetTickCount();
        const int inference_status=sample.motion.recovery_mode ?
            RecoveryActorMean(preview.observation,action) : network_run(preview.observation, action);
        working_debug.inference_end_tick_ms=xTaskGetTickCount();
        if (inference_status != 0 ||
            (uint32_t)(working_debug.inference_end_tick_ms-sample.tick_ms)>RL_CONTROL_COMMAND_MAX_AGE_MS) {
            working_debug.failure = 5u;
            publish_debug(&sample);
            publish_command(&sample, NULL, NULL, 0);
            commit_debug();
            continue;
        }
        for (size_t i = 0; i < RL_MOTION31_ACTION_SIZE; ++i) {
            working_debug.raw_action[i] = action[i];
        }
        working_debug.stage = 3u;
        if ((sample.motion.recovery_mode ? RLMotion31DecodeRecoveryMean(action,preview.target) :
             RLMotion31DecodeOutput(action, sample.motion.command_height_m,preview.target)) != 0) {
            working_debug.failure = 6u;
            publish_debug(&sample);
            publish_command(&sample, NULL, NULL, 0);
            commit_debug();
            continue;
        }
        for (size_t i = 0; i < RL_MOTION31_ACTION_SIZE; ++i) {
            working_debug.decoded_target[i] = preview.target[i];
        }
        if (RLMotion31MapPolicyTargets(preview.target,
                                       preview.hardware_joint_target_rad,
                                       preview.motor_wheel_target_rad_s) != 0) {
            working_debug.failure = 7u;
            publish_debug(&sample);
            publish_command(&sample, NULL, NULL, 0);
            commit_debug();
            continue;
        }
        working_debug.rl_wheel_target_rad_s[0] = preview.motor_wheel_target_rad_s[0];
        working_debug.rl_wheel_target_rad_s[1] = preview.motor_wheel_target_rad_s[1];
        working_debug.rl_wheel_target_valid = 1u;
        working_debug.rl_target_valid = 1u;
        publish_target_swing(&sample, preview.target);
        working_debug.stage = 4u;
        /* Publish a valid policy target for the next motor-control update. */
        publish_debug(&sample);
        working_debug.failure = sample.recovery_session ? 0u : publish_preview_torque(&sample, preview.target);
        if(sample.recovery_session) working_debug.rl_torque_valid=1u; /* PD belongs to chassis owner */
        working_debug.stage = working_debug.rl_torque_valid ? 5u : 4u;
        publish_command(&sample, action, &preview, working_debug.rl_torque_valid);
        commit_debug();
    }
}
