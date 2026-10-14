#include "rl_shadow.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "network.h"
#include "network_data.h"
#include "stm32h7xx.h"

#include <math.h>
#include <string.h>

#define RL_SHADOW_PERIOD_TICKS pdMS_TO_TICKS(8u)
#define RL_SHADOW_MAX_AGE_TICKS pdMS_TO_TICKS(24u)

_Static_assert(AI_NETWORK_IN_1_SIZE == RL_SHADOW_OBSERVATION_SIZE, "policy input size mismatch");
_Static_assert(AI_NETWORK_OUT_1_SIZE == RL_SHADOW_ACTION_SIZE, "policy output size mismatch");

volatile RLShadowDebug g_rl_shadow_debug;

static StaticQueue_t snapshot_queue_control;
static uint8_t snapshot_queue_storage[sizeof(RLShadowSnapshot)];
static QueueHandle_t snapshot_queue;
static StaticQueue_t command_queue_control;
static uint8_t command_queue_storage[sizeof(RLShadowCommand)];
static QueueHandle_t command_queue;

AI_ALIGNED(32) static uint8_t network_activations[AI_NETWORK_DATA_ACTIVATION_1_SIZE];
static ai_handle network_handle = AI_HANDLE_NULL;
static ai_buffer *network_input;
static ai_buffer *network_output;

void RLShadowQueueInit(void)
{
    snapshot_queue = xQueueCreateStatic(1u, sizeof(RLShadowSnapshot),
                                        snapshot_queue_storage, &snapshot_queue_control);
    command_queue = xQueueCreateStatic(1u, sizeof(RLShadowCommand),
                                       command_queue_storage, &command_queue_control);
    configASSERT(snapshot_queue != NULL);
    configASSERT(command_queue != NULL);
}

void RLShadowPublish(const RLShadowSnapshot *sample)
{
    if (snapshot_queue != NULL && sample != NULL) {
        /* Length one means the 8 ms consumer always receives the latest 5 ms sample. */
        (void)xQueueOverwrite(snapshot_queue, sample);
    }
}

int RLShadowReadCommand(RLShadowCommand *command)
{
    return (command != NULL && command_queue != NULL &&
            xQueuePeek(command_queue, command, 0) == pdPASS) ? 0 : -1;
}

void RLShadowTaskCreateFailed(void)
{
    g_rl_shadow_debug.status = RL_SHADOW_TASK_CREATE_FAILED;
}

static int network_init(void)
{
    ai_handle activations[] = {network_activations};
    ai_error error = ai_network_create_and_init(&network_handle, activations, NULL);
    if (error.type != AI_ERROR_NONE) {
        return -1;
    }
    network_input = ai_network_inputs_get(network_handle, NULL);
    network_output = ai_network_outputs_get(network_handle, NULL);
    return (network_input != NULL && network_output != NULL &&
            network_input[0].data != AI_HANDLE_NULL &&
            network_output[0].data != AI_HANDLE_NULL) ? 0 : -1;
}

static int network_run(const float observation[RL_SHADOW_OBSERVATION_SIZE],
                       float action[RL_SHADOW_ACTION_SIZE])
{
    memcpy(network_input[0].data, observation, AI_NETWORK_IN_1_SIZE_BYTES);
    if (ai_network_run(network_handle, network_input, network_output) != 1) {
        return -1;
    }
    memcpy(action, network_output[0].data, AI_NETWORK_OUT_1_SIZE_BYTES);
    return 0;
}

static void publish_debug(const RLShadowSnapshot *sample, const float *observation,
                          const float *action, const RLShadowOutputPreview *preview,
                          int status, uint32_t missed)
{
    RLShadowCommand command = {0};
    command.output_tick_ms = xTaskGetTickCount();
    if (sample != NULL) {
        command.snapshot_sequence = sample->sequence;
        command.snapshot_tick_ms = sample->tick_ms;
    }
    if (status == RL_SHADOW_OK && sample != NULL && sample->chassis_ready &&
        action != NULL && preview != NULL) {
        command.valid = 1u;
        memcpy(command.raw_action, action, sizeof(command.raw_action));
        command.output = *preview;
    }
    (void)xQueueOverwrite(command_queue, &command);

    volatile RLShadowDebug *d = &g_rl_shadow_debug;
    d->publish_seq++;
    __DMB();
    d->status = status;
    d->missed_periods = missed;
    d->stack_free_min_words = uxTaskGetStackHighWaterMark(NULL);
    d->heap_free_min_bytes = xPortGetMinimumEverFreeHeapSize();
    d->inference_tick_ms = xTaskGetTickCount();
    if (sample != NULL) {
        d->snapshot_seq = sample->sequence;
        d->snapshot_tick_ms = sample->tick_ms;
        d->chassis_mode = sample->chassis_mode;
        d->chassis_ready = sample->chassis_ready;
        d->rl_active = sample->rl_active;
        d->rl_activation_sequence = sample->rl_activation_sequence;
        d->odom_s_m = sample->odom_s_m;
        d->target_s_m = sample->target_s_m;
        d->position_error_m = sample->target_s_m - sample->odom_s_m;
        d->remote_vx_m_s = sample->remote_vx_m_s;
        d->heading_vx_m_s = sample->heading_vx_m_s;
        d->lqr_speed_m_s = sample->lqr_speed_m_s;
        d->target_vx_m_s = sample->target_vx_m_s;
        d->lqr_wheel_torque_nm[0] = sample->lqr_torque_nm[0];
        d->lqr_wheel_torque_nm[1] = sample->lqr_torque_nm[1];
        for (unsigned i = 0; i < 4; ++i) {
            d->measured_joint_angle_rad[i] = sample->joint_angle_rad[i];
            d->lqr_torque_nm[i] = sample->lqr_torque_nm[i];
            d->motor_torque_nm[i] = sample->motor_torque_nm[i];
        }
        for (unsigned i = 0; i < 2; ++i) {
            d->wheel_current_cmd[i] = sample->wheel_current_cmd[i];
        }
    }
    if (observation != NULL) {
        for (unsigned i = 0; i < RL_SHADOW_OBSERVATION_SIZE; ++i) {
            d->observation[i] = observation[i];
        }
    } else {
        for (unsigned i = 0; i < RL_SHADOW_OBSERVATION_SIZE; ++i) {
            d->observation[i] = 0.0f;
        }
    }
    if (action != NULL) {
        for (unsigned i = 0; i < RL_SHADOW_ACTION_SIZE; ++i) {
            d->raw_action[i] = action[i];
        }
    } else {
        for (unsigned i = 0; i < RL_SHADOW_ACTION_SIZE; ++i) {
            d->raw_action[i] = 0.0f;
        }
    }
    if (preview != NULL) {
        d->leg_scale_rad_per_action = preview->leg_scale_rad_per_action;
        for (unsigned i = 0; i < 4; ++i) {
            d->policy_joint_target_angle_rad[i] = preview->joint_target_angle_rad[i];
        }
        for (unsigned i = 0; i < 2; ++i) {
            d->policy_wheel_torque_nm[i] = preview->wheel_forward_torque_nm[i];
        }
    } else {
        d->leg_scale_rad_per_action = 0.0f;
        for (unsigned i = 0; i < 4; ++i) {
            d->policy_joint_target_angle_rad[i] = 0.0f;
        }
        for (unsigned i = 0; i < 2; ++i) {
            d->policy_wheel_torque_nm[i] = 0.0f;
        }
    }
    __DMB();
    d->publish_seq++;
}

void RLShadowTask(void const *argument)
{
    (void)argument;
    RLShadowSnapshot sample;
    float observation[RL_SHADOW_OBSERVATION_SIZE];
    float action[RL_SHADOW_ACTION_SIZE];
    RLShadowOutputPreview preview;
    RLShadowPositionTarget position_target = {0};
    uint32_t last_sequence = 0;
    uint32_t missed = 0;
    TickType_t wake = xTaskGetTickCount();
    int ai_ready = (network_init() == 0);

    configASSERT(snapshot_queue != NULL);
    for (;;) {
        vTaskDelayUntil(&wake, RL_SHADOW_PERIOD_TICKS);
        TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - wake) >= RL_SHADOW_PERIOD_TICKS) {
            missed += (uint32_t)((now - wake) / RL_SHADOW_PERIOD_TICKS);
            wake = now;
        }
        if (xQueuePeek(snapshot_queue, &sample, 0) != pdPASS) {
            publish_debug(NULL, NULL, NULL, NULL, RL_SHADOW_NO_SAMPLE, missed);
            continue;
        }
        if (sample.sequence == last_sequence ||
            (TickType_t)(now - sample.tick_ms) > RL_SHADOW_MAX_AGE_TICKS) {
            publish_debug(&sample, NULL, NULL, NULL, RL_SHADOW_STALE_SAMPLE, missed);
            continue;
        }
        last_sequence = sample.sequence;
        float target_s_m;
        if (RLShadowUpdatePositionTarget(&position_target, &sample, now,
                                         &target_s_m) != 0) {
            publish_debug(&sample, NULL, NULL, NULL, RL_SHADOW_INVALID_INPUT, missed);
            continue;
        }
        sample.target_s_m = target_s_m;
        float target_vx_m_s;
        if (RLShadowPositionPID(sample.odom_s_m, sample.target_s_m,
                                &target_vx_m_s) != 0) {
            publish_debug(&sample, NULL, NULL, NULL, RL_SHADOW_INVALID_INPUT, missed);
            continue;
        }
        sample.target_vx_m_s = target_vx_m_s;
        if (RLShadowBuildObservation(&sample, observation) != 0) {
            publish_debug(&sample, NULL, NULL, NULL, RL_SHADOW_INVALID_INPUT, missed);
            continue;
        }
        if (!ai_ready) {
            publish_debug(&sample, observation, NULL, NULL, RL_SHADOW_AI_INIT_FAILED, missed);
            continue;
        }
        if (network_run(observation, action) != 0) {
            publish_debug(&sample, observation, NULL, NULL, RL_SHADOW_AI_RUN_FAILED, missed);
            continue;
        }
        int valid_output = 1;
        for (unsigned i = 0; i < RL_SHADOW_ACTION_SIZE; ++i) {
            if (!isfinite(action[i])) {
                valid_output = 0;
                break;
            }
        }
        if (valid_output && RLShadowConvertOutput(sample.target_vx_m_s, action, &preview) != 0) {
            valid_output = 0;
        }
        publish_debug(&sample, observation, valid_output ? action : NULL,
                      valid_output ? &preview : NULL,
                      valid_output ? RL_SHADOW_OK : RL_SHADOW_INVALID_OUTPUT, missed);
    }
}
