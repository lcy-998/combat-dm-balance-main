#ifndef RL_HOST_TASK_H
#define RL_HOST_TASK_H
#include "FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
uint32_t ulTaskNotifyTake(int, TickType_t);
void xTaskNotifyGive(TaskHandle_t);
void vTaskSuspend(void *);
#endif
