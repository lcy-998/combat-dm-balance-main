#ifndef RL_HOST_TASK_H
#define RL_HOST_TASK_H
#include "FreeRTOS.h"
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *,TickType_t);
void vTaskSuspend(void *);
#endif
