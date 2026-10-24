#ifndef RL_HOST_QUEUE_H
#define RL_HOST_QUEUE_H
#include "FreeRTOS.h"
typedef struct {unsigned size;void *storage;} StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;
QueueHandle_t xQueueCreateStatic(unsigned,unsigned,uint8_t *,StaticQueue_t *);
int xQueuePeek(QueueHandle_t,void *,unsigned);
int xQueueOverwrite(QueueHandle_t,const void *);
#endif
