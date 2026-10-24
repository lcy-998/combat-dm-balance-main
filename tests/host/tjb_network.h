#ifndef RL_HOST_NETWORK_H
#define RL_HOST_NETWORK_H
#include <stdint.h>
typedef void *ai_handle;
typedef struct {void *data;} ai_buffer;
typedef struct {int type;} ai_error;
#define AI_HANDLE_NULL ((void *)0)
#define AI_ERROR_NONE 0
#define AI_ALIGNED(n) __attribute__((aligned(n)))
#define AI_TJB_NETWORK_IN_1_SIZE 31u
#define AI_TJB_NETWORK_OUT_1_SIZE 6u
#define AI_TJB_NETWORK_IN_1_SIZE_BYTES (31u*sizeof(float))
#define AI_TJB_NETWORK_OUT_1_SIZE_BYTES (6u*sizeof(float))
ai_error ai_tjb_network_create_and_init(ai_handle *,ai_handle *,void *);
ai_buffer *ai_tjb_network_inputs_get(ai_handle,void *);
ai_buffer *ai_tjb_network_outputs_get(ai_handle,void *);
int ai_tjb_network_run(ai_handle,ai_buffer *,ai_buffer *);
#endif
