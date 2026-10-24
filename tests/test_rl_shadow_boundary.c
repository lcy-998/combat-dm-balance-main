/* Execute the real RLShadowTask; only RTOS and the network runtime are mocked.
   This is a control-ownership boundary test, not an ST inference accuracy test. */
#include "rl_control.h"
#include "queue.h"
#include "tjb_network.h"
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

static float input[31],output[6];
static ai_buffer in={input},out={output};
static unsigned inference_calls,iterations,command_writes;
static TickType_t tick;
static RLShadowSnapshot snapshot;
static RLShadowCommand published;
static jmp_buf finished;
static QueueHandle_t snapshot_queue,command_queue;

QueueHandle_t xQueueCreateStatic(unsigned n,unsigned size,uint8_t *storage,StaticQueue_t *queue)
{assert(n==1);queue->size=size;queue->storage=storage;
 if (!snapshot_queue) snapshot_queue=queue;else command_queue=queue;return queue;}
int xQueuePeek(QueueHandle_t queue,void *dst,unsigned timeout)
{(void)timeout;if(queue==snapshot_queue){memcpy(dst,&snapshot,sizeof(snapshot));return pdPASS;}return 0;}
int xQueueOverwrite(QueueHandle_t queue,const void *value)
{if(queue==command_queue){memcpy(&published,value,sizeof(published));command_writes++;}return pdPASS;}
TickType_t xTaskGetTickCount(void){return tick;}
void vTaskDelayUntil(TickType_t *last,TickType_t period)
{
    assert(period==10);
    if (++iterations>3) longjmp(finished,1);
    tick=*last+period;*last=tick;
    memset(&snapshot,0,sizeof(snapshot));snapshot.sequence=iterations;snapshot.tick_ms=tick;
    snapshot.control_epoch=7;snapshot.chassis_ready=1;snapshot.rl_active=1;
    snapshot.feedback_valid_mask=0x3f;snapshot.imu_update_sequence=1;snapshot.imu_update_tick_ms=tick;
    for(unsigned j=0;j<6;++j)snapshot.feedback_receive_tick_ms[j]=tick;
    snapshot.motion.gravity_model[1]=-1;snapshot.motion.command_height_m=.34f;
    snapshot.motion.previous_target_valid=1;
    snapshot.previous_source=iterations==1?RL_PREVIOUS_INIT_PROXY:RL_PREVIOUS_ADOPTED;
    snapshot.adopted_sequence=iterations==1?0:99;
    /* Must use owner-supplied cache, never the last fake network output (.8). */
    snapshot.motion.previous_target[0]=.03f*(float)iterations;
}
void vTaskSuspend(void *task){(void)task;assert(!"unexpected task suspension");}
ai_error ai_tjb_network_create_and_init(ai_handle *handle,ai_handle *a,void *unused)
{(void)a;(void)unused;*handle=input;return (ai_error){0};}
ai_buffer *ai_tjb_network_inputs_get(ai_handle h,void *unused){(void)h;(void)unused;return &in;}
ai_buffer *ai_tjb_network_outputs_get(ai_handle h,void *unused){(void)h;(void)unused;return &out;}
int ai_tjb_network_run(ai_handle h,ai_buffer *a,ai_buffer *b)
{
    (void)h;(void)a;(void)b;
    if (inference_calls++==0) {
        const float golden[6]={.007079028f,.019301796f,.000087026f,-.003934855f,-.009344813f,-.012714249f};
        memcpy(output,golden,sizeof(output));return 1;
    }
    assert(fabsf(input[16]-(.03f*(float)iterations/1.5f))<1e-6f);
    assert(input[22]==0 && input[23]==0 && input[24]==.68f);
    for(unsigned i=0;i<6;++i)output[i]=.8f;
    return 1;
}
void getPhi(float a,float b,float c,float d,float e,float f,float g,float *p2,float *p3,float *length,float *theta)
{(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;(void)g;*p2=*p3=0;*length=.28f;*theta=1.5f;}
int main(void)
{
    RLShadowQueueInit();
    if (!setjmp(finished))RLShadowTask(NULL);
    assert(inference_calls==4 && command_writes==3 && published.valid);
    assert(published.control_epoch==7 && published.previous_sequence==99);
    assert(published.preview.target[0]==1.2f && published.preview.target[2]==80);
    assert(g_rl_shadow_debug.frame_begin==g_rl_shadow_debug.frame_end);
    assert(!(g_rl_shadow_debug.frame_begin&1u) && g_rl_shadow_debug.previous_target_is_virtual==0);
    puts("Real RLShadowTask: owner cache, target-only publication, atomic debug boundary passed");
    return 0;
}
