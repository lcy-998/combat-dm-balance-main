/* Execute production RLShadowTask; only RTOS and AI inference are mocked. */
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
static TickType_t tick;
static unsigned iterations,inferences,writes,scenario;
static RLShadowSnapshot snapshot;
static RLShadowCommand published;
static QueueHandle_t snapshots,commands;
static jmp_buf done;

QueueHandle_t xQueueCreateStatic(unsigned n,unsigned size,uint8_t *storage,StaticQueue_t *q)
{ assert(n==1); q->size=size; q->storage=storage;
  if(!snapshots)snapshots=q; else commands=q; return q; }
int xQueuePeek(QueueHandle_t q,void *dst,unsigned timeout)
{ (void)timeout; if(q==snapshots){memcpy(dst,&snapshot,sizeof(snapshot));return pdPASS;}return 0; }
int xQueueOverwrite(QueueHandle_t q,const void *v)
{ if(q==commands){memcpy(&published,v,sizeof(published));++writes;}return pdPASS; }
TickType_t xTaskGetTickCount(void){return tick;}
void vTaskDelayUntil(TickType_t *last,TickType_t period)
{
    assert(period==10);
    if(++iterations>3)longjmp(done,1);
    tick=*last+period; *last=tick;
    memset(&snapshot,0,sizeof(snapshot));
    snapshot.sequence=100u+iterations; snapshot.tick_ms=tick; snapshot.chassis_ready=1;
    snapshot.rl_active=scenario!=1; snapshot.control_epoch=7;
    snapshot.controller_owner=iterations>1 && !scenario?RL_OWNER_RL:RL_OWNER_LQR;
    snapshot.previous_source=iterations==1?RL_PREVIOUS_INIT_PROXY:RL_PREVIOUS_ADOPTED;
    snapshot.adopted_sequence=iterations==1?0:99;
    snapshot.motion.gravity_model[1]=-1; snapshot.motion.command_height_m=.34f;
    snapshot.motion.previous_target_valid=1;
    snapshot.feedback_valid_mask=0x3f; snapshot.imu_update_sequence=1;
    snapshot.imu_update_tick_ms=tick;
    snapshot.leg_length_m[0]=snapshot.leg_length_m[1]=.28f;
    snapshot.leg_spread_rad[0]=snapshot.leg_spread_rad[1]=.5f;
    for(unsigned i=0;i<6;++i){
        snapshot.feedback_receive_tick_ms[i]=tick;
        snapshot.motion.previous_target[i]=.03f*(float)(i+iterations);
        snapshot.motion.joint_velocity_rad_s[i]=(float)i-3;
    }
    if(scenario==2)snapshot.tick_ms=tick-25u;
    if(scenario==3)snapshot.motion.gravity_model[1]=100;
}
void vTaskSuspend(void *task){(void)task;assert(!"unexpected suspension");}
ai_error ai_tjb_network_create_and_init(ai_handle *h,ai_handle *a,void *u)
{(void)a;(void)u;*h=input;return (ai_error){0};}
ai_buffer *ai_tjb_network_inputs_get(ai_handle h,void *u){(void)h;(void)u;return &in;}
ai_buffer *ai_tjb_network_outputs_get(ai_handle h,void *u){(void)h;(void)u;return &out;}
int ai_tjb_network_run(ai_handle h,ai_buffer *a,ai_buffer *b)
{
    (void)h;(void)a;(void)b; ++inferences;
    const unsigned legs[4]={0,1,3,4};
    for(unsigned i=0;i<6;++i)assert(fabsf(input[10+i]-.05f*snapshot.motion.joint_velocity_rad_s[i])<1e-6f);
    for(unsigned i=0;i<4;++i)assert(fabsf(input[16+i]-snapshot.motion.previous_target[legs[i]]/1.5f)<1e-6f);
    assert(fabsf(input[20]-snapshot.motion.previous_target[2]/100)<1e-6f);
    assert(fabsf(input[21]-snapshot.motion.previous_target[5]/100)<1e-6f);
    for(unsigned i=0;i<6;++i)output[i]=.8f;
    return 1;
}
void getPhi(float a,float b,float c,float d,float e,float f,float g,float *p2,float *p3,float *length,float *theta)
{(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;(void)g;*p2=*p3=0;*length=.28f;*theta=1.5f;}
int main(int argc,char **argv)
{
    if(argc==2)scenario=(unsigned)(argv[1][0]-'0');
    RLShadowQueueInit(); if(!setjmp(done))RLShadowTask(NULL);
    assert(writes==3);
    if(!scenario){
        assert(inferences==3 && published.valid);
        assert(published.previous_source==RL_PREVIOUS_ADOPTED && published.previous_sequence==99);
        assert(fabsf(published.preview.target[0]-(-.02911608294f+.12f))<1e-6f);
        assert(published.preview.target[2]==64);
    }else assert(!inferences && !published.valid);
    assert(g_rl_shadow_debug.controller_owner==(!scenario?RL_OWNER_RL:RL_OWNER_LQR));
    assert(g_rl_shadow_debug.frame_begin==g_rl_shadow_debug.frame_end);
    assert(!(g_rl_shadow_debug.frame_begin&1));
    puts("PASS supported015 real task: adopted cache, real dq and invalid/inactive boundaries");
    return 0;
}
