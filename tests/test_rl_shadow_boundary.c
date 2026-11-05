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
static RLControlState controller;
static jmp_buf finished;
static QueueHandle_t snapshot_queue,command_queue;

QueueHandle_t xQueueCreateStatic(unsigned n,unsigned size,uint8_t *storage,StaticQueue_t *queue)
{assert(n==1);queue->size=size;queue->storage=storage;
 if (!snapshot_queue) snapshot_queue=queue;else command_queue=queue;return queue;}
int xQueuePeek(QueueHandle_t queue,void *dst,unsigned timeout)
{(void)timeout;if(queue==snapshot_queue){memcpy(dst,&snapshot,sizeof(snapshot));return pdPASS;}return 0;}
int xQueueOverwrite(QueueHandle_t queue,const void *value)
{
    if(queue==command_queue) {
        memcpy(&published,value,sizeof(published));command_writes++;
        assert(published.valid==(iterations==2 || iterations==5 || iterations==6));
        if (published.valid) {
            if (iterations==2 || controller.epoch!=snapshot.control_epoch) {
                memset(&controller,0,sizeof(controller));
                controller.ramp_completed=1;controller.timing=1;
                controller.wait_start_tick=tick-5000u;
                controller.epoch=snapshot.control_epoch;
            }
            assert(published.previous_source==RL_PREVIOUS_INFERENCE);
            assert(published.previous_sequence==0u);
            RLShadowSnapshot current=snapshot;
            RLControlFillPrevious(&controller,&current);
            RLControlFrame frame;
            assert(RLControlExecute(&controller,tick,1,&current,&published,&frame)==0);
            assert(frame.owner==RL_OWNER_RL && controller.active);
            assert(controller.adopted_sequence==snapshot.sequence);
        }
    }
    return pdPASS;
}
TickType_t xTaskGetTickCount(void){return tick;}
void vTaskDelayUntil(TickType_t *last,TickType_t period)
{
    assert(period==10);
    if (++iterations>6) longjmp(finished,1);
    if (iterations==2) assert(inference_calls==0 && !g_rl_shadow_debug.rl_torque_valid);
    if (iterations==3) {
        const float spring=16.5f*sinf(.5f);
        assert(g_rl_shadow_debug.rl_torque_valid && g_rl_shadow_debug.stage==5);
        assert(fabsf(g_rl_shadow_debug.rl_leg_torque_nm[0]-(-32.0f-spring))<1e-4f);
        assert(g_rl_shadow_debug.controller_owner==RL_OWNER_LQR);
    }
    if (iterations==4 || iterations==5) {
        assert(!g_rl_shadow_debug.rl_torque_valid);
        if (iterations==4) assert(g_rl_shadow_debug.failure==9u);
        for (unsigned i=0;i<4;++i) assert(g_rl_shadow_debug.rl_leg_torque_nm[i]==0);
        for (unsigned i=0;i<2;++i) assert(g_rl_shadow_debug.rl_wheel_torque_nm[i]==0);
    }
    if (iterations>1) assert(published.valid==(iterations==3 || iterations==6));
    tick=*last+period;*last=tick;
    memset(&snapshot,0,sizeof(snapshot));snapshot.sequence=iterations;snapshot.tick_ms=tick;
    snapshot.control_epoch=7;snapshot.chassis_ready=1;snapshot.rl_active=(iterations!=1 && iterations!=4);
    if (iterations==6) snapshot.control_epoch=8;
    snapshot.feedback_valid_mask=0x3f;snapshot.imu_update_sequence=1;snapshot.imu_update_tick_ms=tick;
    for(unsigned j=0;j<6;++j)snapshot.feedback_receive_tick_ms[j]=tick;
    snapshot.motion.gravity_model[1]=-1;snapshot.motion.command_height_m=.34f;
    if (iterations==2) {
        snapshot.motion.joint_velocity_rad_s[0]=1.0f;
        snapshot.motion.joint_velocity_rad_s[1]=-2.0f;
        snapshot.motion.joint_velocity_rad_s[3]=3.0f;
        snapshot.motion.joint_velocity_rad_s[4]=-4.0f;
    }
    snapshot.motion.previous_target_valid=1;
    snapshot.leg_length_m[0]=snapshot.leg_length_m[1]=iterations==3?0.0f:.28f;
    snapshot.leg_spread_rad[0]=snapshot.leg_spread_rad[1]=.5f;
    snapshot.previous_source=iterations==1?RL_PREVIOUS_INIT_PROXY:RL_PREVIOUS_ADOPTED;
    snapshot.adopted_sequence=iterations==1?0:99;
    /* Deliberately different from policy history; this cache must be ignored. */
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
    inference_calls++;
    if (iterations==2) {
        assert(input[10]==0.0f);
        assert(input[11]==0.0f);
        assert(input[13]==0.0f);
        assert(input[14]==0.0f);
    }
    const float previous = iterations==3 ? .4f : 0.0f;
    for (unsigned i=16;i<22;++i) assert(fabsf(input[i]-previous)<1e-6f);
    assert(input[22]==0 && input[23]==0 && input[24]==.68f);
    for(unsigned i=0;i<6;++i)output[i]=fminf(1.0f,.2f*(float)iterations);
    return 1;
}
void getPhi(float a,float b,float c,float d,float e,float f,float g,float *p2,float *p3,float *length,float *theta)
{(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;(void)g;*p2=*p3=0;*length=.28f;*theta=1.5f;}
int main(void)
{
    RLShadowQueueInit();
    if (!setjmp(finished))RLShadowTask(NULL);
    assert(RL_CONTROL_OUTPUT_ENABLED==1u);
    assert(inference_calls==4 && command_writes==6 && published.valid);
    assert(g_rl_shadow_debug.rl_target_valid);
    assert(g_rl_shadow_debug.rl_torque_valid && g_rl_shadow_debug.failure==0);
    const float spring=16.5f*sinf(.5f);
    const float expected[4]={-40.0f,-40.0f+spring,40.0f,-40.0f};
    for (unsigned i=0;i<4;++i)
        assert(fabsf(g_rl_shadow_debug.rl_leg_torque_nm[i]-expected[i])<1e-4f);
    for (unsigned i=0;i<2;++i)
        assert(fabsf(g_rl_shadow_debug.rl_wheel_torque_nm[i]+
                     RL_CONTROL_WHEEL_RAW_LIMIT/RL_MOTION31_WHEEL_CURRENT_PER_NM)<1e-5f);
    assert(g_rl_shadow_debug.controller_owner==RL_OWNER_LQR);
    assert(g_rl_shadow_debug.decoded_target[0]==1.5f);
    assert(g_rl_shadow_debug.decoded_target[2]==100.0f);
    assert(g_rl_shadow_debug.frame_begin==g_rl_shadow_debug.frame_end);
    assert(!(g_rl_shadow_debug.frame_begin&1u));
    assert(g_rl_shadow_debug.previous_source==RL_PREVIOUS_INFERENCE);
    assert(g_rl_shadow_debug.previous_target_is_virtual==1);
    puts("Real RLShadowTask: delayed inference, previous inference feedback, torque diagnostics and immediate RL adoption passed");
    return 0;
}
