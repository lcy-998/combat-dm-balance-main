/* Production control/decoder/gate, synthetic feedback only; no RTOS or motors. */
#include "rl_control.h"
#include "rl_recovery_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#ifdef RL_RECOVERY_SHADOW_HOST
#include "host/queue.h"
#include "host/tjb_network.h"
#include "recovery2100/recovery_actor.h"
#include <setjmp.h>
#endif

static unsigned failures;
#define CHECK(x) do { if (!(x)) { \
    fprintf(stderr, "FAIL %s:%d: %s\n", __func__, __LINE__, #x); \
    ++failures; return; } } while (0)
static int near(float a, float b) { return fabsf(a-b)<2e-5f; }
static const unsigned legs[4]={0,1,3,4};

static RLHandoffSample quality(uint32_t now)
{
    RLHandoffSample q={.tick_ms=now,.valid=1,.height_m=.34f,
        .normal_force_N={20,21}};
    return q;
}

static RLShadowSnapshot feedback(const RLControlState *state, uint32_t now)
{
    RLShadowSnapshot x={0};
    x.tick_ms=now; x.sequence=now+1u; x.chassis_ready=x.rl_active=1;
    x.leg_length_m[0]=.28f; x.leg_length_m[1]=.31f;
    x.leg_spread_rad[0]=.8f; x.leg_spread_rad[1]=-.6f;
    x.motion.gravity_model[1]=-1; x.motion.command_height_m=.34f;
    x.feedback_valid_mask=0x3f; x.imu_update_sequence=1;
    x.imu_update_tick_ms=now;
    for(unsigned i=0;i<6;i++)x.feedback_receive_tick_ms[i]=now;
    /* UNKNOWN external support evidence unless the test qualifies it. */
    RLControlFillPrevious(state,&x);
    return x;
}

static RLShadowCommand packet(const RLControlState *state,uint32_t now,
                              uint32_t sequence,const float action[6],int probe)
{
    RLShadowSnapshot x=feedback(state,now);
    RLShadowCommand c={.valid=1,.snapshot_sequence=sequence,
        .snapshot_tick_ms=now,.output_tick_ms=now,.actor_input_tick_ms=now,
        .control_epoch=x.control_epoch,.previous_source=x.previous_source,
        .previous_sequence=x.adopted_sequence,.recovery_mode=x.motion.recovery_mode};
    if(action)memcpy(c.action,action,sizeof(c.action));
    assert(!RLMotion31BuildPreview(&x.motion,c.action,&c.preview));
    if(probe){
        x.motion.recovery_mode=0;
        /* A genuinely different Motion candidate, within the jump envelope. */
        c.motion_action[0]=.1f; c.motion_action[2]=.05f;
        assert(!RLMotion31BuildPreview(&x.motion,c.motion_action,&c.motion_preview));
        c.motion_valid=1;
    }
    return c;
}

static int step(RLControlState *state,uint32_t now,const RLShadowCommand *c,
                int qualified,RLControlFrame *f)
{
    RLShadowSnapshot x=feedback(state,now);
    if(qualified)x.handoff=quality(now);
    return RLControlExecute(state,now,1,&x,c,f);
}

static int zero_output(const RLControlFrame *f)
{
    for(unsigned i=0;i<4;i++)if(f->final_motor_nm[i]!=0)return 0;
    return f->wheel_current_raw[0]==0 && f->wheel_current_raw[1]==0;
}

/* Compare to the ordinary015 output, not PD+twice the diagnostic term. */
static void check_spring(const RLShadowSnapshot *x,const RLControlFrame *f,int valid)
{
    if(!valid){
        for(unsigned j=0;j<4;j++)CHECK(f->spring_motor_nm[j]==0 && f->final_motor_nm[j]==0);
        return;
    }
    float motor[4],expected[4],spring[4],zero[4]={0};
    CHECK(!RLMotion31MapPolicyLegTorqueToMotorOrder(f->tau_limited_nm,motor));
    CHECK(!RLControlCompensateSpring(motor,x->leg_length_m,x->leg_spread_rad,expected));
    CHECK(!RLControlCompensateSpring(zero,x->leg_length_m,x->leg_spread_rad,spring));
    for(unsigned j=0;j<4;j++){
        CHECK(near(f->spring_motor_nm[j],spring[j]));
        CHECK(near(f->final_motor_nm[j],expected[j]));
        CHECK(fabsf(f->final_motor_nm[j])<=40);
    }
}

static void test_explicit_start_and_contracts(void)
{
    RLControlState s={0};
    CHECK(RLControlStartRecovery(NULL)<0);
    for(unsigned mode=0;mode<3;mode++){
        RLControlState busy={0};
        if(mode==0)busy.active=1;
        if(mode==1)busy.timing=1;
        if(mode==2)busy.fault=RL_FAULT_INPUT;
        const RLControlState before=busy;
        CHECK(RLControlStartRecovery(&busy)<0);
        CHECK(!memcmp(&busy,&before,sizeof(busy)));
    }
    CHECK(!RLControlStartRecovery(&s));
    CHECK(s.recovery_session && s.recovering && s.ready && s.epoch==1);
    CHECK(near(RLControlLegRampStep(&s,8000,1,.19f),.19f));
    RLShadowSnapshot x=feedback(&s,100);
    CHECK(x.recovery_session && x.motion.recovery_mode && !x.motion_probe);
    const float a[6]={1,-1,.25f,-1,1,-.25f};
    RLMotion31Preview recovery,motion;
    CHECK(!RLMotion31BuildPreview(&x.motion,a,&recovery));
    x.motion.recovery_mode=0;
    CHECK(!RLMotion31BuildPreview(&x.motion,a,&motion));
    CHECK(recovery.observation[25]==0 && recovery.observation[26]==1);
    CHECK(motion.observation[25]==1 && motion.observation[26]==0);
    for(unsigned i=0;i<25;i++)CHECK(near(recovery.observation[i],motion.observation[i]));
    const float ends[4]={1.5f,-1.5f,-1.5f,1.5f};
    const float centers[4]={-.1309740543f,-.1511767805f,.1309740543f,-.1511767805f};
    for(unsigned j=0;j<4;j++){
        CHECK(near(recovery.target[legs[j]],ends[j]));
        CHECK(near(motion.target[legs[j]],centers[j]+.15f*a[legs[j]]));
    }
    /* Non-endpoint and zero probes distinguish cubic from linear1.5*mean. */
    const float midmean[6]={.5f,-.5f,.1f,.25f,-.25f,-.1f};
    float cubic[6];CHECK(!RLMotion31DecodeRecoveryMean(midmean,cubic));
    for(unsigned j=0;j<4;j++){
        const float d=midmean[legs[j]];
        const float reach=d>=0?1.5f-centers[j]:centers[j]+1.5f;
        CHECK(near(cubic[legs[j]],centers[j]+.15f*d+(reach-.15f)*d*d*d));
        CHECK(fabsf(cubic[legs[j]]-1.5f*d)>.01f);
    }
    const float zeromean[6]={0};CHECK(!RLMotion31DecodeRecoveryMean(zeromean,cubic));
    for(unsigned j=0;j<4;j++)CHECK(near(cubic[legs[j]],centers[j]));
    CHECK(recovery.target[2]==25 && recovery.target[5]==-25);
    CHECK(motion.target[2]==20 && motion.target[5]==-20);
    float target[6],bad[6]={0}; bad[5]=NAN;
    CHECK(RLMotion31DecodeRecoveryMean(bad,target)<0);
    bad[5]=1.01f;
    CHECK(RLMotion31DecodeRecoveryMean(bad,target)<0);
}

/* Both modes call the same015 adoption/PD path; target takes effect on this
   control tick, and repeated target PD uses the newly received q/dq. */
static void test_latest_adoption_and_pd(void)
{
    RLControlState s={0};RLControlFrame f;CHECK(!RLControlStartRecovery(&s));
    RLShadowSnapshot x=feedback(&s,100);
    x.motion.joint_position_rad[0]=.2f;x.motion.joint_velocity_rad_s[2]=7;
    RLControlFillPrevious(&s,&x);
    CHECK(x.previous_source==RL_PREVIOUS_INIT_PROXY && near(x.motion.previous_target[0],.2f));
    CHECK(x.motion.previous_target[2]==7);
    CHECK(RLControlExecute(&s,100,1,&x,NULL,&f)==1 && zero_output(&f));
    const float a[6]={.4f,-.3f,1,-.2f,.3f,-1};
    RLShadowCommand c=packet(&s,105,1,a,0);
    x=feedback(&s,105);
    CHECK(!RLControlExecute(&s,105,1,&x,&c,&f));
    CHECK(s.active && s.adopted_sequence==1 && s.adopted_tick_ms==105);
    CHECK(!memcmp(s.adopted_target,c.preview.target,sizeof(s.adopted_target)));
    CHECK(near(f.tau_limited_nm[2],4.8f) && near(f.tau_limited_nm[5],-4.8f));
    CHECK(near(f.wheel_current_raw[0],-4.8f*RL_MOTION31_WHEEL_CURRENT_PER_NM));
    CHECK(near(f.wheel_current_raw[1],-4.8f*RL_MOTION31_WHEEL_CURRENT_PER_NM));
    check_spring(&x,&f,1);
    const RLControlState before=s;
    /* Shadow decode cannot affect actual previous. */
    RLMotion31Preview preview;CHECK(!RLMotion31BuildPreview(&x.motion,a,&preview));
    RLControlFillPrevious(&s,&x);CHECK(!memcmp(&before,&s,sizeof(s)));
    CHECK(!memcmp(x.motion.previous_target,c.preview.target,sizeof(s.adopted_target)));
    x=feedback(&s,110);x.motion.joint_position_rad[0]=.07f;
    x.motion.joint_velocity_rad_s[0]=.4f;x.leg_spread_rad[0]=1.2f;
    CHECK(!RLControlExecute(&s,110,1,&x,NULL,&f));
    CHECK(near(f.tau_pd_nm[0],40*(c.preview.target[0]-.07f)-2*.4f));
    CHECK(s.adopted_tick_ms==105);check_spring(&x,&f,1);
    c=packet(&s,115,2,NULL,0);x=feedback(&s,115);
    CHECK(!RLControlExecute(&s,115,1,&x,&c,&f));
    CHECK(s.adopted_sequence==2 && s.adopted_tick_ms==115);
    CHECK(!memcmp(s.adopted_target,c.preview.target,sizeof(s.adopted_target)));
}

static void test_motion_matches_ordinary015(void)
{
    const uint32_t starts[]={100,UINT32_MAX-19u};
    for(unsigned phase=0;phase<2;phase++){
        RLControlState plain={.timing=1,.ready=1,.ramp_completed=1,.epoch=7};
        RLControlState session=plain;session.recovery_session=1;
        RLControlFrame a,b;RLShadowCommand c={0};
        for(unsigned t=0;t<150;t+=5){
            const uint32_t now=starts[phase]+t;
            if(t%10==0){
                float mean[6]={.3f,-.2f,.15f,-.2f,.4f,-.16f};
                mean[0]+=.001f*t;c=packet(&plain,now,t/10+1,mean,0);
            }
            RLShadowSnapshot x=feedback(&plain,now),y=feedback(&session,now);
            for(unsigned j=0;j<4;j++)x.motion.joint_position_rad[j]=y.motion.joint_position_rad[j]=.001f*t;
            for(unsigned j=0;j<6;j++)x.motion.joint_velocity_rad_s[j]=y.motion.joint_velocity_rad_s[j]=.01f*t;
            x.leg_spread_rad[0]=y.leg_spread_rad[0]=.6f+.001f*t;
            CHECK(!RLControlExecute(&plain,now,1,&x,&c,&a));
            CHECK(!RLControlExecute(&session,now,1,&y,&c,&b));
            CHECK(!memcmp(a.adopted_target,b.adopted_target,sizeof(a.adopted_target)));
            CHECK(!memcmp(a.tau_pd_nm,b.tau_pd_nm,sizeof(a.tau_pd_nm)));
            CHECK(!memcmp(a.tau_limited_nm,b.tau_limited_nm,sizeof(a.tau_limited_nm)));
            CHECK(!memcmp(a.spring_motor_nm,b.spring_motor_nm,sizeof(a.spring_motor_nm)));
            CHECK(!memcmp(a.final_motor_nm,b.final_motor_nm,sizeof(a.final_motor_nm)));
            CHECK(!memcmp(a.wheel_current_raw,b.wheel_current_raw,sizeof(a.wheel_current_raw)));
            CHECK(plain.adopted_sequence==session.adopted_sequence);
        }
    }
}

static void test_handoff(void)
{
    RLControlState s={0};RLControlFrame f;CHECK(!RLControlStartRecovery(&s));
    RLShadowCommand c={0};
    for(uint32_t now=100;now<=300;now+=5){
        if(now%10==0)c=packet(&s,now,now/10,NULL,0);
        CHECK(!step(&s,now,&c,1,&f));
    }
    CHECK(s.recovering && s.handoff.requested && !s.handoff.committed);
    const RLControlState before=s;
    /* All probe identity fields remain causal; jump/epoch failures retain Recovery. */
    for(unsigned k=0;k<7;k++){
        RLControlState trial=s;c=packet(&trial,305,40,NULL,1);
        if(k==0)c.control_epoch--;
        if(k==1)c.snapshot_tick_ms=299;
        if(k==2)c.previous_sequence++;
        if(k==3)c.previous_source=3;
        if(k==4)c.motion_valid=0;
        if(k==5)c.snapshot_sequence=trial.adopted_sequence;
        if(k==6){c.motion_action[2]=1;RLMotion31Snapshot m=feedback(&trial,305).motion;m.recovery_mode=0;
            CHECK(!RLMotion31BuildPreview(&m,c.motion_action,&c.motion_preview));}
        CHECK(!step(&trial,305,&c,1,&f));
        CHECK(trial.recovering && !trial.handoff.committed && trial.epoch==before.epoch);
    }
    c=packet(&s,305,40,NULL,1);
    CHECK(!step(&s,305,&c,1,&f));
    CHECK(!s.recovering && s.recovery_session && s.handoff.committed);
    CHECK(s.epoch==before.epoch+1 && s.adopted_tick_ms==305 && s.adopted_sequence==40);
    CHECK(!memcmp(s.adopted_target,c.motion_preview.target,sizeof(s.adopted_target)));
    RLShadowSnapshot x=feedback(&s,305);check_spring(&x,&f,1);
    CHECK(x.motion.recovery_mode==0 && !x.motion_probe && x.previous_source==RL_PREVIOUS_ADOPTED);
    CHECK(!memcmp(x.motion.previous_target,c.motion_preview.target,sizeof(s.adopted_target)));
    RLShadowCommand stale=packet(&s,310,41,NULL,0);stale.control_epoch--;
    CHECK(!step(&s,310,&stale,0,&f) && s.adopted_sequence==40);
    c=packet(&s,315,42,NULL,0);CHECK(!step(&s,315,&c,0,&f));
    CHECK(s.adopted_sequence==42 && s.epoch==before.epoch+1);
}

static void test_faults_and_no_partial_handoff(void)
{
    for(unsigned mode=0;mode<7;mode++){
        RLControlState s={0};RLControlFrame f;CHECK(!RLControlStartRecovery(&s));
        RLShadowCommand c=packet(&s,100,1,NULL,0);CHECK(!step(&s,100,&c,0,&f));
        const RLControlState before=s;
        RLShadowSnapshot x=feedback(&s,105);unsigned reason=RL_FAULT_INPUT;
        if(mode==0)x.feedback_valid_mask=0;
        if(mode==1)x.imu_update_tick_ms=80;
        if(mode==2)x.motion.joint_velocity_rad_s[0]=NAN;
        if(mode==3){x.leg_length_m[0]=NAN;reason=RL_FAULT_COMPENSATION;}
        if(mode==4){x.leg_spread_rad[1]=NAN;reason=RL_FAULT_COMPENSATION;}
        if(mode==5)reason=RL_FAULT_MODE;
        if(mode==6){x=feedback(&s,125);reason=RL_FAULT_COMMAND;}
        CHECK(RLControlExecute(&s,mode==6?125:105,mode!=5,&x,NULL,&f)<0);
        CHECK(s.fault==reason && s.recovery_session && !s.active && f.owner==RL_OWNER_SAFE && zero_output(&f));
        CHECK(!memcmp(s.adopted_target,before.adopted_target,sizeof(s.adopted_target)));
        CHECK(step(&s,130,NULL,0,&f)<0 && zero_output(&f));
        RLControlReset(&s);CHECK(!s.fault && !s.recovery_session);
    }
    /* Failure on the same tick as a valid Motion candidate cannot commit it. */
    RLControlState s={0};RLControlFrame f;CHECK(!RLControlStartRecovery(&s));
    RLShadowCommand c=packet(&s,100,1,NULL,0);CHECK(!step(&s,100,&c,0,&f));
    s.handoff_source=1;
    s.handoff=(RLHandoffState){.dwelling=1,.since_ms=0,.requested=1,.request_tick_ms=100,.observed=1,.last_ms=100};
    const RLControlState before=s;c=packet(&s,105,2,NULL,1);
    RLShadowSnapshot x=feedback(&s,105);x.handoff=quality(105);
    RLControlState valid=s;CHECK(!RLControlExecute(&valid,105,1,&x,&c,&f));
    CHECK(!valid.recovering && valid.handoff.committed);
    x.leg_length_m[0]=NAN;
    CHECK(RLControlExecute(&s,105,1,&x,&c,&f)<0);
    CHECK(s.recovering && !s.handoff.committed && s.adopted_sequence==before.adopted_sequence);
    CHECK(s.fault==RL_FAULT_COMPENSATION && f.owner==RL_OWNER_SAFE && zero_output(&f));
}

#ifdef RL_RECOVERY_SHADOW_HOST
/* Compile this same test with production rl_shadow.c/recovery_actor.c and
   -DRL_RECOVERY_SHADOW_HOST -Itests/host to exercise the real infinite task.
   Only RTOS, geometry diagnostics and the unavailable Motion AI are stubbed. */
static float shadow_input[31],shadow_output[6];
static ai_buffer shadow_in={shadow_input},shadow_out={shadow_output};
static RLShadowSnapshot task_snapshot;
static QueueHandle_t task_snapshots,task_commands;
static TickType_t task_tick;
static uint32_t task_clock_base;
static unsigned task_iteration,task_scenario,task_writes,motion_calls;
static jmp_buf task_done;

static unsigned motion_latency(void)
{
    if(task_scenario==8)return 4;
    if(task_scenario==9)return 24;
    if(task_scenario==10 || task_scenario==11)return 25;
    if(task_scenario==12 || task_scenario==13)return 7;
    return 0;
}

QueueHandle_t xQueueCreateStatic(unsigned n,unsigned size,uint8_t *storage,StaticQueue_t *q)
{
    assert(n==1);q->size=size;q->storage=storage;
    if(!task_snapshots)task_snapshots=q;else task_commands=q;
    return q;
}
int xQueuePeek(QueueHandle_t q,void *dst,unsigned timeout)
{
    (void)timeout;
    if(q==task_snapshots){memcpy(dst,&task_snapshot,sizeof(task_snapshot));return pdPASS;}
    return 0;
}
static unsigned received_iteration(void)
{
    return task_iteration;
}
static void check_task_observation(const float *o,int recovery)
{
    const unsigned received=received_iteration();
    assert(near(o[6],.01f*(float)received/1.5f));
    assert(near(o[10],.05f*(float)received));
    /* Physical feedback, previous and commands all come from latest snapshot. */
    for(unsigned j=0;j<4;j++)
        assert(near(o[16+j],task_snapshot.motion.previous_target[legs[j]]/1.5f));
    assert(near(o[20],task_snapshot.motion.previous_target[2]/100));
    assert(near(o[21],task_snapshot.motion.previous_target[5]/100));
    assert(near(o[22],task_snapshot.motion.command_vx_m_s/4));
    assert(near(o[23],task_snapshot.motion.command_yaw_rad_s/4));
    assert(near(o[24],task_snapshot.motion.command_height_m/.5f));
    assert(o[25]==(recovery?0:1) && o[26]==(recovery?1:0));
}
int xQueueOverwrite(QueueHandle_t q,const void *value)
{
    assert(q==task_commands);++task_writes;
    const RLShadowCommand *c=value;
    assert(c->snapshot_sequence==task_snapshot.sequence);
    assert(c->snapshot_tick_ms==task_snapshot.tick_ms && c->output_tick_ms==task_tick);
    if((task_scenario>=3 && task_scenario<=5 && task_iteration>1) ||
       (task_scenario==6 && task_iteration>1)){
        /* Newest invalid state is rejected immediately, no historical fallback. */
        assert(!c->valid);return pdPASS;
    }
    if(task_scenario==10 || task_scenario==11){
        assert(!c->valid && !c->motion_valid);
        assert((uint32_t)(c->output_tick_ms-c->snapshot_tick_ms)==25);
        return pdPASS;
    }
    if(!c->valid)fprintf(stderr,"Real task invalid packet: scenario=%u iteration=%u\n",task_scenario,task_iteration);
    assert(c->valid && c->snapshot_sequence==task_snapshot.sequence);
    assert((uint32_t)(c->output_tick_ms-c->snapshot_tick_ms)==motion_latency());
    assert(c->control_epoch==task_snapshot.control_epoch);
    assert(c->previous_source==RL_PREVIOUS_ADOPTED);
    assert(c->previous_sequence==task_snapshot.adopted_sequence);
    assert(c->recovery_mode==task_snapshot.motion.recovery_mode);
    assert(c->actor_input_tick_ms==task_clock_base+10u*received_iteration());
    check_task_observation(c->preview.observation,c->recovery_mode);
    float expected[6],target[6];
    if(c->recovery_mode){
        assert(!RecoveryActorMean(c->preview.observation,expected));
        assert(!RLMotion31DecodeRecoveryMean(expected,target));
    }else{
        for(unsigned i=0;i<6;i++)expected[i]=.1f*(float)(i+1);
        assert(!RLMotion31DecodeOutput(expected,task_snapshot.motion.command_height_m,target));
    }
    for(unsigned i=0;i<6;i++){
        assert(near(c->action[i],expected[i]));
        assert(near(c->preview.target[i],target[i]));
    }
    assert(c->motion_valid==(task_snapshot.motion_probe && task_scenario!=12));
    if(c->motion_valid){
        check_task_observation(c->motion_preview.observation,0);
        for(unsigned i=0;i<6;i++)expected[i]=.1f*(float)(i+1);
        assert(!RLMotion31DecodeOutput(expected,task_snapshot.motion.command_height_m,target));
        for(unsigned i=0;i<6;i++){
            assert(near(c->motion_action[i],expected[i]));
            assert(near(c->motion_preview.target[i],target[i]));
        }
    }
    return pdPASS;
}
TickType_t xTaskGetTickCount(void){return task_tick;}
void vTaskDelayUntil(TickType_t *last,TickType_t period)
{
    assert(period==10);
    if(++task_iteration>6)longjmp(task_done,1);
    *last+=period;task_tick=*last;
    RLControlState state={.active=1,.epoch=7,.adopted_sequence=100u+task_iteration,
        .recovery_session=1,.recovering=1};
    if(task_scenario==1 || (task_scenario>=8 && task_scenario!=11))state.handoff.requested=1;
    if(task_scenario==11)state.recovering=0;
    if(task_scenario==2 && task_iteration>=4){state.recovering=0;state.epoch=8;state.handoff.committed=1;}
    if(task_scenario==14 && task_iteration>=4)state.epoch=9;
    if(task_scenario==7 && (task_iteration==3 || task_iteration==4)){
        state.recovery_session=state.recovering=0;
    }
    for(unsigned i=0;i<6;i++)state.adopted_target[i]=.01f*(float)(i+1)*task_iteration;
    task_snapshot=feedback(&state,task_tick);
    task_snapshot.leg_length_m[0]=task_snapshot.leg_length_m[1]=.28f;
    task_snapshot.sequence=200u+task_iteration;
    task_snapshot.motion.joint_position_rad[0]=.01f*(float)task_iteration;
    task_snapshot.motion.joint_velocity_rad_s[0]=(float)task_iteration;
    task_snapshot.motion.command_vx_m_s=.1f*(float)task_iteration;
    task_snapshot.motion.command_yaw_rad_s=-.1f*(float)task_iteration;
    task_snapshot.motion.command_height_m=.30f+.01f*(float)task_iteration;
    /* Establish a real inference start in this invocation before suppressing
       inference; deadline checks must not depend on prior test clock resets. */
    if(task_iteration>1){
        if(task_scenario==3)task_snapshot.feedback_valid_mask=0;
        if(task_scenario==4)task_snapshot.rl_active=0;
        if(task_scenario==5)task_snapshot.tick_ms-=25u;
        if(task_scenario==6)task_snapshot.motion.gravity_model[1]=NAN;
    }
}
void vTaskSuspend(void *task){(void)task;assert(!"unexpected task suspension");}
ai_error ai_tjb_network_create_and_init(ai_handle *h,ai_handle *a,void *unused)
{(void)a;(void)unused;*h=shadow_input;return (ai_error){0};}
ai_buffer *ai_tjb_network_inputs_get(ai_handle h,void *unused)
{(void)h;(void)unused;return &shadow_in;}
ai_buffer *ai_tjb_network_outputs_get(ai_handle h,void *unused)
{(void)h;(void)unused;return &shadow_out;}
int ai_tjb_network_run(ai_handle h,ai_buffer *a,ai_buffer *b)
{
    (void)h;(void)a;(void)b;++motion_calls;
    check_task_observation(shadow_input,0);
    task_tick+=motion_latency();
    if(task_scenario==12)return 0; /* Motion probe failure must retain Recovery output. */
    for(unsigned i=0;i<6;i++)shadow_output[i]=.1f*(float)(i+1);
    return 1;
}
void getPhi(float a,float b,float c,float d,float e,float f,float g,
            float *p2,float *p3,float *length,float *theta)
{(void)a;(void)b;(void)c;(void)d;(void)e;(void)f;(void)g;*p2=*p3=0;*length=.28f;*theta=1.5f;}

static void test_real_recovery_shadow_task(void)
{
    const unsigned failures_before=failures;
    for(task_scenario=0;task_scenario<15;task_scenario++){
        task_iteration=task_writes=motion_calls=0;
        task_clock_base=task_scenario==13?UINT32_MAX-15u:0;
        task_tick=task_clock_base;
        task_snapshots=task_commands=NULL;
        const uint32_t misses_before=g_rl_shadow_debug.inference_deadline_miss_count;
        RLShadowQueueInit();
        if(!setjmp(task_done))RLShadowTask(NULL);
        CHECK(task_writes==6);
        const unsigned expected_calls=task_scenario==1 || task_scenario>=8?6:
            task_scenario==2?3:task_scenario==7?2:0;
        CHECK(motion_calls==expected_calls);
        if(task_scenario>=8){
            const uint32_t misses=g_rl_shadow_debug.inference_deadline_miss_count-misses_before;
            if(misses!=(motion_latency()>10?6u:0u)){
                fprintf(stderr,"Shadow deadline: scenario=%u expected=%u actual=%u\n",
                        task_scenario,motion_latency()>10?6u:0u,(unsigned)misses);
                ++failures;
            }
            CHECK(g_rl_shadow_debug.inference_end_tick_ms==task_tick);
        }
        if(task_scenario>=3 && task_scenario<=6){
            const uint32_t misses=g_rl_shadow_debug.inference_deadline_miss_count-misses_before;
            if(misses){
                fprintf(stderr,"Shadow deadline without inference: scenario=%u actual=%u\n",
                        task_scenario,(unsigned)misses);
                ++failures;
            }
        }
        CHECK(g_rl_shadow_debug.publish_seq==g_rl_shadow_debug.frame_end);
        CHECK(!(g_rl_shadow_debug.publish_seq&1));
    }
    printf("%s real Recovery shadow task: 9 routing/input scenarios + 6 latency/failure/wrap scenarios\n",
           failures==failures_before?"PASS":"FAIL");
}
#endif

static void test_proprioceptive_readiness(void)
{
    RLHandoffState g={0};float q[4]={.24f,.20f,-.235f,.21f};
    float dq[6]={0},gyro[3]={0},gravity[3]={0,-1,0},center[4];double dc[4];
    CHECK(!RLMotion31SupportCenter(.34,dc));
    for(unsigned i=0;i<4;i++)center[i]=(float)dc[i];
    const uint32_t base=UINT32_MAX-99u;
    for(unsigned t=0;t<=200;t++) {
        const int ready=RLHandoffObservePosture(&g,&RL_HANDOFF_CONFIG,base+t,1,
                                               q,dq,gyro,gravity,center,.5f);
        CHECK(ready==(t==200));
    }
    CHECK(g.requested && !g.committed);
    RLHandoffReset(&g);
    for(unsigned i=0;i<1000;i++)
        CHECK(!RLHandoffObservePosture(&g,&RL_HANDOFF_CONFIG,100,1,q,dq,gyro,gravity,center,.5f));
    CHECK(!g.requested); /* Repeating one physical sample cannot manufacture dwell. */
    for(unsigned variant=0;variant<7;variant++){
        RLHandoffReset(&g);float bq[4],bv[6],bg[3],bgrav[3];
        memcpy(bq,q,sizeof(q));memcpy(bv,dq,sizeof(dq));
        memcpy(bg,gyro,sizeof(gyro));memcpy(bgrav,gravity,sizeof(gravity));
        int valid=1;
        if(variant==0)valid=0;
        if(variant==1)bq[0]=center[0]+.51f;
        if(variant==2)bv[0]=3.01f;
        if(variant==3)bg[2]=1.01f;
        if(variant==4)bgrav[1]=1;
        if(variant==5)bq[0]=NAN;
        if(variant==6)bgrav[2]=.5f;
        for(unsigned t=0;t<400;t++)
            CHECK(!RLHandoffObservePosture(&g,&RL_HANDOFF_CONFIG,1000+t,valid,
                                           bq,bv,bg,bgrav,center,.5f));
    }
    /* Execute the real control route, then fresh Motion probe commits once. */
    RLControlState s={0};RLControlFrame f;CHECK(!RLControlStartRecovery(&s));
    RLShadowCommand c={0};
    for(unsigned t=0;t<=130;t+=5){
        const uint32_t now=1000+t;
        if(t%10==0)c=packet(&s,now,t/10+1,NULL,t>=110);
        RLShadowSnapshot x=feedback(&s,now);x.handoff_posture_enabled=1;
        memcpy(x.motion.joint_position_rad,q,sizeof(q));
        CHECK(!RLControlExecute(&s,now,1,&x,&c,&f));
    }
    CHECK(s.handoff_source==2 && s.handoff.committed && !s.recovering);
    /* A controller tick is not a sensor update: oldest received timestamp wins. */
    memset(&s,0,sizeof(s));CHECK(!RLControlStartRecovery(&s));
    for(unsigned t=0;t<RL_RECOVERY_POSTURE_DWELL_MS;t+=5){
        const uint32_t now=2000+t;
        if(t%10==0)c=packet(&s,now,t/10+1,NULL,0);
        RLShadowSnapshot x=feedback(&s,now);x.handoff_posture_enabled=1;
        memcpy(x.motion.joint_position_rad,q,sizeof(q));
        const uint32_t oldest=2000+(t/10)*10;
        x.imu_update_tick_ms=oldest;
        for(unsigned i=0;i<6;i++)x.feedback_receive_tick_ms[i]=oldest;
        CHECK(!RLControlExecute(&s,now,1,&x,&c,&f));
        CHECK(s.handoff.last_ms==oldest && !s.handoff.requested);
    }
}

static void test_broader_motion_readiness(void)
{
    RLControlState s={0};RLControlFrame f;CHECK(!RLControlStartRecovery(&s));
    RLShadowCommand c={0};
    for(unsigned t=0;t<=40;t+=5){
        const uint32_t now=3000+t;
        if(t%10==0)c=packet(&s,now,t/10+1,NULL,t>=30);
        RLShadowSnapshot x=feedback(&s,now);x.handoff_posture_enabled=1;
        /*20degree cone,3rad/s gyro,8rad/s dq region; old limits reject this. */
        x.motion.gravity_model[1]=-cosf(.25f);x.motion.gravity_model[2]=sinf(.25f);
        x.motion.gyro_model_rad_s[0]=2.f;
        x.motion.joint_velocity_rad_s[0]=6.f;
        CHECK(!RLControlExecute(&s,now,1,&x,&c,&f));
        if(t<RL_RECOVERY_POSTURE_DWELL_MS)CHECK(!s.handoff.requested);
    }
    CHECK(s.handoff_source==2 && s.handoff.committed && !s.recovering);
}

int main(void)
{
    test_explicit_start_and_contracts();
    test_latest_adoption_and_pd();
    test_motion_matches_ordinary015();
    test_handoff();
    test_faults_and_no_partial_handoff();
    test_proprioceptive_readiness();
    test_broader_motion_readiness();
#ifdef RL_RECOVERY_SHADOW_HOST
    test_real_recovery_shadow_task();
#endif
    if(failures){fprintf(stderr,"Recovery/handoff: %u failed cases\n",failures);return 1;}
    puts("PASS shared015 Recovery/Motion: latest feedback/target, no added queues, PD, spring, actual previous and handoff");
    return 0;
}
