/* Production control/decoder/gate, synthetic feedback only; no RTOS or motors. */
#include "rl_control.h"
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
    CHECK(recovery.target[2]==25 && recovery.target[5]==-25);
    CHECK(motion.target[2]==20 && motion.target[5]==-20);
    float target[6],bad[6]={0}; bad[5]=NAN;
    CHECK(RLMotion31DecodeRecoveryMean(bad,target)<0);
    bad[5]=1.01f;
    CHECK(RLMotion31DecodeRecoveryMean(bad,target)<0);
}

static void test_shadow_previous_is_read_only(void)
{
    RLControlState s={0}; RLControlFrame f;
    CHECK(!RLControlStartRecovery(&s));
    RLShadowCommand c=packet(&s,100,1,NULL,0);
    for(uint32_t t=100;t<=110;t++)CHECK(!step(&s,t,&c,0,&f));
    CHECK(s.adopted_sequence==1);
    const RLControlState before=s;
    RLShadowSnapshot x=feedback(&s,111);
    RLMotion31Preview preview;
    const float shadow_action[6]={.8f,-.7f,.6f,-.5f,.4f,-.3f};
    for(unsigned pass=0;pass<3;pass++){
        x.motion.recovery_mode=pass==0;
        CHECK(!RLMotion31BuildPreview(&x.motion,shadow_action,&preview));
        /* Inference output is never fed back as an adopted previous. */
        RLControlFillPrevious(&s,&x);
        CHECK(!memcmp(x.motion.previous_target,before.adopted_target,sizeof(s.adopted_target)));
        CHECK(x.previous_source==RL_PREVIOUS_ADOPTED && x.adopted_sequence==1);
        CHECK(!memcmp(&s,&before,sizeof(s)));
    }
    c=packet(&s,111,2,shadow_action,0);
    CHECK(!step(&s,111,&c,0,&f));
    CHECK(s.recovery_request_sequence==2 && s.adopted_sequence==1);
    x=feedback(&s,111);
    CHECK(!memcmp(x.motion.previous_target,before.adopted_target,sizeof(s.adopted_target)));
    CHECK(memcmp(x.motion.previous_target,c.preview.target,sizeof(s.adopted_target)));
}

static void test_startup_previous_is_proxy(void)
{
    RLControlState s={0}; RLControlFrame f;
    CHECK(!RLControlStartRecovery(&s));
    RLShadowSnapshot x=feedback(&s,101);
    x.motion.joint_position_rad[0]=.2f;
    x.motion.joint_velocity_rad_s[2]=7;
    x.motion.joint_velocity_rad_s[5]=-9;
    RLControlFillPrevious(&s,&x);
    CHECK(x.previous_source==RL_PREVIOUS_INIT_PROXY && !x.adopted_sequence);
    CHECK(near(x.motion.previous_target[0],.2f));
    CHECK(x.motion.previous_target[2]==0 && x.motion.previous_target[5]==0);
    CHECK(RLControlExecute(&s,101,1,&x,NULL,&f)==1);
    CHECK(f.owner==RL_OWNER_SAFE && !f.fault && f.control_epoch==s.epoch && zero_output(&f));
    CHECK(s.recovery_initialized && !s.adopted_sequence && !s.recovery_pd_valid);
    RLControlFillPrevious(&s,&x);
    /* Queue seeding is initialization, not an actual target adoption. */
    CHECK(x.previous_source==RL_PREVIOUS_INIT_PROXY && !x.adopted_sequence);
    RLShadowCommand c=packet(&s,102,1,NULL,0);
    for(uint32_t t=102;t<=109;t++){
        CHECK(!step(&s,t,&c,0,&f));
        x=feedback(&s,t);
        if(x.previous_source!=RL_PREVIOUS_INIT_PROXY)
            fprintf(stderr,"Startup previous: now=%u active=%u pd_valid=%u adopted_sequence=%u source=%u\n",
                    (unsigned)t,(unsigned)s.active,(unsigned)s.recovery_pd_valid,
                    (unsigned)s.adopted_sequence,(unsigned)x.previous_source);
        CHECK(!s.adopted_sequence && x.previous_source==RL_PREVIOUS_INIT_PROXY);
    }
    CHECK(!step(&s,110,&c,0,&f));
    x=feedback(&s,110);
    CHECK(x.previous_source==RL_PREVIOUS_ADOPTED && x.adopted_sequence==1);
    CHECK(!memcmp(x.motion.previous_target,c.preview.target,sizeof(s.adopted_target)));
}

static void test_command_mode_boundaries(void)
{
    const float action[6]={.2f,0,.1f,0,0,0};
    RLControlState recovery={0};
    CHECK(!RLControlStartRecovery(&recovery));
    RLShadowCommand c=packet(&recovery,100,1,action,0);
    RLControlFrame f;
    c.recovery_mode=2;
    CHECK(step(&recovery,100,&c,0,&f)==1);
    CHECK(!recovery.recovery_request_sequence && !recovery.active && f.owner==RL_OWNER_SAFE);
    for(unsigned mode=1;mode<=2;mode++){
        RLControlState motion={.ramp_completed=1,.timing=1,.ready=1,.epoch=7};
        RLShadowSnapshot x=feedback(&motion,100);
        RLMotion31Snapshot input=x.motion;input.recovery_mode=1;
        c=(RLShadowCommand){.valid=1,.snapshot_sequence=1,.snapshot_tick_ms=100,
            .output_tick_ms=100,.control_epoch=7,.previous_source=RL_PREVIOUS_INIT_PROXY,
            .recovery_mode=(uint8_t)mode};
        memcpy(c.action,action,sizeof(action));
        CHECK(!RLMotion31BuildPreview(&input,c.action,&c.preview));
        CHECK(!RLControlStep(&motion,100,1,&c).active);
        CHECK(RLControlExecute(&motion,100,1,&x,&c,&f)==1);
        CHECK(!motion.active && !motion.adopted_sequence && !motion.fault);
    }
}

/* Independent elapsed-time oracle: targets travel 5ms, PD updates every 5ms,
   then physical right/left wheel torque travels another 7/11ms. */
static void schedule(uint32_t base)
{
    RLControlState s={0}; RLControlFrame f;
    float requested[90][6]={{0}},pd_history[90][6]={{0}},adopted[6]={0},pd[6]={0};
    uint32_t sequences[90]={0},adopted_sequence=0,last_pd=0;
    int pd_started=0;
    CHECK(!RLControlStartRecovery(&s));
    RLShadowCommand c={0};
    for(unsigned t=0;t<90;t++){
        const uint32_t now=base+t;
        if(t%10u==0){
            float a[6]={0};
            for(unsigned i=0;i<6;i++)a[i]=(t/10u)%2u?.06f:-.04f;
            c=packet(&s,now,t/10u+1u,a,0);
        }
        memcpy(requested[t],c.preview.target,sizeof(requested[t]));
        sequences[t]=c.snapshot_sequence;
        RLShadowSnapshot x=feedback(&s,now);
        x.motion.joint_position_rad[0]=.0001f*(float)t;
        x.motion.joint_velocity_rad_s[0]=.001f*(float)t;
        if((!pd_started && now%5u==0u) || (pd_started && (uint32_t)(now-last_pd)==5u)){
            if(t>=5){memcpy(adopted,requested[t-5],sizeof(adopted));adopted_sequence=sequences[t-5];}
            for(unsigned j=0;j<4;j++)pd[legs[j]]=40*adopted[legs[j]];
            pd[0]-=40*x.motion.joint_position_rad[0]+2*x.motion.joint_velocity_rad_s[0];
            pd[2]=.2f*adopted[2]; pd[5]=.2f*adopted[5];
            last_pd=now; pd_started=1;
        }
        memcpy(pd_history[t],pd,sizeof(pd));
        CHECK(!RLControlExecute(&s,now,1,&x,&c,&f));
        CHECK(!s.fault && s.recovery_session && s.recovering && f.owner==RL_OWNER_RL);
        CHECK(s.adopted_sequence==adopted_sequence);
        if(pd_started && s.adopted_tick_ms!=last_pd)
            fprintf(stderr,"PD cadence: base=%u now=%u expected_last=%u actual_last=%u\n",
                    (unsigned)base,(unsigned)now,(unsigned)last_pd,(unsigned)s.adopted_tick_ms);
        CHECK(!pd_started || s.adopted_tick_ms==last_pd);
        for(unsigned i=0;i<6;i++)CHECK(near(s.adopted_target[i],adopted[i]));
        for(unsigned i=0;i<6;i++)CHECK(near(f.tau_pd_nm[i],pd[i]));
        for(unsigned j=0;j<4;j++)CHECK(near(f.tau_limited_nm[legs[j]],pd[legs[j]]));
        const float right=t>=7?pd_history[t-7][2]:0;
        const float left=t>=11?pd_history[t-11][5]:0;
        CHECK(near(f.tau_limited_nm[2],right) && near(f.tau_limited_nm[5],left));
        CHECK(fabsf(f.wheel_current_raw[0]-left*RL_MOTION31_WHEEL_CURRENT_PER_NM)<.001f);
        CHECK(fabsf(f.wheel_current_raw[1]+right*RL_MOTION31_WHEEL_CURRENT_PER_NM)<.001f);
        for(unsigned j=0;j<4;j++)CHECK(f.spring_motor_nm[j]==0);
    }
}

static void test_wheel_session_cap(void)
{
    RLControlState s={0}; RLControlFrame f;
    CHECK(!RLControlStartRecovery(&s));
    const float a[6]={0,0,1,0,0,-1};
    RLShadowCommand c=packet(&s,100,1,a,0);
    for(uint32_t t=100;t<=120;t++){
        CHECK(!step(&s,t,&c,0,&f));
        CHECK(near(f.tau_limited_nm[2],t>=112?4.8f:0));
        CHECK(near(f.tau_limited_nm[5],t>=116?-4.8f:0));
        CHECK(near(f.tau_pd_nm[2],t>=105?20:0));
        CHECK(near(f.tau_pd_nm[5],t>=105?-20:0));
        for(unsigned j=0;j<4;j++)CHECK(f.spring_motor_nm[j]==0);
    }
    CHECK(s.adopted_target[2]==100 && s.adopted_target[5]==-100);
    CHECK(fabsf(f.wheel_current_raw[0])<=RL_CONTROL_WHEEL_RAW_LIMIT);
    CHECK(fabsf(f.wheel_current_raw[1])<=RL_CONTROL_WHEEL_RAW_LIMIT);
}

static void test_faults_latch_zero(void)
{
    for(unsigned mode=0;mode<11;mode++){
        RLControlState s={0}; RLControlFrame f;
        CHECK(!RLControlStartRecovery(&s));
        RLShadowCommand c=packet(&s,100,1,NULL,0);
        for(uint32_t t=100;t<=120;t++)CHECK(!step(&s,t,&c,0,&f));
        uint32_t now=mode==6?122:121;
        RLShadowSnapshot x=feedback(&s,now);
        unsigned reason=RL_FAULT_INPUT;
        if(mode==0)x.feedback_valid_mask&=~1u;
        if(mode==1)x.feedback_receive_tick_ms[5]=now-25u;
        if(mode==2)x.imu_update_sequence=0;
        if(mode==3)x.motion.joint_velocity_rad_s[5]=NAN;
        if(mode==4){
            for(uint32_t t=121;t<=124;t++)CHECK(!step(&s,t,NULL,0,&f));
            now=125; x=feedback(&s,now); reason=RL_FAULT_COMMAND;
        }
        if(mode==5){now=120; x=feedback(&s,now); reason=RL_FAULT_PD;}
        if(mode==6)reason=RL_FAULT_PD;
        if(mode==7)x.imu_update_tick_ms=now-25u;
        if(mode==8)x.tick_ms=now-25u;
        if(mode==9)x.motion.gravity_model[1]=100;
        CHECK(RLControlExecute(&s,now,1,mode==10?NULL:&x,NULL,&f)<0);
        CHECK(s.fault==reason && !s.active && s.recovery_session && zero_output(&f));
        CHECK(f.owner==RL_OWNER_SAFE && f.fault==reason && f.control_epoch==s.epoch);
        const uint32_t epoch=s.epoch;
        for(unsigned retry=1;retry<=3;retry++){
            c=packet(&s,now+retry,100+retry,NULL,0);
            CHECK(step(&s,now+retry,&c,1,&f)<0);
            CHECK(s.fault==reason && s.epoch==epoch && s.recovery_session && zero_output(&f));
            CHECK(f.owner==RL_OWNER_SAFE && f.fault==reason && f.control_epoch==epoch);
        }
        /* Session remains selected: chassis owner must use its zero-output
           error branch, rather than the ordinary Motion/LQR execution path. */
        RLControlReset(&s);
        CHECK(!s.fault && !s.recovery_session && s.epoch==epoch+1u);
    }
}

static void test_quality_rejections(void)
{
    CHECK(RL_HANDOFF_CONFIG.dwell_ms==200);
    /* Each bad stream lasts longer than the dwell; elapsed time alone is insufficient. */
    for(unsigned mode=0;mode<13;mode++){
        RLHandoffState h={0};
        for(unsigned t=0;t<=400;t+=10){
            RLHandoffSample q=quality(1000+t);
            switch(mode){
            case 0:q.valid=0;break; /* UNKNOWN */
            case 1:q.height_m=NAN;break;
            case 2:q.pitch_rad=NAN;break;
            case 3:q.normal_force_N[1]=NAN;break;
            case 4:q.maximum_leg_speed_rad_s=NAN;break;
            case 5:q.maximum_leg_speed_rad_s=3.01f;break;
            case 6:q.normal_force_N[0]=2;break;
            case 7:q.angular_speed_rad_s=1.01f;break;
            case 8:q.roll_rad=.101f;break;
            case 9:q.height_m=.37f;break;
            case 10:q.tick_ms=1000;break; /* repeated */
            case 11:q.tick_ms=1000+3*t;break; /* gaps >20ms */
            case 12:q.tick_ms=1000-t;break; /* backwards */
            }
            CHECK(!RLHandoffObserve(&h,&RL_HANDOFF_CONFIG,&q));
            CHECK(!h.requested && !h.committed);
            const float target[6]={0};
            CHECK(!RLHandoffCanCommit(&h,&RL_HANDOFF_CONFIG,q.tick_ms,target,target));
        }
    }
    /* Lost quality or a gap destroys accumulated dwell, including over wrap. */
    for(unsigned wrap=0;wrap<2;wrap++)for(unsigned gap=0;gap<2;gap++){
        RLHandoffState h={0};
        uint32_t base=wrap?UINT32_MAX-150u:1000u;
        for(unsigned t=0;t<200;t+=10){
            RLHandoffSample q=quality(base+t);
            CHECK(!RLHandoffObserve(&h,&RL_HANDOFF_CONFIG,&q));
        }
        RLHandoffSample q=quality(base+220u);
        if(!gap)q.valid=0;
        CHECK(!RLHandoffObserve(&h,&RL_HANDOFF_CONFIG,&q));
        base+=gap?220u:230u;
        for(unsigned t=0;t<=200;t+=10){
            q=quality(base+t);
            CHECK(RLHandoffObserve(&h,&RL_HANDOFF_CONFIG,&q)==(t==200));
        }
        CHECK(h.requested && h.request_tick_ms==base+200u);
    }
}

static void test_unqualified_session_never_hands_off(void)
{
    for(unsigned mode=0;mode<5;mode++){
        RLControlState s={0}; RLControlFrame f; RLShadowCommand c={0};
        CHECK(!RLControlStartRecovery(&s));
        for(uint32_t now=100;now<=500;now++){
            if(now%10u==0)c=packet(&s,now,now/10u,NULL,1);
            RLShadowSnapshot x=feedback(&s,now);
            x.handoff=quality(now);
            if(mode==0)x.handoff.valid=0;
            if(mode==1)x.handoff.roll_rad=NAN;
            if(mode==2)x.handoff.tick_ms=now-21u;
            if(mode==3)x.handoff.tick_ms=100; /* repeated evidence, fresh motor loop */
            if(mode==4)x.handoff.tick_ms=100+((now-100)/21u)*21u;
            CHECK(!RLControlExecute(&s,now,1,&x,&c,&f));
            CHECK(s.recovering && !s.handoff.requested && !s.handoff.committed && !s.fault);
            RLControlFillPrevious(&s,&x);
            CHECK(!x.motion_probe && x.motion.recovery_mode);
        }
    }
}

/* End-to-end gate -> Motion probe -> epoch -> delayed adoption. */
static void test_handoff(void)
{
    RLControlState s={0}; RLControlFrame f;
    CHECK(!RLControlStartRecovery(&s));
    RLShadowCommand c={0};
    const float recovery_action[6]={.05f,-.03f,.02f,.04f,-.02f,-.03f};
    for(uint32_t t=100;t<=300;t++){
        if(t%10u==0 && t<300)c=packet(&s,t,t/10u,recovery_action,1);
        CHECK(!step(&s,t,&c,1,&f));
        if(t<300)CHECK(s.recovering && !s.handoff.requested);
    }
    CHECK(s.handoff.requested && s.recovering && !s.handoff.committed);
    RLShadowSnapshot x=feedback(&s,301);
    CHECK(x.motion_probe && x.motion.recovery_mode);
    const uint32_t epoch=s.epoch;
    /* Candidates must be sampled after the qualified request, be causal,
       have the right epoch, and stay inside BOTH physical jump caps. */
    for(unsigned mode=0;mode<11;mode++){
        RLControlState trial=s;
        c=packet(&trial,301,40,NULL,1);
        if(mode==0)c.control_epoch--;
        if(mode==1)c.snapshot_tick_ms=299;
        if(mode==2)c.previous_sequence++;
        if(mode==3)c.previous_source=3; /* shadow history */
        if(mode==4){
            c.motion_action[2]=1;
            RLMotion31Snapshot m=x.motion;m.recovery_mode=0;
            CHECK(!RLMotion31BuildPreview(&m,c.motion_action,&c.motion_preview));
        }
        if(mode==5)c.motion_valid=0;
        if(mode==6)c.recovery_mode=0;
        if(mode==7){
            /* Put actual previous outside Motion's leg jump envelope. */
            trial.adopted_target[0]=1.0f;
            c.previous_sequence=trial.adopted_sequence;
        }
        if(mode==8)c.motion_preview.target[5]=NAN;
        if(mode==9)c.snapshot_sequence=trial.recovery_request_sequence;
        if(mode==10)c.preview.target[0]+=.1f; /* wrong primary decoder */
        CHECK(!step(&trial,301,&c,1,&f));
        CHECK(trial.recovering && !trial.handoff.committed && trial.epoch==epoch);
    }
    c=packet(&s,301,40,NULL,1);
    const RLControlState before=s;
    CHECK(!step(&s,301,&c,1,&f));
    CHECK(!s.recovering && s.handoff.committed && s.epoch==epoch+1u);
    CHECK(s.recovery_session && s.active && f.owner==RL_OWNER_RL);
    CHECK(!memcmp(s.adopted_target,before.adopted_target,sizeof(s.adopted_target)));
    CHECK(s.adopted_sequence==before.adopted_sequence);
    CHECK(!memcmp(s.recovery_request,c.motion_preview.target,sizeof(s.recovery_request)));
    for(unsigned slot=0;slot<32;slot++)if(slot!=301u%32u){
        CHECK(!memcmp(s.recovery_targets[slot],before.recovery_targets[slot],sizeof(s.recovery_targets[slot])));
        CHECK(s.recovery_target_sequence[slot]==before.recovery_target_sequence[slot]);
    }
    CHECK(!memcmp(s.recovery_wheels,before.recovery_wheels,sizeof(s.recovery_wheels)));
    x=feedback(&s,302);
    CHECK(!x.motion_probe && !x.motion.recovery_mode && x.recovery_session);
    CHECK(!memcmp(x.motion.previous_target,before.adopted_target,sizeof(s.adopted_target)));
    /* Even a mode-correct Motion packet from the OLD epoch is rejected. */
    RLShadowCommand stale=packet(&s,302,41,NULL,0);stale.control_epoch=epoch;
    CHECK(!step(&s,302,&stale,1,&f));
    CHECK(s.recovery_request_sequence==40 && s.epoch==epoch+1u);
    for(uint32_t t=303;t<=309;t++){
        CHECK(!step(&s,t,NULL,1,&f));
        CHECK(s.adopted_sequence==before.adopted_sequence);
    }
    CHECK(!step(&s,310,NULL,1,&f));
    CHECK(s.adopted_sequence==40);
    CHECK(!memcmp(s.adopted_target,c.motion_preview.target,sizeof(s.adopted_target)));
    c=packet(&s,311,42,NULL,0);
    CHECK(!step(&s,311,&c,1,&f));
    CHECK(s.recovery_request_sequence==42 && !s.recovering && !s.fault);
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
    if(task_scenario==7 && task_iteration>=5)return 5;
    if(!task_snapshot.recovery_session)return task_iteration;
    return task_iteration>2?task_iteration-2:1;
}
static void check_task_observation(const float *o,int recovery)
{
    const unsigned received=received_iteration();
    assert(near(o[6],.01f*(float)received/1.5f));
    assert(near(o[10],.05f*(float)received));
    /* Previous and commands must come from NOW, not the delayed packet or
       the previous shadow prediction. All six channels have distinct values. */
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
       (task_scenario==6 && task_iteration>=4)){
        /* Invalid state reaches the Actor through the same two-packet delay;
           transport validity and inactive metadata are checked immediately. */
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
    if(task_scenario==2 && task_iteration>=4){state.recovering=0;state.epoch=8;}
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
    for(task_scenario=0;task_scenario<14;task_scenario++){
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
        CHECK(g_rl_shadow_debug.frame_begin==g_rl_shadow_debug.frame_end);
        CHECK(!(g_rl_shadow_debug.frame_begin&1));
    }
    printf("%s real Recovery shadow task: 8 routing/input scenarios + 6 latency/failure/wrap scenarios\n",
           failures==failures_before?"PASS":"FAIL");
}
#endif

int main(void)
{
    test_explicit_start_and_contracts();
    test_startup_previous_is_proxy();
    test_command_mode_boundaries();
    test_shadow_previous_is_read_only();
    for(unsigned phase=0;phase<5;phase++)schedule(100u+phase);
    schedule(UINT32_MAX-19u);
    test_wheel_session_cap();
    test_faults_latch_zero();
    test_quality_rejections();
    test_unqualified_session_never_hands_off();
    test_handoff();
#ifdef RL_RECOVERY_SHADOW_HOST
    test_real_recovery_shadow_task();
#endif
    if(failures){fprintf(stderr,"Recovery/handoff: %u failed cases\n",failures);return 1;}
    puts("PASS Recovery/handoff: decoder/skill, actual previous, transport/PD, latched zero, quality and epoch");
    return 0;
}
