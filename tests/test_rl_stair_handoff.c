#include "rl_control.h"
#include "rl_policy_core.h"
#include "rl_stair.h"
#include "stair_actor.h"
#include "leg_const.hpp"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* RL_KINEMATICS_HOST links the actual five-bar solver. */
void getPhi(float, float, float, float, float, float, float,
            float *, float *, float *, float *);

static unsigned decoder_calls;
int __real_RLPolicyDecode(const RLPolicyDecodeConfig *, const float *, float, float *);
int __wrap_RLPolicyDecode(const RLPolicyDecodeConfig *cfg, const float *raw, float h, float *out)
{
    ++decoder_calls;
    return __real_RLPolicyDecode(cfg,raw,h,out);
}

static void step(RLStairHandoff *h, RLStairSample *s)
{
    s->tick_ms+=5;
    ++s->imu_sequence;
    RLStairUpdate(h,s);
}

static void flat(RLStairSample *s, float height)
{
    s->pitch=s->roll=s->pitch_rate=s->roll_rate=s->omega_squared=0;
    s->swing[0]=s->swing[1]=0;
    s->wheel_to_root_m[0]=s->wheel_to_root_m[1]=height;
    s->wheel_up_estimate_n[0]=s->wheel_up_estimate_n[1]=90;
    s->vx_estimate=1.2f;
}

static void hit(RLStairSample *s)
{
    s->pitch=.15f;
    s->pitch_rate=.4f;
    s->omega_squared=.16f;
    s->swing[0]=.30f;
    s->vx_estimate=.7f;
}

static void ready(RLStairHandoff *h, RLStairSample *s, uint32_t start)
{
    memset(h,0,sizeof(*h));
    *s=(RLStairSample){.tick_ms=start,.armed=1,.link_fresh=1,.feedback_fresh=1,
                       .can_enter=1,.vx_cmd=2.0f};
    flat(s,.40f);
    for (unsigned i=0;i<5;++i) step(h,s);
    hit(s);
    for (unsigned i=0;i<5;++i) {
        step(h,s);
        assert(h->phase==(i<4?RL_STAIR_IDLE:RL_STAIR_ACTIVE));
    }
    assert(h->impact_tick-h->impact_start_tick==RL_STAIR_IMPACT_MS);
}

static void fast_impact(void)
{
    RLStairHandoff h={0};
    RLStairSample s={.tick_ms=100,.armed=1,.link_fresh=1,.feedback_fresh=1,.can_enter=1,
                     .vx_cmd=2,.wheel_forward_mps={2.0f,2.0f}};
    flat(&s,.40f);
    for (unsigned i=0;i<12;++i) step(&h,&s);
    s.wheel_forward_mps[0]=1.0f;
    step(&h,&s);
    assert(h.phase==RL_STAIR_IDLE); /* wheel deceleration alone */
    for (unsigned i=0;i<5;++i) step(&h,&s);
    s.pitch_rate=.55f;s.swing[0]=.08f;
    step(&h,&s);
    assert(h.phase==RL_STAIR_IDLE); /* expired impact evidence */
    s.pitch_rate=0;s.swing[0]=0;s.wheel_forward_mps[0]=2.0f;
    step(&h,&s);
    s.wheel_forward_mps[0]=1.1f;
    step(&h,&s);
    const uint32_t wheel_edge=s.tick_ms;
    s.pitch=-.03f;s.pitch_rate=.55f;s.omega_squared=.55f*.55f;
    s.swing[1]=.08f; /* wrong side cannot corroborate */
    step(&h,&s);
    assert(h.phase==RL_STAIR_IDLE);
    s.swing[0]=.055f;
    step(&h,&s);
    assert(h.phase==RL_STAIR_ACTIVE && h.impact_start_tick==wheel_edge);
    assert(h.impact_tick-wheel_edge==10);
}

static void approach_anchor(void)
{
    RLStairHandoff h={0};
    RLStairSample s={.tick_ms=100,.armed=1,.link_fresh=1,.feedback_fresh=1,.can_enter=1,
                     .vx_cmd=1.50f};
    flat(&s,.36f);
    s.omega_squared=1.0f; /* level turn at 1 rad/s, no tilt rates */
    step(&h,&s);
    assert(h.have_anchor && h.anchor_tick==s.tick_ms);
    const uint32_t anchor=h.anchor_tick;

    /* Recent support cannot be refreshed while the body is rolling quickly. */
    s.roll_rate=.50f; s.omega_squared=1.25f;
    for (unsigned i=0;i<51;++i) step(&h,&s);
    assert(h.anchor_tick==anchor);
    s.roll_rate=0;
    hit(&s);
    for (unsigned i=0;i<5;++i) step(&h,&s);
    assert(h.phase==RL_STAIR_IDLE); /* previous upright sample is >250 ms old */

    flat(&s,.36f);
    s.omega_squared=1.0f;
    s.wheel_up_estimate_n[0]=0;
    step(&h,&s);
    assert(h.anchor_tick==anchor);
    s.wheel_up_estimate_n[0]=90;
    step(&h,&s);
    assert(h.anchor_tick==s.tick_ms);
    hit(&s);
    s.omega_squared+=1.0f;
    for (unsigned i=0;i<5;++i) step(&h,&s);
    assert(h.phase==RL_STAIR_ACTIVE && h.impact_tick-h.impact_start_tick==20u);
}

static void handling(void)
{
    RLStairHandoff h;
    RLStairSample s;
    ready(&h,&s,UINT32_MAX-37u);    /* actual edge spans the tick wrap */
    const uint32_t impact_tick=h.impact_tick;
    RLStairOnAdopt(&h,RL_ACTOR_MOTION,s.tick_ms);
    assert(!h.stair_adopted);
    RLStairOnAdopt(&h,RL_ACTOR_STAIR,s.tick_ms+5);
    assert(h.stair_adopted && h.stair_adopted_tick-impact_tick==5u);
    s.armed=0; s.can_enter=0;
    /* Raise the body .051 m with a triangular vertical velocity profile.
       No height or contact truth is passed to the controller. */
    for (unsigned i=0;i<64;++i) {
        s.accel_up=i<32?2.0f:-2.0f;
        step(&h,&s);
        assert(h.phase==RL_STAIR_ACTIVE);
    }
    s.accel_up=0;
    flat(&s,.25f);
    s.wheel_up_estimate_n[1]=0;
    for (unsigned i=0;i<110;++i) {
        step(&h,&s);
        assert(h.phase==RL_STAIR_ACTIVE);
    }
    assert(fabsf(h.tread_estimate_m[0]-.2012f)<.002f);
    s.wheel_up_estimate_n[1]=90;
    s.vx_estimate=0.0f;
    step(&h,&s);
    const uint32_t first_support=s.tick_ms;
    for (unsigned i=0;i<94;++i) {
        step(&h,&s);
        assert(h.phase==RL_STAIR_ACTIVE);
    }
    step(&h,&s);
    assert(h.phase==RL_STAIR_RETURN && h.supported_tick-first_support==475u);

    /* A loss while Motion is computing revokes permission before adoption. */
    s.wheel_up_estimate_n[0]=NAN;
    step(&h,&s);
    RLStairOnAdopt(&h,RL_ACTOR_MOTION,s.tick_ms);
    assert(h.phase==RL_STAIR_ACTIVE && !h.motion_adopted_tick);
    s.wheel_up_estimate_n[0]=90;
    for (unsigned i=0;i<96;++i) step(&h,&s);
    assert(h.phase==RL_STAIR_RETURN);
    RLStairOnAdopt(&h,RL_ACTOR_MOTION,s.tick_ms+5);
    assert(h.phase==RL_STAIR_WAIT_RELEASE && h.motion_adopted_tick-h.supported_tick==5u);
    s.armed=1; s.can_enter=1; hit(&s);
    for (unsigned i=0;i<8;++i) step(&h,&s);
    assert(h.phase==RL_STAIR_WAIT_RELEASE);
    s.armed=0; step(&h,&s);
    assert(h.phase==RL_STAIR_IDLE);
    s.armed=1; step(&h,&s);
    assert(h.phase==RL_STAIR_IDLE); /* must first see a new clean approach */

    /* Debounce rejects brief/negative signatures and requires arming. */
    RLStairReset(&h,0);
    flat(&s,.40f); s.armed=0;
    for (unsigned i=0;i<6;++i) step(&h,&s);
    hit(&s);
    for (unsigned i=0;i<8;++i) step(&h,&s);
    assert(h.phase==RL_STAIR_IDLE);
    s.armed=1; flat(&s,.40f); step(&h,&s);
    hit(&s); step(&h,&s); step(&h,&s);
    s.pitch=-.15f; step(&h,&s);
    hit(&s);
    for (unsigned i=0;i<4;++i) step(&h,&s);
    assert(h.phase==RL_STAIR_IDLE);

    ready(&h,&s,100u);
    s.link_fresh=0; step(&h,&s);
    assert(h.phase==RL_STAIR_STOPPED && h.fault==RL_STAIR_STALE);
    s.link_fresh=1; step(&h,&s);
    assert(h.phase==RL_STAIR_STOPPED);
    RLStairReset(&h,1);
    assert(h.phase==RL_STAIR_WAIT_RELEASE); /* stop with arm still held */
    ready(&h,&s,100u);
    s.feedback_fresh=0; step(&h,&s);
    assert(h.phase==RL_STAIR_STOPPED && h.fault==RL_STAIR_STALE);
    ready(&h,&s,100u);
    s.pitch=1.0f; step(&h,&s);
    assert(h.phase==RL_STAIR_STOPPED && h.fault==RL_STAIR_POSE);

    /* Same-floor support in the target pose is not a successful ascent.
       Initial .40m -> .25m, lowering the body without rising the wheel plane. */
    ready(&h,&s,100u);
    RLStairOnAdopt(&h,RL_ACTOR_STAIR,s.tick_ms);
    for (unsigned i=0;i<100;++i) {
        s.accel_up=i<50?-2.4f:2.4f;
        step(&h,&s);
    }
    s.accel_up=0;
    flat(&s,.25f);
    for (unsigned i=0;i<110;++i) step(&h,&s);
    assert(fabsf(h.tread_estimate_m[0])<.003f && h.phase==RL_STAIR_ACTIVE);
    while (s.tick_ms-h.impact_tick<RL_STAIR_TIMEOUT_MS) step(&h,&s);
    assert(h.phase==RL_STAIR_STOPPED && h.fault==RL_STAIR_TIMEOUT);
}

/* Establish a repeatable stand-above-tread state using the same causal
   simulated acceleration and wheel geometry as the end-to-end gate test. */
static void recovered(RLStairHandoff *h,RLStairSample *s)
{
    ready(h,s,200u);
    RLStairOnAdopt(h,RL_ACTOR_STAIR,s->tick_ms);
    for (unsigned i=0;i<64;++i) {
        s->accel_up=i<32?2.0f:-2.0f;
        step(h,s);
    }
    s->accel_up=0;
    flat(s,.25f);
    step(h,s);
}

static void support_window(void)
{
    RLStairHandoff h;
    RLStairSample s;
    recovered(&h,&s);
    const uint32_t first=s.tick_ms;
    for (unsigned i=1;i<=100;++i) {
        /* 20 ms loss, then another 5 ms: credit neither, and do not impose
           a second gap threshold in addition to 475 ms of genuine support. */
        s.wheel_up_estimate_n[1]=(i>=9 && i<=12) || i==75 ? 0 : 90;
        step(&h,&s);
        assert(h.phase==(i==100 ? RL_STAIR_RETURN : RL_STAIR_ACTIVE));
    }
    assert(h.supported_tick-first==500u);
    assert(h.support.duration_ms==500u && h.support.loaded_ms==475u);

    /* The car can stand and turn or pitch under Motion. The current packet
       still must have support, and a dropout cannot earn the elapsed time. */
    s.wheel_up_estimate_n[1]=0;
    step(&h,&s);
    RLStairOnAdopt(&h,RL_ACTOR_MOTION,s.tick_ms);
    assert(h.phase==RL_STAIR_ACTIVE && !h.motion_adopted_tick);
    s.wheel_up_estimate_n[1]=90;
    for (unsigned i=2;i<10;++i) {
        step(&h,&s);
        assert(h.phase==RL_STAIR_ACTIVE); /* six recent misses = 94% */
    }
    step(&h,&s);
    assert(h.phase==RL_STAIR_RETURN && h.supported_tick-first==550u);

    /* Preserve Motion's existing leg envelope without discarding valid load
       history. Recovering that envelope can authorize a switch immediately. */
    s.swing[0]=0.9f;
    step(&h,&s);
    assert(h.phase==RL_STAIR_ACTIVE && h.support.loaded_ms>=475u);
    s.swing[0]=0.65f;
    s.pitch=0.30f;
    s.roll=-0.18f;
    s.omega_squared=2.25f; /* e.g. 1.50 rad/s yaw */
    s.vx_estimate=0.0f;
    step(&h,&s);
    assert(h.phase==RL_STAIR_RETURN);

    /* Sustained loss ages out any previous support. */
    s.wheel_up_estimate_n[0]=0;
    for (unsigned i=0;i<102;++i) step(&h,&s);
    assert(h.phase==RL_STAIR_ACTIVE && !h.support.loaded_ms);
    s.wheel_up_estimate_n[0]=90;
    step(&h,&s);
    const uint32_t restored=s.tick_ms;
    for (unsigned i=0;i<94;++i) {
        step(&h,&s);
        assert(h.phase==RL_STAIR_ACTIVE);
    }
    step(&h,&s);
    assert(h.phase==RL_STAIR_RETURN && h.supported_tick-restored==475u);

    /* At an irregular 7 ms cadence, confirm on the first usable control tick
       after 475 ms, not after filling the 500 ms history. */
    recovered(&h,&s);
    const uint32_t jitter_begin=s.tick_ms;
    for (unsigned i=1;i<=68;++i) {
        s.tick_ms+=7;
        ++s.imu_sequence;
        RLStairUpdate(&h,&s);
        assert(h.phase==(i==68 ? RL_STAIR_RETURN : RL_STAIR_ACTIVE));
    }
    assert(h.supported_tick-jitter_begin==476u && h.support.duration_ms==476u);
}

static void lift_then_return(void)
{
    RLStairHandoff h;
    RLStairSample s;
    ready(&h,&s,100u);
    RLStairOnAdopt(&h,RL_ACTOR_STAIR,s.tick_ms);
    /* .20 m tread + .18 m crouched body. From the old .40 m root datum
       descend .020 m while putting both wheels on the higher surface. */
    for (unsigned i=0;i<40;++i) {
        s.accel_up=i<20?-2.0f:2.0f;
        step(&h,&s);
    }
    s.accel_up=0;
    flat(&s,.18f);
    s.pitch=.20f;
    s.roll=.12f;
    s.omega_squared=4.0f;
    s.vx_estimate=-0.18f;
    for (unsigned i=0;i<102;++i) {
        step(&h,&s);
        assert(h.phase==RL_STAIR_ACTIVE);
    }
    assert(h.support.loaded_ms==500u);
    assert(fabsf(h.tread_estimate_m[0]-.20f)<.002f);
    /* Raise the body by roughly .023 m under load. A brief pitch/yaw transient
       and vertical acceleration over the old settling limit do not delay handoff. */
    for (unsigned i=0;i<24;++i) {
        s.accel_up=i<12?6.4f:-6.4f;
        const float t=(i+1)*.005f;
        const float left_time=.12f-t;
        const float dh=i<12 ? 3.2f*t*t : .02304f-3.2f*left_time*left_time;
        s.wheel_to_root_m[0]=s.wheel_to_root_m[1]=.18f+dh;
        step(&h,&s);
        if (s.wheel_to_root_m[0] >= RL_STAIR_RETURN_MIN_BODY_M) {
            assert(h.phase==RL_STAIR_RETURN && h.support.loaded_ms==500u);
            return;
        }
        assert(h.phase==RL_STAIR_ACTIVE);
    }
    assert(0);
}

static void hub(float a,float b,float *point,float *links)
{
    float length,theta;
    getPhi(a,b,L_1,L_2,L_3,L_4,L_5,&links[0],&links[1],&length,&theta);
    point[0]=length*cosf(theta); point[1]=length*sinf(theta);
}

static void measured_force(void)
{
    /* Virtual work using finite differences of the real forward kinematics:
       verify the distinct hip/active contributions and projected force sign. */
    const float p=.14f, r=.04f, a=2.26f, b=.88f, delta=.001f;
    const float fx=-21, fy=85;
    float q[2],l[2],plus[2],minus[2],unused[2],tau[2];
    hub(a,b,q,l);
    for (unsigned i=0;i<2;++i) {
        hub(a+(i==0?delta:0),b+(i==1?delta:0),plus,unused);
        hub(a-(i==0?delta:0),b-(i==1?delta:0),minus,unused);
        tau[i]=(fx*(plus[0]-minus[0])+fy*(plus[1]-minus[1]))/(2*delta);
    }
    const float vertical=(-sinf(p)*fx+cosf(p)*fy)*cosf(r);
    assert(fabsf(RLStairWheelLoad(a,b,l[0],l[1],tau[0],tau[1],p,r)-vertical)<.12f);
    assert(isnan(RLStairWheelLoad(a,b,1.0f,1.0f,tau[0],tau[1],p,r)));
}

static void feedback_age(RLShadowSnapshot *s, uint32_t tick, uint32_t seq)
{
    s->tick_ms=s->imu_update_tick_ms=tick;
    s->sequence=seq;
    s->imu_update_sequence=seq;
    s->feedback_valid_mask=0x3f;
    for (unsigned i=0;i<6;++i) s->feedback_receive_tick_ms[i]=tick;
}

static RLShadowCommand produce(const RLControlState *ctrl, RLShadowSnapshot *s, uint8_t actor)
{
    RLControlFillPrevious(ctrl,s);
    RLShadowCommand c={.snapshot_tick_ms=s->tick_ms,.output_tick_ms=s->tick_ms,
        .snapshot_sequence=s->sequence,.valid=1,.actor=actor,.control_epoch=ctrl->epoch,
        .previous_source=s->previous_source,.previous_sequence=s->adopted_sequence};
    if (actor==RL_ACTOR_STAIR) {
        assert(!RLStairBuildObservation(&s->motion,c.preview.observation));
        assert(!StairActorPhysical(c.preview.observation,c.action,c.preview.target));
    } else {
        c.action[0]=.35f;c.action[1]=.3f;c.action[2]=-.25f;
        assert(!RLMotion31BuildPreview(&s->motion,c.action,&c.preview));
    }
    return c;
}

static void adopted_blend(const RLControlState *ctrl,const float origin[6],
                          const RLShadowCommand *c, float fraction)
{
    for (unsigned i=0;i<6;++i) {
        assert(fabsf(ctrl->adopted_target[i]-(origin[i]+fraction*(c->preview.target[i]-origin[i])))<1e-5f);
    }
}

static void execution(void)
{
    RLControlState ctrl={.active=1,.timing=1,.ready=1,.ramp_started=1,.ramp_completed=1,
                         .epoch=5,.adopted_sequence=100,.adopted_tick_ms=20,
                         .adopted_target={.30f,.10f,-30.0f,-.30f,.12f,-32.0f}};
    RLShadowSnapshot s={.leg_length_m={.28f,.28f},.leg_spread_rad={1.0f,1.0f},
                        .motion={.gravity_model={0,-1,0},.command_height_m=.40f}};
    RLControlFrame f;
    float origin[6];memcpy(origin,ctrl.adopted_target,sizeof(origin));
    feedback_age(&s,25,101);
    RLShadowCommand c=produce(&ctrl,&s,RL_ACTOR_MOTION);
    assert(RLControlSelectActor(&ctrl,RL_ACTOR_STAIR));
    assert(!RLControlExecute(&ctrl,25,1,&s,&c,&f));
    assert(f.owner==RL_OWNER_RL && !memcmp(ctrl.adopted_target,origin,sizeof(origin)));
    c.control_epoch=ctrl.epoch; /* still wrong Actor even with a matching epoch */
    assert(!RLControlExecute(&ctrl,25,1,&s,&c,&f) && ctrl.adopted_sequence==100);

    for (unsigned i=1;i<=3;++i) {
        feedback_age(&s,25+10*i,101+i);
        c=produce(&ctrl,&s,RL_ACTOR_STAIR);
        const unsigned calls_after_inference=decoder_calls;
        assert(c.previous_sequence==ctrl.adopted_sequence);
        RLShadowCommand bad=c;
        bad.previous_sequence--;
        assert(!RLControlExecute(&ctrl,s.tick_ms,1,&s,&bad,&f));
        assert(ctrl.adopted_sequence==c.previous_sequence);
        assert(!RLControlExecute(&ctrl,s.tick_ms,1,&s,&c,&f));
        assert(decoder_calls==calls_after_inference); /* no re-decode at PD/adoption */
        assert(f.owner==RL_OWNER_STAIR && ctrl.adopted_sequence==c.snapshot_sequence);
        adopted_blend(&ctrl,origin,&c,(float)i/3.0f);
        RLControlFillPrevious(&ctrl,&s);
        assert(s.previous_source==RL_PREVIOUS_ADOPTED && s.controller_owner==RL_OWNER_STAIR);
        assert(!memcmp(s.motion.previous_target,f.adopted_target,sizeof(origin)));
        for (unsigned w=0;w<2;++w) assert(fabsf(f.wheel_current_raw[w])<=RL_CONTROL_WHEEL_RAW_LIMIT);
        for (unsigned j=0;j<4;++j) assert(fabsf(f.final_motor_nm[j])<=40.0f);
    }
    assert(decoder_calls==3);
    memcpy(origin,ctrl.adopted_target,sizeof(origin));
    assert(RLControlSelectActor(&ctrl,RL_ACTOR_MOTION));
    feedback_age(&s,60,107);
    assert(!RLControlExecute(&ctrl,60,1,&s,&c,&f) && f.owner==RL_OWNER_STAIR);
    s.motion.command_height_m=.25f;
    c=produce(&ctrl,&s,RL_ACTOR_MOTION);
    assert(!memcmp(s.motion.previous_target,origin,sizeof(origin)));
    assert(!RLControlExecute(&ctrl,60,1,&s,&c,&f));
    assert(f.owner==RL_OWNER_RL && ctrl.ready && ctrl.adopted_height_m==.25f);
    adopted_blend(&ctrl,origin,&c,1.0f/3.0f);
}

int main(void)
{
    fast_impact();
    approach_anchor();
    handling();
    support_window();
    lift_then_return();
    measured_force();
    execution();
    puts("PASS atomic Stair detector, bilateral load, epoch, decode, adopted previous and both handoffs");
    return 0;
}
