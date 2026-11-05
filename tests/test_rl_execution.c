#include "rl_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static RLShadowSnapshot feedback(uint32_t now)
{
    RLShadowSnapshot s={0};
    s.tick_ms=now;s.sequence=now+1u;s.chassis_ready=1;
    s.motion.gravity_model[1]=-1;s.motion.command_height_m=.34f;
    s.feedback_valid_mask=0x3fu;s.imu_update_sequence=1;s.imu_update_tick_ms=now;
    for (unsigned i=0;i<6u;++i) s.feedback_receive_tick_ms[i]=now;
    s.leg_length_m[0]=s.leg_length_m[1]=.28f;
    s.leg_spread_rad[0]=s.leg_spread_rad[1]=.5f;
    return s;
}

static RLShadowCommand command(RLControlState *state,uint32_t seq,uint32_t now,float action)
{
    RLShadowSnapshot s=feedback(now);RLControlFillPrevious(state,&s);
    RLShadowCommand c={0};c.valid=1;c.snapshot_sequence=seq;
    c.snapshot_tick_ms=c.output_tick_ms=now;c.control_epoch=s.control_epoch;
    c.previous_sequence=s.adopted_sequence;c.previous_source=s.previous_source;
    c.action[0]=action;c.action[2]=.1f;c.action[5]=-.1f;
    assert(RLMotion31BuildPreview(&s.motion,c.action,&c.preview)==0);
    return c;
}

static void test_startup_timeline(void)
{
    RLControlState state={0};
    assert(RLControlLegRampStep(&state,100,0,.16f)==.16f);
    assert(RLControlLegRampStep(&state,200,1,.16f)==.16f);
    (void)RLControlStep(&state,200,1,NULL);
    assert(!state.timing && !state.ready);
    assert(fabsf(RLControlLegRampStep(&state,2700,1,.16f)-.22f)<1e-6f);
    (void)RLControlStep(&state,2700,1,NULL);
    assert(!state.ready);
    assert(RLControlLegRampStep(&state,5200,1,.21f)==RL_CONTROL_LEG_TARGET_M);
    (void)RLControlStep(&state,5200,1,NULL);
    assert(state.timing && !state.ready);
    assert(RLControlLegRampStep(&state,10199,1,.20f)==RL_CONTROL_LEG_TARGET_M);
    (void)RLControlStep(&state,10199,1,NULL);
    assert(!state.ready);
    (void)RLControlStep(&state,10200,1,NULL);
    assert(state.ready && !state.active);
    (void)RLControlStep(&state,10201,0,NULL);
    assert(!state.ready && !state.ramp_started && !state.ramp_completed);
    assert(RLControlLegRampStep(&state,10300,1,.18f)==.18f);
}

static void test_counter_boundaries(void)
{
    RLControlState state={.ramp_completed=1,.timing=1,.ready=1,.epoch=7};
    RLControlFrame frame;
    RLShadowSnapshot s=feedback(6000);RLControlFillPrevious(&state,&s);
    RLShadowCommand c=command(&state,0x80000020u,6000,.1f);
    assert(!RLControlExecute(&state,6000,1,&s,&c,&frame));
    assert(state.active && state.adopted_sequence==0x80000020u);

    /* Completed startup wait must not become false after uint32 clock wrap. */
    state=(RLControlState){.ramp_completed=1,.timing=1,.ready=1,.active=1,.epoch=7,
        .wait_start_tick=200,.adopted_tick_ms=200,.adopted_sequence=99};
    s=feedback(205);RLControlFillPrevious(&state,&s);
    assert(!RLControlExecute(&state,205,1,&s,NULL,&frame));
    assert(state.active && state.ready);

    /* A new causal sequence immediately after wrap is still newer. */
    state=(RLControlState){.ramp_completed=1,.timing=1,.ready=1,.active=1,.epoch=7,
        .wait_start_tick=UINT32_MAX-6000u,.adopted_tick_ms=UINT32_MAX-2u,
        .adopted_sequence=UINT32_MAX-1u};
    s=feedback(2);RLControlFillPrevious(&state,&s);
    c=command(&state,1,2,.1f);
    assert(!RLControlExecute(&state,2,1,&s,&c,&frame));
    assert(state.adopted_sequence==1 && !state.fault);
}

static void test_schedule_phases(void)
{
    unsigned cases=0;
    for(unsigned wrap=0;wrap<2;++wrap)for(unsigned order=0;order<2;++order)
    for(unsigned phase=0;phase<10;++phase)for(unsigned cost=0;cost<10;++cost) {
        const uint32_t base=wrap?UINT32_MAX-90u:10000u;
        RLControlState state={.ramp_completed=1,.timing=1,.ready=1,.epoch=7,
            .wait_start_tick=base-6000u};
        RLControlFrame frame;
        uint32_t sequence=100;
        RLShadowSnapshot sample=feedback(base);sample.sequence=sequence++;
        RLControlFillPrevious(&state,&sample);
        RLShadowCommand pending={0},delivered={0};
        unsigned inflight=0,completion=0,adoptions=0;
        uint32_t previous=0;
        for(unsigned tick=0;tick<400;++tick) {
            const uint32_t now=base+tick;
            for(unsigned slot=0;slot<2;++slot) {
                if(slot==order) {
                    if(inflight && tick==completion){delivered=pending;inflight=0;}
                    if(tick%10==phase) {
                        assert(!inflight);
                        pending=(RLShadowCommand){.valid=1,.snapshot_sequence=sample.sequence,
                            .snapshot_tick_ms=sample.tick_ms,.output_tick_ms=now+cost,
                            .control_epoch=sample.control_epoch,.previous_source=sample.previous_source,
                            .previous_sequence=sample.adopted_sequence};
                        pending.action[0]=.1f;
                        assert(!RLMotion31BuildPreview(&sample.motion,pending.action,&pending.preview));
                        if(cost){completion=tick+cost;inflight=1;}else delivered=pending;
                    }
                } else if(tick%5==0) {
                    RLShadowSnapshot current=feedback(now);current.sequence=sequence++;
                    RLControlFillPrevious(&state,&current);
                    const int rc=RLControlExecute(&state,now,1,&current,&delivered,&frame);
                    assert(rc>=0 && !state.fault);
                    if(state.active) {
                        assert(rc==0 && frame.owner==RL_OWNER_RL);
                        assert(fabsf(frame.tau_pd_nm[0]-40*state.adopted_target[0])<1e-5f);
                        if(state.adopted_sequence!=previous){++adoptions;previous=state.adopted_sequence;}
                    }
                    RLControlFillPrevious(&state,&current);sample=current;
                }
            }
        }
        assert(state.active && adoptions>=10);
        ++cases;
    }
    assert(cases==400);
    puts("PASS 400 Actor/PD phase, inference-cost, owner-order and clock-wrap schedules");
}

int main(void)
{
    test_startup_timeline();
    test_counter_boundaries();
    test_schedule_phases();
    RLControlState state={.ramp_completed=1};RLControlFrame out;
    RLShadowSnapshot s=feedback(0);RLControlFillPrevious(&state,&s);
    assert(s.previous_source==RL_PREVIOUS_INIT_PROXY && !s.adopted_sequence);
    assert(RLControlExecute(&state,0,1,&s,NULL,&out)==1);
    assert(!state.active && state.epoch==1);
    assert(!state.ready);
    (void)RLControlStep(&state,4999,1,NULL);
    assert(!state.ready);
    (void)RLControlStep(&state,5000,1,NULL);
    assert(state.ready && !state.active);
    RLShadowCommand c=command(&state,101,5000,.1f);
    s=feedback(5000);RLControlFillPrevious(&state,&s);
    assert(RLControlExecute(&state,5000,1,&s,&c,&out)==0);
    assert(state.active && out.owner==RL_OWNER_RL && state.adopted_sequence==101);
    assert(fabsf(out.tau_pd_nm[0]-40.0f*(-.02911608294f+.015f))<1e-5f);
    assert(fabsf(out.wheel_current_raw[0])<=8192 && fabsf(out.wheel_current_raw[1])<=8192);
    RLControlFillPrevious(&state,&s);
    assert(s.previous_source==RL_PREVIOUS_ADOPTED && s.adopted_sequence==101);
    assert(fabsf(s.motion.previous_target[0]-(-.02911608294f+.015f))<1e-6f);
    assert(s.motion.previous_target[2]==4);

    /* No new Actor action: PD still uses the new 200Hz feedback. */
    s.tick_ms=5005;s.motion.joint_position_rad[0]=state.adopted_target[0];s.motion.joint_velocity_rad_s[0]=2;
    assert(RLControlExecute(&state,5005,1,&s,&c,&out)==0);
    assert(fabsf(out.tau_pd_nm[0]+4)<1e-5f);
    assert(state.adopted_sequence==101);
    assert(fabsf(out.adopted_target[0]-(-.02911608294f+.015f))<1e-6f);

    c=command(&state,102,5010,.2f);s=feedback(5010);RLControlFillPrevious(&state,&s);
    assert(RLControlExecute(&state,5010,1,&s,&c,&out)==0);
    assert(state.adopted_sequence==102);
    assert(fabsf(state.adopted_target[0]-(-.02911608294f+.03f))<1e-6f);
    const float adopted=state.adopted_target[0];
    c=command(&state,103,5015,.9f);c.previous_source=RL_PREVIOUS_INIT_PROXY;c.previous_sequence=0;
    s.tick_ms=5015;
    assert(RLControlExecute(&state,5015,1,&s,&c,&out)==0);
    assert(state.adopted_sequence==102 && state.adopted_target[0]==adopted);

    s=feedback(5035);RLControlFillPrevious(&state,&s);
    assert(RLControlExecute(&state,5035,1,&s,&c,&out)<0);
    assert(!state.active && state.fault==RL_FAULT_COMMAND);
    c=command(&state,104,5040,0);s=feedback(5040);RLControlFillPrevious(&state,&s);
    assert(RLControlExecute(&state,5040,1,&s,&c,&out)==1 && !state.active);
    assert(RLControlExecute(&state,5045,0,&s,NULL,&out)==1);
    assert(state.fault && !state.ready && !state.timing);
    RLControlReset(&state);
    assert(!state.fault);

    /* Invalid input or partial output never commits a six-axis target. */
    state.ramp_completed=1;s=feedback(5100);RLControlFillPrevious(&state,&s);
    assert(RLControlExecute(&state,5100,1,&s,NULL,&out)==1);
    c=command(&state,200,10100,.1f);s=feedback(10100);RLControlFillPrevious(&state,&s);
    s.motion.joint_position_rad[0]=1e30f;
    assert(RLControlExecute(&state,10100,1,&s,&c,&out)<0);
    assert(state.adopted_sequence==0 && state.fault==RL_FAULT_INPUT && !state.active);
    puts("RL execution: actual adoption, fresh PD, causal previous and latched faults passed");
    return 0;
}
