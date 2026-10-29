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
    assert(fabsf(RLControlLegRampStep(&state,2700,1,.16f)-.21f)<1e-6f);
    (void)RLControlStep(&state,2700,1,NULL);
    assert(!state.ready);
    assert(RLControlLegRampStep(&state,5200,1,.21f)==.26f);
    (void)RLControlStep(&state,5200,1,NULL);
    assert(state.timing && !state.ready);
    assert(RLControlLegRampStep(&state,10199,1,.20f)==.26f);
    (void)RLControlStep(&state,10199,1,NULL);
    assert(!state.ready);
    (void)RLControlStep(&state,10200,1,NULL);
    assert(state.ready && !state.active);
    (void)RLControlStep(&state,10201,0,NULL);
    assert(!state.ready && !state.ramp_started && !state.ramp_completed);
    assert(RLControlLegRampStep(&state,10300,1,.18f)==.18f);
}

int main(void)
{
    test_startup_timeline();
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
    assert(fabsf(out.tau_pd_nm[0]-6)<1e-5f);
    assert(fabsf(out.wheel_current_raw[0])<=8192 && fabsf(out.wheel_current_raw[1])<=8192);
    RLControlFillPrevious(&state,&s);
    assert(s.previous_source==RL_PREVIOUS_ADOPTED && s.adopted_sequence==101);
    assert(s.motion.previous_target[0]==.15f && s.motion.previous_target[2]==10);

    /* No new Actor action: PD still uses the new 200Hz feedback. */
    s.tick_ms=5005;s.motion.joint_position_rad[0]=.1f;s.motion.joint_velocity_rad_s[0]=2;
    assert(RLControlExecute(&state,5005,1,&s,&c,&out)==0);
    assert(fabsf(out.tau_pd_nm[0]+2)<1e-5f);
    assert(state.adopted_sequence==101 && out.adopted_target[0]==.15f);

    c=command(&state,102,5010,.2f);s=feedback(5010);RLControlFillPrevious(&state,&s);
    assert(RLControlExecute(&state,5010,1,&s,&c,&out)==0);
    assert(state.adopted_sequence==102 && state.adopted_target[0]==.3f);
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
