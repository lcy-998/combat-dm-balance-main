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

int main(void)
{
    RLControlState state={0};RLControlLegRamp ramp={.completed=1};RLControlFrame out;
    RLShadowSnapshot s=feedback(0);RLControlFillPrevious(&state,&s);
    assert(s.previous_source==RL_PREVIOUS_INIT_PROXY && !s.adopted_sequence);
    assert(RLControlExecute(&state,&ramp,0,1,&s,NULL,&out)==1);
    assert(!state.active && state.epoch==1);
    RLShadowCommand c=command(&state,101,5000,.1f);
    s=feedback(5000);RLControlFillPrevious(&state,&s);
    assert(RLControlExecute(&state,&ramp,5000,1,&s,&c,&out)==0);
    assert(state.active && out.owner==RL_OWNER_RL && state.adopted_sequence==101);
    assert(fabsf(out.tau_pd_nm[0]-6)<1e-5f);
    assert(fabsf(out.wheel_current_raw[0])<=8192 && fabsf(out.wheel_current_raw[1])<=8192);
    RLControlFillPrevious(&state,&s);
    assert(s.previous_source==RL_PREVIOUS_ADOPTED && s.adopted_sequence==101);
    assert(s.motion.previous_target[0]==.15f && s.motion.previous_target[2]==10);

    /* No new Actor action: PD still uses the new 200Hz feedback. */
    s.tick_ms=5005;s.motion.joint_position_rad[0]=.1f;s.motion.joint_velocity_rad_s[0]=2;
    assert(RLControlExecute(&state,&ramp,5005,1,&s,&c,&out)==0);
    assert(fabsf(out.tau_pd_nm[0]+2)<1e-5f);
    assert(state.adopted_sequence==101 && out.adopted_target[0]==.15f);

    c=command(&state,102,5010,.2f);s=feedback(5010);RLControlFillPrevious(&state,&s);
    assert(RLControlExecute(&state,&ramp,5010,1,&s,&c,&out)==0);
    assert(state.adopted_sequence==102 && state.adopted_target[0]==.3f);
    const float adopted=state.adopted_target[0];
    c=command(&state,103,5015,.9f);c.previous_source=RL_PREVIOUS_INIT_PROXY;c.previous_sequence=0;
    s.tick_ms=5015;
    assert(RLControlExecute(&state,&ramp,5015,1,&s,&c,&out)==0);
    assert(state.adopted_sequence==102 && state.adopted_target[0]==adopted);

    s=feedback(5035);RLControlFillPrevious(&state,&s);
    assert(RLControlExecute(&state,&ramp,5035,1,&s,&c,&out)<0);
    assert(!state.active && state.fault==RL_FAULT_COMMAND);
    c=command(&state,104,5040,0);s=feedback(5040);RLControlFillPrevious(&state,&s);
    assert(RLControlExecute(&state,&ramp,5040,1,&s,&c,&out)==1 && !state.active);
    assert(RLControlExecute(&state,&ramp,5045,0,&s,NULL,&out)==1);
    assert(state.fault && !ramp.completed);
    RLControlReset(&state,&ramp);
    assert(!state.fault);

    /* Invalid input or partial output never commits a six-axis target. */
    ramp.completed=1;s=feedback(5100);RLControlFillPrevious(&state,&s);
    assert(RLControlExecute(&state,&ramp,5100,1,&s,NULL,&out)==1);
    c=command(&state,200,10100,.1f);s=feedback(10100);RLControlFillPrevious(&state,&s);
    s.motion.joint_position_rad[0]=1e30f;
    assert(RLControlExecute(&state,&ramp,10100,1,&s,&c,&out)<0);
    assert(state.adopted_sequence==0 && state.fault==RL_FAULT_INPUT && !state.active);
    puts("RL execution: actual adoption, fresh PD, causal previous and latched faults passed");
    return 0;
}
