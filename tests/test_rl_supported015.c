#include "rl_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void near(float a, float b) { assert(fabsf(a-b) < 2e-5f); }

static RLShadowSnapshot feedback(uint32_t now, RLControlState *state)
{
    RLShadowSnapshot s={0}; s.tick_ms=now; s.sequence=now+1u;
    s.motion.gravity_model[1]=-1; s.motion.command_height_m=.34f;
    s.feedback_valid_mask=0x3f; s.imu_update_sequence=1; s.imu_update_tick_ms=now;
    for(unsigned i=0;i<6;++i) s.feedback_receive_tick_ms[i]=now;
    s.leg_length_m[0]=s.leg_length_m[1]=.28f;
    RLControlFillPrevious(state,&s); return s;
}

int main(void)
{
    _Static_assert(RL_MOTION31_OUTPUT_VERSION==RL_MOTION31_OUTPUT_SUPPORTED015,
                   "compile this test with output version 2");
    const unsigned legs[4]={0,1,3,4};
    const float expected[6]={-.029116082936525345f,-.09327510744333267f,0,
                             .029116082936525345f,-.09327510744333267f,0};
    float action[6]={0}, target[6], again[6], requested[6], limited[6], obs[31];
    assert(!RLMotion31DecodeOutputAtHeight(action,.34f,target));
    for(unsigned i=0;i<6;++i) near(target[i],expected[i]);
    RLControlState state={.ramp_completed=1,.timing=1,.ready=1,.epoch=7};
    RLShadowSnapshot s=feedback(6000,&state);
    for(unsigned i=0;i<4;++i) s.motion.joint_position_rad[i]=target[legs[i]];
    assert(!RLMotion31ComputePdTorque(&s.motion,target,requested,limited));
    for(unsigned i=0;i<6;++i) near(requested[i],0);
    for(unsigned i=0;i<6;++i) s.motion.joint_velocity_rad_s[i]=(float)i-2;
    memcpy(s.motion.previous_target,target,sizeof(target));
    s.motion.previous_target[2]=20; s.motion.previous_target[5]=-20;
    assert(!RLMotion31BuildObservation(&s.motion,obs));
    for(unsigned i=0;i<4;++i) {
        near(obs[6+i],target[legs[i]]/1.5f);
        near(obs[16+i],target[legs[i]]/1.5f);
    }
    for(unsigned i=0;i<6;++i) near(obs[10+i],.05f*s.motion.joint_velocity_rad_s[i]);
    near(obs[20],.2f); near(obs[21],-.2f); near(obs[24],.68f);
    assert(!RLMotion31ComputePdTorque(&s.motion,target,requested,limited));
    for(unsigned i=0;i<4;++i) near(requested[legs[i]],-2*s.motion.joint_velocity_rad_s[legs[i]]);

    for(unsigned i=0;i<6;++i) action[i]=(i&1)?-2:2;
    assert(!RLMotion31DecodeOutput(action,target));
    assert(!RLMotion31DecodeOutput(action,again));
    for(unsigned i=0;i<6;++i) {
        const float width=(i==2 || i==5)?40:.15f;
        near(target[i],expected[i]+((i&1)?-width:width));
        near(again[i],target[i]); /* no accumulation or second Tanh */
    }
    assert(RLMotion31DecodeOutputAtHeight(action,.30f,target)<0);
    assert(RLMotion31DecodeOutputAtHeight(action,NAN,target)<0);
    action[0]=NAN; assert(RLMotion31DecodeOutput(action,target)<0); action[0]=.2f;
    assert(RLMotion31DecodeOutput(NULL,target)<0);
    assert(RLMotion31DecodeOutput(action,NULL)<0);

    s=feedback(6000,&state);
    RLShadowCommand c={.valid=1,.snapshot_sequence=100,.snapshot_tick_ms=6000,
        .output_tick_ms=6000,.control_epoch=7,.previous_source=RL_PREVIOUS_INIT_PROXY};
    c.action[0]=.2f; c.action[2]=.1f;
    assert(!RLMotion31BuildPreview(&s.motion,c.action,&c.preview));
    RLControlFrame frame;
    RLShadowCommand invalid=c; invalid.previous_source=RL_PREVIOUS_INFERENCE;
    assert(RLControlExecute(&state,6000,1,&s,&invalid,&frame)==1 && !state.active);
    invalid=c; invalid.preview.target[0]=1.5f*c.action[0];
    assert(RLControlExecute(&state,6000,1,&s,&invalid,&frame)==1 && !state.active);
    invalid=c; invalid.preview.observation[24]=.60f;
    assert(RLControlExecute(&state,6000,1,&s,&invalid,&frame)==1 && !state.active);
    assert(!RLControlExecute(&state,6000,1,&s,&c,&frame) && state.active);
    near(state.adopted_target[0],expected[0]+.03f);
    near(state.adopted_target[2],4);
    s=feedback(6005,&state);
    near(s.motion.previous_target[0],state.adopted_target[0]);
    assert(s.previous_source==RL_PREVIOUS_ADOPTED);
    s.motion.joint_position_rad[0]=state.adopted_target[0];
    s.motion.joint_velocity_rad_s[0]=1;
    assert(!RLControlExecute(&state,6005,1,&s,&c,&frame));
    near(frame.tau_pd_nm[0],-2); /* fresh feedback, unchanged 200Hz PD */
    puts("PASS supported015: center/scale, real dq, physical previous, PD and causal adoption");
    return 0;
}
