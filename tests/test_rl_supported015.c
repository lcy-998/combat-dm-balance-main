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
    assert(!strcmp(RL_MOTION31_ACTION_CONTRACT,
                   "motion_multiheight_support_center_poly5_015_80_v1"));
    const unsigned legs[4]={0,1,3,4};
    const float expected[6]={-0.13097405433655f,-0.15117678046227f,0,
                             0.13097405433655f,-0.15117678046227f,0};
    float action[6]={0}, target[6], again[6], requested[6], limited[6], obs[31];
    assert(!RLMotion31DecodeOutput(action,.34f,target));
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
    s.motion.previous_target_valid=0;
    assert(RLMotion31BuildObservation(&s.motion,obs)<0);
    s.motion.previous_target_valid=1;
    assert(!RLMotion31ComputePdTorque(&s.motion,target,requested,limited));
    for(unsigned i=0;i<4;++i) near(requested[legs[i]],-2*s.motion.joint_velocity_rad_s[legs[i]]);

    for(unsigned i=0;i<6;++i) action[i]=(i&1)?-2:2;
    assert(!RLMotion31DecodeOutput(action,.34f,target));
    assert(!RLMotion31DecodeOutput(action,.34f,again));
    for(unsigned i=0;i<6;++i) {
        const float width=(i==2 || i==5)?80:.15f;
        near(target[i],expected[i]+((i&1)?-width:width));
        near(again[i],target[i]); /* no accumulation or second Tanh */
    }
    assert(RLMotion31DecodeOutput(action,.29f,target)<0);
    assert(RLMotion31DecodeOutput(action,NAN,target)<0);
    action[0]=NAN; assert(RLMotion31DecodeOutput(action,.34f,target)<0); action[0]=.2f;
    assert(RLMotion31DecodeOutput(NULL,.34f,target)<0);
    assert(RLMotion31DecodeOutput(action,.34f,NULL)<0);

    s=feedback(6000,&state);
    RLShadowCommand c={.valid=1,.snapshot_sequence=100,.snapshot_tick_ms=6000,
        .output_tick_ms=6000,.control_epoch=7,.previous_source=RL_PREVIOUS_INIT_PROXY};
    c.action[0]=.2f; c.action[2]=.1f;
    assert(!RLMotion31BuildPreview(&s.motion,c.action,&c.preview));
    RLControlFrame frame;
    RLShadowCommand invalid=c; invalid.previous_source=3u /* retired shadow source */;
    assert(RLControlExecute(&state,6000,1,&s,&invalid,&frame)==1 && !state.active);
    invalid=c; invalid.preview.target[0]=1.5f*c.action[0];
    assert(RLControlExecute(&state,6000,1,&s,&invalid,&frame)==1 && !state.active);
    invalid=c; invalid.preview.target[2]=40.0f*c.action[2];
    assert(RLControlExecute(&state,6000,1,&s,&invalid,&frame)==1 && !state.active);
    invalid=c; invalid.preview.observation[24]=.60f;
    assert(RLControlExecute(&state,6000,1,&s,&invalid,&frame)==1 && !state.active);
    assert(!RLControlExecute(&state,6000,1,&s,&c,&frame) && state.active);
    near(state.adopted_target[0],expected[0]+.03f);
    near(state.adopted_target[2],8);
    s=feedback(6005,&state);
    near(s.motion.previous_target[0],state.adopted_target[0]);
    assert(s.previous_source==RL_PREVIOUS_ADOPTED);
    s.motion.joint_position_rad[0]=state.adopted_target[0];
    s.motion.joint_velocity_rad_s[0]=1;
    assert(!RLControlExecute(&state,6005,1,&s,&c,&frame));
    near(frame.tau_pd_nm[0],-2); /* fresh feedback, unchanged 200Hz PD */

    /* Each leg's own P/D channels, both signs, all torque clamps. */
    s=feedback(6010,&state);
    for(unsigned i=0;i<4;++i) {
        const unsigned ch=legs[i];
        for(int sign=-1;sign<=1;sign+=2) {
            float p[6]={0},v[6]={0},t[6]={0};
            t[ch]=.1f*(float)sign;
            memset(s.motion.joint_position_rad,0,sizeof(s.motion.joint_position_rad));
            memset(s.motion.joint_velocity_rad_s,0,sizeof(s.motion.joint_velocity_rad_s));
            assert(!RLMotion31ComputePdTorque(&s.motion,t,p,v));
            for(unsigned j=0;j<6;++j)near(p[j],j==ch?4.0f*(float)sign:0);
            t[ch]=0;s.motion.joint_velocity_rad_s[ch]=(float)sign;
            assert(!RLMotion31ComputePdTorque(&s.motion,t,p,v));
            for(unsigned j=0;j<6;++j)near(p[j],j==ch?-2.0f*(float)sign:0);
            t[ch]=100*(float)sign;
            assert(!RLMotion31ComputePdTorque(&s.motion,t,p,v));
            near(v[ch],40*(float)sign);
        }
    }
    memset(s.motion.joint_position_rad,0,sizeof(s.motion.joint_position_rad));
    memset(s.motion.joint_velocity_rad_s,0,sizeof(s.motion.joint_velocity_rad_s));
    for(unsigned ch=2;ch<6;ch+=3) {
        for(int sign=-1;sign<=1;sign+=2) {
            float p[6]={0},v[6]={0},t[6]={0};t[ch]=10*(float)sign;
            assert(!RLMotion31ComputePdTorque(&s.motion,t,p,v));near(p[ch],2*(float)sign);
            t[ch]=100*(float)sign;
            assert(!RLMotion31ComputePdTorque(&s.motion,t,p,v));near(v[ch],5.5f*(float)sign);
        }
    }
    /* A failure in any last channel/compensation leaves the adopted six-axis
       target and sequence untouched; only the existing fault/owner changes. */
    const RLControlState committed=state;
    for(unsigned mode=0;mode<4;++mode) {
        state=committed;s=feedback(6010,&state);
        c=(RLShadowCommand){.valid=1,.snapshot_sequence=101,.snapshot_tick_ms=6010,
            .output_tick_ms=6010,.control_epoch=state.epoch,.previous_source=RL_PREVIOUS_ADOPTED,
            .previous_sequence=state.adopted_sequence};
        assert(!RLMotion31BuildPreview(&s.motion,c.action,&c.preview));
        if(mode==0)s.motion.joint_velocity_rad_s[5]=NAN;
        if(mode==1)s.motion.command_height_m=.29f;
        if(mode==2)s.leg_length_m[1]=NAN;
        if(mode==3)s.imu_update_tick_ms=6010-25;
        assert(RLControlExecute(&state,6010,1,&s,&c,&frame)<0);
        assert(!state.active && state.fault && frame.owner==RL_OWNER_LQR);
        assert(state.adopted_sequence==committed.adopted_sequence);
        assert(!memcmp(state.adopted_target,committed.adopted_target,sizeof(state.adopted_target)));
    }
    /* Full wheel targets are adopted in physical units. Output scale does not
       alter previous normalization, Kv, fresh PD feedback or current limits. */
    state=(RLControlState){.ramp_completed=1,.timing=1,.ready=1,.epoch=8};
    s=feedback(7000,&state);
    c=(RLShadowCommand){.valid=1,.snapshot_sequence=200,.snapshot_tick_ms=7000,
        .output_tick_ms=7000,.control_epoch=8,.previous_source=RL_PREVIOUS_INIT_PROXY};
    c.action[2]=1; c.action[5]=-1;
    assert(!RLMotion31BuildPreview(&s.motion,c.action,&c.preview));
    near(c.preview.motor_wheel_target_rad_s[0],-80);
    near(c.preview.motor_wheel_target_rad_s[1],-80);
    assert(!RLControlExecute(&state,7000,1,&s,&c,&frame));
    near(state.adopted_target[2],80); near(state.adopted_target[5],-80);
    near(frame.tau_pd_nm[2],16); near(frame.tau_pd_nm[5],-16);
    near(frame.tau_limited_nm[2],5.5f); near(frame.tau_limited_nm[5],-5.5f);
    near(fabsf(frame.wheel_current_raw[0]),8192);
    near(fabsf(frame.wheel_current_raw[1]),8192);
    s=feedback(7005,&state);
    assert(!RLMotion31BuildObservation(&s.motion,obs));
    near(obs[20],.8f); near(obs[21],-.8f);
    s.motion.joint_velocity_rad_s[2]=70; s.motion.joint_velocity_rad_s[5]=-70;
    assert(!RLControlExecute(&state,7005,1,&s,&c,&frame));
    near(frame.tau_pd_nm[2],2); near(frame.tau_pd_nm[5],-2);
    near(state.adopted_target[2],80); near(state.adopted_target[5],-80);
    puts("PASS supported015/80: center/scale, real dq, physical previous, PD and causal adoption");
    return 0;
}
