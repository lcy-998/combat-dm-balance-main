#include "rl_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void near(float a,float b) {assert(fabsf(a-b)<2e-5f);}

static RLShadowSnapshot feedback(const RLControlState *state,uint32_t now)
{
    RLShadowSnapshot s={0};s.tick_ms=now;s.sequence=now+1;s.chassis_ready=1;
    s.motion.gravity_model[1]=-1;s.motion.command_height_m=RLControlCommandHeight(state);
    /* Nonzero motion and physical velocity are deliberate: height is NOT
       gated on stopping, stationarity, pitch settling, or a zero joystick. */
    s.motion.command_vx_m_s=1.2f;s.motion.command_yaw_rad_s=.4f;
    for(unsigned i=0;i<6;++i)s.motion.joint_velocity_rad_s[i]=.2f+.1f*i;
    s.feedback_valid_mask=0x3f;s.imu_update_sequence=1;s.imu_update_tick_ms=now;
    for(unsigned i=0;i<6;++i)s.feedback_receive_tick_ms[i]=now;
    s.leg_length_m[0]=s.leg_length_m[1]=.28f;
    RLControlFillPrevious(state,&s);return s;
}

static RLShadowCommand query(const RLControlState *state,const RLShadowSnapshot *s,uint32_t sequence)
{
    RLShadowCommand c={0};c.valid=1;c.snapshot_sequence=sequence;
    c.snapshot_tick_ms=c.output_tick_ms=s->tick_ms;c.control_epoch=state->epoch;
    c.previous_source=s->previous_source;c.previous_sequence=s->adopted_sequence;
    const float actions[6]={.1f,-.2f,.3f,-.4f,.5f,-.6f};memcpy(c.action,actions,sizeof(actions));
    assert(!RLMotion31BuildPreview(&s->motion,c.action,&c.preview));return c;
}

static void pd_matches_absolute(const RLShadowSnapshot *s,const RLControlFrame *frame)
{
    const unsigned legs[4]={0,1,3,4};
    for(unsigned j=0;j<4;++j) {
        unsigned i=legs[j];
        near(frame->tau_pd_nm[i],40*(frame->adopted_target[i]-s->motion.joint_position_rad[j])-2*s->motion.joint_velocity_rad_s[i]);
    }
    near(frame->tau_pd_nm[2],.2f*(frame->adopted_target[2]-s->motion.joint_velocity_rad_s[2]));
    near(frame->tau_pd_nm[5],.2f*(frame->adopted_target[5]-s->motion.joint_velocity_rad_s[5]));
}

static void test_each_height_standing_or_motion_uses_matching_center_and_input(void)
{
    const unsigned legs[4]={0,1,3,4};
    for(unsigned j=0;j<=100;++j) {
        float h=.30f+.001f*j;
        RLControlState state={.ramp_completed=1,.timing=1,.ready=1,.epoch=7};
        state.height.initialized=1;state.height.command_m=h;
        RLShadowSnapshot s=feedback(&state,100);RLShadowCommand c=query(&state,&s,1);RLControlFrame frame;
        near(c.preview.observation[24],2*h);near(c.preview.observation[22],.3f);near(c.preview.observation[23],.1f);
        double center[4];assert(!RLMotion31SupportCenter(h,center));
        for(unsigned i=0;i<4;++i)near(c.preview.target[legs[i]],(float)center[i]+.15f*c.action[legs[i]]);
        assert(!RLControlExecute(&state,100,1,&s,&c,&frame));pd_matches_absolute(&s,&frame);
        near(state.adopted_height_m,h);RLControlFillPrevious(&state,&s);
        for(unsigned i=0;i<6;++i)near(s.motion.previous_target[i],state.adopted_target[i]);
    }
}

static void test_dynamic_height_during_motion_preserves_previous_and_delayed_target_identity(void)
{
    RLControlState state={.ramp_completed=1,.timing=1,.ready=1,.epoch=7};RLControlFrame frame;
    RLShadowSnapshot s=feedback(&state,100);RLShadowCommand c=query(&state,&s,1);
    assert(!RLControlExecute(&state,100,1,&s,&c,&frame));
    float original[6];memcpy(original,state.adopted_target,sizeof(original));
    s=feedback(&state,105);RLShadowCommand pending=query(&state,&s,2);
    /* Arbitrary requested step: no slew cap and no pending-adoption gate. */
    assert(!RLControlSetCommandHeight(&state,.38f));
    assert(!memcmp(original,state.adopted_target,sizeof(original)));
    s=feedback(&state,110);float new_h=RLControlCommandHeight(&state);near(new_h,.38f);
    assert(!memcmp(original,s.motion.previous_target,sizeof(original)));
    /* An original .34m inference returns after the live request changed.
       Keep its original absolute target and .34m identity; never re-decode
       with .38m, and never relabel previous as the new reference center. */
    float old_request[6];memcpy(old_request,pending.preview.target,sizeof(old_request));
    assert(!RLControlExecute(&state,110,1,&s,&pending,&frame));pd_matches_absolute(&s,&frame);
    assert(!memcmp(old_request,state.adopted_target,sizeof(old_request)));
    near(state.adopted_height_m,.34f);
    RLControlFillPrevious(&state,&s);
    assert(!memcmp(s.motion.previous_target,old_request,sizeof(old_request)));
    RLShadowCommand new_request=query(&state,&s,3);
    assert(fabsf(new_request.preview.target[0]-old_request[0])>.01f);
    float pending_new[6];memcpy(pending_new,new_request.preview.target,sizeof(pending_new));
    assert(!RLControlSetCommandHeight(&state,.30f));
    s=feedback(&state,115);
    assert(!RLControlExecute(&state,115,1,&s,&new_request,&frame));pd_matches_absolute(&s,&frame);
    near(state.adopted_height_m,.38f);
    assert(!memcmp(state.adopted_target,pending_new,sizeof(pending_new)));
    RLControlFillPrevious(&state,&s);
    assert(!memcmp(s.motion.previous_target,pending_new,sizeof(pending_new)));
    RLShadowCommand lower=query(&state,&s,4);
    assert(!RLControlExecute(&state,115,1,&s,&lower,&frame));pd_matches_absolute(&s,&frame);
    near(state.adopted_height_m,.30f);
    near(s.motion.command_vx_m_s,1.2f);near(s.motion.command_yaw_rad_s,.4f);
}

static void test_full_domain_direct_requests_no_dwell_or_slew_and_reset(void)
{
    RLControlState state={.ramp_completed=1,.timing=1,.ready=1,.epoch=7};
    RLShadowSnapshot s=feedback(&state,100);RLControlFrame frame;RLShadowCommand c=query(&state,&s,1);
    assert(!RLControlExecute(&state,100,1,&s,&c,&frame));
    float actual[6];memcpy(actual,state.adopted_target,sizeof(actual));
    const float requests[]={.30f,.40f,.32f,.38f,.36f,.34f};
    for(unsigned i=0;i<sizeof(requests)/sizeof(requests[0]);++i) {
        /* Same tick immediate changes accepted without confirmation timers. */
        assert(!RLControlSetCommandHeight(&state,requests[i]));
        near(RLControlCommandHeight(&state),requests[i]);
        assert(!memcmp(actual,state.adopted_target,sizeof(actual)));
    }
    assert(RLControlSetCommandHeight(&state,.29f)<0);
    assert(RLControlSetCommandHeight(&state,.41f)<0);
    assert(RLControlSetCommandHeight(&state,NAN)<0);
    assert(RLControlSetCommandHeight(NULL,.34f)<0);
    near(RLControlCommandHeight(&state),.34f);
    RLControlReset(&state);near(RLControlCommandHeight(&state),.34f);
    assert(!state.active && state.adopted_sequence==0);
}

int main(void)
{
    test_each_height_standing_or_motion_uses_matching_center_and_input();
    test_dynamic_height_during_motion_preserves_previous_and_delayed_target_identity();
    test_full_domain_direct_requests_no_dwell_or_slew_and_reset();
    puts("PASS runtime height: full .30-.40m, motion allowed, original packet center, actual previous and absolute PD, no dwell/slew gate");
    return 0;
}
