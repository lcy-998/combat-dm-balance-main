#include "rl_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void near(float a,float b) {assert(fabsf(a-b)<2e-5f);}

static RLShadowSnapshot sample(RLControlState *state)
{
    RLShadowSnapshot s={0};s.tick_ms=100;s.sequence=2;s.chassis_ready=1;
    s.motion.command_height_m=.34f;s.motion.gravity_model[1]=-1;
    s.leg_length_m[0]=s.leg_length_m[1]=.28f;
    s.feedback_valid_mask=0x3f;s.imu_update_sequence=1;s.imu_update_tick_ms=100;
    for(unsigned i=0;i<6;++i)s.feedback_receive_tick_ms[i]=100;
    RLControlFillPrevious(state,&s);return s;
}

static void test_height_zero_and_axis_roundtrip(void)
{
    const unsigned legs[4]={0,1,3,4};
    const float pi=3.14159265358979323846f;
    const float hip=(90-47.1033888f)*pi/180,active=(90+47.1080837f)*pi/180;
    /* Driver offsets are copied from current chassis.c for an independent
       raw encoder -> chassis angle -> model -> output -> encoder roundtrip.
       This proves algebra, not the physical calibration of these constants. */
    const float offset[4]={1.640323924f,-2.643448303f,
        1.092286447f-.618772388f+pi/2+.03721809f,
        1.313378777f-1.5654273f+pi/2-.0021917f};
    for(unsigned k=0;k<=100;++k) {
        float h=.30f+.001f*(float)k;
        const float action[6]={-.7f,.4f,.5f,.6f,-.3f,-.2f};
        float target[6],hardware[4],wheel[2],torque[6],limited[6];
        double center[4];
        assert(!RLMotion31SupportCenter(h,center));
        assert(!RLMotion31DecodeOutput(action,h,target));
        for(unsigned i=0;i<4;++i)near(target[legs[i]],(float)center[i]+.15f*action[legs[i]]);
        assert(!RLMotion31MapPolicyTargets(target,hardware,wheel));
        float raw[4];
        for(unsigned i=0;i<4;++i)raw[i]=(i<2)?offset[i]-hardware[i]:hardware[i]-offset[i];
        const float lf=-raw[0]+offset[0],lb=-raw[1]+offset[1];
        const float rf=raw[2]+offset[2],rb=raw[3]+offset[3];
        RLMotion31Snapshot s={0};
        RLMotion31MapJointAngles(&s,rb,rf,lb,lf,0,0,0,0);
        RLMotion31MapWheelMotorSpeeds(&s,wheel[0],wheel[1]);
        for(unsigned i=0;i<4;++i)near(s.joint_position_rad[i],target[legs[i]]);
        assert(!RLMotion31ComputePdTorque(&s,target,torque,limited));
        for(unsigned i=0;i<6;++i)near(torque[i],0); /* center is NOT a PD zero */
        /* Actual adopted physical previous is not recentered when h changes. */
        s.gravity_model[1]=-1;s.command_height_m=.34f;s.previous_target_valid=1;
        memcpy(s.previous_target,target,sizeof(target));float obs[31];
        assert(!RLMotion31BuildObservation(&s,obs));
        for(unsigned i=0;i<4;++i)near(obs[16+i],target[legs[i]]/1.5f);
        near(obs[20],target[2]/100);near(obs[21],target[5]/100);
        for(unsigned axis=0;axis<4;++axis) {
            float a[4]={hip,active,hip,active};float v[4]={0};
            /* Inverse of model q and raw motor speed, lf/lb/rf/rb order. */
            a[0]=hip-target[0];a[1]=active+target[1];
            a[2]=hip+target[3];a[3]=active+target[4];
            const unsigned raw_to_model[4]={4,3,1,0};
            const float sign[4]={-1,-1,1,-1};
            float raw_v[4]={0};raw_v[axis]=.1f;
            v[0]=raw_v[3];v[1]=raw_v[2];v[2]=-raw_v[1];v[3]=-raw_v[0];
            RLMotion31MapJointAngles(&s,a[0],a[1],a[2],a[3],v[0],v[1],v[2],v[3]);
            s.joint_velocity_rad_s[2]=target[2];s.joint_velocity_rad_s[5]=target[5];
            assert(!RLMotion31ComputePdTorque(&s,target,torque,limited));
            near(torque[raw_to_model[axis]],-2*.1f*sign[axis]);
            float motor[4];assert(!RLMotion31MapPolicyLegTorqueToMotorOrder(torque,motor));
            near(motor[axis],-.2f); /* D term always opposes raw motor motion */
            /* Raw encoder position displaced positively by .01rad, rates
               zero: every raw-motor P torque must restore by -.4Nm. */
            float shifted[4];memcpy(shifted,raw,sizeof(raw));shifted[axis]+=.01f;
            RLMotion31MapJointAngles(&s,shifted[3]+offset[3],shifted[2]+offset[2],
                -shifted[1]+offset[1],-shifted[0]+offset[0],0,0,0,0);
            assert(!RLMotion31ComputePdTorque(&s,target,torque,limited));
            assert(!RLMotion31MapPolicyLegTorqueToMotorOrder(torque,motor));
            near(motor[axis],-.4f);
        }
    }
    double center[4];assert(RLMotion31SupportCenter(NAN,center)<0);
    assert(RLMotion31SupportCenter(.29,center)<0);assert(RLMotion31SupportCenter(.41,center)<0);
    assert(RLMotion31SupportCenter(.34,NULL)<0);
}

static void test_outside_contract_height_command_is_rejected(void)
{
    const float invalid_heights[]={.29f,.41f,-1.0f,NAN,INFINITY};
    for(unsigned h=0;h<5;++h) {
        RLControlState state={.ramp_completed=1,.timing=1,.ready=1,.epoch=7};
        RLShadowSnapshot s=sample(&state);RLShadowCommand c={0};
        c.valid=1;c.snapshot_sequence=2;c.snapshot_tick_ms=c.output_tick_ms=100;
        c.control_epoch=7;c.previous_source=RL_PREVIOUS_INIT_PROXY;
        assert(!RLMotion31BuildPreview(&s.motion,c.action,&c.preview));
        /* Out-of-contract/invalid heights must be rejected. Other legal
           packet heights may overlap the live command during transport. */
        c.preview.observation[24]=invalid_heights[h]*2;
        /* Corrupt height while keeping an otherwise finite valid target. */
        assert(RLMotion31DecodeOutput(c.action,invalid_heights[h],c.preview.target)!=0);
        assert(!RLControlStep(&state,100,1,&c).active);
        RLControlFrame frame;
        assert(RLControlExecute(&state,100,1,&s,&c,&frame)!=0);
        assert(!state.active && state.adopted_sequence==0);
        /* An already active controller may hold its valid adopted target;
           an invalid-height new packet must never rewrite it or previous. */
        c.preview.observation[24]=.68f;
        assert(!RLMotion31DecodeOutput(c.action,.34f,c.preview.target));
        assert(!RLControlExecute(&state,100,1,&s,&c,&frame));
        const RLControlState committed=state;
        c.snapshot_sequence=3;c.previous_source=RL_PREVIOUS_ADOPTED;c.previous_sequence=2;
        c.preview.observation[24]=invalid_heights[h]*2;
        /* Corrupt height while keeping an otherwise finite valid target. */
        assert(RLMotion31DecodeOutput(c.action,invalid_heights[h],c.preview.target)!=0);
        RLControlFillPrevious(&state,&s);
        assert(!RLControlExecute(&state,100,1,&s,&c,&frame));
        assert(state.adopted_sequence==committed.adopted_sequence);
        assert(!memcmp(state.adopted_target,committed.adopted_target,sizeof(state.adopted_target)));
        RLControlFillPrevious(&state,&s);
        assert(!memcmp(s.motion.previous_target,committed.adopted_target,sizeof(state.adopted_target)));
    }
}

int main(void)
{
    test_height_zero_and_axis_roundtrip();
    test_outside_contract_height_command_is_rejected();
    puts("PASS C0 center: height/encoder roundtrip, PD zero and motor damping, physical previous, height-domain adoption guard");
    return 0;
}
