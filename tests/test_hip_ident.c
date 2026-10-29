#include "hip_ident.h"
#include "../application/balance/leg_const.hpp"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

void getPhi(float,float,float,float,float,float,float,float *,float *,float *,float *);
static const float initial_q[4]={0.18f,0.165f,-0.225f,0.15f};
static const unsigned selected=HIP_IDENT_SIDE==0u?3u:0u;
static const unsigned debug_first=HIP_IDENT_SIDE==0u?2u:0u;
static const unsigned motor_first=HIP_IDENT_SIDE==0u?0u:2u;
static const float lengths[]=HIP_IDENT_LEG_LENGTHS_M;
static RLShadowSnapshot feedback(uint32_t now)
{
    RLShadowSnapshot f={0};
    f.tick_ms=now;f.imu_update_sequence=1;f.imu_update_tick_ms=now;
    f.feedback_valid_mask=0x3fu;
    for(unsigned i=0;i<6u;++i)f.feedback_receive_tick_ms[i]=now;
    f.motion.gravity_model[1]=-1;
    memcpy(f.motion.joint_position_rad,initial_q,sizeof(initial_q));
    f.motion.previous_target[0]=initial_q[0];f.motion.previous_target[1]=initial_q[1];
    f.motion.previous_target[3]=initial_q[2];f.motion.previous_target[4]=initial_q[3];
    f.motion.joint_velocity_rad_s[selected]=0.123f;
    f.leg_length_m[0]=0.26f;f.leg_length_m[1]=0.27f;
    f.leg_spread_rad[0]=1.4f;f.leg_spread_rad[1]=1.3f;
    const float motor_torque[4]={1.0f,-2.0f,3.0f,-4.0f}; /* lf/lb/rf/rb */
    memcpy(f.driver_reported_leg_torque_motor_nm,motor_torque,sizeof(motor_torque));
    return f;
}
static int step(HipIdentState *s,uint32_t t,int stop,int link,HipIdentFrame *o)
{
    RLShadowSnapshot f=feedback(t);
    return HipIdentStep(s,t,stop,link,&f,o);
}
static void near(float a,float b){assert(fabsf(a-b)<2e-5f);}
static void hw_from_action(const float a[6],float hw[4])
{
    float target[6],wheel[2];
    assert(!RLMotion31DecodeOutput(a,target));assert(!RLMotion31MapPolicyTargets(target,hw,wheel));
}

int main(void)
{
    /* Five actual lengths, not five height-command labels; preserve reference leg angle. */
    float ref[6]={0.12f,0.11f,0,-0.15f,0.10f,0},ref_hw[4],p2,p3,l,theta_ref;
    hw_from_action(ref,ref_hw);
    getPhi(ref_hw[motor_first],ref_hw[motor_first+1u],L_1,L_2,L_3,L_4,L_5,&p2,&p3,&l,&theta_ref);
    for(unsigned c=0;c<5u;++c) {
        float a[6],hw[4],theta;
        assert(!HipIdentCenterAction(c,a));hw_from_action(a,hw);
        getPhi(hw[motor_first],hw[motor_first+1u],L_1,L_2,L_3,L_4,L_5,&p2,&p3,&l,&theta);
        near(l,lengths[c]);near(theta,theta_ref);
    }
    float invalid[6];assert(HipIdentCenterAction(5,invalid));

    HipIdentState s={0};HipIdentFrame o;
    assert(!step(&s,0,0,1,&o));assert(s.stage==HIP_WAIT);
    assert(!step(&s,5,1,1,&o));assert(s.armed);
    assert(step(&s,10,0,1,&o));assert(s.stage==HIP_RAMP);
    assert(step(&s,3010,0,1,&o));assert(s.stage==HIP_SETTLE);
    assert(step(&s,5010,0,1,&o));assert(s.stage==HIP_SWEEP);
    near(o.status.frequency_hz,0.5f);
    near(o.debug.observation_velocity[debug_first],0.05f*0.123f);
    assert(o.status.observation_valid);
    assert(sizeof(HipIdentDebug)==22u*sizeof(float));
    const float sl=16.5f*sinf(1.4f),sr=16.5f*sinf(1.3f);
    near(o.debug.feedback_torque_nm[0],HIP_IDENT_SIDE==1u?4.0f-sr:4.0f);
    near(o.debug.feedback_torque_nm[1],HIP_IDENT_SIDE==1u?3.0f-sr:3.0f);
    near(o.debug.feedback_torque_nm[2],HIP_IDENT_SIDE==0u?2.0f+sl:2.0f);
    near(o.debug.feedback_torque_nm[3],HIP_IDENT_SIDE==0u?-1.0f-sl:-1.0f);
    assert(o.status.feedback_torque_valid_mask==0x0fu);

    /* Synthetic feedback equal to the motor command must recover PD before
       compensation when unsaturated. Exercise both spread signs and saturation. */
    for(unsigned scenario=0;scenario<4u;++scenario) {
        HipIdentState local=s;local.stage=HIP_SWEEP;local.start_ms=10000;
        RLShadowSnapshot f=feedback(10000);
        const unsigned channels[4]={0u,1u,3u,4u};
        float a[6],target[6],policy[6]={0},motor[4],sent[4],length[2],spread[2],spring[4];
        assert(!HipIdentCenterAction(0,a));assert(!RLMotion31DecodeOutput(a,target));
        const float requested_pd[4]={3.0f,-4.0f,-5.0f,6.0f};
        for(unsigned i=0;i<4u;++i) {
            policy[channels[i]]=scenario==3u?(i==2u?-39.0f:39.0f):requested_pd[i];
            f.motion.joint_position_rad[i]=target[channels[i]]-policy[channels[i]]/40.0f;
            f.motion.joint_velocity_rad_s[channels[i]]=0.0f;
        }
        const float angle=scenario==0u?1.2f:scenario==1u?-1.2f:scenario==2u?0.0f:1.2f;
        f.leg_spread_rad[HIP_IDENT_SIDE]=angle;
        f.leg_length_m[HIP_IDENT_SIDE]=0.22f;
        length[HIP_IDENT_SIDE]=0.22f;length[1u-HIP_IDENT_SIDE]=0.22f;
        spread[HIP_IDENT_SIDE]=angle;spread[1u-HIP_IDENT_SIDE]=0.0f;
        const float zero[4]={0};
        assert(!RLMotion31MapPolicyLegTorqueToMotorOrder(policy,motor));
        assert(!RLControlCompensateSpring(motor,length,spread,sent));
        assert(!RLControlCompensateSpring(zero,length,spread,spring));
        memcpy(f.driver_reported_leg_torque_motor_nm,sent,sizeof(sent));
        assert(HipIdentStep(&local,10000,0,1,&f,&o));
        const unsigned motor_index[4]={3u,2u,1u,0u};
        const float sign[4]={-1.0f,1.0f,-1.0f,-1.0f};
        for(unsigned i=debug_first;i<debug_first+2u;++i) {
            near(o.debug.pre_spring_torque_nm[i],policy[channels[i]]);
            if(scenario<3u)near(o.debug.feedback_torque_nm[i],policy[channels[i]]);
            else {
                near(fabsf(sent[motor_index[i]]),40.0f);
                near(o.debug.feedback_torque_nm[i],sign[i]*(sent[motor_index[i]]-spring[motor_index[i]]));
                assert(fabsf(o.debug.feedback_torque_nm[i])<39.0f);
            }
        }
        for(unsigned i=motor_first;i<motor_first+2u;++i)near(o.motor_request_nm[i],sent[i]);
        /* No compensation is applied while stopped, so do not invent a subtraction. */
        assert(!HipIdentStep(&local,10000,1,1,&f,&o));
        for(unsigned i=debug_first;i<debug_first+2u;++i)
            near(o.debug.feedback_torque_nm[i],sign[i]*sent[motor_index[i]]);
    }

    /* Integrated linear frequency produces the phase; both motor angles remain equal magnitude. */
    for(unsigned direction=0;direction<2u;++direction) {
        s.direction=direction;s.start_ms=6000;s.stage=HIP_SWEEP;
        const uint32_t offset_ms=3250u;
        assert(step(&s,6000u+offset_ms,0,1,&o));
        const float t=offset_ms*0.001f;
        const float slope=(HIP_IDENT_SWEEP_END_HZ-HIP_IDENT_SWEEP_START_HZ)/(HIP_IDENT_SWEEP_MS*0.001f);
        near(o.status.frequency_hz,HIP_IDENT_SWEEP_START_HZ+slope*t);
        const float offset=HIP_IDENT_AMPLITUDE*sinf(2.0f*PI*(HIP_IDENT_SWEEP_START_HZ*t+0.5f*slope*t*t));
        float center[6],a[6]={0},hw0[4],hw1[4];
        assert(!HipIdentCenterAction(0,center));
        near(o.debug.mean_actor[debug_first]-center[selected],offset);
        a[0]=o.debug.mean_actor[0];a[1]=o.debug.mean_actor[1];
        a[3]=o.debug.mean_actor[2];a[4]=o.debug.mean_actor[3];
        hw_from_action(center,hw0);hw_from_action(a,hw1);
        const float front=hw1[motor_first]-hw0[motor_first];
        const float back=hw1[motor_first+1u]-hw0[motor_first+1u];
        near(fabsf(front),1.5f*fabsf(offset));near(front,direction==0u?back:-back);
        RLShadowSnapshot f=feedback(6000+offset_ms);
        float target[6],request[6],limited[6];
        assert(!RLMotion31DecodeOutput(a,target));
        assert(!RLMotion31ComputePdTorque(&f.motion,target,request,limited));
        near(o.debug.pre_spring_torque_nm[debug_first],limited[selected]);
        for(unsigned i=0;i<4u;++i)if(i<motor_first || i>=motor_first+2u)near(o.motor_request_nm[i],0);
    }
    /* Fades are zero at start/end, one in the interior; endpoint is exactly 4 Hz. */
    s.stage=HIP_SWEEP;s.start_ms=10000;
    assert(step(&s,10000,0,1,&o));near(o.status.envelope,0);
    assert(step(&s,11000,0,1,&o));near(o.status.envelope,0.5f);
    assert(step(&s,10000+HIP_IDENT_SWEEP_MS-1000,0,1,&o));near(o.status.envelope,0.5f);
    assert(step(&s,10000+HIP_IDENT_SWEEP_MS,0,1,&o));
    near(o.status.frequency_hz,4.0f);near(o.status.envelope,0);assert(s.stage==HIP_GAP);

    memset(&s,0,sizeof(s));step(&s,0,1,1,&o);
    unsigned seen[5][2]={{0}},done=0,sweep_count=0;
    unsigned last_stage=HIP_WAIT,last_center=0,last_direction=0;
    float last_frequency[5][2]={{0}};
    for(uint32_t t=5;t<=400000;t+=5) {
        assert(step(&s,t,0,1,&o));
        for(unsigned i=motor_first;i<motor_first+2u;++i)assert(isfinite(o.motor_request_nm[i]) && fabsf(o.motor_request_nm[i])<=40.0f);
        near(o.status.target_leg_length_m,lengths[s.center]);
        near(o.status.measured_leg_length_m,HIP_IDENT_SIDE==0u?0.26f:0.27f);
        near(o.debug.leg_length_m[0],0.26f);
        near(o.debug.leg_length_m[1],0.27f);
        if(s.stage==HIP_SWEEP) {
            const unsigned c=s.center,d=s.direction;
            if(last_stage!=HIP_SWEEP) {
                assert(sweep_count<6u);
                assert(c==(sweep_count==0u?0u:sweep_count-1u));
                assert(d==(sweep_count==0u?1u:0u));
                ++sweep_count;
            } else {
                assert(c==last_center && d==last_direction);
            }
            assert(o.status.frequency_hz>=0.5f && o.status.frequency_hz<=4.0f);
            assert(o.status.frequency_hz>=last_frequency[c][d]);
            last_frequency[c][d]=o.status.frequency_hz;seen[c][d]++;
            float a[6];assert(!HipIdentCenterAction(c,a));
            assert(fabsf(o.debug.mean_actor[debug_first]-a[selected])<=HIP_IDENT_AMPLITUDE+1e-5f);
        }
        last_stage=s.stage;last_center=s.center;last_direction=s.direction;
        if(s.stage==HIP_DONE){done=1;assert(t>=127000u && t<128000u);break;}
    }
    assert(done);
    assert(sweep_count==6u);
    assert(seen[0][1]>=HIP_IDENT_SWEEP_MS/5u);
    for(unsigned c=0;c<5u;++c) {
        assert(seen[c][0]>=HIP_IDENT_SWEEP_MS/5u);
        if(c>0u)assert(seen[c][1]==0u);
    }
    assert(step(&s,400005,0,1,&o));assert(o.status.stage==HIP_DONE);
    for(unsigned stage=HIP_RAMP;stage<=HIP_DONE;++stage) {
        s.stage=stage;s.start_ms=500000;
        assert(!step(&s,500000,1,1,&o));assert(s.stage==HIP_WAIT);
    }
    step(&s,500005,0,1,&o);
    RLShadowSnapshot stale=feedback(500010);stale.feedback_receive_tick_ms[selected]=499900;
    assert(!HipIdentStep(&s,500010,0,1,&stale,&o));assert(o.status.fault==HIP_BAD_FEEDBACK);
    assert(isnan(o.debug.feedback_torque_nm[debug_first]));
    assert(!(o.status.feedback_torque_valid_mask&(1u<<debug_first)));
    assert(!step(&s,500015,0,1,&o));assert(s.stage==HIP_FAULT);
    step(&s,500020,1,1,&o);assert(step(&s,500025,0,1,&o));
    assert(!step(&s,500030,0,0,&o));assert(o.status.fault==HIP_COMMAND_LOST);
    assert(!step(&s,500035,0,1,&o));
    step(&s,500040,1,1,&o);step(&s,500045,0,1,&o);
    RLShadowSnapshot bad=feedback(500050);bad.motion.joint_velocity_rad_s[selected]=NAN;
    assert(!HipIdentStep(&s,500050,0,1,&bad,&o));
    memset(&s,0,sizeof(s));step(&s,0xfffffff0u,1,1,&o);
    assert(step(&s,0xfffffff5u,0,1,&o));
    assert(step(&s,0xfffffff5u+3000u,0,1,&o));assert(s.stage==HIP_SETTLE);
    printf("hip identification side %u: five lengths, chirp phase/frequency/fade, signals, signs, stop/fault and rollover passed\n",HIP_IDENT_SIDE);
    return 0;
}
