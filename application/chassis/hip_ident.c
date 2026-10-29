#include "hip_ident.h"
#include "../balance/leg_const.hpp"
#include <math.h>
#include <string.h>

/* Use the actual shared five-bar implementation without pulling in HAL/RTOS headers. */
void getPhi(float,float,float,float,float,float,float,float *,float *,float *,float *);

volatile HipIdentDebug g_hip_ident_debug;
volatile HipIdentStatus g_hip_ident_status;
static const float reference_center[4] = HIP_IDENT_REFERENCE_CENTER;
static const float leg_lengths[] = HIP_IDENT_LEG_LENGTHS_M;
static const unsigned joints[4]={0u,1u,3u,4u};
#define COUNT(a) (sizeof(a)/sizeof((a)[0]))
_Static_assert(HIP_IDENT_SIDE<=1u,"HIP_IDENT_SIDE must be 0 (left) or 1 (right)");
_Static_assert(COUNT(leg_lengths)==5u,"configure exactly five leg lengths");

int HipIdentCenterAction(uint32_t index,float a[6])
{
    if(index>=COUNT(leg_lengths) || a==NULL || L_5!=0.0f)return -1;
    memset(a,0,6u*sizeof(float));
    for(unsigned i=0;i<4u;++i)a[joints[i]]=reference_center[i];
    float target[6],hw[4],wheel[2],p2,p3,reference_length,angle;
    if(RLMotion31DecodeOutput(a,target) || RLMotion31MapPolicyTargets(target,hw,wheel))return -1;
    const unsigned j=HIP_IDENT_SIDE==0u?0u:2u;
    getPhi(hw[j],hw[j+1u],L_1,L_2,L_3,L_4,L_5,&p2,&p3,&reference_length,&angle);
    const float length=leg_lengths[index];
    if(!isfinite(angle) || !isfinite(length) || length<=0.0f)return -1;
    const float front_cos=(length*length+L_1*L_1-L_2*L_2)/(2.0f*length*L_1);
    const float back_cos=(length*length+L_4*L_4-L_3*L_3)/(2.0f*length*L_4);
    if(fabsf(front_cos)>1.0f || fabsf(back_cos)>1.0f)return -1;
    const float front=angle+acosf(front_cos),back=angle-acosf(back_cos);
    /* Map a unit action through production code to obtain its exact angle slopes;
       this avoids duplicating calibrated zero offsets or right hip signs. */
    const unsigned first=HIP_IDENT_SIDE==0u?3u:0u;
    float unit[6],target_unit[6],hw_unit[4];
    memcpy(unit,a,sizeof(unit));
    unit[first]+=0.01f;unit[first+1u]+=0.01f;
    if(RLMotion31DecodeOutput(unit,target_unit) ||
       RLMotion31MapPolicyTargets(target_unit,hw_unit,wheel))return -1;
    const float back_slope=(hw_unit[j+1u]-hw[j+1u])/0.01f;
    const float front_slope=(hw_unit[j]-hw[j])/0.01f;
    if(fabsf(back_slope)<1.0f || fabsf(front_slope)<1.0f)return -1;
    a[first]+=(back-hw[j+1u])/back_slope;
    a[first+1u]+=(front-hw[j])/front_slope;
    if(RLMotion31DecodeOutput(a,target) || RLMotion31MapPolicyTargets(target,hw,wheel))return -1;
    float check_length,check_angle;
    getPhi(hw[j],hw[j+1u],L_1,L_2,L_3,L_4,L_5,&p2,&p3,&check_length,&check_angle);
    return isfinite(check_length) && fabsf(check_length-length)<1e-4f &&
           fabsf(remainderf(check_angle-angle,2.0f*PI))<1e-4f?0:-1;
}

static int pose_valid(const float action[6])
{
    float target[6], hw[4], wheel[2], p2,p3,length,angle;
    for(unsigned i=0;i<6u;++i)
        if(!isfinite(action[i]) || fabsf(action[i])>1.0f) return 0;
    if(RLMotion31DecodeOutput(action,target) || RLMotion31MapPolicyTargets(target,hw,wheel))return 0;
    const unsigned j=HIP_IDENT_SIDE==0u?0u:2u;
    getPhi(hw[j],hw[j+1u],L_1,L_2,L_3,L_4,L_5,&p2,&p3,&length,&angle);
    return isfinite(p2)&&isfinite(p3)&&isfinite(length)&&isfinite(angle)&&length>0.001f;
}

static void add_motion(float a[6], unsigned direction, float offset)
{
    const unsigned first=HIP_IDENT_SIDE==0u?3u:0u;
    /* Hardware: left lb=zero+q3, lf=zero+q4;
       right rb=zero-q0, rf=zero+q1. Same physical rotation swings the leg. */
    const float second_sign=(direction==0u)==(HIP_IDENT_SIDE==0u)?1.0f:-1.0f;
    a[first]+=offset;
    a[first+1u]+=second_sign*offset;
}

static int config_valid(void)
{
    if(HIP_IDENT_RAMP_MS==0u || HIP_IDENT_HOLD_MS==0u || HIP_IDENT_FADE_MS==0u ||
       HIP_IDENT_SWEEP_MS<=2u*HIP_IDENT_FADE_MS ||
       !isfinite(HIP_IDENT_AMPLITUDE) || HIP_IDENT_AMPLITUDE<=0.0f || HIP_IDENT_AMPLITUDE>1.0f ||
       !isfinite(HIP_IDENT_SWEEP_START_HZ) || !isfinite(HIP_IDENT_SWEEP_END_HZ) ||
       HIP_IDENT_SWEEP_START_HZ<=0.0f || HIP_IDENT_SWEEP_END_HZ<=HIP_IDENT_SWEEP_START_HZ ||
       HIP_IDENT_SWEEP_END_HZ>20.0f)return 0;
    for(unsigned i=0;i<4u;++i)
        if(!isfinite(reference_center[i]) || fabsf(reference_center[i])>0.98f)return 0;
    for(unsigned c=0;c<COUNT(leg_lengths);++c) {
        float center[6];
        if(HipIdentCenterAction(c,center))return 0;
        if(c>0u && leg_lengths[c]<=leg_lengths[c-1u])return 0;
        for(unsigned d=0;d<2u;++d) {
            for(unsigned k=0;k<=200u;++k) {
                float a[6];memcpy(a,center,sizeof(a));
                add_motion(a,d,HIP_IDENT_AMPLITUDE*sinf(2.0f*PI*(float)k/200.0f));
                if(!pose_valid(a))return 0;
            }
        }
    }
    return 1;
}

static int feedback_valid(const RLShadowSnapshot *s,uint32_t now)
{
    const unsigned first=HIP_IDENT_SIDE==0u?3u:0u;
    if(s==NULL || (uint32_t)(now-s->tick_ms)>HIP_IDENT_FEEDBACK_TIMEOUT_MS ||
       !s->imu_update_sequence ||
       (uint32_t)(now-s->imu_update_tick_ms)>HIP_IDENT_FEEDBACK_TIMEOUT_MS ||
       RLMotion31ValidateSnapshot(&s->motion))return 0;
    for(unsigned i=first;i<first+2u;++i)
        if(!(s->feedback_valid_mask&(1u<<i)) ||
           (uint32_t)(now-s->feedback_receive_tick_ms[i])>HIP_IDENT_FEEDBACK_TIMEOUT_MS)return 0;
    return isfinite(s->leg_length_m[HIP_IDENT_SIDE])&&s->leg_length_m[HIP_IDENT_SIDE]>0.001f&&
           isfinite(s->leg_spread_rad[HIP_IDENT_SIDE]);
}

static void fail(HipIdentState *s,unsigned fault)
{
    s->stage=HIP_FAULT;s->fault=fault;s->armed=0u;
}

int HipIdentStep(HipIdentState *s,uint32_t now,int emergency,int link_ok,
                 const RLShadowSnapshot *feedback,HipIdentFrame *out)
{
    memset(out,0,sizeof(*out));
    HipIdentDebug *o=&out->debug;
    HipIdentStatus *status=&out->status;
    status->tick_ms=now;status->side=HIP_IDENT_SIDE;
    o->leg_length_m[0]=NAN;o->leg_length_m[1]=NAN;
    /* Inverse of RLMotion31MapPolicyLegTorqueToMotorOrder:
       RH=-rb, RA=rf, LH=-lb, LA=-lf. Keep signs, never fabsf. */
    const unsigned motor_index[4]={3u,2u,1u,0u};
    const float motor_sign[4]={-1.0f,1.0f,-1.0f,-1.0f};
    for(unsigned i=0;i<4u;++i) {
        o->feedback_torque_nm[i]=NAN;
        if(feedback && (feedback->feedback_valid_mask&(1u<<joints[i])) &&
           (uint32_t)(now-feedback->feedback_receive_tick_ms[joints[i]])<=HIP_IDENT_FEEDBACK_TIMEOUT_MS &&
           isfinite(feedback->driver_reported_leg_torque_motor_nm[motor_index[i]])) {
            o->feedback_torque_nm[i]=motor_sign[i]*feedback->driver_reported_leg_torque_motor_nm[motor_index[i]];
            status->feedback_torque_valid_mask|=1u<<i;
        }
    }
    if(feedback) {
        float observation[31];
        memcpy(o->leg_length_m,feedback->leg_length_m,sizeof(o->leg_length_m));
        status->measured_leg_length_m=feedback->leg_length_m[HIP_IDENT_SIDE];
        status->observation_valid=RLMotion31BuildObservation(&feedback->motion,observation)==0;
        if(status->observation_valid)for(unsigned i=0;i<4u;++i) {
            o->observation_position[i]=observation[6u+i];
            o->observation_velocity[i]=observation[10u+joints[i]];
        }
    }
    if(!link_ok) fail(s,HIP_COMMAND_LOST);
    else if(emergency) {
        memset(s,0,sizeof(*s));s->armed=1u;
    } else if(s->stage!=HIP_FAULT && (s->armed || s->stage!=HIP_WAIT)) {
        if(!feedback_valid(feedback,now))fail(s,HIP_BAD_FEEDBACK);
        else if(s->stage==HIP_WAIT) {
            if(!config_valid())fail(s,HIP_BAD_CONFIG);
            else {
                for(unsigned i=0;i<4u;++i)s->ramp_from[joints[i]]=feedback->motion.joint_position_rad[i]/1.5f;
                memcpy(s->previous_target,feedback->motion.previous_target,sizeof(s->previous_target));
                /* One initial extension sweep, then swing at all five lengths. */
                s->direction=1u;
                s->stage=HIP_RAMP;s->start_ms=now;s->armed=0u;
            }
        }
    }
    if(s->stage!=HIP_WAIT && s->stage!=HIP_FAULT) {
        float a[6];
        if(HipIdentCenterAction(s->center,a))fail(s,HIP_BAD_POSE);
        else {
            const uint32_t elapsed=now-s->start_ms;
            if(s->stage==HIP_RAMP) {
                const float t=fminf(1.0f,(float)elapsed/HIP_IDENT_RAMP_MS);
                const float blend=t*t*(3.0f-2.0f*t);
                const unsigned first=HIP_IDENT_SIDE==0u?3u:0u;
                for(unsigned i=first;i<first+2u;++i)a[i]=s->ramp_from[i]+blend*(a[i]-s->ramp_from[i]);
                if(elapsed>=HIP_IDENT_RAMP_MS){s->stage=HIP_SETTLE;s->start_ms=now;}
            } else if(s->stage==HIP_SETTLE && elapsed>=HIP_IDENT_HOLD_MS) {
                s->center_length_m=feedback->leg_length_m[HIP_IDENT_SIDE];
                s->stage=HIP_SWEEP;s->start_ms=now;
                status->frequency_hz=HIP_IDENT_SWEEP_START_HZ;
            } else if(s->stage==HIP_SWEEP) {
                const float t=fminf((float)elapsed,(float)HIP_IDENT_SWEEP_MS)*0.001f;
                const float duration=HIP_IDENT_SWEEP_MS*0.001f;
                const float slope=(HIP_IDENT_SWEEP_END_HZ-HIP_IDENT_SWEEP_START_HZ)/duration;
                const float phase=2.0f*PI*(HIP_IDENT_SWEEP_START_HZ*t+0.5f*slope*t*t);
                status->frequency_hz=HIP_IDENT_SWEEP_START_HZ+slope*t;
                status->sweep_elapsed_s=t;
                float env=1.0f;
                if(elapsed<HIP_IDENT_FADE_MS)
                    env=0.5f-0.5f*cosf(PI*(float)elapsed/HIP_IDENT_FADE_MS);
                else if(elapsed>HIP_IDENT_SWEEP_MS-HIP_IDENT_FADE_MS)
                    env=0.5f-0.5f*cosf(PI*fmaxf(0.0f,(float)HIP_IDENT_SWEEP_MS-(float)elapsed)/HIP_IDENT_FADE_MS);
                status->envelope=env;
                if(elapsed>=HIP_IDENT_SWEEP_MS){s->stage=HIP_GAP;s->start_ms=now;}
                else add_motion(a,s->direction,HIP_IDENT_AMPLITUDE*env*sinf(phase));
            } else if(s->stage==HIP_GAP && elapsed>=HIP_IDENT_HOLD_MS) {
                s->start_ms=now;
                if(s->direction==1u) {
                    s->direction=0u;s->stage=HIP_SWEEP;
                    status->frequency_hz=HIP_IDENT_SWEEP_START_HZ;
                } else if(s->center+1u<COUNT(leg_lengths)) {
                    memcpy(s->ramp_from,a,sizeof(a));++s->center;
                    s->direction=0u;s->stage=HIP_RAMP;
                } else s->stage=HIP_DONE;
            }
            RLMotion31Snapshot sample=feedback->motion;
            memcpy(sample.previous_target,s->previous_target,sizeof(s->previous_target));
            sample.command_vx_m_s=0;sample.command_yaw_rad_s=0;
            sample.command_height_m=feedback->leg_length_m[HIP_IDENT_SIDE];
            RLMotion31Preview preview;
            float requested[6],limited[6],motor[4],compensated[4],spring[4],length[2],spread[2];
            const float zero[4]={0};
            memcpy(length,feedback->leg_length_m,sizeof(length));
            memcpy(spread,feedback->leg_spread_rad,sizeof(spread));
            length[1u-HIP_IDENT_SIDE]=length[HIP_IDENT_SIDE];spread[1u-HIP_IDENT_SIDE]=0;
            if(!pose_valid(a) || RLMotion31BuildPreview(&sample,a,&preview))fail(s,HIP_BAD_POSE);
            else if(RLMotion31ComputePdTorque(&sample,preview.target,requested,limited) ||
                    RLMotion31MapPolicyLegTorqueToMotorOrder(limited,motor) ||
                    RLControlCompensateSpring(zero,length,spread,spring) ||
                    RLControlCompensateSpring(motor,length,spread,compensated))fail(s,HIP_BAD_PD);
            else {
                const unsigned first=HIP_IDENT_SIDE==0u?0u:2u;
                for(unsigned i=first;i<first+2u;++i)out->motor_request_nm[i]=compensated[i];
                for(unsigned i=0;i<4u;++i) {
                    o->mean_actor[i]=a[joints[i]];
                    o->observation_position[i]=preview.observation[6u+i];
                    o->observation_velocity[i]=preview.observation[10u+joints[i]];
                    /* Only the selected leg's PD torque is executed. */
                    const int selected=HIP_IDENT_SIDE==0u?i>=2u:i<2u;
                    o->pre_spring_torque_nm[i]=selected?limited[joints[i]]:0.0f;
                    /* Remove the signed additive spring command in the motor axis,
                       then express feedback in the same axis as the PD command.
                       Do not clip the result or take its absolute value. */
                    if(selected && (status->feedback_torque_valid_mask&(1u<<i)))
                        o->feedback_torque_nm[i]=motor_sign[i]*
                            (feedback->driver_reported_leg_torque_motor_nm[motor_index[i]]-
                             spring[motor_index[i]]);
                }
                status->observation_valid=1u;
                memcpy(s->previous_target,preview.target,sizeof(s->previous_target));
                status->output_enabled=1u;
            }
        }
    }
    status->stage=s->stage;status->fault=s->fault;status->center=s->center;status->direction=s->direction;
    status->target_leg_length_m=leg_lengths[s->center];
    status->center_length_m=s->center_length_m;
    if(s->stage==HIP_FAULT || s->stage==HIP_WAIT)status->output_enabled=0u;
    return (int)status->output_enabled;
}
