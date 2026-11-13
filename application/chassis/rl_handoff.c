#include "rl_handoff.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
static int valid_config(const RLHandoffConfig *c) {
    return c && c->dwell_ms>0 && c->dwell_ms<=1000u &&
        isfinite(c->height_min_m) && isfinite(c->height_max_m) &&
        isfinite(c->command_height_m) && isfinite(c->height_error_m) &&
        isfinite(c->tilt_rad) && isfinite(c->angular_speed_rad_s) &&
        isfinite(c->leg_speed_rad_s) && isfinite(c->support_min_N) &&
        isfinite(c->maximum_leg_target_jump_rad) && isfinite(c->maximum_wheel_target_jump_rad_s) &&
        c->height_min_m>0 && c->height_max_m>=c->height_min_m &&
        c->command_height_m>=c->height_min_m && c->command_height_m<=c->height_max_m &&
        c->height_error_m>0 && c->tilt_rad>0 && c->angular_speed_rad_s>0 &&
        c->leg_speed_rad_s>0 && c->support_min_N>=0 &&
        c->maximum_leg_target_jump_rad>0 && c->maximum_wheel_target_jump_rad_s>0;
}
void RLHandoffReset(RLHandoffState *s) { if(s) memset(s,0,sizeof(*s)); }
int RLHandoffObserve(RLHandoffState *s,const RLHandoffConfig *c,const RLHandoffSample *x) {
    if(!s||!x)return 0;
    if(!valid_config(c)){s->dwelling=s->requested=0;return 0;}
    if(s->committed)return 1;
    /* Missing/repeated/backward samples never add dwell; >20ms gaps break it. */
    if(s->observed && (int32_t)(x->tick_ms-s->last_ms)<=0)return s->requested;
    if(s->observed && (uint32_t)(x->tick_ms-s->last_ms)>20u){s->dwelling=0;s->requested=0;}
    s->last_ms=x->tick_ms;s->observed=1;
    const int good=x->valid && c->dwell_ms>0 && c->dwell_ms<=1000u &&
        isfinite(x->height_m)&&isfinite(x->pitch_rad)&&isfinite(x->roll_rad)&&
        isfinite(x->angular_speed_rad_s)&&isfinite(x->maximum_leg_speed_rad_s)&&
        isfinite(x->normal_force_N[0])&&isfinite(x->normal_force_N[1])&&
        x->height_m>=c->height_min_m&&x->height_m<=c->height_max_m&&
        fabsf(x->height_m-c->command_height_m)<=c->height_error_m&&
        fabsf(x->pitch_rad)<=c->tilt_rad&&fabsf(x->roll_rad)<=c->tilt_rad&&
        x->angular_speed_rad_s>=0&&x->angular_speed_rad_s<=c->angular_speed_rad_s&&
        x->maximum_leg_speed_rad_s>=0&&x->maximum_leg_speed_rad_s<=c->leg_speed_rad_s&&
        x->normal_force_N[0]>c->support_min_N&&x->normal_force_N[1]>c->support_min_N;
    if(!good){s->dwelling=0;s->requested=0;return 0;}
    if(!s->dwelling){s->since_ms=x->tick_ms;s->dwelling=1;}
    if((uint32_t)(x->tick_ms-s->since_ms)>=c->dwell_ms){
        if(!s->requested)s->request_tick_ms=x->tick_ms;
        s->requested=1;
    }
    return s->requested;
}
int RLHandoffCanCommit(const RLHandoffState *s,const RLHandoffConfig *c,
                       uint32_t tick,const float previous[6],const float target[6]) {
    if(!s||!valid_config(c)||!previous||!target||!s->requested||s->committed||
       (uint32_t)(s->last_ms-s->request_tick_ms)>1000u||
       (int32_t)(tick-s->request_tick_ms)<0||!isfinite(c->maximum_leg_target_jump_rad)||
       !isfinite(c->maximum_wheel_target_jump_rad_s))return 0;
    for(unsigned i=0;i<6;i++){
        if(!isfinite(previous[i])||!isfinite(target[i]))return 0;
        const float cap=(i==2||i==5)?c->maximum_wheel_target_jump_rad_s:c->maximum_leg_target_jump_rad;
        if(cap<=0||fabsf(target[i]-previous[i])>cap)return 0;
    }
    return 1;
}

int RLHandoffObservePosture(RLHandoffState *s,const RLHandoffConfig *c,
                            uint32_t tick,int valid,const float q[4],const float dq[6],
                            const float gyro[3],const float gravity[3],
                            const float center[4],float error)
{
    if(!s)return 0;
    if(!valid_config(c)){s->dwelling=s->requested=0;return 0;}
    if(s->committed)return 1;
    if(s->observed && (int32_t)(tick-s->last_ms)<=0)return s->requested;
    if(s->observed && (uint32_t)(tick-s->last_ms)>20u)s->dwelling=s->requested=0;
    s->last_ms=tick;s->observed=1;
    int good=valid && q && dq && gyro && gravity && center && isfinite(error) && error>0 && error<=.5f;
    float gn=0,wn=0;
    if(good){
        for(unsigned i=0;i<3;i++){
            if(!isfinite(gyro[i])||!isfinite(gravity[i]))good=0;
            gn+=gravity[i]*gravity[i];wn+=gyro[i]*gyro[i];
        }
        /* Upright model gravity is[0,-1,0]; compare cone without Euler axes. */
        if(!isfinite(gn)||!isfinite(wn)||fabsf(gn-1.f)>.02f ||
           -gravity[1]<cosf(c->tilt_rad) || wn>c->angular_speed_rad_s*c->angular_speed_rad_s)good=0;
        const unsigned legs[4]={0,1,3,4};
        for(unsigned i=0;i<6;i++)if(!isfinite(dq[i]))good=0;
        for(unsigned i=0;i<4;i++){
            if(!isfinite(q[i])||!isfinite(center[i]) || fabsf(q[i]-center[i])>error ||
               fabsf(dq[legs[i]])>c->leg_speed_rad_s)good=0;
        }
    }
    if(!good){s->dwelling=s->requested=0;return 0;}
    if(!s->dwelling){s->since_ms=tick;s->dwelling=1;}
    if((uint32_t)(tick-s->since_ms)>=c->dwell_ms){
        if(!s->requested)s->request_tick_ms=tick;
        s->requested=1;
    }
    return s->requested;
}
