#include "rl_heading_velocity.h"
#include <assert.h>
#include <stdio.h>

static void close_to(float a, float b) { assert(fabsf(a-b)<2e-5f); }

int main(void)
{
    RLShadowSnapshot s={0};
    s.joint_angle_rad[0]=s.joint_angle_rad[2]=RL_SHADOW_ACTIVE_ZERO_RAD;
    s.joint_angle_rad[1]=s.joint_angle_rad[3]=RL_SHADOW_HIP_ZERO_RAD;
    float v, sides[2];
    assert(RLHeadingVelocity(&s,&v,sides)==0); close_to(v,0);
    s.wheel_rate_rad_s[0]=s.wheel_rate_rad_s[1]=-5;
    assert(RLHeadingVelocity(&s,&v,sides)==0);
    close_to(v,.3f); close_to(sides[0],.3f); close_to(sides[1],.3f);
    /* Each contact rolls at a different speed during yaw. Transport each to
     * the SAME root reference before averaging; pure yaw is zero translation. */
    s.yaw_rate_rad_s=1.7f;
    for (unsigned i=0; i<2; ++i) {
        float lateral=(i ? RL_VX_HALF_TRACK_M:-RL_VX_HALF_TRACK_M)-RL_VX_REFERENCE_RIGHT_M;
        s.wheel_rate_rad_s[i]=-s.yaw_rate_rad_s*lateral/RL_VX_WHEEL_RADIUS_M;
    }
    assert(RLHeadingVelocity(&s,&v,sides)==0);
    close_to(v,0); close_to(sides[0],0); close_to(sides[1],0);
    /* The estimator changes ONLY obs6. All other policy inputs, including
     * gravity, commands, wheel-rate signs and history, stay byte-identical. */
    float old_observation[25], new_observation[25];
    s.pitch_rate_rad_s=1.0f;
    assert(RLShadowBuildObservation(&s,old_observation)==0);
    assert(RLHeadingVelocity(&s,&s.heading_vx_m_s,sides)==0);
    assert(RLShadowBuildObservation(&s,new_observation)==0);
    for (unsigned j=0; j<25; ++j)
        if (j!=6) assert(old_observation[j]==new_observation[j]);
    assert(fabsf(old_observation[6]-new_observation[6])>.1f);
    s.joint_rate_rad_s[2]=NAN;
    assert(RLHeadingVelocity(&s,&v,sides)!=0 && isnan(v) && isnan(sides[0]) && isnan(sides[1]));
    s.joint_rate_rad_s[2]=0; s.pitch_rad=1.57079632679f;
    assert(RLHeadingVelocity(&s,&v,sides)!=0); /* heading undefined at vertical */
    assert(RLHeadingVelocity(NULL,&v,sides)!=0);
    puts("RL heading velocity physical sign, turning, and invalid-input tests passed");
    return 0;
}
