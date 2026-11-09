#ifndef RL_HEADING_VELOCITY_H
#define RL_HEADING_VELOCITY_H

#include "rl_shadow.h"
#include <math.h>
#include <stddef.h>

/* Training root-rigid-body COM reference POINT relative to midpoint of hip
 * pivots, in the policy's forward/right/up coordinates, metres. This is NOT
 * a measurement of the real robot's mass centre. Geometry from nominal asset.
 * Wheel stator attaches to LOWER leg D->C, not virtual leg O->C.
 * Assumptions: two wheels in rolling contact, no longitudinal slip. No filter
 * or extra delay is introduced. LQR odometry and its KF are separate.
 */
#define RL_VX_REFERENCE_FORWARD_M 0.0055423f
#define RL_VX_REFERENCE_RIGHT_M (-0.0052237f)
#define RL_VX_REFERENCE_UP_M 0.1277691f
#define RL_VX_HALF_TRACK_M 0.2215f
#define RL_VX_WHEEL_RADIUS_M 0.06f

static inline void RLHeadingCross(const float a[3], const float b[3], float out[3])
{
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

/* q and dq are physical lf/lb/rf/rb angle coordinates, already calibrated.
 * Wheel rates use RL model signs (both negative when rolling forward).
 * Returns 0 on success; failure leaves every output NAN for input rejection.
 */
static inline int RLHeadingVelocity(const RLShadowSnapshot *s,
                                     float *heading_vx, float side_vx[2])
{
    if (heading_vx == NULL || side_vx == NULL) return -1;
    *heading_vx = NAN;
    side_vx[0] = side_vx[1] = NAN;
    if (s == NULL) return -1;
    const float attitude[5] = {s->roll_rad, s->pitch_rad, s->roll_rate_rad_s,
                              s->pitch_rate_rad_s, s->yaw_rate_rad_s};
    for (unsigned i=0; i<5; ++i) if (!isfinite(attitude[i])) return -1;
    for (unsigned i=0; i<4; ++i)
        if (!isfinite(s->joint_angle_rad[i]) || !isfinite(s->joint_rate_rad_s[i])) return -1;
    for (unsigned i=0; i<2; ++i) if (!isfinite(s->wheel_rate_rad_s[i])) return -1;

    const float cr = cosf(s->roll_rad);
    const float g[3] = {sinf(s->pitch_rad)*cr, sinf(s->roll_rad),
                        -cosf(s->pitch_rad)*cr};
    float h[3] = {1.0f-g[0]*g[0], -g[0]*g[1], -g[0]*g[2]};
    const float hn = sqrtf(h[0]*h[0]+h[1]*h[1]+h[2]*h[2]);
    const float cn = sqrtf(g[0]*g[0]+g[2]*g[2]);
    if (hn < 1e-5f || cn < 1e-5f) return -1;
    for (unsigned i=0; i<3; ++i) h[i] /= hn;
    const float contact[3] = {RL_VX_WHEEL_RADIUS_M*g[0]/cn, 0.0f,
                              RL_VX_WHEEL_RADIUS_M*g[2]/cn};
    const float omega[3] = {s->roll_rate_rad_s, -s->pitch_rate_rad_s,
                            s->yaw_rate_rad_s};
    float results[2];
    for (unsigned side=0; side<2; ++side) {
        const unsigned i=side*2;
        const float a=s->joint_angle_rad[i], b=s->joint_angle_rad[i+1];
        const float da=fabsf(s->joint_rate_rad_s[i])<.022f ? 0.0f:s->joint_rate_rad_s[i];
        const float db=fabsf(s->joint_rate_rad_s[i+1])<.022f ? 0.0f:s->joint_rate_rad_s[i+1];
        const float m=.5f*(a+b), d=.5f*(a-b), dm=.5f*(da+db), dd=.5f*(da-db);
        const float sd=sinf(d), cd=cosf(d), sm=sinf(m), cm=cosf(m);
        const float radicand=.25f*.25f-.21f*.21f*sd*sd;
        if (radicand <= 1e-10f) return -1;
        const float height=sqrtf(radicand);
        const float length=.21f*cd+height;
        const float dl=(-.21f*sd-.21f*.21f*sd*cd/height)*dd;
        const float x=length*cm, y=length*sm;
        const float dx=dl*cm-length*sm*dm, dy=dl*sm+length*cm*dm;
        const float ex=x-.21f*cosf(b), ey=y-.21f*sinf(b);
        const float edx=dx+.21f*sinf(b)*db, edy=dy-.21f*cosf(b)*db;
        const float en=ex*ex+ey*ey;
        if (en <= 1e-10f) return -1;
        const float carrier=(ex*edy-ey*edx)/en;
        const float r[3] = {-x-RL_VX_REFERENCE_FORWARD_M,
                            (side ? RL_VX_HALF_TRACK_M:-RL_VX_HALF_TRACK_M)-RL_VX_REFERENCE_RIGHT_M,
                            -y-RL_VX_REFERENCE_UP_M};
        const float dr[3] = {-dx, 0.0f, -dy};
        const float wheel_omega[3] = {omega[0], omega[1]+carrier+s->wheel_rate_rad_s[side], omega[2]};
        float rolling[3], transport[3];
        RLHeadingCross(wheel_omega, contact, rolling);
        RLHeadingCross(omega, r, transport);
        /* Policy axes have determinant -1 relative to source: physical cross
         * is minus coordinate cross. Thus v_ref = w_wheel x rho + w x r - dr.
         */
        results[side]=0.0f;
        for (unsigned j=0; j<3; ++j) results[side] += (rolling[j]+transport[j]-dr[j])*h[j];
        if (!isfinite(results[side])) return -1;
    }
    const float mean=.5f*results[0]+.5f*results[1];
    if (!isfinite(mean)) return -1;
    side_vx[0]=results[0]; side_vx[1]=results[1];
    *heading_vx=mean;
    return 0;
}
#endif
