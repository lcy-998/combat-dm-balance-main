#include "rl_policy_core.h"
#include "rl_motion31.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
static int finite_values(const float *p, size_t n) {
    if (!p) return 0;
    for (size_t i=0;i<n;++i) if (!isfinite(p[i])) return 0;
    return 1;
}
uint32_t RLPolicyCoreAbi(void) { return RL_POLICY_CORE_ABI; }
int RLPolicyEncode(const float *gyro,const float *gravity,const float *q,const float *dq,
                   const float *adopted,const float *command,uint32_t skill,uint32_t jump,float *o) {
    if (!o || skill>2 || jump>2 || !finite_values(gyro,3) || !finite_values(gravity,3) ||
        !finite_values(q,4) || !finite_values(dq,6) || !finite_values(adopted,6) || !finite_values(command,3)) return -1;
    const unsigned leg[4]={0,1,3,4};
    for (unsigned i=0;i<3;++i) { o[i]=gyro[i]*.25f; o[3+i]=gravity[i]; }
    for (unsigned i=0;i<4;++i) { o[6+i]=q[i]/1.5f; o[16+i]=adopted[leg[i]]/1.5f; }
    for (unsigned i=0;i<6;++i) { o[10+i]=dq[i]*.05f; o[25+i]=0; }
    o[20]=adopted[2]/100.f; o[21]=adopted[5]/100.f;
    o[22]=command[0]/4.f; o[23]=command[1]/4.f; o[24]=command[2]/.5f;
    o[25+skill]=1; o[28+jump]=1;
    return finite_values(o,31)?0:-1;
}
int RLPolicyDecode(const RLPolicyDecodeConfig *c,const float *raw,float h,float *target) {
    if (!c || !target || c->abi!=RL_POLICY_CORE_ABI || !finite_values(raw,6) || !isfinite(h) ||
        h<c->height_min-1e-7 || h>c->height_max+1e-7 || (c->center_kind==RL_POLICY_CENTER_TABLE_WIDE ?
            (c->leg_scale!=.5f || c->wheel_scale!=80.f) : c->leg_scale!=.15f) ||
        (c->wheel_scale!=40.f && c->wheel_scale!=80.f && !(c->center_kind==RL_POLICY_RECOVERY_CUBIC && c->wheel_scale==100.f))) return -1;
    if (c->center_kind==RL_POLICY_RECOVERY_CUBIC) {
        const unsigned indices[4]={0,1,3,4};
        target[2]=c->wheel_scale*fminf(1,fmaxf(-1,raw[2]));
        target[5]=c->wheel_scale*fminf(1,fmaxf(-1,raw[5]));
        for (unsigned j=0;j<4;++j) {
            const unsigned i=indices[j];
            if (!isfinite(c->offset[j]) || !isfinite(c->leg_lower[j]) || !isfinite(c->leg_upper[j]) ||
                c->leg_lower[j]>c->offset[j] || c->leg_upper[j]<c->offset[j]) return -1;
            const float v=fminf(1,fmaxf(-1,raw[i]));
            const float reach=(float)(v>=0?c->leg_upper[j]-c->offset[j]:c->offset[j]-c->leg_lower[j]);
            target[i]=(float)c->offset[j]+c->leg_scale*v+(reach-c->leg_scale)*v*v*v;
        }
        return finite_values(target,6)?0:-1;
    }
    double legs[4];
    if (c->center_kind==RL_POLICY_CENTER_POLY5) {
        if (RLMotion31SupportCenter(h,legs)) return -1;
    } else {
        const double *g=c->geometry;
        for (unsigned i=0;i<6;++i) if (!isfinite(g[i])) return -1;
        const double length=h+g[0]-g[1];
        if (length<=0 || g[2]<=0 || g[3]<=0) return -1;
        const double cosine=(length*length+g[2]*g[2]-g[3]*g[3])/(2*length*g[2]);
        if (cosine < -1 || cosine > 1) return -1;
        const double beta=acos(cosine), radians=3.14159265358979323846/180.;
        legs[0]=beta-g[4]*radians; legs[1]=beta-g[5]*radians;
        legs[2]=-legs[0]; legs[3]=legs[1];
        if (c->center_kind==RL_POLICY_CENTER_CONSTANT) {
            for (unsigned i=0;i<4;++i) { if (!isfinite(c->offset[i])) return -1; legs[i]+=c->offset[i]; }
        } else if ((c->center_kind==RL_POLICY_CENTER_TABLE || c->center_kind==RL_POLICY_CENTER_TABLE_WIDE)) {
            unsigned n=c->node_count;
            if (n<2 || n>RL_POLICY_MAX_NODES) return -1;
            for (unsigned i=0;i<n;++i) {
                if (!isfinite(c->heights[i]) || !isfinite(c->offsets[i][0]) || !isfinite(c->offsets[i][1]) ||
                    (i && c->heights[i]<=c->heights[i-1])) return -1;
            }
            if (h<c->heights[0]-1e-7 || h>c->heights[n-1]+1e-7) return -1;
            unsigned i=1; while (i<n-1 && h>=c->heights[i]) ++i;
            const double ratio=(h-c->heights[i-1])/(c->heights[i]-c->heights[i-1]);
            double o[2]; for (unsigned j=0;j<2;++j) o[j]=c->offsets[i-1][j]+ratio*(c->offsets[i][j]-c->offsets[i-1][j]);
            legs[0]+=o[0];legs[1]+=o[1];legs[2]-=o[0];legs[3]+=o[1];
        } else return -1;
    }
    const unsigned leg[4]={0,1,3,4};
    for (unsigned i=0;i<6;++i) target[i]=(i==2 || i==5?c->wheel_scale:c->leg_scale)*fminf(1,fmaxf(-1,raw[i]));
    for (unsigned i=0;i<4;++i) target[leg[i]]+=(float)legs[i];
    return finite_values(target,6)?0:-1;
}
int RLPolicyProtect(const float *request,const float *lower,const float *upper,float *out) {
    if (!out || !finite_values(request,6) || !finite_values(lower,6) || !finite_values(upper,6)) return -1;
    for (unsigned i=0;i<6;++i) { if (lower[i]>upper[i]) return -1; out[i]=fmaxf(lower[i],fminf(upper[i],request[i])); }
    return 0;
}
int RLPolicyAdopt(const float *protected_target,float *adopted) {
    if (!adopted || !finite_values(protected_target,6)) return -1;
    memcpy(adopted,protected_target,6*sizeof(float)); return 0;
}
int RLPolicyPd(const float *q,const float *dq,const float *target,const float *gains,const float *caps,float *raw,float *limited) {
    if (!raw || !limited || !finite_values(q,4) || !finite_values(dq,6) || !finite_values(target,6) ||
        !finite_values(gains,3) || !finite_values(caps,6)) return -1;
    if (gains[0]<=0 || gains[1]<0 || gains[2]<0) return -1;
    for (unsigned i=0;i<6;++i) {
        if (caps[i]<0) return -1;
        raw[i]=(i==2 || i==5)?gains[2]*(target[i]-dq[i]):gains[0]*(target[i]-q[i<2?i:i-1])-gains[1]*dq[i];
        if (!isfinite(raw[i])) return -1;
        limited[i]=fmaxf(-caps[i],fminf(caps[i],raw[i]));
    }
    return 0;
}
