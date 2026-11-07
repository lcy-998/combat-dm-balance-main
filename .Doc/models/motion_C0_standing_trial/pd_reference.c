#include <math.h>
int motion31_pd_request(const float target[6],const float q[6],const float dq[6],float out[6]){
 for(int i=0;i<6;i++){if(!isfinite(target[i])||!isfinite(q[i])||!isfinite(dq[i]))return -1;
 out[i]=(i==2||i==5)? .2f*(target[i]-dq[i]):40.f*(target[i]-q[i])-2.f*dq[i];}return 0;}
