/* FP32 reference only. No IO, allocation, motor drivers or timing. */
#include <math.h>
#include "actor_weights.h"
static void dense(const float*x,float*y,const float*w,const float*b,int ni,int no,int elu){
 for(int o=0;o<no;o++){float z=b[o];for(int i=0;i<ni;i++)z+=w[o*ni+i]*x[i];y[o]=(elu&&z<0)?expm1f(z):z;}}
int motion31_actor(const float input[31],float output[6]){
 for(int i=0;i<31;i++)if(!isfinite(input[i]))return -1;
 float a[128],b[64],c[32],d[6];
 dense(input,a,W0,B0,31,128,1);dense(a,b,W1,B1,128,64,1);
 dense(b,c,W2,B2,64,32,1);dense(c,d,W3,B3,32,6,0);
 for(int i=0;i<6;i++){if(!isfinite(d[i]))return -1;d[i]=tanhf(d[i]);}
 for(int i=0;i<6;i++)output[i]=d[i];return 0;
}
/* center[4] comes from the frozen output_contract, at commanded height.
 * limits and center are model-coordinate radians; output is six physical targets.
 * This function does NOT update previous: the actual target adopter must do so. */
int motion31_decode(const float mean[6],const float center[4],const float lo[4],const float hi[4],float target[6]){
 const int ix[4]={0,1,3,4};float v[6];
 for(int i=0;i<6;i++){if(!isfinite(mean[i]))return -1;v[i]=80.f*fmaxf(-1.f,fminf(1.f,mean[i]));}
 for(int i=0;i<4;i++){
  if(!isfinite(center[i])||!isfinite(lo[i])||!isfinite(hi[i])||lo[i]>hi[i])return -1;
  v[ix[i]]=fmaxf(lo[i],fminf(hi[i],center[i]+.15f*fmaxf(-1.f,fminf(1.f,mean[ix[i]]))));}
 for(int i=0;i<6;i++)target[i]=v[i];return 0;
}
