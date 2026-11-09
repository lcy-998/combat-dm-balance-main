#include "rl_spring_compensation.h"
#include "dm_position_pd.h"
#include <assert.h>
#include <stdio.h>
int main(void){
 float t[4]; const float signs[4]={-1,-1,1,-1};
 for(int k=0;k<101;k++){
  float a=.5f+k*.02f,b=.6f+k*.018f;
  assert(!RLSpringDifferenceTorque(16.5f,20.f,a,b,4.31f,t));
  float mechanical[4]={-16.5f*sinf(a),16.5f*sinf(a),-20.f*sinf(b),-20.f*sinf(b)};
  float wanted[4]={-4.31f,0,-4.31f,0};
  for(int j=0;j<4;j++)assert(fabsf(signs[j]*t[j]+mechanical[j]-wanted[j])<3e-6f);
 }
 assert(RLSpringDifferenceTorque(NAN,16.5,1,1,4.31,t)==-1);
 for(int j=0;j<4;j++)assert(t[j]==0);
 float e;assert(DMPositionPDTorque(1,0,0,0,60,2,20,40,&e)==40);
 assert(DMPositionPDTorque(-1,0,0,0,60,2,-20,40,&e)==-40);
 puts("spring difference:101 paired geometry checks + invalid input + total torque clamp passed");
}
