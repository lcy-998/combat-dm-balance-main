#include "wheel_friction.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void near(float a,float b){assert(fabsf(a-b)<.004f);}
int main(void){
 float c,i,prev=0;
 for(unsigned side=0;side<2;++side){
  float sign=side==0?-1.f:1.f;
  float d=side==0?.0494515f:.05133043f;
  float f=side==0?.053699f:.051843f;
  assert(!WheelRLCompensate(side,0,0,&c,&i));near(i,0);near(c,0);
  assert(!WheelRLCompensate(side,30,0,&c,&i));near(c,-d*30+f);near(i,sign*c*WHEEL_CURRENT_PER_NM);
  assert(!WheelRLCompensate(side,-30,0,&c,&i));near(c,d*30-f);
  /* 5.5 drive must survive mapping until compensation: preclip gives wrong result. */
  assert(!WheelRLCompensate(side,30,sign*5.5f*WHEEL_CURRENT_PER_NM,&c,&i));
  near(i,sign*(5.5f-d*30+f)*WHEEL_CURRENT_PER_NM);
  assert(!WheelRLCompensate(side,-30,sign*5.5f*WHEEL_CURRENT_PER_NM,&c,&i));near(i,sign*16384);
  for(int k=-600;k<=600;++k){
   assert(!WheelRLCompensate(side,k*.1f,0,&c,&i));assert(fabsf(i)<=16384);
   if(k>-600) assert(fabsf(i-prev)<170);
   prev=i;
  }
  assert(WheelRLCompensate(side,NAN,100,&c,&i)!=0);near(i,0);
 }
 assert(WheelRLCompensate(2,0,0,&c,&i)!=0);
 near(WHEEL_TORQUE_LIMIT_NM,4.893896f);
 puts("RL single-clip damping/Coulomb compensation tests passed");
}
