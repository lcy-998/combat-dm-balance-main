#include <math.h>
#include "motion_coordinates.h"
static const int LEG[4]={0,1,3,4};
static int finite_array(const float*x,int n){if(!x)return 0;for(int i=0;i<n;i++)if(!isfinite(x[i]))return 0;return 1;}
static int valid(const motion31_leg_calibration*c){if(!c||c->known!=1)return -2;return finite_array(c->delta_rad,4)?0:-1;}
static int q_convert(const float*x,const motion31_leg_calibration*c,float*y,float sign){
 int e=valid(c);if(e)return e;if(!y||!finite_array(x,4))return -1;
 float a[4];for(int i=0;i<4;i++){a[i]=x[i]+sign*c->delta_rad[i];if(!isfinite(a[i]))return -1;}
 for(int i=0;i<4;i++)y[i]=a[i];return 0;
}
int motion31_q_model_from_report(const float*x,const motion31_leg_calibration*c,float*y){return q_convert(x,c,y,-1.f);}
int motion31_q_report_from_model(const float*x,const motion31_leg_calibration*c,float*y){return q_convert(x,c,y,1.f);}
int motion31_observation_model_from_report(const float*x,const motion31_leg_calibration*c,float*y){
 int e=valid(c);if(e)return e;if(!y||!finite_array(x,31))return -1;
 float a[31];for(int i=0;i<31;i++)a[i]=x[i];
 for(int i=0;i<4;i++){a[6+i]-=c->delta_rad[i]/1.5f;a[16+i]-=c->delta_rad[i]/1.5f;}
 if(!finite_array(a,31))return -1;for(int i=0;i<31;i++)y[i]=a[i];return 0;
}
static int target_convert(const float*x,const motion31_leg_calibration*c,float*y,float sign){
 int e=valid(c);if(e)return e;if(!y||!finite_array(x,6))return -1;
 float a[6];for(int i=0;i<6;i++)a[i]=x[i];for(int i=0;i<4;i++)a[LEG[i]]+=sign*c->delta_rad[i];
 if(!finite_array(a,6))return -1;for(int i=0;i<6;i++)y[i]=a[i];return 0;
}
int motion31_target_report_from_model(const float*x,const motion31_leg_calibration*c,float*y){return target_convert(x,c,y,1.f);}
int motion31_target_model_from_report(const float*x,const motion31_leg_calibration*c,float*y){return target_convert(x,c,y,-1.f);}
int motion31_limits_model_from_report(const float*x,const motion31_leg_calibration*c,float*y){
 int e=valid(c);if(e)return e;if(!y||!finite_array(x,8))return -1;
 float a[8];for(int i=0;i<4;i++){if(x[2*i]>x[2*i+1])return -1;for(int j=0;j<2;j++)a[2*i+j]=x[2*i+j]-c->delta_rad[i];}
 if(!finite_array(a,8))return -1;for(int i=0;i<8;i++)y[i]=a[i];return 0;
}
int motion31_center_from_unprotected_request(const float*m,const float*t,int unprotected,int clipped,float*c){
 if(!unprotected||clipped)return -4;if(!c||!finite_array(m,6)||!finite_array(t,6))return -1;
 for(int i=0;i<6;i++)if(fabsf(m[i])>1.000001f)return -1;
 float a[4];for(int i=0;i<4;i++)a[i]=t[LEG[i]]-.15f*m[LEG[i]];
 if(!finite_array(a,4))return -1;for(int i=0;i<4;i++)c[i]=a[i];return 0;
}
