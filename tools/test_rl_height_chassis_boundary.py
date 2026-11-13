"""CPU test of exact CurrentRLSnapshot/ChassisSetRLHeight source boundaries.

Only hardware/RTOS and leg-geometry IO are doubles. Never flash or mj_step.
"""
import os
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def function(source,name):
    match=re.search(r'(?:static\s+)?(?:RLShadowSnapshot|int)\s+'+re.escape(name)+r'\([^)]*\)\s*\{',source)
    if match is None:raise ValueError('Missing public boundary '+name)
    pos=match.end();depth=1
    while depth:
        if source[pos]=='{':depth+=1
        if source[pos]=='}':depth-=1
        pos+=1
    return source[match.start():pos]


class ChassisHeightBoundary(unittest.TestCase):
    def test_actual_snapshot_and_external_height_api_keep_axes_previous_and_motion(self):
        source=(ROOT/'application/chassis/chassis.c').read_text()
        macros='\n'.join(line for line in source.splitlines() if re.match(r'#define (?:LF|LB|RF|RB)_ENCODE_OFFSET\b|#define REDUCE_RATIO\b|#define LEG_LENGTH_(?:SPEED_M_S|STEP_M)\b',line))
        c='''
#include "rl_control.h"
#include "rl_recovery_config.h"
#include <math.h>
#include <string.h>
#include <assert.h>
#define PI 3.14159265358979323846f
#define DEGREE_2_RAD (PI/180.0f)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define CHASSIS_ZERO_FORCE 0
#define L_1 .21f
#define L_2 .25f
#define L_3 .25f
#define L_4 .21f
#define L_5 .06f
typedef struct {struct {float pos,vel,tor,speed_aps,real_current;} measure;uint32_t feedback_receive_tick_ms,feedback_sequence;} DMMotorInstance;
static DMMotorInstance joints[4],wheels[2];
static DMMotorInstance *jiont_motorlf=&joints[0],*jiont_motorlb=&joints[1],*jiont_motorrf=&joints[2],*jiont_motorrb=&joints[3];
static DMMotorInstance *motor_l=&wheels[0],*motor_r=&wheels[1];
static struct {float Pitch,Roll,Gyro[3];uint32_t update_tick_ms,update_sequence;} imu;
static __typeof__(imu) *IMU_data=&imu;
static struct {int changeL,chassis_mode;} chassis_cmd_recv;
static RLControlState rl_control_state;
static RLHandoffSample rl_handoff_evidence;
static float rl_height_request_m=RL_MOTION31_HEIGHT_M;
static float v_cmd=1.2f,w_cmd=.4f;
static int chassis_status,upstair_flag,jumpingflag,upstair_flag0;
static int ChassisNormalLQRState(void){return 1;}
static uint32_t xTaskGetTickCount(void){return 100;}
static void getPhi(float a,float b,float c,float d,float e,float f,float g,float *p2,float *p3,float *length,float *theta){
 (void)a;(void)b;(void)c;(void)d;(void)e;(void)f;(void)g;
 *p2=*p3=*theta=0;*length=.28f;
}
'''+macros+'\n'+function(source,'ChassisSetRLHeight')+'\n'+function(source,'CurrentRLSnapshot')+'''
int main(void){
  const float prev[6]={.11f,.12f,13,.14f,.15f,-16};
  const float hip=(90-47.1033888f)*PI/180,active=(90+47.1080837f)*PI/180;
  joints[0].measure.pos=LF_ENCODE_OFFSET-active;joints[1].measure.pos=LB_ENCODE_OFFSET-hip;
  joints[2].measure.pos=active-(RF_ENCODE_OFFSET);joints[3].measure.pos=hip-(RB_ENCODE_OFFSET);
  for(int i=0;i<4;++i){joints[i].feedback_receive_tick_ms=100;joints[i].feedback_sequence=1;joints[i].measure.vel=.4f;}
  for(int i=0;i<2;++i){wheels[i].feedback_receive_tick_ms=100;wheels[i].feedback_sequence=1;wheels[i].measure.speed_aps=300;}
  imu.update_tick_ms=100;imu.update_sequence=1;chassis_cmd_recv.chassis_mode=1;
  rl_control_state.active=1;rl_control_state.ready=1;rl_control_state.adopted_sequence=99;
  memcpy(rl_control_state.adopted_target,prev,sizeof(prev));
  assert(!ChassisSetRLHeight(.40f));RLShadowSnapshot a=CurrentRLSnapshot(1);
  assert(a.motion.command_height_m==.40f);
  assert(a.motion.command_vx_m_s==1.2f && a.motion.command_yaw_rad_s==.4f);
  assert(a.previous_source==RL_PREVIOUS_ADOPTED && a.adopted_sequence==99);
  assert(!memcmp(a.motion.previous_target,prev,sizeof(prev)));
  for(int i=0;i<4;++i)assert(fabsf(a.motion.joint_position_rad[i])<1e-6f);
  assert(a.motion.joint_velocity_rad_s[0]==-.4f);
  assert(a.motion.joint_velocity_rad_s[1]==.4f);
  assert(a.motion.joint_velocity_rad_s[3]==-.4f);
  assert(a.motion.joint_velocity_rad_s[4]==-.4f);
  assert(!ChassisSetRLHeight(.30f));RLShadowSnapshot b=CurrentRLSnapshot(1);
  assert(b.motion.command_height_m==.30f);
  assert(!memcmp(b.motion.previous_target,prev,sizeof(prev)));
  chassis_cmd_recv.changeL=-1;b=CurrentRLSnapshot(1);
  assert(fabsf(b.motion.command_height_m-(.30f+.048f*.005f))<1e-7f);
  assert(b.motion.command_vx_m_s==1.2f && b.motion.command_yaw_rad_s==.4f);
  assert(ChassisSetRLHeight(.41f)<0);assert(ChassisSetRLHeight(NAN)<0);
  return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'actual_snapshot.c';path.write_text(c)
            subprocess.run([os.getenv('CC','cc'),'-std=gnu11','-O2','-Wall','-Wextra','-Werror',
                '-I'+str(ROOT/'application/chassis'),str(path),str(ROOT/'application/chassis/rl_motion31.c'),
                str(ROOT/'application/chassis/rl_control.c'),str(ROOT/'application/chassis/rl_handoff.c'),'-lm','-o',str(Path(tmp)/'snapshot')],check=True)
            subprocess.run([str(Path(tmp)/'snapshot')],check=True)


if __name__=='__main__':unittest.main()
