"""CPU test of exact height, snapshot and Stair command boundaries.

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
    match=re.search(r'(?:static\s+)?(?:RLShadowSnapshot|int|void)\s+'+re.escape(name)+r'\([^)]*\)\s*\{',source)
    if match is None:
        raise ValueError('Missing public boundary '+name)
    pos=match.end()
    depth=1
    while depth:
        if source[pos]=='{':
            depth+=1
        if source[pos]=='}':
            depth-=1
        pos+=1
    return source[match.start():pos]


class ChassisHeightBoundary(unittest.TestCase):
    def test_actual_snapshot_and_external_height_api_keep_axes_previous_and_motion(self):
        source=(ROOT/'application/chassis/chassis.c').read_text()
        macros='\n'.join(line for line in source.splitlines() if re.match(r'#define (?:LF|LB|RF|RB)_ENCODE_OFFSET\b|#define REDUCE_RATIO\b|#define LEG_LENGTH_(?:STEP_M|SPEED_M_S)\b',line))
        c='''
#include "rl_control.h"
#include "rl_stair.h"
#include "stair_actor.h"
#include "leg_const.hpp"
#include <math.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#define DEGREE_2_RAD (PI/180.0f)
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
enum { CHASSIS_ZERO_FORCE=0, CHASSIS_ROTATE, CHASSIS_NO_FOLLOW, CHASSIS_FOLLOW_GIMBAL_YAW };
typedef struct {struct {float pos,vel,tor,speed_aps,real_current;uint8_t state;} measure;uint32_t feedback_receive_tick_ms,feedback_sequence;} DMMotorInstance;
static DMMotorInstance joints[4],wheels[2];
static DMMotorInstance *jiont_motorlf=&joints[0],*jiont_motorlb=&joints[1],*jiont_motorrf=&joints[2],*jiont_motorrb=&joints[3];
static DMMotorInstance *motor_l=&wheels[0],*motor_r=&wheels[1];
static struct {float Pitch,Roll,Gyro[3],Accel[3];uint32_t update_tick_ms,update_sequence;} imu;
static __typeof__(imu) *IMU_data=&imu;
static struct {int changeL,chassis_mode;} chassis_cmd_recv;
static RLControlState rl_control_state;
static float rl_height_request_m=RL_MOTION31_HEIGHT_M;
static float v_cmd=1.2f,w_cmd=.4f;
static int chassis_status,jumpingflag;
static RLStairHandoff stair_handoff;
static uint8_t stair_capture_prepared;
static uint32_t now=100;
static int ChassisNormalLQRState(void){return 1;}
static uint32_t xTaskGetTickCount(void){return now;}
static void getPhi(float a,float b,float c,float d,float e,float f,float g,float *p2,float *p3,float *length,float *theta){
 (void)a;(void)b;(void)c;(void)d;(void)e;(void)f;(void)g;
 *p2=*p3=*theta=0;*length=.28f;
}
'''+macros+'\n'+ '\n'.join(function(source, name) for name in (
            'ChassisSetRLHeight', 'ChassisCanDetectStair', 'CurrentRLSnapshot', 'ChassisStairFrameAdopted',
        ))+'''
/* Real snapshot, Stair detector, decode, PD adoption and commit callback.
   Feed synthetic wheel loads and attitude; Motion means are stubbed. */
static float stair_tick(RLStairSample *plant)
{
  now+=5;
  imu.update_tick_ms=now;
  ++imu.update_sequence;
  for(unsigned i=0;i<4;++i) joints[i].feedback_receive_tick_ms=now;
  for(unsigned i=0;i<2;++i) wheels[i].feedback_receive_tick_ms=now;

  RLStairSample io={0};
  RLShadowSnapshot s=CurrentRLSnapshot(&io);
  plant->can_enter=ChassisCanDetectStair(1);
  plant->link_fresh=1;
  plant->feedback_fresh=io.feedback_fresh && RLControlFeedbackFresh(&s,now);
  plant->tick_ms=s.tick_ms;
  plant->imu_sequence=s.imu_update_sequence;
  plant->vx_cmd=s.motion.command_vx_m_s;
  RLStairUpdate(&stair_handoff,plant);
  const uint8_t actor=stair_handoff.phase==RL_STAIR_ACTIVE ? RL_ACTOR_STAIR : RL_ACTOR_MOTION;
  (void)RLControlSelectActor(&rl_control_state,actor);
  const int engaged=RLStairEngaged(&stair_handoff);
  if (engaged) s.motion.command_height_m=RL_STAIR_GOAL_M;
  RLControlFillPrevious(&rl_control_state,&s);
  RLShadowCommand packet={
    .valid=1,.actor=actor,.snapshot_sequence=s.sequence,
    .snapshot_tick_ms=s.tick_ms,.output_tick_ms=now,.control_epoch=s.control_epoch,
    .previous_source=s.previous_source,.previous_sequence=s.adopted_sequence
  };
  if (actor==RL_ACTOR_STAIR) {
    assert(!RLStairBuildObservation(&s.motion,packet.preview.observation));
    assert(!StairActorPhysical(packet.preview.observation,packet.action,packet.preview.target));
  } else {
    assert(!RLMotion31BuildPreview(&s.motion,packet.action,&packet.preview));
  }
  RLControlFrame frame;
  assert(!RLControlExecute(&rl_control_state,now,1,&s,&packet,&frame));
  assert(rl_control_state.adopted_height_m==s.motion.command_height_m);
  if (engaged) ChassisStairFrameAdopted(rl_control_state.actor,frame.tick_ms);
  return s.motion.command_height_m;
}

static void continuous_stairs(void)
{
  const float approach_height=.38f;
  assert(!ChassisSetRLHeight(approach_height)); /* one height request for the flight */
  RLControlReset(&rl_control_state);
  RLStairReset(&stair_handoff);
  rl_control_state.ramp_completed=rl_control_state.timing=rl_control_state.ready=1u;
  chassis_cmd_recv.chassis_mode=CHASSIS_NO_FOLLOW;
  chassis_cmd_recv.changeL=0;
  joints[0].measure.state=1;
  v_cmd=1.60f;w_cmd=0;
  RLStairSample s={
    .vx_estimate=1.2f, .wheel_to_root_m={approach_height,approach_height},
    .wheel_up_estimate_n={90,90}
  };
  assert(stair_tick(&s)==approach_height);
  for (unsigned i=0;i<3;++i) {
    s.pitch=s.roll=s.pitch_rate=s.omega_squared=s.swing[0]=0;
    s.wheel_to_root_m[0]=s.wheel_to_root_m[1]=approach_height;
    assert(stair_tick(&s)==approach_height);
    assert(stair_handoff.have_anchor && fabsf(stair_handoff.tread_estimate_m[0])<1e-6f);
    stair_capture_prepared=1u;
    s.pitch=.15f;s.pitch_rate=.4f;s.omega_squared=.16f;s.swing[0]=.30f;
    for (unsigned j=0;j<5;++j)
      assert(stair_tick(&s)==(j==4 ? RL_STAIR_GOAL_M : approach_height));
    assert(RLControlOwner(&rl_control_state)==RL_OWNER_STAIR && stair_handoff.stair_adopted);

    /* Raise the body about .07 m while the wheels move from the lower tread
       to a tread .20 m higher, leaving .25 m of body clearance. */
    for (unsigned j=0;j<64;++j) {
      s.accel_up=j<32?2.734375f:-2.734375f;
      assert(stair_tick(&s)==RL_STAIR_GOAL_M);
    }
    s.pitch=s.pitch_rate=s.omega_squared=s.swing[0]=s.accel_up=0;
    s.wheel_to_root_m[0]=s.wheel_to_root_m[1]=RL_STAIR_GOAL_M;
    assert(stair_tick(&s)==RL_STAIR_GOAL_M);
    const uint32_t loaded_since=now;
    while ((uint32_t)(now-loaded_since)<RL_STAIR_SUPPORT_MIN_MS) {
      assert(RLControlOwner(&rl_control_state)==RL_OWNER_STAIR);
      assert(stair_tick(&s)==RL_STAIR_GOAL_M); /* first Motion packet stays at .25 */
    }
    assert(RLControlOwner(&rl_control_state)==RL_OWNER_RL &&
           stair_handoff.motion_adopted_tick==now && !stair_capture_prepared);
    assert(!stair_handoff.have_anchor && !ChassisCanDetectStair(1));
    assert(stair_tick(&s)==approach_height); /* no new height or arm command */
    assert(ChassisCanDetectStair(1) && !stair_handoff.have_anchor);
    assert(stair_tick(&s)==approach_height && !stair_handoff.have_anchor); /* body still low */
  }
  assert(rl_height_request_m==approach_height);
  puts("PASS three sequential Stair/Motion handoffs with one Motion height request");
}

int main(void){
  const float prev[6]={.11f,.12f,13,.14f,.15f,-16};
  const float hip=(90-47.1033888f)*PI/180,active=(90+47.1080837f)*PI/180;
  joints[0].measure.pos=LF_ENCODE_OFFSET-active;joints[1].measure.pos=LB_ENCODE_OFFSET-hip;
  joints[2].measure.pos=active-(RF_ENCODE_OFFSET);joints[3].measure.pos=hip-(RB_ENCODE_OFFSET);
  for(int i=0;i<4;++i){
    joints[i].feedback_receive_tick_ms=100;joints[i].feedback_sequence=1;
    joints[i].measure.vel=.4f;joints[i].measure.state=1;
  }
  for(int i=0;i<2;++i){wheels[i].feedback_receive_tick_ms=100;wheels[i].feedback_sequence=1;wheels[i].measure.speed_aps=300;}
  imu.update_tick_ms=100;imu.update_sequence=1;chassis_cmd_recv.chassis_mode=CHASSIS_NO_FOLLOW;
  rl_control_state.active=1;rl_control_state.ready=1;rl_control_state.adopted_sequence=99;
  memcpy(rl_control_state.adopted_target,prev,sizeof(prev));
  assert(!ChassisSetRLHeight(.40f));RLShadowSnapshot a=CurrentRLSnapshot(NULL);
  assert(a.motion.command_height_m==.40f);
  assert(a.motion.command_vx_m_s==1.2f && a.motion.command_yaw_rad_s==.4f);
  assert(a.previous_source==RL_PREVIOUS_ADOPTED && a.adopted_sequence==99);
  assert(!memcmp(a.motion.previous_target,prev,sizeof(prev)));
  for(int i=0;i<4;++i)assert(fabsf(a.motion.joint_position_rad[i])<1e-6f);
  assert(a.motion.joint_velocity_rad_s[0]==-.4f);
  assert(a.motion.joint_velocity_rad_s[1]==.4f);
  assert(a.motion.joint_velocity_rad_s[3]==-.4f);
  assert(a.motion.joint_velocity_rad_s[4]==-.4f);
  assert(!ChassisSetRLHeight(.30f));RLShadowSnapshot b=CurrentRLSnapshot(NULL);
  assert(b.motion.command_height_m==.30f);
  assert(!memcmp(b.motion.previous_target,prev,sizeof(prev)));
  chassis_cmd_recv.changeL=-1;b=CurrentRLSnapshot(NULL);
  assert(fabsf(b.motion.command_height_m-(.30f+.048f*.005f))<1e-7f);
  assert(b.motion.command_vx_m_s==1.2f && b.motion.command_yaw_rad_s==.4f);
  chassis_cmd_recv.changeL=0;
  assert(!ChassisSetRLHeight(.36f));
  RLStairSample stair={0};
  IMU_data->Accel[2]=G;
  IMU_data->Gyro[1]=.2f;
  b=CurrentRLSnapshot(&stair);
  assert(b.motion.command_height_m==.36f && fabsf(stair.accel_up)<1e-6f);
  assert(stair.feedback_fresh && stair.wheel_forward_mps[0]<0 && stair.wheel_forward_mps[1]>0);
  assert(stair.roll_rate==-.2f); /* measured tilt rate, in chassis convention */

  v_cmd=1.60f; w_cmd=1.0f;
  rl_control_state.adopted_height_m=.35f;
  assert(!ChassisCanDetectStair(1)); /* requested .36, but not yet adopted */
  rl_control_state.adopted_height_m=.36f;
  assert(ChassisCanDetectStair(1) && !ChassisCanDetectStair(0)); /* no arm command */
  w_cmd=-1.0f;
  assert(ChassisCanDetectStair(1));
  w_cmd=-1.01f;
  assert(!ChassisCanDetectStair(1));
  w_cmd=0;
  v_cmd=1.50f;
  assert(!ChassisCanDetectStair(1));
  chassis_cmd_recv.changeL=2;
  assert(!ChassisCanDetectStair(1)); /* retired switch cannot override a gate */
  chassis_cmd_recv.changeL=0;
  v_cmd=nextafterf(1.50f,INFINITY);
  assert(ChassisCanDetectStair(1));
  v_cmd=2.21f; rl_control_state.adopted_height_m=.40f;
  assert(ChassisCanDetectStair(1)); /* no independent 2.20 m/s Stair ceiling */
  v_cmd=INFINITY;
  assert(!ChassisCanDetectStair(1));
  v_cmd=1.60f;
  chassis_cmd_recv.chassis_mode=CHASSIS_ROTATE;
  assert(!ChassisCanDetectStair(1));
  chassis_cmd_recv.chassis_mode=CHASSIS_FOLLOW_GIMBAL_YAW;
  assert(ChassisCanDetectStair(1));
  stair_handoff.phase=RL_STAIR_RETURN;
  assert(!ChassisCanDetectStair(1));
  stair_handoff.phase=RL_STAIR_IDLE;
  rl_control_state.actor=RL_ACTOR_STAIR;
  assert(!ChassisCanDetectStair(1));
  rl_control_state.actor=RL_ACTOR_MOTION;
  rl_control_state.active=0;
  assert(!ChassisCanDetectStair(1));
  rl_control_state.active=1;
  assert(ChassisSetRLHeight(.44f)<0);
  assert(CurrentRLSnapshot(&stair).motion.command_height_m==.36f);
  v_cmd=1.2f;w_cmd=.4f;
  chassis_cmd_recv.changeL=0;
  joints[0].measure.state=0;
  (void)CurrentRLSnapshot(&stair);
  assert(!stair.feedback_fresh);
  assert(!memcmp(b.motion.previous_target,prev,sizeof(prev)));
  stair_handoff.phase=RL_STAIR_ACTIVE;
  b=CurrentRLSnapshot(&stair);
  assert(b.motion.command_height_m==.25f && b.motion.command_vx_m_s==1.2f);
  stair_handoff.phase=RL_STAIR_IDLE;
  assert(!ChassisSetRLHeight(.20f));b=CurrentRLSnapshot(NULL);
  assert(b.motion.command_height_m==.20f);
  assert(!memcmp(b.motion.previous_target,prev,sizeof(prev)));
  assert(ChassisSetRLHeight(.19f)<0);
  assert(ChassisSetRLHeight(.41f)<0);assert(ChassisSetRLHeight(NAN)<0);
  continuous_stairs();
  return 0;
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'actual_snapshot.c'
            path.write_text(c)
            subprocess.run([os.getenv('CC','cc'),'-std=gnu11','-O2','-Wall','-Wextra','-Werror',
                '-I'+str(ROOT/'application/chassis'),'-I'+str(ROOT/'application/balance'),str(path),
                str(ROOT/'application/chassis/rl_policy_core.c'),str(ROOT/'application/chassis/rl_stair.c'),str(ROOT/'application/chassis/rl_motion31.c'),
                str(ROOT/'application/chassis/rl_control.c'),str(ROOT/'application/chassis/stair_actor.c'),'-lm','-o',str(Path(tmp)/'snapshot')],check=True)
            subprocess.run([str(Path(tmp)/'snapshot')],check=True)


if __name__=='__main__':
    unittest.main()
