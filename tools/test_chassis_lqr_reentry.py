"""Exercise actual chassis target/standup/output boundaries with real LQR and RL.

Hardware IO and clocks are stubbed; no firmware flashing or physical dynamics.
"""
import os
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def block_after(source, marker):
    start = source.index("{", source.index(marker))
    end, depth = start + 1, 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def harness(source):
    declarations = "\n".join(
        line for line in source.splitlines()
        if re.match(r"static uint8_t lqr_reentry_pending\b", line)
    )
    # Extract the real selection/commit transaction, including the fallback
    # wheel conversion and output limit. Do not duplicate its decisions here.
    start = source.index("float selected_leg[4]")
    end = source.index("taskEXIT_CRITICAL();", start) + len("taskEXIT_CRITICAL();")
    return r'''
#include "rl_control.h"
#include "LQR.h"
#include <assert.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define V_CMD_MAX_ACCEL 100.0f
typedef struct {
    struct {float total_angle;} measure;
    struct {unsigned host_position_pd;float kp_set,kd_set,pos_set,vel_set,tor_set;} ctrl;
} DMMotorInstance;
static DMMotorInstance joints[4], wheels[2];
static DMMotorInstance *jiont_motorlf=&joints[0],*jiont_motorlb=&joints[1];
static DMMotorInstance *jiont_motorrf=&joints[2],*jiont_motorrb=&joints[3];
static DMMotorInstance *motor_l=&wheels[0],*motor_r=&wheels[1];
static struct {float L0;} left_result={.16f},right_result={.16f};
static struct {__typeof__(left_result) *result;} left_leg={&left_result},right_leg={&right_result};
static __typeof__(left_leg) *Lleg=&left_leg,*Rleg=&right_leg;
static struct {float vx;} chassis_cmd_recv;
static RLControlState rl_control_state;
static RLControlFrame rl_control_frame;
static RLShadowSnapshot rl_pd_snapshot;
static RLShadowCommand queued_command,rl_consumed_command;
static int queued,normal=1,rl_command_consumed,chassis_lqr_output_valid;
static unsigned now=100,balance_dwt_cnt;
static float s,s_dot,phi,phi_dot,theta_l=.04f,theta_l_dot,theta_r=.04f,theta_r_dot,theta_b,theta_b_dot;
static float s_tar,phi_tar,v_cmd,w_cmd,v_cmd_last,chassis_wz_cmd,del_t=.005f;
static float theta_bal,l_xx=.16f,thetal=.04f,thetar=.04f;
static float lwt,rwt,lwi,rwi,lfjt,lbjt,rfjt,rbjt,Rwheel,Lwheel;
static int done=1,step_3,upstair_flag0,upstair_flag0_cnt,upstair_flag0_latch;
static float DWT_GetDeltaT(unsigned *counter){(void)counter;return .005f;}
static unsigned xTaskGetTickCount(void){return now;}
static int ChassisNormalLQRState(void){return normal;}
static void Unsetlegkd(void){}
static void DMMotorSetTorque(DMMotorInstance *motor,float value){motor->ctrl.tor_set=value;}
int RLShadowReadCommand(RLShadowCommand *out){*out=queued_command;return queued?0:-1;}
''' + declarations + '\nvoid setTargetData(void)\n' + block_after(source, 'void setTargetData()') + r'''
static void finish_standup(void)
''' + block_after(source, 'if (theta_l < 0.50f && theta_r < 0.50f && fabs(theta_b) < 0.01)') + r'''
static void select_output(void) {
    /* Same torque getters used by legLengthCalc before outputTorque. */
    lwt=get_lw_torque();rwt=get_rw_torque();
''' + source[start:end] + r'''
}
static void feedback(void) {
    memset(&rl_pd_snapshot,0,sizeof(rl_pd_snapshot));
    rl_pd_snapshot.tick_ms=now;rl_pd_snapshot.sequence=now+1;
    rl_pd_snapshot.feedback_valid_mask=0x3f;
    rl_pd_snapshot.imu_update_sequence=1;rl_pd_snapshot.imu_update_tick_ms=now;
    for(unsigned i=0;i<6;++i)rl_pd_snapshot.feedback_receive_tick_ms[i]=now;
    rl_pd_snapshot.motion.gravity_model[1]=-1;
    rl_pd_snapshot.motion.command_height_m=.34f;
    rl_pd_snapshot.leg_length_m[0]=rl_pd_snapshot.leg_length_m[1]=.16f;
    RLControlFillPrevious(&rl_control_state,&rl_pd_snapshot);
}
static void adopt_rl(void) {
    RLControlReset(&rl_control_state);
    rl_control_state.ramp_completed=rl_control_state.timing=rl_control_state.ready=1;
    feedback();
    memset(&queued_command,0,sizeof(queued_command));
    queued_command.valid=1;queued_command.snapshot_sequence=now+1;
    queued_command.snapshot_tick_ms=queued_command.output_tick_ms=now;
    queued_command.control_epoch=rl_control_state.epoch;
    queued_command.previous_source=RL_PREVIOUS_INIT_PROXY;
    assert(!RLMotion31BuildPreview(&rl_pd_snapshot.motion,queued_command.action,&queued_command.preview));
    queued=1;setTargetData();select_output();queued=0;
    assert(rl_control_state.active && rl_control_frame.owner==RL_OWNER_RL);
}
static void assert_no_position_kick(void) {
    assert(fabsf(s_tar-s)<1e-6f);
    assert(fabsf(get_lw_torque())<1e-6f && fabsf(get_rw_torque())<1e-6f);
    assert(fabsf(lwi)<1e-3f && fabsf(rwi)<1e-3f);
    assert(rl_control_frame.owner==RL_OWNER_LQR);
}
int main(void) {
    /* Repeated recovery, both encoder directions: the cached old s must be
       rebased in the same transaction as both wheel origins and the target. */
    for(int sign=-1;sign<=1;sign+=2) {
        s=sign*3.0f;s_tar=sign*7.0f;
        motor_l->measure.total_angle=sign*10000;
        motor_r->measure.total_angle=-sign*8000;
        finish_standup();
        assert(step_3 && s==0 && s_tar==0);
        assert(Lwheel==motor_l->measure.total_angle && Rwheel==motor_r->measure.total_angle);
        setTargetData();select_output();assert_no_position_kick();
    }
    /* A command timeout is discovered after LQR was already calculated. */
    s_tar=-4;s=2;adopt_rl();
    s=3;now+=30;feedback();setTargetData();
    assert(rl_control_state.active);select_output();
    assert(rl_control_state.fault==RL_FAULT_COMMAND);assert_no_position_kick();
    /* Once LQR has output, its normal position hold must not follow drift. */
    s=3.2f;setTargetData();select_output();
    assert(s_tar==3.0f && fabsf(lwi)>1.0f);
    /* Invalid feedback also fails during selection, not target calculation. */
    s=-2;adopt_rl();now+=5;s=-3;feedback();
    rl_pd_snapshot.motion.gravity_model[1]=0;
    setTargetData();select_output();
    assert(rl_control_state.fault==RL_FAULT_INPUT);assert_no_position_kick();
    /* Fall/cutoff clears RL state before LQR is allowed to output again. */
    s=1;adopt_rl();normal=0;
    (void)RLControlStep(&rl_control_state,now,normal,NULL);
    s=4;setTargetData(); /* inhibited frame: no output selection */
    s=5;setTargetData();
    normal=1;s=6;setTargetData();select_output();assert_no_position_kick();
    s=6.1f;setTargetData();assert(s_tar==6);
    /* Safe-mode reset immediately after first adoption must retain reentry. */
    s=2;adopt_rl();RLControlReset(&rl_control_state);
    s=-5;setTargetData();select_output();assert_no_position_kick();
    /* A moving command must not overwrite the first fallback's anchor.
       Keep the existing nonzero-command target behavior after that frame. */
    s=1;adopt_rl();now+=30;s=8;feedback();chassis_cmd_recv.vx=60;
    setTargetData();assert(s_tar==s);select_output();
    assert(rl_control_state.fault==RL_FAULT_COMMAND);
    setTargetData();assert(fabsf(s_tar-(s+v_cmd*.003f+1.2f))<1e-6f);
    puts("PASS chassis recovery, same-frame RL faults, inhibited reentry and subsequent position hold");
    return 0;
}
'''


class ChassisLqrReentry(unittest.TestCase):
    def test_actual_chassis_boundaries(self):
        source = (ROOT / 'application/chassis/chassis.c').read_text(encoding='utf-8')
        cc = os.environ.get('CC') or shutil.which('gcc') or 'cc'
        cxx = os.environ.get('CXX') or shutil.which('g++') or 'c++'
        with tempfile.TemporaryDirectory(prefix='chassis_lqr_reentry_') as tmp:
            path = Path(tmp)
            boundary = path / 'boundary.c'
            boundary.write_text(harness(source), encoding='utf-8')
            sources = [boundary, ROOT / 'application/chassis/rl_motion31.c',
                       ROOT / 'application/chassis/rl_control.c']
            objects = []
            for index, src in enumerate(sources):
                obj = path / f'{index}.o'
                subprocess.run([cc, '-std=gnu11', '-O2', '-Wall', '-Wextra', '-Werror',
                                '-I' + str(ROOT / 'application/chassis'),
                                '-I' + str(ROOT / 'application/balance'),
                                '-c', str(src), '-o', str(obj)], check=True)
                objects.append(str(obj))
            exe = path / ('boundary.exe' if os.name == 'nt' else 'boundary')
            subprocess.run([cxx, '-std=c++11', '-O2', *objects,
                            str(ROOT / 'application/balance/lqr.cpp'), '-lm', '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
