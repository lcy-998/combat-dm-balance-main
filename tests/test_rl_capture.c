#include "rl_capture.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(int argc,char **argv)
{
    RLCaptureArm(3);
    RLShadowSnapshot s={0};RLControlFrame frame={0};RLShadowCommand c={0};
    s.motion.joint_position_rad[0]=.2f;
    for(unsigned i=0;i<RL_CAPTURE_CAPACITY+10u;++i){
        frame.tick_ms=i*5u;RLCapturePush(&s,NULL,&frame);
    }
    assert(g_rl_capture.count==RL_CAPTURE_CAPACITY && !g_rl_capture.triggered);
    frame.owner=RL_OWNER_RL;frame.tick_ms+=5;
    frame.tau_pd_nm[0]=8;frame.final_motor_nm[0]=2;
    c.actor_input_tick_ms=17;c.recovery_mode=1;c.motion_valid=1;
    c.valid=1;c.snapshot_sequence=99;c.previous_source=RL_PREVIOUS_INIT_PROXY;
    c.preview.observation[24]=.68f;
    RLCapturePush(&s,&c,&frame);
    assert(g_rl_capture.triggered && !g_rl_capture.frozen && g_rl_capture.remaining==3);
    for(unsigned i=0;i<3;++i){frame.tick_ms+=5;RLCapturePush(&s,&c,&frame);}
    assert(g_rl_capture.frozen);
    const unsigned count=g_rl_capture.records_written;
    RLCapturePush(&s,&c,&frame);assert(g_rl_capture.records_written==count);
    const unsigned last=(g_rl_capture.write_index+RL_CAPTURE_CAPACITY-1u)%RL_CAPTURE_CAPACITY;
    assert(g_rl_capture.records[last].frame_begin==g_rl_capture.records[last].frame_end);
    assert(!(g_rl_capture.records[last].frame_begin&1));
    assert(g_rl_capture.schema_version==3);
    assert(g_rl_capture.records[last].actor_input_tick_ms==17);
    assert(g_rl_capture.records[last].recovery_mode==1 && g_rl_capture.records[last].motion_candidate_valid==1);
    assert(g_rl_capture.records[last].policy_sequence==99);
    assert(g_rl_capture.records[last].valid_mask==7);
    assert(g_rl_capture.records[last].pd_q[0]==.2f && g_rl_capture.records[last].final_motor_command_nm[0]==2);
    if(argc>1){FILE *f=fopen(argv[1],"wb");assert(f);
        assert(fwrite((const void *)&g_rl_capture,sizeof(g_rl_capture),1,f)==1);assert(!fclose(f));}
    printf("capture record_bytes=%zu buffer_bytes=%zu; wrap, trigger, freeze, guards passed\n",
           sizeof(RLCaptureRecord),sizeof(RLCaptureBuffer));
    RLCaptureArm(RL_CAPTURE_CAPACITY-17u);
    memset(&s,0,sizeof(s));memset(&frame,0,sizeof(frame));
    s.recovery_session=1;frame.owner=RL_OWNER_RL;
    for(unsigned t=0;t<=1100;t++){
        frame.tick_ms=1000+t;s.handoff_committed=t>=500;
        RLCapturePush(&s,NULL,&frame);
        if(t==400)assert(!g_rl_capture.triggered && !g_rl_capture.frozen);
        if(t==500)assert(g_rl_capture.triggered && !g_rl_capture.frozen);
    }
    assert(g_rl_capture.frozen && g_rl_capture.records_written>200);
    const unsigned tail=(g_rl_capture.write_index+RL_CAPTURE_CAPACITY-1u)%RL_CAPTURE_CAPACITY;
    assert(g_rl_capture.records[tail].tick_ms==2055);
    assert(g_rl_capture.records[tail].handoff_committed==1);
    puts("PASS Recovery capture:200Hz, handoff trigger and555ms post-switch retained");
    return 0;
}
