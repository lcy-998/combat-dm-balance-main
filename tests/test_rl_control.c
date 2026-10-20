#include "rl_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static RLShadowCommand fresh(uint32_t tick, int recovery)
{
    RLShadowCommand command = {0};
    command.valid = 1u;
    command.recovery_mode = recovery;
    command.snapshot_sequence = tick + 1u;
    command.snapshot_tick_ms = tick;
    command.output_tick_ms = tick;
    return command;
}

int main(void)
{
    RLControlState state = {0};
    RLShadowCommand command = fresh(100u, 1);
    assert(!RLControlStep(&state, 100u, 1, &command).active);
    command = fresh(101u, 1);
    assert(RLControlStep(&state, 101u, 1, &command).active);
    /* Recovery stays active beyond the former stand-up handover interval. */
    command = fresh(901u, 1);
    RLControlDecision d = RLControlStep(&state, 901u, 1, &command);
    assert(d.recovering && d.active);
    command = fresh(903u, 0);
    assert(!RLControlStep(&state, 903u, 1, &command).active);
    command = fresh(904u, 1);
    assert(RLControlStep(&state, 904u, 1, &command).active);
    assert(!RLControlStep(&state, 929u, 1, &command).active);
    command = fresh(930u, 1);
    assert(RLControlStep(&state, 930u, 1, &command).active);
    command.leg_torque_motor_nm[0] = NAN;
    assert(!RLControlStep(&state, 931u, 1, &command).active);
    command = fresh(932u, 1);
    assert(RLControlStep(&state, 932u, 1, &command).active);
    d = RLControlStep(&state, 932u, 0, &command);
    assert(!d.active && d.exited && !state.enabled);
    /* Re-enable starts recovery and rejects outputs preceding enable. */
    command = fresh(931u, 1);
    assert(!RLControlStep(&state, 933u, 1, &command).active);
    command = fresh(934u, 0);
    assert(!RLControlStep(&state, 934u, 1, &command).active);
    command = fresh(935u, 1);
    assert(RLControlStep(&state, 935u, 1, &command).active);
    d = RLControlStep(&state, 935u, 0, &command);
    assert(!d.active && d.exited);
    /* Tick wrap still rejects old commands and accepts fresh commands. */
    assert(!RLControlStep(&state, UINT32_MAX - 2u, 1, NULL).active);
    command = fresh(1u, 1);
    assert(RLControlStep(&state, 1u, 1, &command).active);
    /* Compensation opposes the spring with opposite signs on each motor pair. */
    const float policy[4] = {1.0f, 2.0f, 3.0f, 4.0f};
    const float lengths[2] = {0.25f, 0.40f};
    float spread[2] = {1.57079632679f, -1.57079632679f};
    float torque[4];
    assert(RLControlCompensateSpring(policy, lengths, spread, torque) == 0);
    assert(fabsf(torque[0] + 15.5f) < 0.001f);
    assert(fabsf(torque[1] - 18.5f) < 0.001f);
    assert(fabsf(torque[2] + 13.5f) < 0.001f);
    assert(fabsf(torque[3] - 20.5f) < 0.001f);
    const float saturated[4] = {-40.0f, 40.0f, -40.0f, 40.0f};
    assert(RLControlCompensateSpring(saturated, lengths, spread, torque) == 0);
    for (unsigned i = 0; i < 4u; ++i) assert(torque[i] == saturated[i]);
    spread[0] = spread[1] = 0.0f;
    assert(RLControlCompensateSpring(policy, lengths, spread, torque) == 0);
    for (unsigned i = 0; i < 4u; ++i) assert(torque[i] == policy[i]);
    spread[0] = NAN;
    assert(RLControlCompensateSpring(policy, lengths, spread, torque) != 0);
    puts("Recovery RL, spring compensation, timeout and emergency-stop tests passed");
    return 0;
}
