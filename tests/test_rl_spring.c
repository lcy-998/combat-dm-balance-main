#include "../application/chassis/rl_control.h"

#include <assert.h>
#include <math.h>

int main(void)
{
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
    return 0;
}
