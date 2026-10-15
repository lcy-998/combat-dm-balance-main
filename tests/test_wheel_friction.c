#include "wheel_friction.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.0001f);
}

int main(void)
{
    float torque, current;

    assert(WheelFrictionCompensate(WHEEL_FRICTION_LEFT, 0.0f, -1000.0f,
                                    &torque, &current) == 0);
    near(torque, 0.0546993f);
    assert(current < -1000.0f);

    assert(WheelFrictionCompensate(WHEEL_FRICTION_LEFT, 0.0f, 1000.0f,
                                    &torque, &current) == 0);
    near(torque, -0.0546993f);
    assert(current > 1000.0f);

    assert(WheelFrictionCompensate(WHEEL_FRICTION_LEFT, 0.5f, 1000.0f,
                                    &torque, &current) == 0);
    near(torque, 0.0025484659f * 0.5f - 0.0546993f);
    assert(WheelFrictionCompensate(WHEEL_FRICTION_LEFT, 1.0f, 1000.0f,
                                    &torque, &current) == 0);
    near(torque, 0.0025484659f + 0.0546993f);

    assert(WheelFrictionCompensate(WHEEL_FRICTION_RIGHT, 10.0f, -1000.0f,
                                    &torque, &current) == 0);
    near(torque, 0.000669568f * 10.0f + 0.0528429f);
    assert(current > -1000.0f);

    assert(WheelFrictionCompensate(WHEEL_FRICTION_RIGHT, -3.0f, 1000.0f,
                                    &torque, &current) == 0);
    near(torque, -0.000669568f * 3.0f - 0.0528429f);
    assert(current < 1000.0f);

    assert(WheelFrictionCompensate(WHEEL_FRICTION_RIGHT, 0.0f, 0.0f,
                                    &torque, &current) == 0);
    near(torque, 0.0f);
    near(current, 0.0f);

    assert(WheelFrictionCompensate(WHEEL_FRICTION_LEFT, 50.0f, -16384.0f,
                                    &torque, &current) == 0);
    near(current, -16384.0f);
    assert(WheelFrictionCompensate(WHEEL_FRICTION_LEFT, NAN, 1000.0f,
                                    &torque, &current) != 0);
    puts("wheel friction tests passed");
    return 0;
}
