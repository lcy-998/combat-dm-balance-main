#include "dm_position_pd.h"
#include <assert.h>
#include <stdio.h>

static void near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

int main(void)
{
    float error;
    near(DMPositionPDTorque(0.1f, 0.0f, 6.2f, 0.0f,
                            60.0f, 2.0f, 0.0f, 40.0f, &error),
         60.0f * (DM_POSITION_TWO_PI - 6.1f));
    near(error, DM_POSITION_TWO_PI - 6.1f);
    near(DMPositionPDTorque(-0.1f, 0.0f, -6.2f, 0.0f,
                            60.0f, 2.0f, 0.0f, 40.0f, &error),
         -60.0f * (DM_POSITION_TWO_PI - 6.1f));
    const float before = DMPositionPDTorque(0.05f, 0.0f, 6.283f, 0.0f,
                                            60.0f, 2.0f, 0.0f, 40.0f, &error);
    const float after = DMPositionPDTorque(0.05f, 0.0f, -6.283f, 0.0f,
                                           60.0f, 2.0f, 0.0f, 40.0f, &error);
    assert(fabsf(before - after) < 0.03f);
    near(DMPositionPDTorque(1.0f, 0.0f, 1.0f, 2.0f,
                            60.0f, 2.0f, 3.0f, 40.0f, &error), -1.0f);
    near(DMPositionPDTorque(1.0f, 0.0f, 0.0f, 0.0f,
                            60.0f, 2.0f, 0.0f, 40.0f, &error), 40.0f);
    near(DMPositionPDTorque(-1.0f, 0.0f, 0.0f, 0.0f,
                            60.0f, 2.0f, 0.0f, 40.0f, &error), -40.0f);
    near(DMPositionPDTorque(NAN, 0.0f, 0.0f, 0.0f,
                            60.0f, 2.0f, 0.0f, 40.0f, &error), 0.0f);
    near(error, 0.0f);
    puts("DM shortest-angle position PD tests passed");
    return 0;
}
