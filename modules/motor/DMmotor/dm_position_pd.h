#ifndef DM_POSITION_PD_H
#define DM_POSITION_PD_H

#include <math.h>

#define DM_POSITION_TWO_PI 6.2831853071795864769f

/* Joint angles are periodic. Compute PD here instead of relying on the
   actuator's raw multi-turn position subtraction. All axes are encoder axes. */
static inline float DMPositionPDTorque(float target, float target_velocity,
                                      float position, float velocity,
                                      float kp, float kd, float feedforward,
                                      float torque_limit, float *error)
{
    *error = 0.0f;
    if (!isfinite(target) || !isfinite(target_velocity) ||
        !isfinite(position) || !isfinite(velocity) || !isfinite(kp) ||
        !isfinite(kd) || !isfinite(feedforward) ||
        !isfinite(torque_limit) || torque_limit <= 0.0f) return 0.0f;
    *error = remainderf(target - position, DM_POSITION_TWO_PI);
    const float torque = feedforward + kp * *error +
                         kd * (target_velocity - velocity);
    if (!isfinite(torque)) return 0.0f;
    return fminf(fmaxf(torque, -torque_limit), torque_limit);
}

#endif
