#ifndef WHEEL_FRICTION_H
#define WHEEL_FRICTION_H

#define WHEEL_FRICTION_SWITCH_SPEED_RAD_S 1.0f
#define WHEEL_CURRENT_PER_NM (19.0f / (0.00036f * (268.0f / 17.0f)))
#define WHEEL_CURRENT_LIMIT 16384.0f
#define WHEEL_TORQUE_LIMIT_NM (WHEEL_CURRENT_LIMIT / WHEEL_CURRENT_PER_NM)

/* RL only: input motor-coordinate current is NOT pre-clipped.
 * physical-forward speed uses the same filtered encoder as policy observation. */
int WheelRLCompensate(unsigned wheel, float forward_speed_rad_s,
                      float drive_current, float *correction_nm, float *motor_current);

enum {
    WHEEL_FRICTION_LEFT = 0,
    WHEEL_FRICTION_RIGHT = 1
};

/* Wheel speed and torque use the physical forward-positive convention. */
int WheelFrictionCompensate(unsigned wheel, float forward_speed_rad_s,
                            float base_current, float *friction_torque_nm,
                            float *compensated_current);

#endif
