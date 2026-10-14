#ifndef WHEEL_FRICTION_H
#define WHEEL_FRICTION_H

#define WHEEL_FRICTION_SWITCH_SPEED_RAD_S 1.0f

enum {
    WHEEL_FRICTION_LEFT = 0,
    WHEEL_FRICTION_RIGHT = 1
};

/* Wheel speed and torque use the physical forward-positive convention. */
int WheelFrictionCompensate(unsigned wheel, float forward_speed_rad_s,
                            float base_current, float *friction_torque_nm,
                            float *compensated_current);

#endif
