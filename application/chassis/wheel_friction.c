#include "wheel_friction.h"

#include <math.h>
#include <stddef.h>


typedef struct {
    float viscous_nm_per_rad_s;
    float coulomb_nm;
} WheelFrictionParams;

/* tools/wheel_motor_identification/wheel_motor_identification_parameters.csv */
static const WheelFrictionParams params[2] = {
    {0.00254846587463745f, 0.0546993266811951f}, /* left */
    {0.000669568028716762f, 0.0528429204730597f} /* right */
};

static float clamp_current(float current)
{
    return fminf(fmaxf(current, -WHEEL_CURRENT_LIMIT), WHEEL_CURRENT_LIMIT);
}

int WheelFrictionCompensate(unsigned wheel, float forward_speed_rad_s,
                            float base_current, float *friction_torque_nm,
                            float *compensated_current)
{
    if (wheel > WHEEL_FRICTION_RIGHT || friction_torque_nm == NULL ||
        compensated_current == NULL || !isfinite(forward_speed_rad_s) ||
        !isfinite(base_current)) {
        return -1;
    }

    /* Left motor current is negative for forward wheel torque; right is positive. */
    const float motor_sign = (wheel == WHEEL_FRICTION_LEFT) ? -1.0f : 1.0f;
    const float requested_torque_nm = motor_sign * base_current / WHEEL_CURRENT_PER_NM;
    const float direction_source =
        (fabsf(forward_speed_rad_s) >= WHEEL_FRICTION_SWITCH_SPEED_RAD_S)
            ? forward_speed_rad_s : requested_torque_nm;
    const float direction = (direction_source > 0.0f) ? 1.0f :
                            (direction_source < 0.0f) ? -1.0f : 0.0f;
    const float compensation = params[wheel].viscous_nm_per_rad_s * forward_speed_rad_s +
                               params[wheel].coulomb_nm * direction;
    *friction_torque_nm = compensation;
    *compensated_current = clamp_current(base_current +
                                         motor_sign * compensation * WHEEL_CURRENT_PER_NM);
    return 0;
}

/* Exact physical-left/right values from the training fit_compensated profile.
 * Active compensation only; mechanical B/C are physically present, not applied
 * a second time in firmware. Total residual D is approximately .052. */
int WheelRLCompensate(unsigned wheel, float speed, float drive_current,
                      float *correction_nm, float *motor_current)
{
    static const float dfb[2] = {0.0494515f, 0.05133043f};
    static const float cff[2] = {0.053699f, 0.051843f};
    if (correction_nm) *correction_nm = 0.0f;
    if (motor_current) *motor_current = 0.0f;
    if (wheel > WHEEL_FRICTION_RIGHT || !correction_nm || !motor_current ||
        !isfinite(speed) || !isfinite(drive_current)) return -1;
    const float sign = wheel == WHEEL_FRICTION_LEFT ? -1.0f : 1.0f;
    const float drive = sign * drive_current / WHEEL_CURRENT_PER_NM;
    *correction_nm = -dfb[wheel] * speed + cff[wheel] * tanhf(speed / 0.1f);
    *motor_current = clamp_current(sign * (drive + *correction_nm) * WHEEL_CURRENT_PER_NM);
    return 0;
}
