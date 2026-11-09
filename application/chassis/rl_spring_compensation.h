#ifndef RL_SPRING_COMPENSATION_H
#define RL_SPRING_COMPENSATION_H
#include <math.h>
/* Experimental: old LQR compensation is not an identified passive spring.
 * Keep disabled until four-joint first100ms max error <=0.5deg is verified.
 * Bound to D5 controller_contract gas_spring_torque_nm=4.31, not final10.775. */
#ifndef RL_SPRING_DIFFERENCE_ENABLED
#define RL_SPRING_DIFFERENCE_ENABLED 0
#endif
#define RL_TRAINED_SPRING_NM 4.31f
/* Input: the existing LQR scalar spring compensation before sin(front-back).
 * Output: additive MOTOR ENCODER torque in physical LF,LB,RF,RB order.
 * delta = trained passive torque - hypothesized real passive torque.
 * Add BEFORE host PD's total +/-40Nm clip. Do not add to last_action. */
static inline int RLSpringDifferenceTorque(float left_scalar, float right_scalar,
                                           float left_angle, float right_angle,
                                           float trained_nm, float out[4])
{
    for (unsigned i=0;i<4;++i) out[i]=0.0f;
    if (!isfinite(left_scalar)||!isfinite(right_scalar)||!isfinite(left_angle)||
        !isfinite(right_angle)||!isfinite(trained_nm)||left_scalar<0||
        right_scalar<0||left_scalar>27||right_scalar>27||trained_nm<0||trained_nm>10.775f)
        return -1;
    const float sl=left_scalar*sinf(left_angle), sr=right_scalar*sinf(right_angle);
    out[0]=trained_nm-sl; out[1]=sl;
    out[2]=sr-trained_nm; out[3]=-sr;
    return 0;
}
#endif
