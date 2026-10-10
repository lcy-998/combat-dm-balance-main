#ifndef __LQR_H__
#define __LQR_H__

#ifdef __cplusplus
extern "C" {
#endif

void set_lqr_data(float s, float s_dot, float phi, float phi_dot,
                  float theta_ll, float theta_ll_dot, float theta_lr,
                  float theta_lr_dot, float theta_b, float theta_b_dot,
                  float l_l, float l_r, float F_N);
void set_lqr_target(float s, float s_dot, float phi, float phi_dot, float theta_ll, float theta_ll_dot,
                    float theta_lr, float theta_lr_dot, float theta_b, float theta_b_dot);

float get_lw_torque();

float get_rw_torque();

float get_lb_torque();

float get_rb_torque();

void calc_lqr();

#ifdef __cplusplus
}
#endif

#endif /* __LQR_H__ */
