#ifndef __LQR_HPP_
#define __LQR_HPP_

#include <array>

#ifdef __cplusplus
extern "C" {
#endif

// C接口，供C代码调用
void set_lqr_data(float s, float s_dot, float phi, float phi_dot, float theta_ll, 
                  float theta_ll_dot, float theta_lr, float theta_lr_dot, 
                  float theta_b, float theta_b_dot, float l_l, float l_r, float F_N);
void set_lqr_target(float s,float s_dot, float phi, float phi_dot, float theta_ll, float theta_ll_dot,
                    float theta_lr, float theta_lr_dot, float theta_b, float theta_b_dot);
float get_lw_torque();
float get_rw_torque();
float get_lb_torque();
float get_rb_torque();
void calc_lqr();

#ifdef __cplusplus
}
#endif

/* C++类定义 */
class Lqr
{
public:
    // 构造函数，初始化成员变量
    Lqr() 
        : s(0), s_dot(0), phi(0), phi_dot(0), theta_ll(0), theta_ll_dot(0), 
          theta_lr(0), theta_lr_dot(0), theta_b(0), theta_b_dot(0),
          l_l(0), l_r(0), target_s_dot(0), target_s(0), target_phi(0), 
          target_phi_dot(0), target_theta_ll(0), target_theta_ll_dot(0),
          target_theta_lr(0), target_theta_lr_dot(0), target_theta_b(0),
          target_theta_b_dot(0), F_N_(0) 
    {
        K_ = {{{0}}};  // 初始化 K_ 数组
        T_ = {0};      // 初始化 T_ 数组
    }

    void Calc();  // 执行 LQR 计算
    void SetData(const float _s, const float _s_dot,
                 const float _phi, const float _phi_dot,
                 const float _theta_ll, const float _theta_ll_dot,
                 const float _theta_lr, const float _theta_lr_dot,
                 const float _theta_b, const float _theta_b_dot,
                 const float _l_l, const float _l_r, const float _F_N)
    {
        s = _s;
        s_dot = _s_dot;
        phi = _phi;
        phi_dot = _phi_dot;
        theta_ll = _theta_ll;
        theta_ll_dot = _theta_ll_dot;
        theta_lr = _theta_lr;
        theta_lr_dot = _theta_lr_dot;
        theta_b = _theta_b;
        theta_b_dot = _theta_b_dot;
        l_l = _l_l;
        l_r = _l_r;
        F_N_ = _F_N;
    }

    // 其他方法
    void SetSpeed(const float _s_dot) { target_s_dot = _s_dot; };
    void SetNowDist(const float _s) { target_s = _s; }
    void SetPhi(const float _phi) { target_phi = _phi; }
    void SetPhiDot(const float _phi_dot) { target_phi_dot = _phi_dot; }
    void SetThetaLL(const float _theta_ll) { target_theta_ll = _theta_ll; }
    void SetThetaLLDot(const float _theta_ll_dot) { target_theta_ll_dot = _theta_ll_dot; }
    void SetThetaLR(const float _theta_lr) { target_theta_lr = _theta_lr; }
    void SetThetaLRDot(const float _theta_lr_dot) { target_theta_lr_dot = _theta_lr_dot; }
    void SetThetaB(const float _theta_b) { target_theta_b = _theta_b; }
    void SetThetaBDot(const float _theta_b_dot) { target_theta_b_dot = _theta_b_dot; }

    float GetLeftWheelTor() { return T_[0]; };
    float GetRightWheelTor() { return T_[1]; };
    float GetLeftLegTor() { return T_[2]; };
    float GetRightLegTor() { return T_[3]; };

private:
    float s, s_dot, phi, phi_dot, theta_ll, theta_ll_dot, theta_lr, theta_lr_dot, theta_b, theta_b_dot, l_l, l_r;
    float target_s_dot, target_s, target_phi, target_phi_dot,
          target_theta_ll, target_theta_ll_dot,
          target_theta_lr, target_theta_lr_dot,
          target_theta_b, target_theta_b_dot, F_N_;
    
    std::array<std::array<float, 10>, 4> K_;  // 状态反馈矩阵
    std::array<float, 4> T_;  // 扭矩： [左轮, 右轮, 左腿, 右腿]
};

#endif /* __LQR_HPP_ */
