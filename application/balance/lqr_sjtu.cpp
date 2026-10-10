/**
 * (C)Copyright Combat Algorithm Team.All rights reserved.
 * @author: kuroneko(kuroneko@njust.edu.cn)
 * @created: 2026.1.18
 * @description: Linear Quadratic Regulator (LQR) controller of SJTU model.
 **/

#include "lqr_sjtu.hpp"

using namespace Leg;

LqrController::LqrController()
{
    // Fill in the LQR fitCoeff with predefined values
    fitCoeff = {0.0f}; // Placeholder values
}

void LqrController::LQR_Calculate()
{
    float l_l = this->state.lengthLeft;
    float l_r = this->state.lengthRight;
    float l_l_square = l_l * l_l;
    float l_r_square = l_r * l_r;
    float l_l_cube = l_l_square * l_l;
    float l_r_cube = l_r_square * l_r;

    float poly[POLY_TERMS] = {
        l_r_cube,
        l_l_cube,
        l_r_square * l_l,
        l_l_square * l_r,
        l_r_square,
        l_l_square,
        l_l * l_r,
        l_r,
        l_l,
        1.0f};

    for (int i = 0; i < INPUT_DIM; ++i)
    {
        for (int j = 0; j < STATE_DIM; ++j)
        {
            float acc = 0.0f;
            int base = (i * STATE_DIM + j) * POLY_TERMS;
            for (int k = 0; k < POLY_TERMS; ++k)
                acc += fitCoeff[base + k] * poly[k];

            K[i][j] = acc;
        }
    }

    std::array<float, 4> T_;
    float x_err[10] = {
        state.s - target.sTarget,
        state.sDot - target.sDotTarget,
        state.phi - target.phiTarget,
        state.phiDot - target.phiDotTarget,
        state.thetaLeft - target.thetaLeftTarget,
        state.thetaLeftDot - target.thetaLeftDotTarget,
        state.thetaRight - target.thetaRightTarget,
        state.thetaRightDot - target.thetaRightDotTarget,
        state.thetaBody - target.thetaBodyTarget,
        state.thetaBodyDot - target.thetaBodyDotTarget};

    for (int i = 0; i < 4; ++i)
    {
        float u = 0.0f;
        for (int j = 0; j < 10; ++j)
            u += K[i][j] * x_err[j];

        T_[i] = -u;
    }

    this->torqueOutputs.leftWheelTorque = T_[0];
    this->torqueOutputs.rightWheelTorque = T_[1];
    this->torqueOutputs.leftBodyTorque = T_[2];
    this->torqueOutputs.rightBodyTorque = T_[3];
}