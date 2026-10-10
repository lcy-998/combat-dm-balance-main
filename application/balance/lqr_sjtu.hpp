/**
 * (C)Copyright Combat Algorithm Team.All rights reserved.
 * @author: kuroneko(kuroneko@njust.edu.cn)
 * @created: 2026.1.18
 * @description: Linear Quadratic Regulator (LQR) controller of SJTU model.
 **/

#ifndef LQR_SJTU_HPP
#define LQR_SJTU_HPP

#include <array>
#include <vector>

#include "leg_const.hpp"

namespace Leg
{
    // Name from SJTU leg model paper
    struct StateVariables
    {
        float s, sDot;
        float phi, phiDot;
        float thetaLeft, thetaLeftDot;
        float thetaRight, thetaRightDot;
        float thetaBody, thetaBodyDot;
        float lengthLeft, lengthRight;
    };

    struct TargetVariables
    {
        float sTarget, sDotTarget;
        float phiTarget, phiDotTarget;
        float thetaLeftTarget, thetaLeftDotTarget;
        float thetaRightTarget, thetaRightDotTarget;
        float thetaBodyTarget, thetaBodyDotTarget;
    };

    struct TorqueOutputs
    {
        float leftWheelTorque;
        float rightWheelTorque;
        float leftBodyTorque;
        float rightBodyTorque;
    };

    class LqrController
    {
    public:
        LqrController();
        ~LqrController() = default;

        // Setters for state and target variables
        void Set_State(const StateVariables &state) { this->state = state; };
        void Set_Target(const TargetVariables &target) { this->target = target; };

        // Getters for state variables
        float Get_S() const { return this->state.s; };
        float Get_SDot() const { return this->state.sDot; };
        float Get_Phi() const { return this->state.phi; };
        float Get_PhiDot() const { return this->state.phiDot; };
        float Get_ThetaLeft() const { return this->state.thetaLeft; };
        float Get_ThetaLeftDot() const { return this->state.thetaLeftDot; };
        float Get_ThetaRight() const { return this->state.thetaRight; };
        float Get_ThetaRightDot() const { return this->state.thetaRightDot; };
        float Get_ThetaBody() const { return this->state.thetaBody; };
        float Get_ThetaBodyDot() const { return this->state.thetaBodyDot; };
        float Get_LengthLeft() const { return this->state.lengthLeft; };
        float Get_LengthRight() const { return this->state.lengthRight; };

        // LQR calculation
        void LQR_Calculate();

        // Get control output torques
        float Get_LeftWheelTorque() const { return this->torqueOutputs.leftWheelTorque; };
        float Get_RightWheelTorque() const { return this->torqueOutputs.rightWheelTorque; };
        float Get_LeftBodyTorque() const { return this->torqueOutputs.leftBodyTorque; };
        float Get_RightBodyTorque() const { return this->torqueOutputs.rightBodyTorque; };
        static constexpr int STATE_DIM = 10;
        static constexpr int INPUT_DIM = 4;
        static constexpr int POLY_TERMS = 10;

    private:
        // state variables
        StateVariables state;

        // target variables
        TargetVariables target;

        // LQR calculation
        std::array<float, 240> fitCoeff;
        std::array<std::array<float, 10>, 4> K; // LQR gain matrix

        // control output torques
        TorqueOutputs torqueOutputs;
    };
}

#endif // LQR_SJTU_HPP