/**
 * (C)Copyright Combat Algorithm Team.All rights reserved.
 * @author: kuroneko(kuroneko@njust.edu.cn)
 * @created: 2026.1.18
 * @description: Virtual Model Controller (VMC) of SJTU model.
 **/

namespace Leg
{
    class VmcController
    {
        // Phi1 is alias of Forward Angel
        // Phi4 is alias of Backward Angel
        struct ConnectingRodState
        {
            float l1, l2, l3, l4, l5;                               // Lengths of connecting rods
            float leftPhi1, leftPhi1Dot, leftPhi4, leftPhi4Dot;     // Left leg connecting rod angles and their derivatives
            float rightPhi1, rightPhi1Dot, rightPhi4, rightPhi4Dot; // Right leg connecting rod angles and their derivatives
        };

        struct ForwardSolveResult
        {
            float lLeft, lLeftDot, lLeftDot2;                // Vitrual left length, its first and second derivative
            float lRight, lRightDot, lRightDot2;             // Vitrual right length, its first and second derivative
            float thetaLeft, thetaLeftDot, thetaLeftDot2;    // Vitrual left angle, its first and second derivative
            float thetaRight, thetaRightDot, thetaRightDot2; // Vitrual right angle, its first and second derivative
            float leftForwardTorque, rightForwardTorque;     // Output forward torques
            float leftBackwardTorque, rightBackwardTorque;   // Output backward torques
        };

        struct VmcTargetEffects
        {
            float leftForce, leftTorque;   // Target effects on left leg
            float rightForce, rightTorque; // Target effects on right leg
        };

    public:
        VmcController() = default;
        ~VmcController() = default;

        // Setters for target effects and connecting rod states
        void Set_TargetEffects(const VmcTargetEffects &effects) { this->targetEffects = effects; };
        void Set_ConnectingRodState(const ConnectingRodState &state) { this->connectingRodState = state; };

        // VMC calculation method
        void Forward_Solve();
        void Difference_Solve();
        void VMC_Calculate_Torque();
        void Update_State();

        // Getter for result body torques
        float Get_LeftForwardTorque() const { return this->forwardSolveResult.leftForwardTorque; };
        float Get_RightForwardTorque() const { return this->forwardSolveResult.rightForwardTorque; };
        float Get_LeftBackwardTorque() const { return this->forwardSolveResult.leftBackwardTorque; };
        float Get_RightBackwardTorque() const { return this->forwardSolveResult.rightBackwardTorque; };

    private:
        ConnectingRodState connectingRodState;
        VmcTargetEffects targetEffects;
        ForwardSolveResult forwardSolveResult;
    };
}