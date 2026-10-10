/**
 * (C)Copyright Combat Algorithm Team.All rights reserved.
 * @author: kuroneko(kuroneko@njust.edu.cn)
 * @created: 2026.1.18
 * @description: Virtual Model Controller (VMC) of SJTU model.
 **/

#include <cmath>

#include "vmc_sjtu.hpp"
#include "leg_const.hpp"

#ifdef __arm__
#include <arm_math.h>
#define sin sin
#define cos cos
#endif

using namespace Leg;

// VMC forward kinematics solver
void VmcController::Forward_Solve()
{
    // Left leg forward kinematics
    {
        float x_B = -L_5 / 2.0 + cos(connectingRodState.leftPhi1) * L_1;
        float y_B = sin(connectingRodState.leftPhi1) * L_1;
        float x_D = L_5 / 2.0 + cos(connectingRodState.leftPhi4) * L_4;
        float y_D = sin(connectingRodState.leftPhi4) * L_4;

        float A_0 = 2.0 * L_2 * (x_D - x_B);
        float B_0 = 2.0 * L_2 * (y_D - y_B);
        float l_BD = sqrtf((x_D - x_B) * (x_D - x_B) + (y_D - y_B) * (y_D - y_B));
        float C_0 = L_2 * L_2 + l_BD * l_BD - L_3 * L_3;

        float phi2 = 2.0 * atan2f(B_0 + sqrtf(A_0 * A_0 + B_0 * B_0 - C_0 * C_0), A_0 + C_0);

        float x_C = -L_5 / 2.0 + L_1 * cos(connectingRodState.leftPhi1) + L_2 * cos(phi2);
        float y_C = 0.0 + L_1 * sin(connectingRodState.leftPhi1) + L_2 * sin(phi2);

        float phi3 = atan2f(y_C - y_D, x_C - x_D);

        forwardSolveResult.lLeft = sqrtf(x_C * x_C + y_C * y_C);
        forwardSolveResult.thetaLeft = atan2f(y_C, x_C);
    }

    // Right leg forward kinematics
    {
        float x_B = -L_5 / 2.0 + cos(connectingRodState.rightPhi1) * L_1;
        float y_B = sin(connectingRodState.rightPhi1) * L_1;
        float x_D = L_5 / 2.0 + cos(connectingRodState.rightPhi4) * L_4;
        float y_D = sin(connectingRodState.rightPhi4) * L_4;

        float A_0 = 2.0 * L_2 * (x_D - x_B);
        float B_0 = 2.0 * L_2 * (y_D - y_B);
        float l_BD = sqrtf((x_D - x_B) * (x_D - x_B) + (y_D - y_B) * (y_D - y_B));
        float C_0 = L_2 * L_2 + l_BD * l_BD - L_3 * L_3;
        float phi2 = 2.0 * atan2f(B_0 + sqrtf(A_0 * A_0 + B_0 * B_0 - C_0 * C_0), A_0 + C_0);

        float x_C = -L_5 / 2.0 + L_1 * cos(connectingRodState.rightPhi1) + L_2 * cos(phi2);
        float y_C = 0.0 + L_1 * sin(connectingRodState.rightPhi1) + L_2 * sin(phi2);

        float phi3 = atan2f(y_C - y_D, x_C - x_D);

        forwardSolveResult.lRight = sqrtf(x_C * x_C + y_C * y_C);
        forwardSolveResult.thetaRight = atan2f(y_C, x_C);
    }
}

void VmcController::Difference_Solve()
{
    // Left leg difference kinematics
    {
        float t8 = L_2 * L_2;
        float t10 = L_1 * cos(connectingRodState.leftPhi1);
        float t11 = L_4 * cos(connectingRodState.leftPhi4);
        float t12 = L_1 * sin(connectingRodState.leftPhi1);
        float t13 = L_4 * sin(connectingRodState.leftPhi4);
        float t19 = 2.0 * L_2 * t10;
        float t20 = 2.0 * L_2 * t11;
        float t33 = t12 - t13;
        float t34 = L_5 + t11 - t10;
        float t35 = t33 * t33;
        float t36 = t34 * t34;
        float t37 = 2.0 * L_2 * t33;
        float t50 = 2.0 * (t10 * t33 + t12 * t34);
        float t51 = 2.0 * (t11 * t33 + t13 * t34);
        float t52 = t8 - L_3 * L_3 + t35 + t36;
        float t54 = 2.0 * L_2 * t12 + t50;
        float t55 = 2.0 * L_2 * t13 + t51;
        float t60 = 2.0 * t50 * t52;
        float t61 = 2.0 * t51 * t52;
        float t58 = 1.0 / (2.0 * L_2 * t34 + t52);
        float t59 = t58 * t58;
        float t64 = 4.0 * t8 * t35 + 4.0 * t8 * t36 - t52 * t52;
        float t69 = 8.0 * t8 * t10 * t33 + 8.0 * t8 * t12 * t34 - t60;
        float t70 = 8.0 * t8 * t11 * t33 + 8.0 * t8 * t13 * t34 - t61;
        float t65 = sqrtf(t64);
        float t90 = -t54 * t59 * (t37 - t65);
        float t91 = -t55 * t59 * (t37 - t65);
        float t72 = atanf(-(t37 - t65) * t58);
        float t94 = 0.5 * (t69 / t65);
        float t95 = 0.5 * (t70 / t65);
        float t80 = 1.0 / (t59 * (t37 - t65) * (t37 - t65) + 1.0);
        float t78 = L_2 * cos(2.0 * t72);
        float t79 = L_2 * sin(2.0 * t72);
        float t86 = t12 + t79;
        float t88 = t10 - L_5 / 2.0 + t78;
        float t112 = -t78 * t80 * (t90 + t58 * (t19 - t94));
        float t113 = -t78 * t80 * (t91 + t58 * (t20 - t95));
        float t114 = -t79 * t80 * (t90 + t58 * (t19 - t94));
        float t115 = -t79 * t80 * (t91 + t58 * (t20 - t95));
        float t100 = t12 + t79;
        float t103 = L_5 / 2.0 - t10 - t78;
        float t104 = 1.0 / sqrtf(t86 * t86 + t88 * t88);
        float t105 = t103 * t103;
        float t106 = 1.0 / t103;
        float t107 = 1.0 / t105;
        float t111 = 1.0 / (t100 * t100 + t105);
        float dt = 0.001f; // Assuming a time step of 1 ms

        float lLeftDotTemp = 0.5 * connectingRodState.leftPhi1Dot * t104 * (2.0 * t86 * (t10 - t78 * t80 * (t90 + t58 * (t19 - t94)) * 2.0) - 2.0 * t88 * (t12 - t79 * t80 * (t90 + t58 * (t19 - t94)) * 2.0)) + 0.5 * connectingRodState.leftPhi4Dot * t104 * (4.0 * t78 * t80 * t86 * (t91 + t58 * (t20 - t95)) - 4.0 * t79 * t80 * t88 * (t91 + t58 * (t20 - t95)));
        float thetaLeftDotTemp = connectingRodState.leftPhi4Dot * t105 * t111 * (-2.0 * t106 * t113 + 2.0 * t100 * t107 * t115) + connectingRodState.leftPhi1Dot * t105 * t111 * (2.0 * t106 * (t10 + 2.0 * t112) - 2.0 * t100 * t107 * (t12 + 2.0 * t114));

        forwardSolveResult.lLeftDot2 = (lLeftDotTemp - forwardSolveResult.lLeftDot) / dt;
        forwardSolveResult.thetaLeftDot2 = (thetaLeftDotTemp - forwardSolveResult.thetaLeftDot) / dt;

        forwardSolveResult.lLeftDot = lLeftDotTemp;
        forwardSolveResult.thetaLeftDot = thetaLeftDotTemp;
    }

    // Right leg difference kinematics
    {
        float t8 = L_2 * L_2;
        float t10 = L_1 * cos(connectingRodState.rightPhi1);
        float t11 = L_4 * cos(connectingRodState.rightPhi4);
        float t12 = L_1 * sin(connectingRodState.rightPhi1);
        float t13 = L_4 * sin(connectingRodState.rightPhi4);
        float t19 = 2.0 * L_2 * t10;
        float t20 = 2.0 * L_2 * t11;
        float t33 = t12 - t13;
        float t34 = L_5 + t11 - t10;
        float t35 = t33 * t33;
        float t36 = t34 * t34;
        float t37 = 2.0 * L_2 * t33;
        float t50 = 2.0 * (t10 * t33 + t12 * t34);
        float t51 = 2.0 * (t11 * t33 + t13 * t34);
        float t52 = t8 - L_3 * L_3 + t35 + t36;
        float t54 = 2.0 * L_2 * t12 + t50;
        float t55 = 2.0 * L_2 * t13 + t51;
        float t60 = 2.0 * t50 * t52;
        float t61 = 2.0 * t51 * t52;
        float t58 = 1.0 / (2.0 * L_2 * t34 + t52);
        float t59 = t58 * t58;
        float t64 = 4.0 * t8 * t35 + 4.0 * t8 * t36 - t52 * t52;
        float t69 = 8.0 * t8 * t10 * t33 + 8.0 * t8 * t12 * t34 - t60;
        float t70 = 8.0 * t8 * t11 * t33 + 8.0 * t8 * t13 * t34 - t61;
        float t65 = sqrtf(t64);
        float t90 = -t54 * t59 * (t37 - t65);
        float t91 = -t55 * t59 * (t37 - t65);
        float t72 = atanf(-(t37 - t65) * t58);
        float t94 = 0.5 * (t69 / t65);
        float t95 = 0.5 * (t70 / t65);
        float t80 = 1.0 / (t59 * (t37 - t65) * (t37 - t65) + 1.0);
        float t78 = L_2 * cos(2.0 * t72);
        float t79 = L_2 * sin(2.0 * t72);
        float t86 = t12 + t79;
        float t88 = t10 - L_5 / 2.0 + t78;
        float t112 = -t78 * t80 * (t90 + t58 * (t19 - t94));
        float t113 = -t78 * t80 * (t91 + t58 * (t20 - t95));
        float t114 = -t79 * t80 * (t90 + t58 * (t19 - t94));
        float t115 = -t79 * t80 * (t91 + t58 * (t20 - t95));
        float t100 = t12 + t79;
        float t103 = L_5 / 2.0 - t10 - t78;
        float t104 = 1.0 / sqrtf(t86 * t86 + t88 * t88);
        float t105 = t103 * t103;
        float t106 = 1.0 / t103;
        float t107 = 1.0 / t105;
        float t111 = 1.0 / (t100 * t100 + t105);
        float dt = 0.001f; // Assuming a time step of 1 ms

        float lRightDotTemp = 0.5 * connectingRodState.rightPhi1Dot * t104 * (2.0 * t86 * (t10 - t78 * t80 * (t90 + t58 * (t19 - t94)) * 2.0) - 2.0 * t88 * (t12 - t79 * t80 * (t90 + t58 * (t19 - t94)) * 2.0)) + 0.5 * connectingRodState.rightPhi4Dot * t104 * (4.0 * t78 * t80 * t86 * (t91 + t58 * (t20 - t95)) - 4.0 * t79 * t80 * t88 * (t91 + t58 * (t20 - t95)));
        float thetaRightDotTemp = connectingRodState.rightPhi4Dot * t105 * t111 * (-2.0 * t106 * t113 + 2.0 * t100 * t107 * t115) + connectingRodState.rightPhi1Dot * t105 * t111 * (2.0 * t106 * (t10 + 2.0 * t112) - 2.0 * t100 * t107 * (t12 + 2.0 * t114));

        forwardSolveResult.lRightDot2 = (lRightDotTemp - forwardSolveResult.lRightDot) / dt;
        forwardSolveResult.thetaRightDot2 = (thetaRightDotTemp - forwardSolveResult.thetaRightDot) / dt;

        forwardSolveResult.lRightDot = lRightDotTemp;
        forwardSolveResult.thetaRightDot = thetaRightDotTemp;
    }
}

void VmcController::Update_State()
{
    Forward_Solve();
    Difference_Solve();
}

void VmcController::VMC_Calculate_Torque()
{
    // Left leg torque calculation
    {
        float theta = PI / 2 - forwardSolveResult.thetaLeft;
        float l = forwardSolveResult.lLeft;

        // In SJTU model, phi1 is the forward angle, phi2 is the PI/2 minus the backward angle
        float phi1 = connectingRodState.leftPhi1;
        float phi2 = PI / 2 - connectingRodState.leftPhi4;
        float xe = l * sin(theta), ye = l * cos(theta);
        float x1 = -L_1 * cos(phi1), y1 = L_1 * sin(phi1);
        float x2 = -L_1 * cos(phi2), y2 = L_1 * sin(phi2);

        float phi1_diff_l = ((xe + x1) * sin(theta) + (ye - y1) * cos(theta)) / (l * (-(xe + x1) * sin(phi1) + (ye - y1) * cos(phi1)));
        float phi1_diff_theta = (l * ((xe + x1) * cos(theta) - (ye - y1) * sin(theta))) / (L_1 * (-(xe + x1) * sin(phi1) + (ye - y1) * cos(phi1)));
        float phi2_diff_l = ((xe - x2) * sin(theta) + (ye - y2) * cos(theta)) / (l * ((xe - x2) * sin(phi2) + (ye - y2) * cos(phi2)));
        float phi2_diff_theta = (l * ((xe - x2) * cos(theta) - (ye - y2) * sin(theta))) / (L_1 * ((xe - x2) * sin(phi2) + (ye - y2) * cos(phi2)));

        float det = phi1_diff_l * phi2_diff_theta - phi2_diff_l * phi1_diff_theta;
        if (det == 0)
            return; // Avoid division by zero

        /*
        J:[a b
           c d]
        */
        float a = phi2_diff_theta / det;
        float b = -phi2_diff_l / det;
        float c = -phi1_diff_theta / det;
        float d = phi1_diff_l / det;

        forwardSolveResult.leftForwardTorque = a * targetEffects.leftForce + b * targetEffects.leftTorque;
        forwardSolveResult.leftBackwardTorque = c * targetEffects.leftForce + d * targetEffects.leftTorque;
    }

    // Right leg torque calculation
    {
        float theta = PI / 2 - forwardSolveResult.thetaRight;
        float l = forwardSolveResult.lRight;

        // In SJTU model, phi1 is the forward angle, phi2 is the PI/2 minus the backward angle
        float phi1 = connectingRodState.rightPhi1;
        float phi2 = PI / 2 - connectingRodState.rightPhi4;
        float xe = l * sin(theta), ye = l * cos(theta);
        float x1 = -L_1 * cos(phi1), y1 = L_1 * sin(phi1);
        float x2 = -L_1 * cos(phi2), y2 = L_1 * sin(phi2);

        float phi1_diff_l = ((xe + x1) * sin(theta) + (ye - y1) * cos(theta)) / (l * (-(xe + x1) * sin(phi1) + (ye - y1) * cos(phi1)));
        float phi1_diff_theta = (l * ((xe + x1) * cos(theta) - (ye - y1) * sin(theta))) / (L_1 * (-(xe + x1) * sin(phi1) + (ye - y1) * cos(phi1)));
        float phi2_diff_l = ((xe - x2) * sin(theta) + (ye - y2) * cos(theta)) / (l * ((xe - x2) * sin(phi2) + (ye - y2) * cos(phi2)));
        float phi2_diff_theta = (l * ((xe - x2) * cos(theta) - (ye - y2) * sin(theta))) / (L_1 * ((xe - x2) * sin(phi2) + (ye - y2) * cos(phi2)));

        float det = phi1_diff_l * phi2_diff_theta - phi2_diff_l * phi1_diff_theta;
        if (det == 0)
            return; // Avoid division by zero

        /*
        J:[a b
           c d]
        */
        float a = phi2_diff_theta / det;
        float b = -phi2_diff_l / det;
        float c = -phi1_diff_theta / det;
        float d = phi1_diff_l / det;

        forwardSolveResult.rightForwardTorque = a * targetEffects.rightForce + b * targetEffects.rightTorque;
        forwardSolveResult.rightBackwardTorque = c * targetEffects.rightForce + d * targetEffects.rightTorque;
    }
}