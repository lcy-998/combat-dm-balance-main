/**
 * (C)Copyright Combat Algorithm Team.All rights reserved.
 * @author: kuroneko(kuroneko@njust.edu.cn)
 * @created: 2026.1.17
 * @description: Macro definitions for leg module constants.
 **/

#ifndef LEG_CONST_HPP
#define LEG_CONST_HPP

/* Macro definitions for mathematical and physical constants */
#define PI        3.14159265358979323846f
#define DEG_2_RAD (PI / 180.0f)
#define RAD_2_DEG (180.0f / PI)
#define G         9.81f

/* Macro definitions for leg-specific constants */
#define R_W 0.06f              // Wheel radius in meters
#define R_L 0.2215f            // Half of distance between wheels in meters
#define L_C 0.03387f           // Distance from the body's centroid to the leg joints in meters
#define M_W 0.29884f           // Mass of the wheel in kilograms
#define M_L 1.398745f          // Mass of the leg in kilograms
#define M_B 16.10888f          // Mass of the body in kilograms
#define I_W 0.00052122406f     // Moment of inertia of the wheel in kg*m^2
#define I_B 0.51247818743f     // Moment of inertia of the body in kg*m^2
#define I_Z 0.70952497810f * 2 // Double moment of inertia of the body around the vertical axis in kg*m^2
#define L_1 0.210f             // Length of the first connecting rod in meters
#define L_2 0.250f             // Length of the second connecting rod in meters
#define L_3 0.250f             // Length of the third connecting rod in meters
#define L_4 0.210f             // Length of the fourth connecting rod in meters
#define L_5 0.0f               // Length of the fifth connecting rod in meters

#endif // LEG_CONST_HPP