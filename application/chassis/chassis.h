#ifndef CHASSIS_H
#define CHASSIS_H

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit();

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask();

/* Ground-to-root command height in meters, full frozen .30-.40m domain.
   Allowed during motion; no stationarity/dwell/slew gate. */
int ChassisSetRLHeight(float command_height_m);

#endif // CHASSIS_H