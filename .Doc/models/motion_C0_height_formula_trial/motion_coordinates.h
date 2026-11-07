#ifndef MOTION31_COORDINATES_H
#define MOTION31_COORDINATES_H
/* Unified model axes, radians; geometric Rhip/Ractive/Lhip/Lactive.
 * known=0 means UNKNOWN even if placeholder numbers are zero.
 * Errors: -1 nonfinite/invalid, -2 unknown calibration, -4 invalid request layer.
 * On error output is unchanged. Constant delta never changes dq or wheel speed.
 */
typedef struct { int known; float delta_rad[4]; } motion31_leg_calibration;
int motion31_q_model_from_report(const float report[4],const motion31_leg_calibration*,float model[4]);
int motion31_q_report_from_model(const float model[4],const motion31_leg_calibration*,float report[4]);
int motion31_observation_model_from_report(const float report[31],const motion31_leg_calibration*,float model[31]);
int motion31_target_report_from_model(const float model[6],const motion31_leg_calibration*,float report[6]);
int motion31_target_model_from_report(const float report[6],const motion31_leg_calibration*,float model[6]);
int motion31_limits_model_from_report(const float report[8],const motion31_leg_calibration*,float model[8]);
int motion31_center_from_unprotected_request(const float mean[6],const float request_model[6],int request_unprotected,int mechanical_clipped,float center[4]);
#endif
