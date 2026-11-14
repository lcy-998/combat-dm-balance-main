#ifndef RL_RECOVERY_CONFIG_H
#define RL_RECOVERY_CONFIG_H
/* Test firmware: each safe->enabled cycle begins with Recovery2100. */
#define RL_RECOVERY_AUTO_START 1u
/* 1=observable encoder/IMU readiness; 0=qualified height/dual-support API only.
   Posture readiness is not proof of terrain height or contact. */
#define RL_RECOVERY_POSTURE_HANDOFF 1u
#define RL_RECOVERY_POSTURE_DWELL_MS 20u
#define RL_RECOVERY_POSTURE_TILT_RAD 0.35f
#define RL_RECOVERY_POSTURE_GYRO_RAD_S 3.0f
#define RL_RECOVERY_POSTURE_LEG_SPEED_RAD_S 8.0f
#define RL_RECOVERY_POSTURE_ERROR_RAD 0.50f
#endif
