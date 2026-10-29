#ifndef HIP_IDENT_CONFIG_H
#define HIP_IDENT_CONFIG_H

/* Dedicated identification firmware. Edit here, rebuild, then export via debugger. */
#define HIP_IDENT_ENABLED 1
#ifndef HIP_IDENT_SIDE
#define HIP_IDENT_SIDE 0u /* 0=physical left, 1=physical right; never both */
#endif
#define HIP_IDENT_REFERENCE_CENTER {0.12f, 0.11f, -0.15f, 0.10f} /* RH, RA, LH, LA; defines center leg angle */
#define HIP_IDENT_LEG_LENGTHS_M {0.22f, 0.25f, 0.28f, 0.31f, 0.34f}
#define HIP_IDENT_SWEEP_START_HZ 0.5f
#define HIP_IDENT_SWEEP_END_HZ 4.0f
#define HIP_IDENT_SWEEP_MS 15000u
#define HIP_IDENT_FADE_MS 2000u
#define HIP_IDENT_AMPLITUDE 0.2f /* existing normalized peak: 0.3 rad per motor */
#define HIP_IDENT_RAMP_MS 3000u
#define HIP_IDENT_HOLD_MS 2000u
#define HIP_IDENT_COMMAND_TIMEOUT_MS 100u
#define HIP_IDENT_FEEDBACK_TIMEOUT_MS 24u

#endif
