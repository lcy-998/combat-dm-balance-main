#ifndef RL_POLICY_CORE_H
#define RL_POLICY_CORE_H
#include <stdint.h>
#define RL_POLICY_CORE_ABI 1u
#define RL_POLICY_CENTER_CONSTANT 1u
#define RL_POLICY_CENTER_TABLE 2u
#define RL_POLICY_CENTER_POLY5 3u
#define RL_POLICY_RECOVERY_CUBIC 4u
#define RL_POLICY_CENTER_TABLE_WIDE 5u
#define RL_POLICY_MAX_NODES 16u
/* Explicit center identity; table interpolation and firmware poly5 never alias. */
typedef struct {
    uint32_t abi, center_kind, node_count;
    float leg_scale, wheel_scale;
    double geometry[6]; /* hip above root, wheel radius, upper, lower, hip zero deg, active zero deg */
    double height_min, height_max;
    double offset[4], heights[RL_POLICY_MAX_NODES], offsets[RL_POLICY_MAX_NODES][2];
    double leg_lower[4], leg_upper[4];
} RLPolicyDecodeConfig;
uint32_t RLPolicyCoreAbi(void);
int RLPolicyEncode(const float *gyro, const float *gravity, const float *q,
                   const float *dq, const float *adopted, const float *command,
                   uint32_t skill, uint32_t jump, float *observation);
int RLPolicyDecode(const RLPolicyDecodeConfig *config, const float *raw, float height, float *target);
/* Protection belongs before adoption; shadow decoding never calls Adopt. */
int RLPolicyProtect(const float *requested, const float *lower, const float *upper, float *protected_target);
int RLPolicyAdopt(const float *protected_target, float *adopted);
int RLPolicyPd(const float *q, const float *dq, const float *adopted,
               const float *gains, const float *caps, float *requested, float *limited);
#endif
