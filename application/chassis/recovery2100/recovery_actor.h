#ifndef RECOVERY_ACTOR_H
#define RECOVERY_ACTOR_H
/* Same normalized31D observation; actual adopted previous; CAD/model order.
 * Mean returns Tanh6; Current actor is c94bb9f tjb_recover31-128-64-32-6.
 * Physical calls the absolute linear1.5/100 decoder exactly ONCE.
 * Caller retains queues, actual adoption, PD/V4 and motor/NET routing. */
int RecoveryActorMean(const float *input, float *mean);
int RecoveryActorPhysical(const float *input, float *mean, float *physical);
#endif
