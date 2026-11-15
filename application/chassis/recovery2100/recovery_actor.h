#ifndef RECOVERY_ACTOR_H
#define RECOVERY_ACTOR_H
/* Same normalized31D observation; actual adopted previous; CAD/model order.
 * Mean returns Tanh6; Physical calls the cubic decoder exactly ONCE.
 * Caller retains queues, actual adoption, PD/V4 and motor/NET routing. */
int RecoveryActorMean(const float *input, float *mean);
int RecoveryActorPhysical(const float *input, float *mean, float *physical);
#endif
