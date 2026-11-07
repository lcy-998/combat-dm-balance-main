#ifndef MOTION31_RUNTIME_H
#define MOTION31_RUNTIME_H
int motion31_actor(const float input[31],float output[6]);
int motion31_decode(const float mean[6],const float center[4],const float lo[4],const float hi[4],float target[6]);
#endif
