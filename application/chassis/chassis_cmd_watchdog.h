#ifndef CHASSIS_CMD_WATCHDOG_H
#define CHASSIS_CMD_WATCHDOG_H

#include <stdint.h>

typedef struct {
    uint32_t last_sequence;
    uint32_t last_rx_tick;
    uint8_t received;
} ChassisCmdWatchdog;

/* The caller observes a sequence increment only after a complete CAN callback. */
static inline int ChassisCmdWatchdogFresh(ChassisCmdWatchdog *watchdog,
                                          uint32_t rx_sequence,
                                          uint32_t rx_tick,
                                          uint32_t now_tick,
                                          uint32_t timeout_ticks)
{
    if (rx_sequence != watchdog->last_sequence) {
        watchdog->last_sequence = rx_sequence;
        watchdog->last_rx_tick = rx_tick;
        watchdog->received = 1u;
    }
    return watchdog->received &&
           (uint32_t)(now_tick - watchdog->last_rx_tick) <= timeout_ticks;
}

#endif
