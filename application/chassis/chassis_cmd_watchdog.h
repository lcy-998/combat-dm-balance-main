#ifndef CHASSIS_CMD_WATCHDOG_H
#define CHASSIS_CMD_WATCHDOG_H

#include <stdint.h>

typedef struct {
    uint32_t last_sequence;
    uint32_t last_rx_tick;
    uint8_t received;
    uint8_t rearm_required;
} ChassisCmdWatchdog;

/* Only a newly received zero-force frame can rearm after startup or timeout. */
static inline int ChassisCmdWatchdogAccept(ChassisCmdWatchdog *watchdog,
                                           uint32_t rx_sequence,
                                           uint32_t rx_tick,
                                           uint32_t now_tick,
                                           uint32_t timeout_ticks,
                                           int zero_force)
{
    if (!watchdog->received ||
        (uint32_t)(now_tick - watchdog->last_rx_tick) > timeout_ticks)
        watchdog->rearm_required = 1u;
    int new_frame = rx_sequence != watchdog->last_sequence;
    if (new_frame) {
        watchdog->last_sequence = rx_sequence;
        watchdog->last_rx_tick = rx_tick;
        watchdog->received = 1u;
    }
    if (!watchdog->received ||
        (uint32_t)(now_tick - watchdog->last_rx_tick) > timeout_ticks) {
        watchdog->rearm_required = 1u;
        return 0;
    }
    if (watchdog->rearm_required) {
        if (new_frame && zero_force)
            watchdog->rearm_required = 0u;
        return 0;
    }
    return 1;
}

#endif
