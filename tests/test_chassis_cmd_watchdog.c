#include "chassis_cmd_watchdog.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    ChassisCmdWatchdog watchdog = {0};
    assert(!ChassisCmdWatchdogAccept(&watchdog, 0u, 0u, 0u, 50u, 0));
    assert(!ChassisCmdWatchdogAccept(&watchdog, 1u, 10u, 10u, 50u, 0));
    assert(!ChassisCmdWatchdogAccept(&watchdog, 2u, 20u, 20u, 50u, 1));
    assert(ChassisCmdWatchdogAccept(&watchdog, 3u, 30u, 30u, 50u, 0));
    assert(ChassisCmdWatchdogAccept(&watchdog, 3u, 30u, 80u, 50u, 0));
    assert(!ChassisCmdWatchdogAccept(&watchdog, 3u, 30u, 81u, 50u, 0));
    assert(!ChassisCmdWatchdogAccept(&watchdog, 4u, 90u, 90u, 50u, 0));
    assert(!ChassisCmdWatchdogAccept(&watchdog, 4u, 90u, 90u, 50u, 1));
    assert(!ChassisCmdWatchdogAccept(&watchdog, 5u, 100u, 100u, 50u, 1));
    assert(ChassisCmdWatchdogAccept(&watchdog, 6u, 110u, 110u, 50u, 0));
    assert(!ChassisCmdWatchdogAccept(&watchdog, 7u, 120u, 200u, 50u, 1));
    assert(!ChassisCmdWatchdogAccept(&watchdog, 8u, 201u, 201u, 50u, 1));
    assert(ChassisCmdWatchdogAccept(&watchdog, 9u, 202u, 202u, 50u, 0));
    /* A new frame after a task stall must not hide the preceding link gap. */
    assert(!ChassisCmdWatchdogAccept(&watchdog, 10u, 300u, 300u, 50u, 0));
    assert(!ChassisCmdWatchdogAccept(&watchdog, 11u, 301u, 301u, 50u, 1));
    assert(ChassisCmdWatchdogAccept(&watchdog, 12u, 302u, 302u, 50u, 0));
    puts("chassis command watchdog tests passed");
    return 0;
}
