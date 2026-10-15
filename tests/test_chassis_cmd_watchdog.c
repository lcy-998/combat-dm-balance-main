#include "chassis_cmd_watchdog.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    ChassisCmdWatchdog watchdog = {0};
    assert(!ChassisCmdWatchdogFresh(&watchdog, 0u, 0u, 0u, 50u));
    assert(ChassisCmdWatchdogFresh(&watchdog, 1u, 10u, 10u, 50u));
    assert(ChassisCmdWatchdogFresh(&watchdog, 1u, 10u, 60u, 50u));
    assert(!ChassisCmdWatchdogFresh(&watchdog, 1u, 10u, 61u, 50u));
    assert(!ChassisCmdWatchdogFresh(&watchdog, 2u, 80u, 150u, 50u));
    assert(ChassisCmdWatchdogFresh(&watchdog, 3u, 150u, 150u, 50u));
    puts("chassis command watchdog tests passed");
    return 0;
}
