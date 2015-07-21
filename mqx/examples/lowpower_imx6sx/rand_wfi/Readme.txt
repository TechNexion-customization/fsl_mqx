--- Introduction ---

The example demo's dual core low power management. i.MX6SX is a dual-core
chip, a Cortex-A9 core and a Cortex-M4 core. A9 is the power management
arbiter, it controls the chip level power mode. M4 and its peripherals act
as a general device attached to A9. M4 will supervise the state of all its
peripheral, when all of them are running in low speed, M4 can enter its own
power saving mode. In this mode M4 will release all the high power resources,
including PLL, QSPI clock, etc. M4 can then inform A9 about this status, A9
can then acturally shut down these resouces based on the system level
situation (including M4 and other peripherals directly managed by A9).

This demo is based on EPIT timer on M4 side. With the timer, the M4 can
switches between high power running mode and low power running mode
continuously.

--- Running the Demo ---

When running this demo, a prompt will be output as the following:

    ************************************************************************
    * i.MX6SX Dual Core Low Power Demo - M4 side                           *
    *   A random timer will change the M4 running speed                    *
    *   Please wait :                                                      *
    *       1) A9 peer is ready                                            *
    *   Then press "S" to start the demo                                   *
    ************************************************************************

    Press "S" to start the demo :

The user should wait to Linux boot up finished at A9 side, then press the 'S'
key to kick off the M4 demo.

Once the demo is running, a random timer will triggle the power mode change.
The default timer is between 5s to 10s. Users can change the 2 macros 
    - PERIOD_MIN
    - PERIOD_MAX
to get customized timer range, the unit of these 2 macro are both second.

The printf counter is output as the following:

    6s passed, A9 low power operation allowed
    7s passed, A9 low power operation denied
    ...

We can see the M4 platform first wait 6s to switches to low power running
mode, then wait 7s to switches to high power running mode. In the backgoround,
the M4 release all high power resources and informs A9 that it in low power
running mode . In this situation, A9 can reduce M4's clock frequency and do
busfreq/suspend test.
In busfreq test, A9 can reduce the bus freq to 24MHz, in suspend test, A9 can
power off all the power mixes. If M4 is running in high speed running mode, we
can invesitgete that A9 can not change the bus freq to 24MHz, or power off all
the power mixes.

M4 can register its peripheral as wakeup source to A9. When A9 enters suspend
mode, it will holds M4 in WFI, but the registered wakeup source will wakeup A9
and let it to release M4 to normal running mode. In this demo, even if A9
enters full suspend mode, the EPIT timer, which is registed as A9 wakeup
source, will drag A9 out of suspend and as a concequence makes M4 into running
mode again.


--- the Demo Configuration ---
The default demo uses a EPT timer to switch between high power and low power
runnign speed. But since the switch is too frequently, there will be
difficulty for users to measure the low power data for the whole system. To
aid on this, a option is provided by a macro
    - DEMO_LPM_MCORE_CONFIGURATION 
This macro is by default set to 0, which will let the demo be a EPT timer
based one as described before. Users can change this macro to 1, which will
avoid the M4 platform from switching power modes continuously. In this case,
the demo will directly makes M4 switching to low power running mode and hold
on this state forever, then users can have adequate time to suspend the A9
then measuring the system level low power data.

The output in this configuration will be as the following:

    ************************************************************************
    * i.MX6SX Dual Core Low Power Demo - M4 side                           *
    *   A random timer will change the M4 running speed                    *
    *   Please wait :                                                      *
    *       1) A9 peer is ready                                            *
    *   Then press "S" to start the demo                                   *
    ************************************************************************

    Press "S" to start the demo : s
    A9 low power operation allowed forever...
