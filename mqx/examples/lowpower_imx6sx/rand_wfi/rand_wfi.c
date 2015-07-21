/*HEADER**********************************************************************
*
* Copyright 2014 Freescale Semiconductor, Inc.
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale MQX RTOS License
* distributed with this Material.
* See the MQX_RTOS_LICENSE file distributed for more details.
*
* Brief License Summary:
* This software is provided in source form for you to use free of charge,
* but it is not open source software. You are allowed to use this software
* but you cannot redistribute it or derivative works of it in source form.
* The software may be used only in connection with a product containing
* a Freescale microprocessor, microcontroller, or digital signal processor.
* See license agreement file for full license terms including other
* restrictions.
*****************************************************************************
*
* Comments:
*
*   This file contains the source for the low power example program.
*
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>
#include <fio.h>
#include <lwevent.h>
#include <stdlib.h>

#if ! BSPCFG_ENABLE_IO_SUBSYSTEM
#error This application requires BSPCFG_ENABLE_IO_SUBSYSTEM defined non-zero in user_config.h. Please recompile BSP with this option.
#endif


#ifndef BSP_DEFAULT_IO_CHANNEL_DEFINED
#error This application requires BSP_DEFAULT_IO_CHANNEL to be not NULL. Please set corresponding BSPCFG_ENABLE_TTYx to non-zero in user_config.h and recompile BSP with this option.
#endif

/*
 * Demo configuration :
 *      If DEMO_LPM_MCORE_CONFIGURATION is set to 0, the demo is running with an EPIT
 *    timer. M4 platform will switch between high power running mode and low power
 *    running mode. This is the default demo mode
 *      If DEMO_LPM_MCORE_CONFIGURATION is set to 1, the demo is running without an
 *    EPIT timer. M4 platform will go directly to low power running mode. This mode
 *    will make it easier to test the system low power data. The user should manually
 *    change the macro to 1 for this purpose
 */
#define DEMO_LPM_MCORE_CONFIGURATION    0

#define MCORE_LPM_DEMO_TASK             5

#define PERIOD_MIN                      5
#define PERIOD_MAX                      10

#define EVENT_MASK_FREQ_CHANGE          0x2

/* Static global variable declaration */
static LWEVENT_STRUCT lwevent;

HWTIMER hwtimer;                               //hwtimer handle

extern void mcore_lpm_demo_task(uint32_t);

const TASK_TEMPLATE_STRUCT  MQX_template_list[] =
{
    /* Task Index,           Function,            Stack,  Priority, Name,           Attributes,          Param, Time Slice */
    { MCORE_LPM_DEMO_TASK,   mcore_lpm_demo_task, 1500,   8,        "freq_change",  MQX_AUTO_START_TASK, 0,     0 },
    { 0 }
};



/*FUNCTION*----------------------------------------------------------------*
* Function Name  : hwtimer_callback
* Returned Value : void
* Comments :
*       Callback for hwtimer, in this example the callback function
*       sets the sync event and stop the hwtimer to prepare for the
*       next round timer set
*END*--------------------------------------------------------------------*/
static void hwtimer_callback(void *p)
{
    hwtimer_stop((HWTIMER_PTR)p);
    _lwevent_set(&lwevent,EVENT_MASK_FREQ_CHANGE);
}

/*------------------------------------------------------------------------------
 * Func Name    : hwtimer_set_random_period
 * Comments     : set timer for a random period (unit : S)
 *------------------------------------------------------------------------------*/
static uint32_t hwtimer_set_random_period(HWTIMER_PTR p_hwtimer, uint32_t min_s, uint32_t max_s)
{
    uint32_t delay_s, delta_s;
    /*Generate the random number*/
    if (max_s <= min_s)
        delay_s = min_s;
    else {
        delta_s = max_s - min_s;
        delay_s = rand() % delta_s + min_s;
    }

    /*Set the timer with delay_s*/
    hwtimer_set_period(p_hwtimer, BSP_HWTIMER_LP_SOURCE_CLK, delay_s * 1000000);
    hwtimer_callback_reg(p_hwtimer,(HWTIMER_CALLBACK_FPTR)hwtimer_callback, p_hwtimer);
    hwtimer_start(p_hwtimer);

    return delay_s;
}


/*TASK*------------------------------------------------------------------------------
* Task Name    : demo_task
* Comments     : the main cycle of this task is switching m4 clock source between PLL_PFD
*                and OSC on EPIT timer event. When switching to OSC, it will also disable
*                non-wakeup interrupts and call _io_mcore_lpm_set_m4_status API to let
*                next idle task scheduling put the core into deep sleep (sticking to WFI
*                in TCM code).
*                EPIT timer also acts as wakeup source. Once timer reaches, it will drag M4 out
*                of TCM WFI. The TCM will handshake with A9 to ensure all necessary
*                resources for M4 to run in full speed in QSPI are ready. After that the
*                timer ISR will be invoked so that all the interrupt can be re-enabled,
*                m4 core clock source will change back to PLL and _io_mcore_lpm_set_m4_status
*                API will be called again to disable idle task to enter deep sleep.
*END*------------------------------------------------------------------------------*/
void mcore_lpm_demo_task
    (
        uint32_t initial_data
    )
{
    char control_char;
    uint32_t elapsed_time;
#if (DEMO_LPM_MCORE_CONFIGURATION == 0)
    uint32_t signal_mask = 0;
    static uint32_t state = 0;
#endif

    printf("\n");
    printf(" ************************************************************************\n");
    printf(" * i.MX6SX Dual Core Low Power Demo - M4 side                           *\n");
    printf(" *   A random timer will change the M4 running speed                    *\n");
    printf(" *   Please wait :                                                      *\n");
    printf(" *       1) A9 peer is ready                                            *\n");
    printf(" *   Then press \"S\" to start the demo                                   *\n");
    printf(" ************************************************************************\n");

    while (TRUE) {
        printf("\nPress \"S\" to start the demo : ");
        control_char = getchar();
        if ((control_char == 's') || (control_char == 'S')) {
            break;
        }
    }
    printf("\n");

#if (DEMO_LPM_MCORE_CONFIGURATION == 0)
    /* create lwevent group */
    if (_lwevent_create(&lwevent, LWEVENT_AUTO_CLEAR) != MQX_OK) {
       printf("\nMake event failed");
       _task_block();
    }
    /*Configure HWTIMER*/
    printf("\nInitialization of hwtimer   :");
    if (MQX_OK != hwtimer_init(&hwtimer, &BSP_HWTIMER1_DEV, BSP_HWTIMER1_ID, (BSP_DEFAULT_MQX_HARDWARE_INTERRUPT_LEVEL_MAX + 1)))
    {
        printf(" FAILED!\n");
    }
    else
    {
        printf(" OK\n");
    }

    while (1) {
        /*
         * Set the timer so that the event can be set at some future time
         */
        elapsed_time = hwtimer_set_random_period(&hwtimer, PERIOD_MIN, PERIOD_MAX);
        /*
         * Wait for timer up
         */
        if (_lwevent_wait_ticks(&lwevent, EVENT_MASK_FREQ_CHANGE, FALSE, 0) != MQX_OK) {
            printf("\nEvent Wait failed");
            _task_block();
        }

        signal_mask = _lwevent_get_signalled();

        if (signal_mask & EVENT_MASK_FREQ_CHANGE) {
            if (_io_mcore_lpm_get_status() == STATUS_NORMAL_RUN) {
                /*
                 * register EPIT1 as A9 wakeup source
                 */
                if (WAKEUP_REGISTER_SUCCESS == _io_mcore_lpm_register_peer_wakeup(INT_EPIT1, WAKEUP_ENABLE)) {
                    /*
                     * Disable peripheral interrupts, all these interrupts will
                     * not drag M4 out of WFI
                     */
                    /*let UART finish its buffer*/
                    printf("%ds passed, A9 low power operation allowed\n", elapsed_time);
                    fflush(stdout);

                    /* disable systick interrupt */
                    SysTick_CSR_REG(SysTick_BASE_PTR) &= ~SysTick_CSR_ENABLE_MASK;
                    /*
                     * Allow idle task to enter Ram WFI
                     */
                    _io_mcore_lpm_set_status(STATUS_LOWPOWER_RUN);
                    /*
                     * Let the task remember low power operation has been performed
                     */
                    state = 1;
                }
            }
            else if (_io_mcore_lpm_get_status() == STATUS_LOWPOWER_RUN) {
                if (state == 1) {
                    /*
                     * unregister EPIT as peer wakeup source
                     */
                    _io_mcore_lpm_register_peer_wakeup(INT_EPIT1, WAKEUP_DISABLE);

                    /*
                     * Enable peripheral interrupts, all these interrupts will
                     * now drag M4 out of WFI
                     */
                    /*enable systick interrupt*/
                    SysTick_CSR_REG(SysTick_BASE_PTR) |= SysTick_CSR_ENABLE_MASK;

                    state = 0;
                    /*
                     * Don't allow idle task to enter Ram WFI
                     */
                    _io_mcore_lpm_set_status(STATUS_NORMAL_RUN);

                    printf("%ds passed, A9 low power operation denied\n", elapsed_time);
                    fflush(stdout);
                }
            }
        }
    }
#elif (DEMO_LPM_MCORE_CONFIGURATION == 1)
    /* disable systick interrupt */
    SysTick_CSR_REG(SysTick_BASE_PTR) &= ~SysTick_CSR_TICKINT_MASK;
    /*
     * Allow idle task to enter Ram WFI
     */
    printf("A9 low power operation allowed forever...\n");
    fflush(stdout);
    _io_mcore_lpm_set_status(STATUS_LOWPOWER_RUN);
#endif
}
/* EOF */
