/*HEADER**********************************************************************
*
* Copyright 2012-2014 Freescale Semiconductor, Inc.
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
*   This file contains the source functions for functions required to
*   specifically initialize the card.
*
*
*END************************************************************************/

#include "mqx_inc.h"
#include "bsp.h"
#include "bsp_prv.h"
#include "io_rev.h"
#include "bsp_rev.h"
#include "core_mutex.h"

#if BSP_ALARM_FREQUENCY == 0
#error Wrong definition of BSP_ALARM_FREQUENCY
#endif

HWTIMER systimer;                                   //System timer handle

void _bsp_systimer_callback(void *dummy);         //callback for system timer

const char      *_mqx_bsp_revision = REAL_NUM_TO_STR(BSP_REVISION);
const char      *_mqx_io_revision  = REAL_NUM_TO_STR(IO_REVISION);

/** Pre initialization - initializing requested modules for basic run of MQX.
 */
int _bsp_pre_init(void) {
    uint32_t result;
    KERNEL_DATA_STRUCT_PTR         kernel_data;

    /* allocate peripheral resources */
    _bsp_rdc_init();

    /* Set the CPU type */
    _mqx_set_cpu_type(MQX_CPU);

#if MQX_EXIT_ENABLED
    /* Set the bsp exit handler, called by _mqx_exit */
    _mqx_set_exit_handler(_bsp_exit_handler);
#endif

    result = _psp_int_init(BSP_FIRST_INTERRUPT_VECTOR_USED, BSP_LAST_INTERRUPT_VECTOR_USED);
    if (result != MQX_OK) {
        return result;
    }

    /* set possible new interrupt vector table - if MQX_ROM_VECTORS = 0 switch to
    ram interrupt table which was initialized in _psp_int_init) */
    _int_set_vector_table(BSP_INTERRUPT_VECTOR_TABLE);

    /*
     * Mutex is the dependency for share clock management so we
     * need to init it before _bsp_clock_manager_init
     */
#if BSPCFG_CORE_MUTEX
    extern const CORE_MUTEX_INIT_STRUCT _core_mutex_init_info;

    result = _core_mutex_install(&_core_mutex_init_info);
    if (result != MQX_OK) {
        return result;
    }
#endif

    /* MU is now dependency of both MCC and Share Clock management*/
    mu_initialize();

    /* Init clock manager */
    _bsp_clock_manager_init();

    /* Install low power support */
#if MQX_ENABLE_LOW_POWER
    _lpm_install (LPM_CPU_OPERATION_MODES, LPM_OPERATION_MODE_RUN);
#endif
 

    /* Initialize , set and run system hwtimer */
    result = hwtimer_init(&systimer, &BSP_SYSTIMER_DEV, BSP_SYSTIMER_ID, BSP_SYSTIMER_ISR_PRIOR); // systick_devif, hwtimer_systick_init
    if (MQX_OK != result) {
        return result;
    }
    result = hwtimer_set_freq(&systimer, BSP_SYSTIMER_SRC_CLK, BSP_ALARM_FREQUENCY);
    if (MQX_OK != result) {
        hwtimer_deinit(&systimer);
        return result;
    }
    result = hwtimer_callback_reg(&systimer,(HWTIMER_CALLBACK_FPTR)_bsp_systimer_callback, NULL);
    if (MQX_OK != result) {
        hwtimer_deinit(&systimer);
        return result;
    }
    result = hwtimer_start(&systimer);
    if (MQX_OK != result) {
        hwtimer_deinit(&systimer);
        return result;
    }

    /* Initialize the system ticks */
    _GET_KERNEL_DATA(kernel_data);
    kernel_data->TIMER_HW_REFERENCE = (BSP_CORE_CLOCK / BSP_ALARM_FREQUENCY);
    _time_set_ticks_per_sec(BSP_ALARM_FREQUENCY);
    _time_set_hwticks_per_tick(hwtimer_get_modulo(&systimer));
    _time_set_hwtick_function(_bsp_get_hwticks, (void *)NULL);

    /* MPU: TODO */

    return 0;
}

/** Initialization - called from init task, usually for io initialization.
 */
int _bsp_init(void) {
    uint32_t result;

    /* Initialize DMA */
/*
    result = dma_init(_bsp_dma_devif_list);
    if (result != MQX_OK) {
        return result;
    }
*/

#if BSPCFG_ENABLE_IO_SUBSYSTEM
    /* Initialize the I/O Sub-system */
    result = _io_init();
    if (result != MQX_OK) {
        return result;
    }

    /* Initialize RTC and MQX time */
#if BSPCFG_ENABLE_RTCDEV
    if (MQX_OK == _bsp_rtc_io_init()) {
        _rtc_init (NULL);
    }
#endif

    /* Extend default memory pool with SRAM(OCRAM or TCML) */
#if MQX_USE_MEM
    result = _mem_extend((void *)__SRAM_POOL_START, (_mem_size)(__SRAM_POOL_END - __SRAM_POOL_START));
    if (result != MQX_OK)
    {
        return result;
    }
#endif

    /** Cache settings **/
    /* nothing to do since cache is enabled when reset */

#if BSPCFG_CORE_MUTEX
/*
    extern const CORE_MUTEX_INIT_STRUCT _core_mutex_init_info;

    result = _core_mutex_install(&_core_mutex_init_info);
    if (result != MQX_OK) {
        return result;
    }
*/
#endif

    /* Install device drivers */

#if BSPCFG_ENABLE_TTYA
    _imx_uart_polled_install("ttya:", &_bsp_sci1_init, _bsp_sci1_init.QUEUE_SIZE);
#endif

#if BSPCFG_ENABLE_TTYB
    _imx_uart_polled_install("ttyb:", &_bsp_sci2_init, _bsp_sci2_init.QUEUE_SIZE);
#endif

#if BSPCFG_ENABLE_ITTYA
   _imx_uart_int_install("ittya:", &_bsp_sci1_init, _bsp_sci1_init.QUEUE_SIZE);
#endif

#if BSPCFG_ENABLE_ITTYB
   _imx_uart_int_install("ittyb:", &_bsp_sci2_init, _bsp_sci2_init.QUEUE_SIZE);
#endif

#if BSPCFG_ENABLE_SPI5
    _io_spi_install("spi5:", &_bsp_spi5_init);
#endif

/* Flashx and QuadSPI only one is allowed to be enabled
 * Flashx interface for QuadSPI flash is preferred */
#if BSPCFG_ENABLE_QUADSPI0 && !BSPCFG_ENABLE_FLASHX
    _io_qspi_install("qspi0:", &_bsp_quadspi0_init);
#endif

/* Flashx and QuadSPI only one is allowed to be enabled
 * Flashx interface for QuadSPI flash is preferred */
#if BSPCFG_ENABLE_QUADSPI1 && !BSPCFG_ENABLE_FLASHX
    _io_qspi_install("qspi1:", &_bsp_quadspi1_init);
#endif

    /* I2C Initialization */
#if BSPCFG_ENABLE_I2C1
    _imx_i2c_polled_install("i2c1:", &_bsp_i2c1_init);
#endif

#if BSPCFG_ENABLE_I2C2
    _imx_i2c_polled_install("i2c2:", &_bsp_i2c2_init);
#endif

#if BSPCFG_ENABLE_I2C3
    _imx_i2c_polled_install("i2c3:", &_bsp_i2c3_init);
#endif

#if BSPCFG_ENABLE_I2C4
    _imx_i2c_polled_install("i2c4:", &_bsp_i2c4_init);
#endif

#if BSPCFG_ENABLE_II2C1
    _imx_i2c_int_install("ii2c1:", &_bsp_i2c1_init);
#endif

#if BSPCFG_ENABLE_II2C2
    _imx_i2c_int_install("ii2c2:", &_bsp_i2c2_init);
#endif

#if BSPCFG_ENABLE_II2C3
    _imx_i2c_int_install("ii2c3:", &_bsp_i2c3_init);
#endif

#if BSPCFG_ENABLE_II2C4
    _imx_i2c_int_install("ii2c4:", &_bsp_i2c4_init);
#endif

#if BSPCFG_ENABLE_SAI
    result = _io_sai_int_install(&_bsp_sai_init);
#endif
   _io_serial_default_init();
#endif

#if MQX_ENABLE_MCORE_LPM
   _io_mcore_lpm_init();
#endif

    return 0;
}


/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_exit_handler
* Returned Value   : none
* Comments         :
*    This function is called when MQX exits
*
*END*----------------------------------------------------------------------*/
void _bsp_exit_handler(void)
{
}


/*FUNCTION*********************************************************************
 *
 * Function Name    : _bsp_systimer_callback
 * Returned Value   : void
 * Comments         :
 *    The system timer callback.
 *
 *END**********************************************************************/

void _bsp_systimer_callback(void *dummy) {
    _time_notify_kernel();
}


/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_get_hwticks
* Returned Value   : none
* Comments         :
*    This function returns the number of hw ticks that have elapsed
* since the last interrupt
*
*END*----------------------------------------------------------------------*/
uint32_t _bsp_get_hwticks(void *param) {
    HWTIMER_TIME_STRUCT time;      //struct for storing time
    hwtimer_get_time(&systimer, &time);
    return time.SUBTICKS;
}

