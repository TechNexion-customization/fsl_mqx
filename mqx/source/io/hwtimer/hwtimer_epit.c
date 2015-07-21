
/*HEADER**********************************************************************
*
* Copyright 2013-2014 Freescale Semiconductor, Inc.
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
* See license agreement file for full license terms including other restrictions.
*****************************************************************************
*
* Comments:
*
*   This file contains functions of the low level EPIT module for hwtimer
*   component.
*
*
*END************************************************************************/

#include "hwtimer.h"
#include "hwtimer_epit.h"
#include <bsp.h>

/*!
 * \cond DOXYGEN_PRIVATE
 * Macro allows the timers to be stopped when the device enters the Debug mode.
 */
#ifndef BSPCFG_HWTIMER_EPIT_FREEZE
/* Timers continue to run in Debug mode */
#define  BSPCFG_HWTIMER_EPIT_FREEZE 0
#endif


extern void     epit_io_init(uint32_t, uint32_t);
extern uint32_t epit_get_vector(uint32_t);
extern void    *epit_get_base_address(uint32_t);
extern CLOCK_NAME epit_get_clock_name(uint32_t);

static void     hwtimer_epit_isr(void *);
static _mqx_int hwtimer_epit_init(HWTIMER_PTR, uint32_t, uint32_t);
static _mqx_int hwtimer_epit_deinit(HWTIMER_PTR);
static _mqx_int hwtimer_epit_set_div(HWTIMER_PTR, uint32_t);
static _mqx_int hwtimer_epit_start(HWTIMER_PTR);
static _mqx_int hwtimer_epit_stop(HWTIMER_PTR);
static _mqx_int hwtimer_epit_get_time(HWTIMER_PTR, HWTIMER_TIME_PTR);

const HWTIMER_DEVIF_STRUCT epit_devif =
{
    hwtimer_epit_init,
    hwtimer_epit_deinit,
    hwtimer_epit_set_div,
    hwtimer_epit_start,
    hwtimer_epit_stop,
    hwtimer_epit_get_time
};

/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief Interrupt service routine.
 *
 * This ISR is used when every channel has its own vector in vector table.
 * Checks whether callback_func is not NULL,
 * and unless callback is blocked by callback_blocked being non-zero it calls the callback function with callback_data as parameter,
 * otherwise callback_pending is set to non-zero value.
 *
 * \param p[in]   Pointer to hwtimer struct.
 *
 * \return void
 *
 * \see hwtimer_epit_deinit
 * \see hwtimer_epit_set_div
 * \see hwtimer_epit_start
 * \see hwtimer_epit_stop
 * \see hwtimer_epit_get_time
 */
static void hwtimer_epit_isr(void *p)
{
    HWTIMER_PTR hwtimer      = (HWTIMER_PTR) p;
    EPIT_MemMapPtr epit_base = (EPIT_MemMapPtr) hwtimer->ll_context[0];

    /* Check if interrupt is enabled for this channel. Cancel spurious interrupt */
    if (!(EPIT_CR_OCIEN_MASK & EPIT_CR_REG(epit_base)))
    {
        return;
    }

    /* Clear interrupt flag */
    EPIT_SR_REG(epit_base) = EPIT_SR_OCIF_MASK;

    /* Following part of function is typically the same for all low level hwtimer drivers */
    hwtimer->ticks++;

    if (NULL != hwtimer->callback_func)
    {
        if (hwtimer->callback_blocked)
        {
            hwtimer->callback_pending = 1;
        }
        else
        {
            /* Run user function*/
            hwtimer->callback_func(hwtimer->callback_data);
        }
    }
}


/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief This function initializes caller allocated structure according to given
 * numerical identifier of the timer.
 *
 * Called by hwtimer_init().
 * Initializes the HWTIMER structure.
 * Sets interrupt priority and registers ISR.
 *
 * \param hwtimer[in]   Returns initialized hwtimer structure handle.
 * \param epit_id[in]    Determines EPIT modul and epit channel.
 * \param isr_prior[in] Interrupt priority for EPIT
 *
 * \return MQX_OK                       Success.
 * \return MQX_INVALID_PARAMETER        When channel number does not exist in epit module.
 * \return MQX_INVALID_COMPONENT_HANDLE Doesnt have any interrupt vectors, or epit does not exist.
 * \return -1                           When epit is used byt PE or when _int_install_isr faild.
 *
 * \see hwtimer_epit_deinit
 * \see hwtimer_epit_set_div
 * \see hwtimer_epit_start
 * \see hwtimer_epit_stop
 * \see hwtimer_epit_get_time
 * \see hwtimer_epit_isr
 */
static _mqx_int hwtimer_epit_init(HWTIMER_PTR hwtimer, uint32_t epit_id, uint32_t isr_prior)
{
    PSP_INTERRUPT_TABLE_INDEX vector;
    EPIT_MemMapPtr epit_base;
    CLOCK_NAME     clock_name;
    /* We need to store epit_id of timer in context struct */
    hwtimer->ll_context[0] = (uint32_t)epit_get_base_address(epit_id);
    hwtimer->ll_context[1] = epit_id;

    clock_name = epit_get_clock_name(epit_id);
    if (clock_name == CLK_MAX) {
       /* clock name equals to CLK_MAX if no clock name is found */
       return MQX_INVALID_PARAMETER;
    }
    hwtimer->clock_hwtimer = clock_get(clock_name);
    epit_base    = (EPIT_MemMapPtr) hwtimer->ll_context[0];

    clock_enable(hwtimer->clock_hwtimer);

    /* software reset for EPIT module and wait until the software reset is finished */
    EPIT_CR_REG(epit_base) |= EPIT_CR_SWR_MASK;
    while ((EPIT_CR_REG(epit_base) & EPIT_CR_SWR_MASK));

    /* Set the clock source and modes */
    EPIT_CR_REG(epit_base) &= ~(EPIT_CR_EN_MASK | (0x3 << 22) | EPIT_CR_OCIEN_MASK |
                                EPIT_CR_IOVW_MASK | (0x3 << 24) | (0xFFF << 4));
    EPIT_CR_REG(epit_base) |= (EPIT_CR_DBGEN_MASK | EPIT_CR_ENMOD_MASK | EPIT_CR_RLD_MASK);

    EPIT_SR_REG(epit_base) = EPIT_SR_OCIF_MASK;

    #if BSPCFG_HWTIMER_EPIT_FREEZE
    /* Allows the timers to be stopped when the device enters the Debug mode. */
    EPIT_CR_REG(epit_base) |= EPIT_CR_DBGEN_MASK;
    #endif

    /* Enable the clock even in stop and wait mode*/
    EPIT_CR_REG(epit_base) |= (1 << 21);
    EPIT_CR_REG(epit_base) |= (1 << 19);

    EPIT_CMPR_REG(epit_base) = 0;

    /* Set isr for timer*/
    vector = (PSP_INTERRUPT_TABLE_INDEX) epit_get_vector(epit_id);

    #if MQX_CHECK_ERRORS
    if (vector == (PSP_INTERRUPT_TABLE_INDEX)NULL)
    {
        clock_disable(hwtimer->clock_hwtimer);
        hwtimer->clock_hwtimer = NULL;
        return MQX_INVALID_COMPONENT_HANDLE;  //doesnt have any interrupt vectors, or epit does not exist
    }
    #endif

    if (NULL == _int_install_isr(vector, (INT_ISR_FPTR) hwtimer_epit_isr, (void *) hwtimer))
    {
        clock_disable(hwtimer->clock_hwtimer);
        hwtimer->clock_hwtimer = NULL;
        return -1;
    }

    _bsp_int_init(vector, isr_prior, 0, TRUE);

    clock_disable(hwtimer->clock_hwtimer);

    return MQX_OK;
}

/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief Initialization of epit timer module
 *
 * Called by hwtimer_deinit.
 * Disables the peripheral.
 * Unregisters ISR.

 *
 * \param hwtimer[in] Pointer to hwtimer structure.
 *
 * \return MQX_OK                       Success.
 * \return MQX_INVALID_COMPONENT_HANDLE When doesnt have any interrupt vectors, or epit does not exist.
 *
 * \see hwtimer_epit_init
 * \see hwtimer_epit_set_div
 * \see hwtimer_epit_start
 * \see hwtimer_epit_stop
 * \see hwtimer_epit_get_time
 * \see hwtimer_epit_isr
 */
static _mqx_int hwtimer_epit_deinit(HWTIMER_PTR hwtimer)
{
    PSP_INTERRUPT_TABLE_INDEX vector;
    uint32_t epit_number = hwtimer->ll_context[1];
    EPIT_MemMapPtr epit_base = (EPIT_MemMapPtr) hwtimer->ll_context[0];

    vector = (PSP_INTERRUPT_TABLE_INDEX) epit_get_vector(epit_number);

    #if MQX_CHECK_ERRORS
    if (vector == (PSP_INTERRUPT_TABLE_INDEX)NULL)
    {
        return MQX_INVALID_COMPONENT_HANDLE;  //doesnt have any interrupt vectors, or epit does not exist
    }
    #endif

    if (hwtimer->clock_hwtimer == NULL)
        return MQX_INVALID_COMPONENT_HANDLE;

    clock_enable(hwtimer->clock_hwtimer);

    EPIT_CR_REG(epit_base) &= ~EPIT_CR_EN_MASK;

    /* Disable interrupt on vector */
    _bsp_int_disable(vector);
    /* Install default isr routine for our epit */
    _int_install_isr(vector, _int_get_default_isr(), NULL);

    clock_disable(hwtimer->clock_hwtimer);
    return MQX_OK;
}

/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief Sets up timer with divider settings closest to the requested total divider factor.
 *
 * Called by hwtimer_set_freq() and hwtimer_set_period().
 * Fills in the divider (actual total divider) and modulo (sub-tick resolution) members of the HWTIMER structure.
 *
 * \param hwtimer[in] Pointer to hwtimer structure.
 * \param divider[in] Value which divide input clock of epit timer module to obtain requested period of timer.
 *
 * \return MQX_OK                Success.
 * \return MQX_INVALID_PARAMETER Divider is equal to zero.
 *
 * \see hwtimer_epit_init
 * \see hwtimer_epit_deinit
 * \see hwtimer_epit_start
 * \see hwtimer_epit_stop
 * \see hwtimer_epit_get_time
 * \see hwtimer_epit_isr
 */
static _mqx_int hwtimer_epit_set_div(HWTIMER_PTR hwtimer, uint32_t divider)
{
    EPIT_MemMapPtr epit_base  = (EPIT_MemMapPtr) hwtimer->ll_context[0];

    #if MQX_CHECK_ERRORS
    if (0 == divider)
    {
        return MQX_INVALID_PARAMETER;
    }
    #endif

    if (hwtimer->clock_hwtimer == NULL)
        return MQX_INVALID_COMPONENT_HANDLE;

    clock_enable(hwtimer->clock_hwtimer);

    /* Select clock source first */
    EPIT_CR_REG(epit_base) &= ~(0x3 << 24);
    if (hwtimer->clock_id == EPIT_CLOCK_LP_HIGH_FREQ)
        EPIT_CR_REG(epit_base) |= (2 << 24);
    else if (hwtimer->clock_id == EPIT_CLOCK_LP_CKIL)
        EPIT_CR_REG(epit_base) |= (3 << 24);
    else
        EPIT_CR_REG(epit_base) |= (1 << 24);

    /* TODO: Low power divider recalculation */

    /* Set load register */
    EPIT_LR_REG(epit_base) = divider - 1;

    hwtimer->divider    = divider;
    hwtimer->modulo     = divider;

    clock_disable(hwtimer->clock_hwtimer);

    return MQX_OK;
}

/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief Start epit timer module
 *
 * This function enables the timer and leaves it running, timer is
 * periodically generating interrupts.
 *
 * \param hwtimer[in] Pointer to hwtimer structure.
 *
 * \return MQX_OK Success.
 *
 * \see hwtimer_epit_init
 * \see hwtimer_epit_deinit
 * \see hwtimer_epit_set_div
 * \see hwtimer_epit_stop
 * \see hwtimer_epit_get_time
 * \see hwtimer_epit_isr
 */
static _mqx_int hwtimer_epit_start(HWTIMER_PTR hwtimer)
{
    EPIT_MemMapPtr epit_base  = (EPIT_MemMapPtr) hwtimer->ll_context[0];

    if (hwtimer->clock_hwtimer == NULL)
        return MQX_INVALID_COMPONENT_HANDLE;

    clock_enable(hwtimer->clock_hwtimer);

    /* force reload counter for low power case */
    EPIT_LR_REG(epit_base) += 0;

    EPIT_CR_REG(epit_base) |= EPIT_CR_OCIEN_MASK | EPIT_CR_EN_MASK;

    return MQX_OK;
}

/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief Stop epit timer module
 *
 * Disable timer and interrupt
 *
 * \param hwtimer[in] Pointer to hwtimer structure.
 *
 * \return MQX_OK Success.
 *
 * \see hwtimer_epit_init
 * \see hwtimer_epit_deinit
 * \see hwtimer_epit_set_div
 * \see hwtimer_epit_start
 * \see hwtimer_epit_get_time
 * \see hwtimer_epit_isr
 */
static _mqx_int hwtimer_epit_stop(HWTIMER_PTR hwtimer)
{
    EPIT_MemMapPtr epit_base = (EPIT_MemMapPtr) hwtimer->ll_context[0];

    if (hwtimer->clock_hwtimer == NULL)
        return MQX_INVALID_COMPONENT_HANDLE;

    clock_enable(hwtimer->clock_hwtimer);

    if (!(EPIT_CR_REG(epit_base) & EPIT_CR_EN_MASK))
    {
        /* Return if hwtimer has been set as disabled */
        clock_disable(hwtimer->clock_hwtimer);
        return MQX_OK;
    }
    /* Disable timer and interrupt */
    EPIT_CR_REG(epit_base) &= ~(EPIT_CR_OCIEN_MASK | EPIT_CR_EN_MASK);
    /* Clear interrupt status */
    EPIT_SR_REG(epit_base) = EPIT_SR_OCIF_MASK;
    /* This disable clock is just for paired with enable clock */
    clock_disable(hwtimer->clock_hwtimer);
    /* The second disabke clock is consistent with the enable clock previous called by epit start*/
    clock_disable(hwtimer->clock_hwtimer);

    return MQX_OK;
}

/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief Atomically captures current time into HWTIMER_TIME_STRUCT structure
 *
 * Corrects/normalizes the values if necessary (interrupt pending, etc.)
 *
 * \param hwtimer[in] Pointer to hwtimer structure.
 * \param time[out]   Pointer to time structure. This value is filled with current value of the timer.
 *
 * \return MQX_OK Success.
 *
 * \see hwtimer_epit_init
 * \see hwtimer_epit_deinit
 * \see hwtimer_epit_set_div
 * \see hwtimer_epit_start
 * \see hwtimer_epit_stop
 * \see hwtimer_epit_isr
 */
static _mqx_int hwtimer_epit_get_time(HWTIMER_PTR hwtimer, HWTIMER_TIME_PTR time)
{
    EPIT_MemMapPtr   epit_base = (EPIT_MemMapPtr) hwtimer->ll_context[0];
    uint32_t         temp_cval;

    if (hwtimer->clock_hwtimer == NULL)
        return MQX_INVALID_COMPONENT_HANDLE;

    /* Disable interrupt from timer*/
    _int_disable();
    time->TICKS = hwtimer->ticks;

    clock_enable(hwtimer->clock_hwtimer);

    temp_cval = EPIT_CNR_REG(epit_base);
    /* Check pending interrupt flag */
    if (EPIT_SR_REG(epit_base) & EPIT_SR_OCIF_MASK)
    {
        _int_enable();
        time->SUBTICKS = hwtimer->modulo - 1;
    }
    else
    {
        _int_enable();
        time->SUBTICKS = EPIT_LR_REG(epit_base) - temp_cval;
    }

    clock_disable(hwtimer->clock_hwtimer);

    return MQX_OK;
}

/* EOF */
