/*HEADER**********************************************************************
*
* Copyright 2008-2014 Freescale Semiconductor, Inc.
* Copyright 2004-2008 Embedded Access Inc.
* Copyright 1989-2008 ARC International
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
*   Revision History:
*   Date             Version  Changes
*   ---------        -------  -------
*   Jan.20/04        2.50     Initial version
*
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>
#include "fsl_flexcan_int.h"
#include "fsl_flexcan_hal.h"
#include "fsl_flexcan_prv.h"
/*!
 * @brief Enables interrupt for requested mailbox.
 *
 * @param   dev_num                FlexCAN device number.
 * @param   mailbox_number         Mailbox number.
 * @return  0 if successful; non-zero failed.
 */
uint32_t flexcan_int_enable
(
    uint8_t dev_num,
    uint32_t mailbox_number
)
{
    volatile uint32_t     index;

    if ( mailbox_number > (CAN_MCR_MAXMB (0xFFFFFFFF)) )
    {
        return (kFlexCan_INVALID_MAILBOX);
    }

    index = _bsp_get_flexcan_vector(dev_num, FLEXCAN_INT_BUF, mailbox_number);
    if (0 == index)
    {
        return (kFlexCan_INT_ENABLE_FAILED);
    }

    if (_bsp_int_init(index, FLEXCAN_MESSBUF_INT_LEVEL, FLEXCAN_MESSBUF_INT_SUBLEVEL, TRUE) !=
        MQX_OK)
    {
        return (kFlexCan_INT_ENABLE_FAILED);
    }

    return(kFlexCan_OK);
}

/*!
 * @brief Masks (disables) interrupt for requested mailbox.
 *
 * @param   dev_num                FlexCAN device number.
 * @param   mailbox_number         Mailbox number.
 * @return  0 if successful; non-zero failed.
 */
uint32_t flexcan_int_disable
(
    uint8_t dev_num,
    uint32_t mailbox_number
)
{
    volatile uint32_t     index;

    if ( mailbox_number > (CAN_MCR_MAXMB (0xFFFFFFFF)) )
    {
        return (kFlexCan_INVALID_MAILBOX);
    }

    index = _bsp_get_flexcan_vector(dev_num, FLEXCAN_INT_BUF, mailbox_number);
    if (0 == index)
    {
        return (kFlexCan_INT_DISABLE_FAILED);
    }

    // Disable the interrupt
    if (_bsp_int_init(index, FLEXCAN_MESSBUF_INT_LEVEL, FLEXCAN_MESSBUF_INT_SUBLEVEL, FALSE) !=
        MQX_OK)
    {
        return (kFlexCan_INT_DISABLE_FAILED);
    }

    return (kFlexCan_OK);
}

/*!
 * @brief Installs interrupt handler for requested mailbox.
 *
 * @param   dev_num                FlexCAN device number.
 * @param   mailbox_number         Mailbox number.
 * @param   isr                    Interrupt service routine.
 * @return  0 if successful; non-zero failed.
 */
uint32_t flexcan_install_isr
(
    uint8_t       dev_num,
    uint32_t      mailbox_number,
    INT_ISR_FPTR isr
)
{
    uint32_t   return_code = kFlexCan_OK;
    INT_ISR_FPTR result;
    volatile CAN_MemMapPtr can_reg_ptr;
    volatile uint32_t          index;

    can_reg_ptr = _bsp_get_flexcan_base_address(dev_num);
    if (NULL == can_reg_ptr)
    {
        return (kFlexCan_INVALID_ADDRESS);
    }

    if ( mailbox_number > (CAN_MCR_MAXMB (0xFFFFFFFF)) )
    {
        return (kFlexCan_INVALID_MAILBOX);
    }

    index = _bsp_get_flexcan_vector(dev_num, FLEXCAN_INT_BUF, mailbox_number);
    if (0 == index)
    {
        return (kFlexCan_INT_INSTALL_FAILED);
    }

    // Install ISR
    result = _int_install_isr(index, isr, (void *)can_reg_ptr);
    if(result == (INT_ISR_FPTR)NULL)
    {
        return_code = _task_get_error();
    }

    return return_code;
}

#if !(defined(BSP_IMX6SX_PELE_M4) || defined(BSP_IMX6SX_SDB_M4) || defined(BSP_VYBRID_AUTOEVB_A5) || defined(BSP_TWR_VF65GS10_A5) || defined(BSP_IMX6SX_AI_M4))
/*!
 * @brief Unmasks (enables) error, wake up & Bus off interrupts.
 *
 * @param   dev_num          FlexCAN device number.
 * @return  0 if successful; non-zero failed.
 */
uint32_t flexcan_error_int_enable
(
    uint8_t dev_num
)
{
    volatile uint32_t     index;

    index = _bsp_get_flexcan_vector (dev_num, FLEXCAN_INT_ERR, 0);
    if (0 == index)
    {
        return (kFlexCan_INT_ENABLE_FAILED);
    }

    if (_bsp_int_init(index, FLEXCAN_ERROR_INT_LEVEL, FLEXCAN_ERROR_INT_SUBLEVEL, TRUE) !=
        MQX_OK)
    {
        return (kFlexCan_INT_ENABLE_FAILED);
    }

    index = _bsp_get_flexcan_vector (dev_num, FLEXCAN_INT_BOFF, 0);
    if (0 == index)
    {
        return (kFlexCan_INT_ENABLE_FAILED);
    }

    if (_bsp_int_init(index, FLEXCAN_BUSOFF_INT_LEVEL, FLEXCAN_BUSOFF_INT_SUBLEVEL, TRUE) !=
        MQX_OK)
    {
        return (kFlexCan_INT_ENABLE_FAILED);
    }

    return (kFlexCan_OK);
}

/*!
 * @brief Masks (disables) error, wake up & Bus off interrupts.
 *
 * @param   dev_num           FlexCAN device number.
 * @return  0 if successful; non-zero failed.
 */
uint32_t flexcan_error_int_disable
(
    uint8_t dev_num
)
{
    volatile CAN_MemMapPtr                 can_reg_ptr;
    volatile uint32_t     index;

    can_reg_ptr = _bsp_get_flexcan_base_address(dev_num);
    if (NULL == can_reg_ptr)
    {
        return (kFlexCan_INVALID_ADDRESS);
    }

    // BOFFMSK = 0x1, ERRMSK = 0x1
    can_reg_ptr->CTRL1 &= ~(CAN_CTRL1_BOFFREC_MASK | CAN_CTRL1_ERRMSK_MASK);

    index = _bsp_get_flexcan_vector(dev_num, FLEXCAN_INT_ERR, 0);
    if (0 == index)
    {
        return (kFlexCan_INT_DISABLE_FAILED);
    }

    if (_bsp_int_init(index, FLEXCAN_ERROR_INT_LEVEL, FLEXCAN_ERROR_INT_SUBLEVEL, FALSE) !=
        MQX_OK)
    {
        return (kFlexCan_INT_DISABLE_FAILED);
    }

    index = _bsp_get_flexcan_vector (dev_num, FLEXCAN_INT_BOFF, 0);
    if (0 == index)
    {
        return (kFlexCan_INT_DISABLE_FAILED);
    }

    if (_bsp_int_init(index, FLEXCAN_BUSOFF_INT_LEVEL, FLEXCAN_BUSOFF_INT_SUBLEVEL, FALSE) !=
        MQX_OK)
    {
        return (kFlexCan_INT_DISABLE_FAILED);
    }

    return ( kFlexCan_OK );
}

/*!
 * @brief Installs interrupt handler for a flexcan error.
 *
 * @param   dev_num                FlexCAN device number.
 * @param   isr                    Interrupt service routine.
 * @return  0 if successful; non-zero failed.
 */
uint32_t flexcan_install_isr_err_int
(
    uint8_t       dev_num,
    INT_ISR_FPTR isr
)
{
    uint32_t   return_code = FLEXCAN_OK;
    INT_ISR_FPTR result;
    volatile CAN_MemMapPtr                 can_reg_ptr;
    volatile uint32_t     index;

    can_reg_ptr = _bsp_get_flexcan_base_address(dev_num);
    if (NULL == can_reg_ptr)
    {
        return (kFlexCan_INVALID_ADDRESS);
    }

    index = _bsp_get_flexcan_vector(dev_num, FLEXCAN_INT_ERR, 0);
    if (0 == index)
    {
        return (kFlexCan_INT_INSTALL_FAILED);
    }

    result = _int_install_isr(index, isr, (void *)can_reg_ptr);
    if(result == (INT_ISR_FPTR)NULL)
    {
        return_code = _task_get_error();
    }

    return return_code;
}

/*!
 * @brief Installs interrupt handler for a flexcan bus off.
 *
 * @param   dev_num                FlexCAN device number.
 * @param   isr                    Interrupt service routine.
 * @return  0 if successful; non-zero failed.
 */
uint32_t flexcan_install_isr_boff_int
(
    uint8_t       dev_num,
    INT_ISR_FPTR isr
)
{
    uint32_t   return_code = FLEXCAN_OK;
    INT_ISR_FPTR result;
    volatile CAN_MemMapPtr                 can_reg_ptr;
    volatile uint32_t     index;

    can_reg_ptr = _bsp_get_flexcan_base_address(dev_num);
    if (NULL == can_reg_ptr)
    {
        return (kFlexCan_INVALID_ADDRESS);
    }

    index = _bsp_get_flexcan_vector(dev_num, FLEXCAN_INT_BOFF, 0);
    if (0 == index)
    {
        return (kFlexCan_INT_INSTALL_FAILED);
    }

    result = _int_install_isr(index, isr, (void *)can_reg_ptr);
    if(result == (INT_ISR_FPTR)NULL)
    {
        return_code = _task_get_error();
    }

    return return_code;
}

/*!
 * @brief Installs interrupt handler for a flexcan wake-up.
 *
 * @param   dev_num                FlexCAN device number.
 * @param   isr                    Interrupt service routine.
 * @return  0 if successful; non-zero failed.
 */
uint32_t flexcan_install_isr_wake_int
(
    uint8_t       dev_num,
    INT_ISR_FPTR isr
)
{
    uint32_t   return_code = FLEXCAN_OK;
    INT_ISR_FPTR result;
    volatile CAN_MemMapPtr                 can_reg_ptr;
    volatile uint32_t     index;

    can_reg_ptr = _bsp_get_flexcan_base_address(dev_num);
    if (NULL == can_reg_ptr)
    {
        return (kFlexCan_INVALID_ADDRESS);
    }

    index = _bsp_get_flexcan_vector(dev_num, FLEXCAN_INT_WAKEUP, 0);
    if (0 == index)
    {
        return (kFlexCan_INT_INSTALL_FAILED);
    }

    result = _int_install_isr(index, isr, (void *)can_reg_ptr);
    if(result == (INT_ISR_FPTR)NULL)
    {
        return_code = _task_get_error();
    }

    return return_code;
}

#endif
