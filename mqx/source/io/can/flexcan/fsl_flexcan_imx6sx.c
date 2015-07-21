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
#include "fsl_flexcan_prv.h"
////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
static flexcan_dev_info_t flexcan_dev_info[2];
/*!
 * @brief Returns pointer to base address of the desired CAN device.
 *
 * @param   dev_num      FlexCAN device number.
 * @return  Pointer to desired CAN device or NULL if not present.
 */
void *_bsp_get_flexcan_base_address
(
    uint8_t dev_num
)
{
    void   *addr;

    switch(dev_num) {
    case 1:
        addr = (void *)CAN1_BASE_PTR;
        break;
    case 2:
        addr = (void *)CAN2_BASE_PTR;
        break;
    default:
        addr = NULL;
    }

    return addr;
}

/*FUNCTION****************************************************************
*
* Function Name    : _bsp_get_flexcan_vector
* Returned Value   : MQX vector number for specified interrupt
* Comments         :
*    This function returns index into MQX interrupt vector table for
*    specified flexcan interrupt. If not known, returns 0.
*
*END*********************************************************************/
/*!
 * @brief Returns index into MQX interrupt vector table for specified flexcan interrupt.
 *
 * @param   dev_num             FlexCAN device number.
 * @param   vector_type         FlexCAN interrupt vector type.
 * @param   vector_index        FlexCAN interrupt vector index.
 * @return  MQX vector number for specified interrupt. If not known, returns 0.
 */
uint32_t _bsp_get_flexcan_vector
(
    uint8_t dev_num,
    uint8_t vector_type,
    uint32_t vector_index
)
{
    uint32_t index = (uint32_t)0;

    switch (dev_num)
    {
       case 1:
           index = (uint32_t)INT_FLEXCAN1;
           break;
        case 2:
            index = (uint32_t)INT_FLEXCAN2;
            break;
      default: break;
    }

    return index;
}

/*!
 * @brief Returns clock name of the desired CAN device.
 *
 * @param   dev_num      FlexCAN device number.
 * @return  clock name to desired CAN device or CLK_MAX if not present.
 */
CLOCK_NAME _bsp_get_flexcan_clock_name
(
   uint8_t dev_num
)
{
   CLOCK_NAME clk_name;
   switch (dev_num)
   {
     case 1:
       clk_name = CLK_FLEXCAN1;
       break;
     case 2:
       clk_name = CLK_FLEXCAN2;
       break;
     default:
       clk_name = CLK_MAX;
   }

   return clk_name;
}

/*!
 * @brief Returns ipg clock name of the desired CAN device.
 *
 * @param   dev_num      FlexCAN device number.
 * @return  ipg clock name to desired CAN device or CLK_MAX if not present.
 */
CLOCK_NAME _bsp_get_flexcan_clock_ipg_name
(
   uint8_t dev_num
)
{
   CLOCK_NAME clk_ipg_name;
   switch (dev_num)
   {
     case 1:
       clk_ipg_name = CLK_FLEXCAN1_IPG;
       break;
     case 2:
       clk_ipg_name = CLK_FLEXCAN2_IPG;
       break;
     default:
       clk_ipg_name = CLK_MAX;
   }

   return clk_ipg_name;
}

/*!
 * @brief Returns device context of the desired CAN device.
 *
 * @param   dev_num      FlexCAN device number.
 * @return  device context to desired CAN device or NULL if not present.
 */
flexcan_dev_info_ptr _bsp_get_flexcan_dev_info(
   uint8_t dev_num)
{
   flexcan_dev_info_ptr flexcan_info_ptr;
   switch (dev_num)
   {
       case 1:
           flexcan_info_ptr = &flexcan_dev_info[0];
           break;
       case 2:
           flexcan_info_ptr = &flexcan_dev_info[1];
           break;
       default:
           flexcan_info_ptr = NULL;
   }

   return flexcan_info_ptr;
}

/*!
 * @brief Request FlexCAN to enter stop mode.
 *
 * @param   dev_num      FlexCAN device number.
 * @return  0 on success
 */
uint32_t _bsp_flexcan_enter_stop_mode(
   uint8_t dev_num)
{
    switch (dev_num)
    {
        case 1:
            IOMUXC_GPR_GPR4 |= IOMUXC_GPR_GPR4_CAN1_STOP_REQ_MASK;
            break;
        case 2:
            IOMUXC_GPR_GPR4 |= IOMUXC_GPR_GPR4_CAN2_STOP_REQ_MASK;
            break;
	default:
            return 1;
    }

    return 0;
}

/*!
 * @brief Request FlexCAN to exit stop mode.
 *
 * @param   dev_num      FlexCAN device number.
 * @return  0 on success
 */

uint32_t _bsp_flexcan_exit_stop_mode(
   uint8_t dev_num)
{
    switch (dev_num)
    {
        case 1:
            IOMUXC_GPR_GPR4 &= ~IOMUXC_GPR_GPR4_CAN1_STOP_REQ_MASK;
            break;
        case 2:
            IOMUXC_GPR_GPR4 &= ~IOMUXC_GPR_GPR4_CAN2_STOP_REQ_MASK;
            break;
	default:
            return 1;
    }

    return 0;
}

/* EOF */
