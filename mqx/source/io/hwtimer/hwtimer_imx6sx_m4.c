
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
* See license agreement file for full license terms including other restrictions.
*****************************************************************************
*
* Comments:
*
*   This file contains imx6sx cortext M4 specific functions of the hwtimer
*   component.
*
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>
#include "hwtimer.h"

/*!
 * \cond DOXYGEN_PRIVATE
 * \brief Array of EPIT register map base address
 */
static EPIT_MemMapPtr epit_base_tab[] = EPIT_BASE_PTRS;

/*!
 * \cond DOXYGEN_PRIVATE
 * \brief Array of EPIT interrupt vectors
 */
const _mqx_int epit_vectors_table[2] = { INT_EPIT1, INT_EPIT2 };

/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief This function get base memory map base for specified EPIT module.
 *
 * \param epit_number Number of EPIT module.
 *
 * \return addr memory map base for specified EPIT module.
 *
 * \see epit_io_init
 */
void *epit_get_base_address
(
    uint32_t epit_number
)
{
    switch (epit_number)
    {
        case 1:
            return (void *)epit_base_tab[0];
        case 2:
            return (void *)epit_base_tab[1];
        default:
            return NULL;
    }
}

/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief This function get vector for specified EPIT module.
 *
 * \param epit_number Number of EPIT module.
 *
 * \return vector vector for specified EPIT module.
 *
 * \see epit_io_init
 */
 _mqx_int epit_get_vector
(
    uint32_t epit_number
)
{
    switch (epit_number)
    {
        case 1:
            return epit_vectors_table[0];
        case 2:
            return epit_vectors_table[1];
        default:
            return 0;
    }
}

/*******************SysTick********************/

/*!
 * \cond DOXYGEN_PRIVATE
 *
 * \brief This function get Interrupt Number.
 *
 * \param pit_vectors_table_ptr[out]  Used to get pit_vectors_table.
 *
 * \return Interrupt Number for SysTick module.
 *
 */
uint32_t systick_get_vector()
{
    return INT_SysTick;
}


/*FUNCTION****************************************************************
*
* Function Name    : epit_get_clock_name
* Returned Value   : clock name if successful, CLK_MAX otherwise
* Comments         :
*    This function returns desired clock name for specified epit device.
*
*END*********************************************************************/
CLOCK_NAME epit_get_clock_name(uint32_t epit_id)
{
   CLOCK_NAME clk_name;

   switch (epit_id) {
      case 1:
        clk_name = CLK_EPIT1;
        break;
      case 2:
        clk_name = CLK_EPIT2;
        break;
      default:
        clk_name = CLK_MAX;
   }

   return clk_name;
}



/*FUNCTION****************************************************************
*
* Function Name    : systick_get_clock_name
* Returned Value   : clock name if successful, CLK_MAX otherwise
* Comments         :
*    This function returns desired clock name for specified systick device.
*
*END*********************************************************************/
CLOCK_NAME systick_get_clock_name(uint32_t systick_id)
{
   CLOCK_NAME clk_name;

   switch (systick_id) {
      case 0:
        clk_name = CLK_M4;
        break;
      default:
        clk_name = CLK_MAX;
   }

   return clk_name;
}
/* EOF */
