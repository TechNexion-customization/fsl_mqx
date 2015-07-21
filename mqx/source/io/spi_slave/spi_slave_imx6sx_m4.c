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
*   This file contains board-specific ECSPI initialization functions. 
*
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>
#include <stdint.h>
#include "spi_slave_ecspi.h"


static const ECSPI_MemMapPtr ecspi_address[] = ECSPI_BASE_PTRS;

static const uint32_t ecspi_vectors[] =
{
    INT_eCSPI1,
    INT_eCSPI2,
    INT_eCSPI3,
    INT_eCSPI4,
    INT_eCSPI5
};

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_get_ecspi_base_address
* Returned Value   : Address upon success, NULL upon failure
* Comments         :
*    This function returns the base register address of the corresponding SPI
*    module.
*
*END*----------------------------------------------------------------------*/

void *_bsp_get_ecspi_base_address(uint32_t cs)
{
    /* channel start from 1 */
    if (cs > 0 && cs <= ELEMENTS_OF(ecspi_address)) {
        return (void *)ecspi_address[cs - 1];
    }
    return NULL;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_get_ecspi_vectors
* Returned Value   : vector number associated with the peripheral
* Comments         :
*    This function returns desired interrupt vector table index for specified
*    SPI module.
*
*END*----------------------------------------------------------------------*/

uint32_t _bsp_get_ecspi_vector(uint32_t cs)
{
    /* channel start from 1 */
    if (cs > 0 && cs <= ELEMENTS_OF(ecspi_vectors)) {
        return ecspi_vectors[cs - 1];
    }
    return 0;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_get_ecspi_clock_name
* Returned Value   : clock name if successful, CLK_MAX otherwise
* Comments         :
*    This function returns desired clock name for specified SPI module.
*
*END*----------------------------------------------------------------------*/

CLOCK_NAME _bsp_get_ecspi_clock_name(uint32_t channel)
{
   CLOCK_NAME clk_name;

   switch (channel) {
      case 1:
        clk_name = CLK_ECSPI1;
        break;
      case 2:
        clk_name = CLK_ECSPI2;
        break;
      case 3:
        clk_name = CLK_ECSPI3;
        break;
      case 4:
        clk_name = CLK_ECSPI4;
        break;
      case 5:
        clk_name = CLK_ECSPI5;
        break;
      default:
        clk_name = CLK_MAX;
   }

   return clk_name;
}

