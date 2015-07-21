/*HEADER**********************************************************************
*
*Copyright 2014 Freescale Semiconductor, Inc.
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
*   This file contains board-specific I2C initialization functions.
*
*
*END************************************************************************/


#include <mqx.h>
#include <bsp.h>
#include "i2c_imx_prv.h"

/*FUNCTION****************************************************************
*
* Function Name    : _bsp_get_i2c_base_address
* Returned Value   : address if successful, NULL otherwise
* Comments         :
*    This function returns the base register address of the corresponding I2C device.
*
*END*********************************************************************/
void * _bsp_get_i2c_base_address(uint8_t channel)
{
   void   *addr;

   switch (channel)
   {
      case 1:
         addr = (void *) I2C1_BASE_PTR;
         break;
      case 2:
         addr = (void *) I2C2_BASE_PTR;
         break;
      case 3:
         addr = (void *) I2C3_BASE_PTR;
         break;
      case 4:
         addr = (void *) I2C4_BASE_PTR;
         break;
      default:
        addr = NULL;
        break;
    }
    return addr;
}



/*FUNCTION****************************************************************
*
* Function Name    : _bsp_get_i2c_vector
* Returned Value   : vector number if successful, 0 otherwise
* Comments         :
*    This function returns desired interrupt vector number for specified I2C device.
*
*END*********************************************************************/

uint32_t _bsp_get_i2c_vector(uint8_t channel)
{
    switch (channel)
    {
      case 1: return (INT_I2C1);
      case 2: return (INT_I2C2);
      case 3: return (INT_I2C3);
      case 4: return (INT_I2C4);
    }
    return 0;
}


/*FUNCTION****************************************************************
*
* Function Name    : _bsp_get_i2c_clock_name
* Returned Value   : clock name if successful, CLK_MAX otherwise
* Comments         :
*    This function returns desired clock name for specified I2C device.
*
*END*********************************************************************/
CLOCK_NAME _bsp_get_i2c_clock_name(uint8_t channel)
{
    CLOCK_NAME clk_name;

    switch (channel) {
       case 1:
          clk_name = CLK_I2C1;
          break;
       case 2:
          clk_name = CLK_I2C2;
          break;
       case 3:
          clk_name = CLK_I2C3;
          break;
       case 4:
          clk_name = CLK_I2C4;
          break;
       default:
          clk_name = CLK_MAX;
    }

    return clk_name;
}

/* EOF */
