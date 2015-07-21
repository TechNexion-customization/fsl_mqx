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
*   This file contains chip-specific lwgpio functions.
*
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>
#include <bsp_prv.h>

#include <lwgpio_igpio.h>

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_lock_lwgpio
* Returned Value   : void
* Comments         :
*    Lock the gpio port with SEMA42 if required
*
*END*----------------------------------------------------------------------*/
void _bsp_lock_lwgpio(uint32_t port)
{
    _bsp_rdc_sema42_lock(RDC_PDAP_GPIO1_ID + port - 1);
}

/*FUNCTION****************************************************************
*
* Function Name    : _bsp_unlock_lwgpio
* Returned Value   : void
* Comments         :
*   Unlock the gpio port with SEMA42 if required
*
*END*********************************************************************/

void _bsp_unlock_lwgpio(uint32_t port)
{
    _bsp_rdc_sema42_unlock(RDC_PDAP_GPIO1_ID + port - 1);
}

/* EOF */
