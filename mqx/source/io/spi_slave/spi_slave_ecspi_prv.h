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
*   This file contains definitions private to the SPI slave driver.
*
*
*END************************************************************************/

#ifndef __SPI_SLAVE_ECSPI_PRV_H__
#define __SPI_SLAVE_ECSPI_PRV_H__

#include "spi_slave_ecspi.h"

/*--------------------------------------------------------------------------*/
/*
**                        FUNCTION PROTOTYPES
*/

#ifdef __cplusplus
extern "C" {
#endif

extern void     *_bsp_get_ecspi_base_address(uint32_t cs);
extern uint32_t  _bsp_get_ecspi_vector(uint32_t cs);
extern CLOCK_NAME _bsp_get_ecspi_clock_name(uint32_t channel);

#ifdef __cplusplus
}
#endif

#endif
