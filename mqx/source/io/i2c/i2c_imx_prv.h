#ifndef _i2c_imx_prv_h
#define _i2c_imx_prv_h 1
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
*   This file contains the definitions of constants and structures
*   required for the I2C drivers for the I.MX family.
*
*
*END************************************************************************/

#include "i2c_imx.h"

/*--------------------------------------------------------------------------*/
/*
**                    DATATYPE DECLARATIONS
*/

/*
** IMX_I2C_INFO_STRUCT
** Run time state information for each serial channel
*/
typedef struct imx_i2c_info_struct
{
   /* Current initialized values */
   IMX_I2C_INIT_STRUCT               INIT;

   /* Peripheral register structure */
   I2C_MemMapPtr                     I2C_PTR;

   /* Clock entry for I2C module */
   void*                             CLOCK_I2C;

   /* The previous interrupt handler and data */
   void                              (_CODE_PTR_ OLD_ISR)(void *);

   void                             *OLD_ISR_DATA;

   /* Interrupt vector */
   uint32_t                          VECTOR;

   /* Actual mode */
   uint8_t                           MODE;

   /* Actual state */
   uint8_t                           STATE;

   /* Destination address */
   uint8_t                           ADDRESSEE;

   /* Operation flags */
   uint8_t                           OPERATION;

   /* Number of bytes requested for receive */
   uint32_t                          RX_REQUEST;

   /* Pointer to the buffer to use for Tx/Rx data */
   unsigned char                    *RX_BUFFER;

   /* Rx index */
   uint32_t                          RX_INDEX;

   /* Rx buffer size */
   uint32_t                          RX_BUFFER_SIZE;

   /* Pointer to the buffer to use for current Tx data */
   unsigned char                    *TX_BUFFER;

   /* Tx index */
   uint32_t                          TX_INDEX;

   /* Tx buffer size */
   uint32_t                          TX_BUFFER_SIZE;

   /* I2C internal synchronize lock */
   LWSEM_STRUCT                      LWSEM;

   /* Statistical information */
   I2C_STATISTICS_STRUCT             STATISTICS;

} IMX_I2C_INFO_STRUCT, * IMX_I2C_INFO_STRUCT_PTR;

#endif
/* EOF */
