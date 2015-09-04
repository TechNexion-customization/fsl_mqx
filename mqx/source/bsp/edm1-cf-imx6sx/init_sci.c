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
*   This file contains the definition for the baud rate for the serial
*   channel
*
*
*END************************************************************************/

#include "mqx.h"
#include "bsp.h"

const IMX_UART_INIT_STRUCT _bsp_sci1_init = {
   /* queue size         */ BSPCFG_SCI1_QUEUE_SIZE,
   /* Channel            */ 1,
   /* Clock Speed        */ BSP_PLL3_UART_CLOCK,
   /* Baud rate          */ BSPCFG_SCI1_BAUD_RATE,
   /* RX/TX Int vect     */ INT_UART1,
   /* ERR Int vect       */ 0, //INT_UART1_ERR,
   /* RX/TX priority     */ 3,
   /* ERR priority       */ 4,
};

const IMX_UART_INIT_STRUCT _bsp_sci2_init = {
   /* queue size         */ BSPCFG_SCI2_QUEUE_SIZE,
   /* Channel            */ 2,
   /* Clock Speed        */ BSP_PLL3_UART_CLOCK,
   /* Baud rate          */ BSPCFG_SCI2_BAUD_RATE,
   /* RX/TX Int vect     */ INT_UART2,
   /* ERR Int vect       */ 0, //INT_UART2_ERR,
   /* RX/TX priority     */ 3,
   /* ERR priority       */ 4,
};
