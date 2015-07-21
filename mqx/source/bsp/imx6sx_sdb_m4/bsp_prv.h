/*HEADER**********************************************************************
*
* Copyright 2011 Freescale Semiconductor, Inc.
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
*   required for initialization of the card.
*
*
*END************************************************************************/
#ifndef _bsp_prv_h
#define _bsp_prv_h 1

#ifdef __cplusplus
extern "C" {
#endif

/*
**  FUNCTION PROTOTYPES
*/

extern uint32_t  _bsp_get_hwticks(void *);
extern void      _bsp_exit_handler(void);
extern void      _bsp_watchdog_disable_powerdown(void);
extern void      _bsp_soft_reset(void);
extern void      _bsp_watchdog_start(uint8_t timeout);
extern void      _bsp_watchdog_service(void);
extern void      _bsp_rdc_init(void);
extern void      _bsp_rdc_sema42_lock(uint32_t pdap);
extern void      _bsp_rdc_sema42_unlock(uint32_t pdap);

/*
**  STRUCTURE DEFINITIONS
*/

extern       HWTIMER systimer;
/* I/O initialization controlled by initialization structures for each channel */
extern const IMX_UART_INIT_STRUCT _bsp_sci1_init;
extern const IMX_UART_INIT_STRUCT _bsp_sci2_init;

extern const LWADC_INIT_STRUCT lwadc1_init;
extern const LWADC_INIT_STRUCT lwadc2_init;

extern const SPI_INIT_STRUCT   _bsp_spi4_init;
extern const SPI_INIT_STRUCT   _bsp_spi5_init;

extern const IMX_I2C_INIT_STRUCT  _bsp_i2c1_init;
extern const IMX_I2C_INIT_STRUCT  _bsp_i2c2_init;
extern const IMX_I2C_INIT_STRUCT  _bsp_i2c3_init;
extern const IMX_I2C_INIT_STRUCT  _bsp_i2c4_init;

#ifdef __cplusplus
}
#endif

#endif
/* EOF */

