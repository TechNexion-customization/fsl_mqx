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
*   This file contains definitions private to the SPI driver.
*
*
*END************************************************************************/

#ifndef __spi_ecspi_prv_h__
#define __spi_ecspi_prv_h__

#include "spi.h"
#include "spi_ecspi.h"


/*--------------------------------------------------------------------------*/
/*
**                    DATATYPE DECLARATIONS
*/

/*
** ECSPI_INFO_STRUCT
** Run time state information for each spi channel
*/
typedef struct ecspi_info_struct
{
    /* Clock entry for ecspi */
    void*                              ECSPI_CLOCK;

    /* Most recently used clock configuration (cached value) */
    BSP_CLOCK_CONFIGURATION            CLOCK_CONFIG;

    /* Most recently used baudrate */
    uint32_t                           BAUDRATE;

    /* Previous clock divider to set */
    uint32_t                           PRE_DIV;

    /* Post clock divider to set */
    uint32_t                           POST_DIV;

    /* Burst length in bit */
    uint32_t                           FRAMESIZE;

    /* The spi device registers */
    ECSPI_MemMapPtr                    ECSPI_PTR;

    /* Pattern to transmit during half-duplex rx transfer */
    uint32_t                           DUMMY_PATTERN;

    /* Additional attributes for the transfer */
    uint32_t                           ATTR;

    /* Pointer to the receiver buffer */
    uint8_t                           *RX_BUF;

    /* Pointer to the transmitter buffer */
    uint8_t                           *TX_BUF;

    /* Remaining byte count for the transmission */
    uint32_t                           BYTES_LEFT;

    /* Remaining byte count for receiver */
    uint32_t                           TX_BYTES;

    /* Event to signal that transmission is done */
    LWSEM_STRUCT                       EVENT_IO_FINISHED;

    /* interrupt vector supported by the peripheral */
    uint32_t                           VECTOR;

} ECSPI_INFO_STRUCT, * ECSPI_INFO_STRUCT_PTR;


/*--------------------------------------------------------------------------*/
/*
**                        FUNCTION PROTOTYPES
*/

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif
