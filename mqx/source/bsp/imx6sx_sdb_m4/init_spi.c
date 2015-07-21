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
*   This file contains the initialization definition for ECSPI driver
*
*
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>

/* Maximum bits of SPI burst. Because some flash commands must finish in one CS
   selection while ECSPI CS is controlled by SPI burst, thus we must try our best
   to put as much data as possible in one burst */
#define MAX_FRAME_SIZE        (4096)

/*
** Parameters for SPI initialization
*/
#if BSPCFG_ENABLE_SPI4
static const ECSPI_INIT_STRUCT _bsp_ecspi4_init = {
    4,                            /* SPI channel */
    CLK_ECSPI4                   /* Relevant module clock name */
};

const SPI_INIT_STRUCT _bsp_spi4_init = {
    &_spi_ecspi_devif,             /* Low level driver interface */
    &_bsp_ecspi4_init,             /* Low level driver init data */
    {                              /* Default parameters: */
        10000000,                  /* Baudrate */
        SPI_CLK_POL_PHA_MODE0,     /* Mode */
        MAX_FRAME_SIZE,            /* Frame size in bits */
        3,                         /* Chip select */
        0,                         /* Attributes */
        0xFFFFFFFF                 /* Dummy pattern */
    }
};
#endif

#if BSPCFG_ENABLE_SPI5
static const ECSPI_INIT_STRUCT _bsp_ecspi5_init = {
    5,                            /* SPI channel */
    CLK_ECSPI5                    /* Relevant module clock source */
};

const SPI_INIT_STRUCT _bsp_spi5_init = {
    &_spi_ecspi_devif,             /* Low level driver interface */
    &_bsp_ecspi5_init,             /* Low level driver init data */
    {                              /* Default parameters: */
        10000000,                  /* Baudrate */
        SPI_CLK_POL_PHA_MODE0,     /* Mode */
        MAX_FRAME_SIZE,            /* Frame size in bits */
        0,                         /* Chip select */
        0,                         /* Attributes */
        0xFFFFFFFF                 /* Dummy pattern */
    }
};
#endif
