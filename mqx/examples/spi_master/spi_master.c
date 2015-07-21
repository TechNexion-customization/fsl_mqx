/*HEADER**********************************************************************
*
* Copyright 2012 Freescale Semiconductor, Inc.
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
*   This file contains the source for a simple example of an
*   application that writes and reads the SPI memory using the SPI driver.
*   It's already configured for onboard SPI flash where available.
*
*
*END************************************************************************/


#include <string.h>
#include <mqx.h>
#include <bsp.h>
#include <spi.h>


#if ! BSPCFG_ENABLE_IO_SUBSYSTEM
#error This application requires BSPCFG_ENABLE_IO_SUBSYSTEM defined non-zero in user_config.h. Please recompile BSP with this option.
#endif


#ifndef BSP_DEFAULT_IO_CHANNEL_DEFINED
#error This application requires BSP_DEFAULT_IO_CHANNEL to be not NULL. Please set corresponding BSPCFG_ENABLE_TTYx to non-zero in user_config.h and recompile BSP with this option.
#endif


#ifndef BSP_SPI_MEMORY_CHANNEL
#error This application requires BSP_SPI_MEMORY_CHANNEL to be defined. Please set it to appropriate SPI channel number in user_config.h and recompile BSP with this option.
#endif


#if BSP_SPI_MEMORY_CHANNEL == 0

    #if ! BSPCFG_ENABLE_SPI0
        #error This application requires BSPCFG_ENABLE_SPI0 defined non-zero in user_config.h. Please recompile kernel with this option.
    #else
        #define TEST_CHANNEL "spi0:"
    #endif

#elif BSP_SPI_MEMORY_CHANNEL == 1

    #if ! BSPCFG_ENABLE_SPI1
        #error This application requires BSPCFG_ENABLE_SPI1 defined non-zero in user_config.h. Please recompile kernel with this option.
    #else
        #define TEST_CHANNEL "spi1:"
    #endif

#elif BSP_SPI_MEMORY_CHANNEL == 2

    #if ! BSPCFG_ENABLE_SPI2
        #error This application requires BSPCFG_ENABLE_SPI2 defined non-zero in user_config.h. Please recompile kernel with this option.
    #else
        #define TEST_CHANNEL  "spi2:"
    #endif

#elif BSP_SPI_MEMORY_CHANNEL == 3

    #if ! BSPCFG_ENABLE_SPI3
        #error This application requires BSPCFG_ENABLE_SPI3 defined non-zero in user_config.h. Please recompile kernel with this option.
    #else
        #define TEST_CHANNEL  "spi3:"
    #endif

#elif BSP_SPI_MEMORY_CHANNEL == 4

    #if ! BSPCFG_ENABLE_SPI4
        #error This application requires BSPCFG_ENABLE_SPI4 defined non-zero in user_config.h. Please recompile kernel with this option.
    #else
        #define TEST_CHANNEL  "spi4:"
    #endif

#elif BSP_SPI_MEMORY_CHANNEL == 5

    #if ! BSPCFG_ENABLE_SPI5
        #error This application requires BSPCFG_ENABLE_SPI5 defined non-zero in user_config.h. Please recompile kernel with this option.
    #else
        #define TEST_CHANNEL  "spi5:"
    #endif

#else

     #error Unsupported SPI channel number. Please check settings of BSP_SPI_MEMORY_CHANNEL in BSP.

#endif


/* Data definitions */
#define BURST_LENGHT    8
uint8_t rx_buffer[1]; 
uint8_t tx_buffer[1]; 

const char *device_mode[] =
{
    "SPI_DEVICE_MASTER_MODE",
    "SPI_DEVICE_SLAVE_MODE",
};

const char *clock_mode[] =
{
    "SPI_CLK_POL_PHA_MODE0",
    "SPI_CLK_POL_PHA_MODE1",
    "SPI_CLK_POL_PHA_MODE2",
    "SPI_CLK_POL_PHA_MODE3"
};

extern void main_task (uint32_t);
_mqx_int tx_rx_burst(MQX_FILE_PTR spifd, uint8_t *tx_buffer, uint8_t *rx_buffer);

const TASK_TEMPLATE_STRUCT  MQX_template_list[] =
{
    /* Task Index,   Function,   Stack,  Priority,   Name,   Attributes,          Param, Time Slice */
    { 10L,          main_task,  1500L,  8L,         "Main", MQX_AUTO_START_TASK, 0,     0  },
    { 0 }
};

/*TASK*-------------------------------------------------------------------
*
* Task Name : main_task
* Comments  :
*
*END*----------------------------------------------------------------------*/
void main_task
   (
      uint32_t dummy
   )
{
    MQX_FILE_PTR           spifd;
    uint32_t               param;
    char                   control_char;
    int32_t                i;

    printf ("\n-------------- SPI master driver example --------------\n\n");
    printf ("This example application demonstrates usage of SPI driver in master mode.\n");
    printf ("It transfers data to/from remote MCU in SPI slave mode.\n");

    /* Open the SPI driver */
    spifd = fopen (TEST_CHANNEL, NULL);

    if (NULL == spifd)
    {
        printf ("Error opening SPI driver!\n");
        _time_delay (200L);
        _task_block ();
    }

    /* Set a baud rate */
    param = 500000;
    printf ("Changing the baud rate to %d Hz ... ", param);
    if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_SET_BAUD, &param))
    {
        printf ("OK\n");
    }
    else
    {
        printf ("ERROR\n");
    }

    /* Set clock mode */
    param = SPI_CLK_POL_PHA_MODE1;
    printf ("Setting clock mode to %s ... ", clock_mode[param]);
    if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_SET_MODE, &param))
    {
        printf ("OK\n");
    }
    else
    {
        printf ("ERROR\n");
    }

    /* Set transfer mode */
    param = SPI_DEVICE_MASTER_MODE;
    printf ("Setting transfer mode to %s ... ", device_mode[param]);
    if (SPI_OK == ioctl (spifd, IO_IOCTL_SPI_SET_TRANSFER_MODE, &param))
    {
        printf ("OK\n");
    }
    else
    {
        printf ("ERROR\n");
    }

    while (TRUE) {
        printf ("Press \"s\" when spi slave is ready.\n");
        control_char = getchar();
        if ((control_char == 's') || (control_char == 'S')) {
            break;
        }
    }

    tx_buffer[0] = 0;
    for(i = 0; i < 20; i++)
    {
        tx_buffer[0]++;
        if ( MQX_OK == tx_rx_burst(spifd, tx_buffer, rx_buffer ))
        {
          printf ("MASTER: Transmited data: %d \n", tx_buffer[0]);
          printf ("      : Received data: %d \n\n", rx_buffer[0]);
        }
        /* Time window for respond generation */
        _time_delay (5);
    }
}

/*FUNCTION*---------------------------------------------------------------
*
* Function Name : tx_rx_burst
* Comments  : This function send and receive one byte to SPI bus
* Return:
*         Status read.
*
*END*----------------------------------------------------------------------*/
_mqx_int tx_rx_burst(MQX_FILE_PTR spifd, uint8_t *tx_buffer, uint8_t *rx_buffer)
{
    _mqx_int result;
    SPI_READ_WRITE_STRUCT rw;
  
    /* Write instruction and read status */
    rw.BUFFER_LENGTH = 1;
    rw.WRITE_BUFFER = (char *)tx_buffer;
    rw.READ_BUFFER = (char *)rx_buffer;

    result = ioctl (spifd, IO_IOCTL_SPI_READ_WRITE, &rw);
    if (result != SPI_OK)
    {
        printf ("ERROR (tx)\n");
    }
    else
    {
        return MQX_OK;
    }

    return MQX_ERROR;
}
