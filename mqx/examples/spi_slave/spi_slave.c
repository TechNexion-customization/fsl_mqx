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
*   This file contains the source for a simple example of an
*   application that responds to SPI master device using the SPI 
*   slave driver.
*
*
*END************************************************************************/

#include <string.h>
#include <mqx.h>
#include <bsp.h>
#include <spi_slave_ecspi.h>

#define FRAME_SIZE    8
#define FIRST_DATA    0xFF
unsigned char rx_buffer;
unsigned char tx_buffer;
ECSPI_SLAVE_INFO_STRUCT_PTR ecspi_info_ptr;
LWSEM_STRUCT  TRANSFER_COMPLETE;  /* Bus synchronization semaphore */

_mqx_int data_transfer(void *app_data_ptr, uint32_t *tx_data_ptr, uint32_t *rx_data_ptr);

/* Initialization structure */
ECSPI_SLAVE_INFO_STRUCT info =
{
    /* User's section */
    4,                           // INSTANCE.
    FRAME_SIZE,                  // FRAME_SIZE.
    3,                           // CS.
    0,                           // SS_POL, Slave select active state: 0-low, 1-high.
    0,                           // SS_CTL, Burst end by: 0-number of bits, 1-SS edge.
    SPI_CLK_POL_PHA_MODE1,       // MODE, clock phase and polarity settings.
    data_transfer,               // CALLBACK, Callback pointer.
    NULL,                        // APP_DATA_PTR, Optional application data for callback.
    /* System section. */
    NULL,                        // ECSPI_REG_PTR, Pointer to register access structure. Initialize by NULL.
    NULL                         // ECSPI_CLOCK, slave ecspi clock node. Initialize by NULL.
};

extern void main_task (uint32_t);

const TASK_TEMPLATE_STRUCT  MQX_template_list[] =
{
    /* Task Index,   Function,   Stack,  Priority,   Name,   Attributes,          Param, Time Slice */
    { 10L,          main_task,  1500L,  8L,         "Main", MQX_AUTO_START_TASK, 0,     0  },
    { 0 }
};


/*TASK*-------------------------------------------------------------------
*
* Task Name : main_task
* Comments  : It initialize ECSPI driver in slave mode and responding to 
*             data requists from master device.
*
*END*----------------------------------------------------------------------*/
void main_task
   (
      uint32_t dummy
   )
{
    uint32_t i = 0;

    printf ("\n-------------- ECSPI slave driver example --------------\n\n");
    printf ("This example application demonstrates usage of ECSPI slave driver.\n");
    printf ("It responding to master via SPI bus.\n\n");

    /* Allocate ecspi_info_ptr */
    /* Initialize internal info data */
    ecspi_info_ptr = (ECSPI_SLAVE_INFO_STRUCT_PTR)_mem_alloc_system_zero((uint32_t)sizeof(ECSPI_SLAVE_INFO_STRUCT));
    if (ecspi_info_ptr == NULL)
    {
        printf("\nError: MQX_OUT_OF_MEMORY.");
        _task_block();
    }

    /* Set memmory type for task aware debugger */
    if (!(_mem_set_type(ecspi_info_ptr, MEM_TYPE_IO_ECSPI_SLAVE_INFO_STRUCT)))
    {
        printf("\nError: MEM TYPE issue.");
        _task_block();
    }

    /* Create synchronization object */
    if (_lwsem_create(&TRANSFER_COMPLETE, 0) != MQX_OK) {
        printf("\n_lwsem_create failed");
        _task_block();
    }

    /* Fill dummy data to tx buffer for first burst. It fill tx data register in init function. */
    tx_buffer = FIRST_DATA;

    /* Initialize ECSPI driver */
    if (ecspi_slave_init(&info) != MQX_OK) {
        printf("\nspi_slave_init failed");
        _task_block();
    }
    
    printf ("SLAVE: Initial transmit data: %d\n\n", tx_buffer);
    
    tx_buffer = 0;

    for (i=0; i<20 ;i++)
    {
        /* Wait for transfer */
        if (_lwsem_wait(&TRANSFER_COMPLETE) != MQX_OK) {
            printf("\n_lwsem_wait failed");
            _task_block();
        }
        printf ("SLAVE: Next step transmit data: %d\n", tx_buffer);
        printf ("     : Currently received data: %d\n\n", rx_buffer);
        tx_buffer = tx_buffer+1;
    }

    /* shutdown ECSPI driver */
    ecspi_slave_shutdown(&info);

    /* Destroy semaphor */
    _lwsem_destroy(&TRANSFER_COMPLETE);

    /* return memmory */
    _mem_free(ecspi_info_ptr);
}


/*FUNCTION**********************************************************************
*
* Function Name    : data_transfer
* Returned Value   : MQX status code
* Comments         : ECSPI driver callback.
*
* param1: Pointer to tx_byte.
* param2: Pointer to rx_byte.
*
*END***************************************************************************/
_mqx_int data_transfer(void *app_data_ptr, uint32_t *tx_data_ptr, uint32_t *rx_data_ptr)
{
    if(rx_data_ptr == NULL)
    {
        /* Fill tx data reigster by first word before first transmission */
        /* This will be executed in ecspi_slave_init function */
        *(uint8_t *)tx_data_ptr = tx_buffer;
    }
    else
    {
        /* Handle interrupt event */
        /* This will be executed in ISR */
        rx_buffer = *(uint8_t *)rx_data_ptr;
        *(uint8_t *)tx_data_ptr = tx_buffer;
        _lwsem_post(&TRANSFER_COMPLETE);
    }
    return MQX_OK;
}
