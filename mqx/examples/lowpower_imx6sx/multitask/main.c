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
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>
#include <i2c.h>
#include "test.h"

#if ! BSPCFG_ENABLE_IO_SUBSYSTEM
#error This application requires BSPCFG_ENABLE_IO_SUBSYSTEM defined non-zero in user_config.h. Please recompile BSP with this option.
#endif

#ifndef MQX_TIME_SLICE_TASK
#error This application requires MQX_TIME_SLICE_TASK to be defined in the BSP. Please recompile BSP with this option.
#endif

/* Task index define */
#define MAIN_TASK   1
#define I2C_TASK    2
#define ADC_TASK    3
#define TX_TASK     4
#define RX_TASK     5
/* Tasks function */
void main_task(uint32_t parameter);
void i2c_task(uint32_t parameter);
void adc_task(uint32_t parameter);
void flexcan_tx_task(uint32_t parameter);
void flexcan_rx_task(uint32_t parameter);

void init_flexcan(void);

/* global variable*/
LWSEM_STRUCT   lwsem;

const TASK_TEMPLATE_STRUCT MQX_template_list[] =
{
   /* Task Index,   Function,  Stack,  Priority, Name,   Attributes,          Param, Time Slice */
   { MAIN_TASK,           main_task,              2000L,    7L,     "Main",               MQX_AUTO_START_TASK, 0,     0 },
   { I2C_TASK,            i2c_task,               2000L,    8L,     "I2c",                MQX_TIME_SLICE_TASK, 0,     0 },
   { ADC_TASK,            adc_task,               2000L,    8L,     "Adc",                MQX_TIME_SLICE_TASK, 0,     0 },
   { TX_TASK,             flexcan_tx_task,        2000L,    8L,  "flexcan tx",            MQX_TIME_SLICE_TASK, 0,     0 },
   { RX_TASK,             flexcan_rx_task,        2000L,    8L,  "flexcan rx",            MQX_TIME_SLICE_TASK, 0,     0 },
   { 0 }
};

uint32_t PRINT(const char* format,...)
{
   va_list arg;
   int done = 0;

   _lwsem_wait(&lwsem);
   va_start(arg, format);
   done = vprintf(format, arg);
   va_end(arg);
   _lwsem_post(&lwsem);

   return done;
}

/*TASK*-------------------------------------------------------------------
*
* Task Name : main_task
* Comments  :
*
*END*----------------------------------------------------------------------*/
void main_task(uint32_t dummy)
{
    uint8_t               i,index;
    _task_id              created_task;

    _lwsem_create(&lwsem, 1);
    PRINT(" %d\t Low power stress test starting... \n", _task_get_index_from_id(_task_get_id()));
    init_flexcan();

    for (i=I2C_TASK; i<=ADC_TASK; i++)
    {
        created_task = _task_create(0, i, 0);
        if (created_task == MQX_NULL_TASK_ID)
        {
            PRINT("\n %d\t %s task creation failed.", _task_get_index_from_id(_task_get_id()), MQX_template_list[i-1].TASK_NAME);
        }
    }

    for (i=TX_TASK; i<=RX_TASK; i++)
    {
        for (index=0; index<=1; index++)
        {
            /* create tx/rx task for instance 1 and 2 */
            created_task = _task_create(0, i, index);
            if (created_task == MQX_NULL_TASK_ID)
            {
                PRINT("\n %d\t %s task creation failed.", _task_get_index_from_id(_task_get_id()), MQX_template_list[i-1].TASK_NAME);
            }
        }
    }

    _task_block();
} /* Endbody */

/* EOF */

