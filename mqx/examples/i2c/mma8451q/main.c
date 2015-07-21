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
*   an application that using the MMA8451Q Accelerometer from
*   freescale
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>
#include <i2c.h>
#include "mma8451q.h"

#if ! BSPCFG_ENABLE_IO_SUBSYSTEM
#error This application requires BSPCFG_ENABLE_IO_SUBSYSTEM defined non-zero in user_config.h. Please recompile BSP with this option.
#endif

#if defined(BSP_IMX6SX_SDB_M4) || defined(BSP_IMX6SX_AI_M4)
    #if BSPCFG_ENABLE_II2C3
        #define I2C_DEVICE "ii2c3:"
    #elif BSPCFG_ENABLE_I2C3
        #define I2C_DEVICE "i2c3:"
    #else
        #error This application requires BSPCFG_ENABLE_I2C3 or BSPCFG_ENABLE_II2C3 defined non-zero in user_config.h. Please recompile BSP with this option.
    #endif
#endif

#define BUFFER_SIZE 256

extern void main_task(uint32_t);

const TASK_TEMPLATE_STRUCT MQX_template_list[] =
{
   /* Task Index,   Function,  Stack,  Priority, Name,   Attributes,          Param, Time Slice */
   { 10,            main_task, 2000L,    8L,     "Main", MQX_AUTO_START_TASK, 0,     0 },
   { 0 }
};

/*TASK*-------------------------------------------------------------------
*
* Task Name : main_task
* Comments  :
*
*END*----------------------------------------------------------------------*/

void main_task(uint32_t dummy)
{
    MQX_FILE_PTR          fd;
    I2C_STATISTICS_STRUCT stats;
    _mqx_int              param, result, c;
    unsigned char        *buffer;

    /* Allocate receive buffer */
    buffer = _mem_alloc_zero(BUFFER_SIZE);
    if (buffer == NULL)
    {
        printf("ERROR getting receive buffer!\n");
        _task_block();
    }

    printf("\n\n-------------- I2C master example --------------\n\n");

    /* Open the I2C driver */
    fd = fopen(I2C_DEVICE, NULL);
    if(fd == NULL)
    {
        printf("ERROR opening the I2C driver!\n");
        _task_block();
    }

    /* Test ioctl commands */
    param = 100000;
    printf("Set current baud rate to %d ... ", param);
    if(I2C_OK == ioctl (fd, IO_IOCTL_I2C_SET_BAUD, &param))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
    }

    printf ("Get current baud rate ... ");
    if(I2C_OK == ioctl(fd, IO_IOCTL_I2C_GET_BAUD, &param))
    {
        printf("%d\n", param);
    }
    else
    {
        printf("ERROR\n");
    }

    printf("Set master mode ... ");
    if(I2C_OK == ioctl (fd, IO_IOCTL_I2C_SET_MASTER_MODE, NULL))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
    }

    printf("Get current mode ... ");
    if (I2C_OK == ioctl (fd, IO_IOCTL_I2C_GET_MODE, &param))
    {
        printf("0x%02x\n", param);
    }
    else
    {
        printf("ERROR\n");
    }

    param = 0x60;
    printf("Set station address to 0x%02x ... ", param);
    if (I2C_OK == ioctl(fd, IO_IOCTL_I2C_SET_STATION_ADDRESS, &param))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
    }

    param = 0x00;
    printf("Get station address ... ");
    if(I2C_OK == ioctl(fd, IO_IOCTL_I2C_GET_STATION_ADDRESS, &param))
    {
        printf("0x%02x\n", param);
    }
    else
    {
        printf("ERROR\n");
    }

    printf ("Clear statistics ... ");
    if(I2C_OK == ioctl(fd, IO_IOCTL_I2C_CLEAR_STATISTICS, NULL))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
    }

    printf("Get statistics ... ");
    if(I2C_OK == ioctl(fd, IO_IOCTL_I2C_GET_STATISTICS, (void *)&stats))
    {
        printf("OK\n  Interrupts:  %d\n", stats.INTERRUPTS);
        printf("  Rx packets:  %d\n", stats.RX_PACKETS);
        printf("  Tx packets:  %d\n", stats.TX_PACKETS);
        printf("  Tx lost arb: %d\n", stats.TX_LOST_ARBITRATIONS);
        printf("  Tx as slave: %d\n", stats.TX_ADDRESSED_AS_SLAVE);
        printf("  Tx naks:     %d\n", stats.TX_NAKS);
    }
    else
    {
        printf("ERROR\n");
    }

    printf("Get current state ... ");
    if (I2C_OK == ioctl(fd, IO_IOCTL_I2C_GET_STATE, &param))
    {
        printf("0x%02x\n", param);
    }
    else
    {
        printf("ERROR\n");
    }

    param = I2C_MMA8451Q_BUS_ADDRESS;
    printf("Set destination address to 0x%02x ... ", param);
    if(I2C_OK == ioctl(fd, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
    }

    param = 0x00;
    printf("Get destination address ... ");
    if (I2C_OK == ioctl(fd, IO_IOCTL_I2C_GET_DESTINATION_ADDRESS, &param))
    {
        printf("0x%02x\n", param);
    }
    else
    {
        printf("ERROR\n");
    }

    printf("Get current state ... ");
    if(I2C_OK == ioctl (fd, IO_IOCTL_I2C_GET_STATE, &param))
    {
        printf ("0x%02x\n", param);
    }
    else
    {
        printf("ERROR\n");
    }

    /* Initialize the MMA8451Q chip and select the mode */
    printf("Mma8451q_init_client ... ");
    result = mma8451q_init_client(fd);
    if(result == I2C_OK)
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
    }

    for(c=0;c<10;c++)
    {
        printf("2G MODE:\n");
        report_abs(fd);
    };

    printf("Get statistics ... ");
    if(I2C_OK == ioctl(fd, IO_IOCTL_I2C_GET_STATISTICS, (void *)&stats))
    {
        printf("OK\n  Interrupts:  %d\n", stats.INTERRUPTS);
        printf("  Rx packets:  %d\n", stats.RX_PACKETS);
        printf("  Tx packets:  %d\n", stats.TX_PACKETS);
        printf("  Tx lost arb: %d\n", stats.TX_LOST_ARBITRATIONS);
        printf("  Tx as slave: %d\n", stats.TX_ADDRESSED_AS_SLAVE);
        printf("  Tx naks:     %d\n", stats.TX_NAKS);
    }
    else
    {
        printf("ERROR\n");
    }

    printf("Get current state ... ");
    if(I2C_OK == ioctl(fd, IO_IOCTL_I2C_GET_STATE, &param))
    {
        printf ("0x%02x\n", param);
    }
    else
    {
        printf("ERROR\n");
    }

    /* Close the driver */
    result = fclose(fd);
    if(result)
    {
        printf("ERROR during close, returned: %d\n", result);
    }

    /* Free buffer */
    _mem_free(buffer);

    printf("Example finished.\n");
    _task_block();
} /* Endbody */

/* EOF */
