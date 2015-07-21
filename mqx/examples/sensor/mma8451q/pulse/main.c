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
    #if defined(BSP_IMX6SX_SDB_M4)
            #define MMA8451Q_INT_GPIO_PIN   LWGPIO_PIN_SD1_DATA0
            #define MMA8451Q_INT_GPIO_MUX   LWGPIO_MUX_SD1_DATA0_GPIO
            #define MMA8451Q_INT_ROUTE      MMA8451Q_PULSE_INT_ROUTE_TO_INT2
    #else // defined BSP_IMX6SX_AI_M4
            #define MMA8451Q_INT_GPIO_PIN   LWGPIO_PIN_LCD1_DATA23
            #define MMA8451Q_INT_GPIO_MUX   LWGPIO_MUX_LCD1_DATA23_GPIO
            #define MMA8451Q_INT_ROUTE      MMA8451Q_PULSE_INT_ROUTE_TO_INT1
    #endif
#endif

#define BUFFER_SIZE 256

void main_task(uint32_t);
static void int_service_routine(void *);

const TASK_TEMPLATE_STRUCT MQX_template_list[] =
{
   /* Task Index,   Function,  Stack,  Priority, Name,   Attributes,          Param, Time Slice */
   { 10,            main_task, 2000L,    8L,     "Main", MQX_AUTO_START_TASK, 0,     0 },
   { 0 }
};

/* Global variables */
static LWSEM_STRUCT lwsem;

/******************************************************************************
*
* Functio Name      : int_service_routine
* Comments          : The interrupt service routine triggered by ACC_INT pin
*
******************************************************************************/
static void int_service_routine(void *pin)
{
    lwgpio_int_clear_flag((LWGPIO_STRUCT_PTR) pin);
    _lwsem_post(&lwsem);
}

/*TASK*-------------------------------------------------------------------
*
* Task Name : main_task
* Comments  :
*
*END*----------------------------------------------------------------------*/

void main_task(uint32_t dummy)
{
    uint8_t                *buffer = NULL;
    _mqx_int                param, i;
    _mqx_int                result;
    MQX_FILE_PTR            fd;
    LWGPIO_STRUCT           acc_int;
    void                   *mma8451q_handle = NULL;
    MMA8451Q_INIT_STRUCT    mma8451q_init_str = {
                                                   .SLAVE_ADDRESS        = MMA8451Q_ADDRESS_SA0_LOW,
                                                   .OUTPUT_DATA_RATE     = MMA8451Q_OUTPUT_DATA_RATE_400HZ,
                                                   .FULL_SCALE_RANGE     = MMA8451Q_FULL_SCALE_RANGE_4G,
                                                   .ACTIVE_POWER_SCHEME  = MMA8451Q_ACTIVE_POWER_SCHEME_NORMAL,
                                                   .BURST_READ_MODE      = MMA8451Q_BURST_READ_MODE_NORMAL,
                                                };

    printf("\n\n-------------- MMA8451Q sensor driver pulse detection example --------------\n\n");

    /* Allocate receive buffer */
    buffer = _mem_alloc_zero(BUFFER_SIZE);
    if (buffer == NULL)
    {
        printf("ERROR getting receive buffer!\n");
        _task_block();
    }

    /* Open the I2C driver */
    fd = fopen(I2C_DEVICE, NULL);
    if (fd == NULL)
    {
        printf("ERROR opening the I2C driver!\n");
        _task_block();
    }

    /* Set bus speed */
    param = 100000;
    printf("Set current baud rate to %d ... ", param);
    if (I2C_OK == ioctl(fd, IO_IOCTL_I2C_SET_BAUD, &param))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
    }

    printf("Set master mode ... ");
    if (I2C_OK == ioctl(fd, IO_IOCTL_I2C_SET_MASTER_MODE, NULL))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
    }

    /* Create the lightweight semaphore */
    result = _lwsem_create(&lwsem, 0);
    if (result != MQX_OK) {
        printf("\nCreating sem failed: 0x%X", result);
        _task_block();
    }

    printf("Mma8451q_init ... ");
    mma8451q_handle = mma8451q_init(&mma8451q_init_str, fd);
    if (mma8451q_handle != NULL)
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    // Initialize Motion detection
    printf("Set_pulse_detect_state... ");
    if(mma8451q_set_pulse_detect_state(mma8451q_handle, MMA8451Q_SINGLE_PULSE_ENABLE_Z))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Set_pulse_threshold... ");
    if(mma8451q_set_pulse_threshold(mma8451q_handle, 42, MMA8451Q_PULSE_AXIS_Z))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Set_pulse_time_limit... ");
    if(mma8451q_set_pulse_time_limit(mma8451q_handle, 0x50))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Set_pulse_latency... ");
    if(mma8451q_set_pulse_latency(mma8451q_handle, 0xF0))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    /******************************************************************************
        Open the pin for input, initialize interrupt
        and set interrupt handler.
    ******************************************************************************/
    /* opening pins for input */
    if (!lwgpio_init(&acc_int, MMA8451Q_INT_GPIO_PIN, LWGPIO_DIR_INPUT, LWGPIO_VALUE_NOCHANGE))
    {
        printf("Initializing acc_int GPIO as input failed.\n");
        _task_block();
    }

    lwgpio_set_functionality(&acc_int, MMA8451Q_INT_GPIO_MUX);
    lwgpio_set_attribute(&acc_int, LWGPIO_ATTR_PULL_UP, LWGPIO_AVAL_ENABLE);

    /* enable gpio functionality for given pin, react on falling edge */
    if (!lwgpio_int_init(&acc_int, LWGPIO_INT_MODE_FALLING))
    {
        printf("Initializing acc_int GPIO for interrupt failed.\n");
        _task_block();
    }

    /* install gpio interrupt service routine */
    _int_install_isr(lwgpio_int_get_vector(&acc_int), int_service_routine, (void *) &acc_int);
    /* set the interrupt level, and unmask the interrupt in interrupt controller */
    _bsp_int_init(lwgpio_int_get_vector(&acc_int), 3, 0, TRUE);

    printf("Set interrupt output mode to push pull...\n");
    if (!mma8451q_set_int_output_mode(mma8451q_handle, MMA8451Q_INT_MODE_PUSH_PULL))
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Set interrupt polarity to active low...\n");
    if (!mma8451q_set_int_polarity(mma8451q_handle, MMA8451Q_INT_POLARITY_ACTIVE_LOW))
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Route pulse detection interrupt to Int pin...\n");
    if (!mma8451q_set_int_pin_route(mma8451q_handle, MMA8451Q_INT_ROUTE))
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Enable pulse detection interrupt ...\n");
    if (!mma8451q_set_int_state(mma8451q_handle, MMA8451Q_PULSE_INT_ENABLE))
    {
        printf("ERROR\n");
        _task_block();
    }

    /* enable interrupt on GPIO peripheral module */
    lwgpio_int_enable(&acc_int, TRUE);

    mma8451q_set_operating_mode(mma8451q_handle, MMA8451Q_OPERATING_MODE_ACTIVE);

    printf("\nTap the board to trigger pulse detection\n");

    for (i = 0; i < 10; i++)
    {
        /* wait for mma8451q interrupt, lwsem is set in lwgpio isr */
        _lwsem_wait(&lwsem);

        buffer[0] = 0x00;
        mma8451q_get_int_source(mma8451q_handle, buffer);
        if (buffer[0] == MMA8451Q_INT_SOURCE_PULSE)
        {
            /* Read ff_mt status to clean pulse detect interrupt flag */
            if(mma8451q_get_pulse_detect_status(mma8451q_handle, buffer))
            {
                printf("Tap detected ");
                if(*buffer & MMA8451Q_PULSE_EVENT_Z_DETECT)
                {
                    printf("on:Z-AXIS\n");
                }
                else if(*buffer & MMA8451Q_PULSE_EVENT_Y_DETECT)
                {
                    printf("on:Y-AXIS\n");
                }
                else if(*buffer & MMA8451Q_PULSE_EVENT_X_DETECT)
                {
                    printf("on:X-AXIS\n");
                }
                else
                {
                    printf("ERROR\n");
                }
            }
            else
            {
                printf("ERROR\n");
                _task_block();
            }
        }
        else
        {
            printf("ERROR\n");
            _task_block();
        }
    }

    mma8451q_set_operating_mode(mma8451q_handle, MMA8451Q_OPERATING_MODE_STANDBY);

    /* Disable gpio interrupt */
    printf("Disable gpio interrupt... \n");
    _bsp_int_disable(lwgpio_int_get_vector(&acc_int));

    printf("Mma8451q deinit ... ");
    if (mma8451q_deinit(mma8451q_handle))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    /* Destroy  semaphore */
    _lwsem_destroy(&lwsem);

    /* Close the driver */
    result = fclose(fd);
    if (result)
    {
        printf("ERROR during close, returned: %d\n", result);
    }

    /* Free buffer */
    _mem_free(buffer);

    printf("Example finished.\n");
    _task_block();
} /* Endbody */

/* EOF */
