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
            #define MMA8451Q_INT_ROUTE      MMA8451Q_FIFO_INT_ROUTE_TO_INT2
    #else // defined BSP_IMX6SX_AI_M4
            #define MMA8451Q_INT_GPIO_PIN   LWGPIO_PIN_LCD1_DATA23
            #define MMA8451Q_INT_GPIO_MUX   LWGPIO_MUX_LCD1_DATA23_GPIO
            #define MMA8451Q_INT_ROUTE      MMA8451Q_FIFO_INT_ROUTE_TO_INT1
    #endif
#endif

#define USE_INTERRUPT

#define BUFFER_SIZE 256

void main_task(uint32_t);
#ifdef USE_INTERRUPT
static void generic_example_int(void);
static void int_service_routine(void *);
#else
static void generic_example(void);
#endif

const TASK_TEMPLATE_STRUCT MQX_template_list[] =
{
   /* Task Index,   Function,  Stack,  Priority, Name,   Attributes,          Param, Time Slice */
   { 10,            main_task, 2000L,    8L,     "Main", MQX_AUTO_START_TASK, 0,     0 },
   { 0 }
};

#ifdef USE_INTERRUPT
/* Global variables */
static LWSEM_STRUCT lwsem;
#endif

/******************************************************************************
*
* Functio Name      : int_service_routine
* Comments          : The interrupt service routine triggered by ACC_INT pin
*
******************************************************************************/
#ifdef USE_INTERRUPT
static void int_service_routine(void *pin)
{
    lwgpio_int_clear_flag((LWGPIO_STRUCT_PTR) pin);
    _lwsem_post(&lwsem);
}
#endif

/*TASK*-------------------------------------------------------------------
*
* Task Name : main_task
* Comments  :
*
*END*----------------------------------------------------------------------*/

void main_task(uint32_t dummy)
{
#ifndef USE_INTERRUPT
    generic_example();
#else
    generic_example_int();
#endif
} /* Endbody */

#ifdef USE_INTERRUPT
static void generic_example_int(void)
{
    uint8_t                *buffer = NULL;
    _mqx_int                param, i;
    _mqx_int                result;
    MQX_FILE_PTR            fd;
    LWGPIO_STRUCT           acc_int;
    void                   *mma8451q_handle = NULL;
    MMA8451Q_INIT_STRUCT    mma8451q_init_str = {
                                                   .SLAVE_ADDRESS        = MMA8451Q_ADDRESS_SA0_LOW,
                                                   .OUTPUT_DATA_RATE     = MMA8451Q_OUTPUT_DATA_RATE_50HZ,
                                                   .FULL_SCALE_RANGE     = MMA8451Q_FULL_SCALE_RANGE_2G,
                                                   .ACTIVE_POWER_SCHEME  = MMA8451Q_ACTIVE_POWER_SCHEME_NORMAL,
                                                   .BURST_READ_MODE      = MMA8451Q_BURST_READ_MODE_NORMAL,
                                                };
    int16_t                 x,y,z;

    printf("\n\n-------------- MMA8451Q sensor driver example with interrupt --------------\n\n");

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

    printf("Set the FIFO to Fill Buffer Mode ...");
    if (!mma8451q_set_fifo_mode(mma8451q_handle, MMA8451Q_FIFO_MODE_FULL_FILL))
    {
        printf("ERROR\n");
        _task_block();
    }

    buffer[0] = 0x00;
    if (mma8451q_get_fifo_mode(mma8451q_handle, buffer) &&
        (MMA8451Q_FIFO_MODE_FULL_FILL == buffer[0]))
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

    printf("Set interrupt output mode to push pull...");
    if (!mma8451q_set_int_output_mode(mma8451q_handle, MMA8451Q_INT_MODE_PUSH_PULL))
    {
        printf("ERROR\n");
        _task_block();
    }

    buffer[0] = 0x00;
    if ((mma8451q_get_int_output_mode(mma8451q_handle, buffer)) &&
        (MMA8451Q_INT_MODE_PUSH_PULL == buffer[0]))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Set interrupt polarity to active low...");
    if (!mma8451q_set_int_polarity(mma8451q_handle, MMA8451Q_INT_POLARITY_ACTIVE_LOW))
    {
        printf("ERROR\n");
        _task_block();
    }

    buffer[0] = 0x00;
    if ((mma8451q_get_int_polarity(mma8451q_handle, buffer)) &&
        (MMA8451Q_INT_POLARITY_ACTIVE_LOW == buffer[0]))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Route FIFO interrupt to Int pin...");
    if (!mma8451q_set_int_pin_route(mma8451q_handle, MMA8451Q_INT_ROUTE))
    {
        printf("ERROR\n");
        _task_block();
    }

    buffer[0] = 0x00;

    if ((mma8451q_get_int_pin_route(mma8451q_handle, buffer)) &&
        (MMA8451Q_INT_ROUTE == buffer[0]))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Enable FIFO interrupt ...");
    if (!mma8451q_set_int_state(mma8451q_handle, MMA8451Q_FIFO_INT_ENABLE))
    {
        printf("ERROR\n");
        _task_block();
    }

    buffer[0] = 0x00;
    if ((mma8451q_get_int_state(mma8451q_handle, buffer)) &&
        (MMA8451Q_FIFO_INT_ENABLE & buffer[0]))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    /* enable interrupt on GPIO peripheral module */
    lwgpio_int_enable(&acc_int, TRUE);

    printf("Switch to active mode \n");
    mma8451q_set_operating_mode(mma8451q_handle, MMA8451Q_OPERATING_MODE_ACTIVE);

    printf("Data Acquisition start: \n");

    for (i = 0; i < 5; i++)
    {
        /* wait for mma8451q interrupt, lwsem is set in lwgpio isr */
        _lwsem_wait(&lwsem);

        buffer[0] = 0x00;
        mma8451q_get_int_source(mma8451q_handle, buffer);
        if (buffer[0] == MMA8451Q_INT_SOURCE_FIFO)
        {
            /* Read fifo status to clean fifo interrupt flag */
            mma8451q_get_fifo_status(mma8451q_handle, buffer);

            if (mma8451q_get_acc_from_fifo(mma8451q_handle, buffer, 32))
            {
                for (int j = 0; j < 32; j++)
                {
                    x = (((int16_t)((buffer[j * 6] << 8) + buffer[j * 6 + 1])) >> MMA8451Q_OUT_DATA_SHIFT);
                    y = (((int16_t)((buffer[j * 6 + 2] << 8) + buffer[j * 6 + 3])) >> MMA8451Q_OUT_DATA_SHIFT);
                    z = (((int16_t)((buffer[j * 6 + 4] << 8) + buffer[j * 6 + 5])) >> MMA8451Q_OUT_DATA_SHIFT);
                    printf("Acc: x = %d y = %d z = %d\n",x,y,z);
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

    printf("Switch to standby mode \n");
    mma8451q_set_operating_mode(mma8451q_handle, MMA8451Q_OPERATING_MODE_STANDBY);

    /* Disable gpio interrupt */
    printf("Disable gpio interrupt... \n");
    _bsp_int_disable(lwgpio_int_get_vector(&acc_int));

    printf("Test mma8451q deinit function... ");
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
#else
static void generic_example(void)
{
    uint8_t                *buffer = NULL;
    _mqx_int                param, i;
    _mqx_int                result;
    MQX_FILE_PTR            fd;
    LWGPIO_STRUCT           acc_int;
    void                   *mma8451q_handle = NULL;
    MMA8451Q_INIT_STRUCT    mma8451q_init_str = {
                                                   .SLAVE_ADDRESS        = MMA8451Q_ADDRESS_SA0_LOW,
                                                   .OUTPUT_DATA_RATE     = MMA8451Q_OUTPUT_DATA_RATE_800HZ,
                                                   .FULL_SCALE_RANGE     = MMA8451Q_FULL_SCALE_RANGE_2G,
                                                   .ACTIVE_POWER_SCHEME  = MMA8451Q_ACTIVE_POWER_SCHEME_NORMAL,
                                                   .BURST_READ_MODE      = MMA8451Q_BURST_READ_MODE_NORMAL,
                                                };
    int16_t                 x,y,z;
    int8_t                  off_x,off_y,off_z;

    printf("\n\n-------------- MMA8451Q sensor driver example without interrupt --------------\n\n");

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

    /* Initialize the MMA8451Q chip and select the mode */
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

    printf("Test slave address set & get function... ");
    if (mma8451q_set_slave_address(mma8451q_handle, MMA8451Q_ADDRESS_SA0_HIGH) &&
        mma8451q_get_slave_address(mma8451q_handle, buffer) &&
        (*buffer == MMA8451Q_ADDRESS_SA0_HIGH))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    // Restore slave address to MMA8451Q_ADDRESS_SA0_LOW
    mma8451q_set_slave_address(mma8451q_handle, MMA8451Q_ADDRESS_SA0_LOW);

    printf("Test single byte write function... ");
    if (mma8451q_write_single_reg(mma8451q_handle, MMA8451Q_OFF_X, 0x55))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test single byte read function... ");
    if (mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_OFF_X, buffer) &&
        (*buffer == 0x55))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    //Restore MMA8451Q_OFF_X to 0x00
    mma8451q_write_single_reg(mma8451q_handle, MMA8451Q_OFF_X, 0x00);

    printf("Test multi byte write function... ");
    buffer[0] = 0x11;
    buffer[1] = 0x22;
    buffer[2] = 0x33;
    if (mma8451q_write_reg(mma8451q_handle, MMA8451Q_OFF_X, buffer, 3))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test multi byte read function... ");
    buffer[0] = 0x00;
    buffer[1] = 0x00;
    buffer[2] = 0x00;
    if (mma8451q_read_reg(mma8451q_handle, MMA8451Q_OFF_X, buffer, 3) &&
        (buffer[0] == 0x11) && (buffer[1] == 0x22) &&(buffer[2] == 0x33))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    // Restore MMA8451Q_OFF_X ~ MMA8451Q_OFF_Z
    buffer[0] = 0x00;
    buffer[1] = 0x00;
    buffer[2] = 0x00;
    mma8451q_write_reg(mma8451q_handle, MMA8451Q_OFF_X, buffer, 3);

    printf("Test mma8451q_get_device_id function... ");
    buffer[0] = 0x00;
    if ((mma8451q_get_device_id(mma8451q_handle, buffer)) &&
        (MMA8451Q_DEVICE_ID == *buffer))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mma8451q_set_user_offset function... ");
    if (mma8451q_set_user_offset(mma8451q_handle, 0x12, 0x34, 0x56))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mma8451q_get_user_offset function... ");
    if ((mma8451q_get_user_offset(mma8451q_handle, &off_x, &off_y, &off_z)) &&
        (0x12 == off_x) && (0x34 == off_y) && (0x56 == off_z))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    // Restore user offset
    mma8451q_set_user_offset(mma8451q_handle, 0x00, 0x00, 0x00);

    printf("Test mma8451q_set_output_data_rate function... ");
    if (mma8451q_set_output_data_rate(mma8451q_handle, MMA8451Q_OUTPUT_DATA_RATE_400HZ))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mma8451q_get_output_data_rate function... ");
    buffer[0] = 0x00;
    if ((mma8451q_get_output_data_rate(mma8451q_handle, buffer)) &&
        (MMA8451Q_OUTPUT_DATA_RATE_400HZ == buffer[0]))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    // Restore output data rate to 800Hz
    mma8451q_set_output_data_rate(mma8451q_handle, MMA8451Q_OUTPUT_DATA_RATE_800HZ);

    printf("Test mma8451q_set_power_scheme function... ");
    if (mma8451q_set_power_scheme(mma8451q_handle, MMA8451Q_ACTIVE_POWER_SCHEME_LOW_POWER))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mma8451q_get_power_scheme function... ");
    buffer[0] = 0x00;
    if ((mma8451q_get_power_scheme(mma8451q_handle, buffer)) &&
        (MMA8451Q_ACTIVE_POWER_SCHEME_LOW_POWER == buffer[0]))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    // Restore active power scheme to normal
    mma8451q_set_power_scheme(mma8451q_handle, MMA8451Q_ACTIVE_POWER_SCHEME_NORMAL);

    printf("Test mma8451q_set_full_scale_range function... ");
    if (mma8451q_set_full_scale_range(mma8451q_handle, MMA8451Q_FULL_SCALE_RANGE_8G))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mma8451q_get_full_scale_range function... ");
    buffer[0] = 0x00;
    if ((mma8451q_get_full_scale_range(mma8451q_handle, buffer)) &&
        (MMA8451Q_FULL_SCALE_RANGE_8G == buffer[0]))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    // Restore full scale range to 2G
    mma8451q_set_full_scale_range(mma8451q_handle, MMA8451Q_FULL_SCALE_RANGE_2G);

    printf("Test mma8451q_set_burst_read_mode function... ");
    if (mma8451q_set_burst_read_mode(mma8451q_handle, MMA8451Q_BURST_READ_MODE_FAST))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mma8451q_get_burst_read_mode function... ");
    buffer[0] = 0x00;
    if ((mma8451q_get_burst_read_mode(mma8451q_handle, buffer)) &&
        (MMA8451Q_BURST_READ_MODE_FAST == buffer[0]))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    // Restore burst read mode
    mma8451q_set_burst_read_mode(mma8451q_handle, MMA8451Q_BURST_READ_MODE_NORMAL);

    printf("Test mma8451q_set_operating_mode function... ");
    if (mma8451q_set_operating_mode(mma8451q_handle, MMA8451Q_OPERATING_MODE_ACTIVE))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mma8451q_get_operating_mode function... ");
    buffer[0] = 0x00;
    if ((mma8451q_get_operating_mode(mma8451q_handle, buffer)) &&
        (MMA8451Q_OPERATING_MODE_ACTIVE == buffer[0]))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test System mode function... ");
    buffer[0] = 0x00;
    if ((mma8451q_get_system_mode(mma8451q_handle, buffer)) &&
        (MMA8451Q_SYSMOD_WAKE == *buffer))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test Data Acquisition function... \n");
    i = 0;
    for (;;)
    {
        buffer[0] = 0x00;
        mma8451q_get_dr_status(mma8451q_handle, buffer);
        if (buffer[0] & MMA8451Q_DATA_READY_ZYXDR)
        {
            if (mma8451q_get_acc_data(mma8451q_handle, &x, &y,& z))
            {
                printf("Acc: x = %d y = %d z = %d\n",x,y,z);
                i++;
                if (i > 30)
                    break;
            }
            else
            {
                printf("ERROR\n");
                _task_block();
            }
        }
        else
        {
            _time_delay(10);
        }
    }

    printf("Test mma8451q set operating mode function... ");
    if (mma8451q_set_operating_mode(mma8451q_handle, MMA8451Q_SYSMOD_STANDBY))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mma8451q sensor reset function... ");
    if (mma8451q_reset_sensor(mma8451q_handle))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Wait for sensor reset finish... ");
    while (1)
    {
        _time_delay(5);
        if (mma8451q_get_senor_reset_state(mma8451q_handle, buffer))
        {
            if (0 == (buffer[0] & MMA8451Q_SENOR_RESET_STATUS))
            {
                printf("OK\n");
                break;
            }
        }
    }

        printf("Test mma8451q deinit function... ");
    if (mma8451q_deinit(mma8451q_handle))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

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
#endif
/* EOF */
