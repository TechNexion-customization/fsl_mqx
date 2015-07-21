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
*   an application that using the MAG3110 Magnetometer from
*   freescale
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>
#include <i2c.h>

#include "mag3110.h"

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
    MQX_FILE_PTR           fd;
    void                  *mag3110_handle = NULL;
    uint8_t               *buffer = NULL;
    _mqx_int               param, i;
    _mqx_int               result;
    int16_t                x,y,z;
    uint8_t                temperature;
    MAG3110_INIT_STRUCT    mag3110_init_str = {
                                                  .SLAVE_ADDRESS        = MAG3110FCR1_ADDRESS,
                                                  .ADC_SAMPLE_RATE      = MAG3110_ADC_SAMPLE_RATE_80HZ,
                                                  .OVER_SAMPLE_RATIO    = MAG3110_OVER_SAMPLE_RATIO_16,
                                                  .BURST_READ_MODE      = MAG3110_BURST_READ_MODE_NORMAL,
                                                  .AUTO_MRST_MODE       = MAG3110_AUTO_MRST_ENABLE,
                                                  .DATA_CORRECTION_MODE = MAG3110_OUTPUT_CORRECT_ENABLE
                                              };

    /* Allocate receive buffer */
    buffer = _mem_alloc_zero(BUFFER_SIZE);
    if(buffer == NULL)
    {
        printf("ERROR getting receive buffer!\n");
        _task_block();
    }

    printf("\n\n-------------- MAG3110 sensor driver example --------------\n\n");
    /* Open the I2C driver */
    fd = fopen(I2C_DEVICE, NULL);
    if(fd == NULL)
    {
        printf("ERROR opening the I2C driver!\n");
        _task_block();
    }

    /* Set bus speed */
    param = 100000;
    printf("Set current baud rate to %d ... ", param);
    if(I2C_OK == ioctl(fd, IO_IOCTL_I2C_SET_BAUD, &param))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
    }

    printf("Set master mode ... ");
    if(I2C_OK == ioctl(fd, IO_IOCTL_I2C_SET_MASTER_MODE, NULL))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
    }

    /* Initialize the MAG3110 chip and select the mode */
    printf("Mag3110_init ... ");
    mag3110_handle = mag3110_init(&mag3110_init_str, fd);
    if(mag3110_handle != NULL)
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test slave address set & get function... ");
    if(mag3110_set_slave_address(mag3110_handle, FXMS3110CDR1_ADDRESS) &&
       mag3110_get_slave_address(mag3110_handle, buffer) &&
       (*buffer == FXMS3110CDR1_ADDRESS))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    // Restore slave address to MAG3110FCR1_ADDRESS
    mag3110_set_slave_address(mag3110_handle, MAG3110FCR1_ADDRESS);

    printf("Test single byte write function... ");
    if(mag3110_write_single_reg(mag3110_handle, MAG3110_OFF_X_MSB, 0x55))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test single byte read function... ");
    if((mag3110_read_single_reg(mag3110_handle, MAG3110_OFF_X_MSB, buffer)) && (*buffer == 0x55))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    //Restore MAG3110_OFF_X_MSB to 0x00
    mag3110_write_single_reg(mag3110_handle, MAG3110_OFF_X_MSB, 0x00);

    printf("Test multi byte write function... ");
    buffer[0] = 0x11;
    buffer[1] = 0x22;
    buffer[2] = 0x33;
    buffer[3] = 0x44;
    buffer[4] = 0x55;
    buffer[5] = 0x66;
    if(mag3110_write_reg(mag3110_handle, MAG3110_OFF_X_MSB, buffer, 6))
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
    buffer[3] = 0x00;
    buffer[4] = 0x00;
    buffer[5] = 0x00;

    if((mag3110_read_reg(mag3110_handle, MAG3110_OFF_X_MSB, buffer, 6)) &&
       (buffer[0] == 0x11) && (buffer[1] == 0x22) && (buffer[2] == 0x33) &&
       (buffer[3] == 0x44) && (buffer[4] == 0x55) && (buffer[5] == 0x66))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    // Restore MAG3110_OFF_X_MSB ~ MAG3110_OFF_Z_LSB
    buffer[0] = 0x00;
    buffer[1] = 0x00;
    buffer[2] = 0x00;
    buffer[3] = 0x00;
    buffer[4] = 0x00;
    buffer[5] = 0x00;
    mag3110_write_reg(mag3110_handle, MAG3110_OFF_X_MSB, buffer, 6);

    printf("Test mag3110_get_device_id function... ");
    buffer[0] = 0x00;
    if((mag3110_get_device_id(mag3110_handle, buffer)) && (MAG3110_DEVICE_ID == *buffer))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mag3110_set_user_offset function... ");
    if(mag3110_set_user_offset(mag3110_handle, 0x1122, 0x2211, 0x1122))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mag3110_get_user_offset function... ");
    if((mag3110_get_user_offset(mag3110_handle, &x, &y, &z)) &&
       (0x1122 == x) && (0x2211 == y) && (0x1122 == z))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    // Restore user offset
    mag3110_set_user_offset(mag3110_handle, 0x0000, 0x0000, 0x0000);

    printf("Test mag3110_set_adc_sample_rate function... ");
    if(mag3110_set_adc_sample_rate(mag3110_handle, MAG3110_ADC_SAMPLE_RATE_160HZ))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mag3110_get_adc_sample_rate function... ");
    buffer[0] = 0x00;
    if((mag3110_get_adc_sample_rate(mag3110_handle, buffer)) &&
       (MAG3110_ADC_SAMPLE_RATE_160HZ == buffer[0]))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    // Restore adc sample rate to 80HZ
    mag3110_set_adc_sample_rate(mag3110_handle, MAG3110_ADC_SAMPLE_RATE_80HZ);

    printf("Test mag3110_set_over_sample_ratio function... ");
    if(mag3110_set_over_sample_ratio(mag3110_handle, MAG3110_OVER_SAMPLE_RATIO_128))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mag3110_get_over_sample_ratio function... ");
    buffer[0] = 0x00;
    if((mag3110_get_over_sample_ratio(mag3110_handle, buffer)) &&
       (MAG3110_OVER_SAMPLE_RATIO_128 == buffer[0]))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    // Restore over sample ratio to 16
    mag3110_set_over_sample_ratio(mag3110_handle, MAG3110_OVER_SAMPLE_RATIO_16);

    printf("Test mag3110_set_burst_read_mode function... ");
    if(mag3110_set_burst_read_mode(mag3110_handle, MAG3110_BURST_READ_MODE_FAST))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mag3110_get_burst_read_mode function... ");
    buffer[0] = 0x00;
    if((mag3110_get_burst_read_mode(mag3110_handle, buffer)) &&
       (MAG3110_BURST_READ_MODE_FAST == buffer[0]))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    // Restore burst read mode
    mag3110_set_burst_read_mode(mag3110_handle, MAG3110_BURST_READ_MODE_NORMAL);

    printf("Test mag3110_set_output_correction function... ");
    if(mag3110_set_output_correction(mag3110_handle, MAG3110_AUTO_MRST_DISABLE))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mag3110_get_output_correction function... ");
    buffer[0] = 0x00;
    if((mag3110_get_output_correction(mag3110_handle, buffer)) &&
       (MAG3110_AUTO_MRST_DISABLE == buffer[0]))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }
    // Restore output correction to MAG3110_AUTO_MRST_ENABLE
    mag3110_set_output_correction(mag3110_handle, MAG3110_AUTO_MRST_ENABLE);

    printf("Test mag3110_set_operating_mode function... ");
    if(mag3110_set_operating_mode(mag3110_handle, MAG3110_OPERATING_MODE_ACTIVE))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mag3110_get_operating_mode function... ");
    buffer[0] = 0x00;
    if((mag3110_get_operating_mode(mag3110_handle, buffer)) &&
       (MAG3110_OPERATING_MODE_ACTIVE == buffer[0]))
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
    if((mag3110_get_system_mode(mag3110_handle, buffer)) &&
       (MAG3110_SYSMOD_ACTIVE_NORMAL == *buffer))
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
    for(;;)
    {
        buffer[0] = 0x00;
        mag3110_get_dr_status(mag3110_handle, buffer);
        if(buffer[0] & MAG3110_DATA_READY_ZYXDR)
        {
            if(mag3110_get_mag_data(mag3110_handle, &x, &y,& z))
            {
                if(mag3110_read_single_reg(mag3110_handle, MAG3110_DIE_TEMP, &temperature))
                {
                    printf("Mag: x = %d y = %d z = %d\n",x,y,z);
                    printf("DIE Temp: %x \n",temperature);
                    i++;
                    if(i > 30)
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
                printf("ERROR\n");
                _task_block();
            }
        }
        else
        {
            _time_delay(10);
        }
    }

    mag3110_set_operating_mode(mag3110_handle, MAG3110_OPERATING_MODE_STANDBY);

    printf("Test Trigger Data Acquisition function... \n");
    // Read to clean data ready status
    mag3110_get_mag_data(mag3110_handle, &x, &y,& z);

    mag3110_get_operating_mode(mag3110_handle, buffer);
    if(*buffer != MAG3110_OPERATING_MODE_STANDBY)
    {
        printf("ERROR\n");
        _task_block();
    }
    if(!mag3110_trigger_measurement(mag3110_handle))
    {
        printf("ERROR\n");
        _task_block();
    }
    i = 0;
    for(;;)
    {
        buffer[0] = 0x00;
        mag3110_get_dr_status(mag3110_handle, buffer);
        if(buffer[0] & MAG3110_DATA_READY_ZYXDR)
        {
            if(mag3110_get_mag_data(mag3110_handle, &x, &y,& z))
            {
                if(mag3110_read_single_reg(mag3110_handle, MAG3110_DIE_TEMP, &temperature))
                {
                    printf("Mag: x = %d y = %d z = %d\n",x,y,z);
                    printf("DIE Temp: 0x%x\n",temperature);
                    i++;
                    if(i > 30)
                        break;
                    mag3110_get_operating_mode(mag3110_handle, buffer);
                    if(*buffer != MAG3110_OPERATING_MODE_STANDBY)
                    {
                        printf("ERROR\n");
                        _task_block();
                    }
                    mag3110_trigger_measurement(mag3110_handle);
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
        else
        {
            mag3110_read_single_reg(mag3110_handle, MAG3110_CTRL_REG1, &temperature);
            printf("CTRL_REG1: 0x%x\n",temperature);
            _time_delay(10);
        }
    }

    printf("Test mag3110 sensor reset function... ");
    if(mag3110_reset_mag_sensor(mag3110_handle))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    printf("Test mag3110 get sensor reset status function... ");
    while(TRUE)
    {
        if(mag3110_get_reset_status(mag3110_handle, buffer))
        {
            if(buffer[0] == 0)
            {
                printf("OK\n");
                break;
            }
            else
            {
                printf("\nWait for Reset finish... \n");
            }
        }
        else
        {
            printf("ERROR\n");
            _task_block();
        }
    }

    printf("Test mag3110 deinit function... ");
    if(mag3110_deinit(mag3110_handle))
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
