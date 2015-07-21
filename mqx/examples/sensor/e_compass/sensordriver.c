// -----------------------------------------------------------------------
// Copyright (c) 2011, 2012, 2013 Freescale Semiconductor, Inc.
// ----------------------------------------------------------------------------
// FILENAME:     sensordriver.c
// CONTENTS:     SENSOR DRIVERS
// DEPARTMENT:   ANALOG AND SENSORS DIVISION
// AUTHOR:       MARK PEDLEY
// ----------------------------------------------------------------------------
// VERSION             DATE           AUTHOR
// V1.0                20 FEB 2012    MARK PEDLEY
// V1.1                1 JUN 2012     MARK PEDLEY
// V1.2                15 JUN 2012    MARK PEDLEY
// V2.0                3 JUL 2012     MARK PEDLEY
// V3.0 (this version) 18 MAR 2013    MARK PEDLEY
// V4.0                1 July 2014    Lionel Xu
// ----------------------------------------------------------------------------
// PURPOSE:
// Tilt-compensated e-compass with magnetic calibration with support
// for Aerospace (NED), Android and Windows 8 coordinate systems.
// This code includes low level drivers for the accelerometer
// and magnetometer sensors.
// ----------------------------------------------------------------------------
// V4.0 ENHANCEMENTS:
// - Replace simulation with real sensor drivers.
// ----------------------------------------------------------------------------

#include "include.h"
#include "mqx.h"
#include "bsp.h"
#include "mma8451q_basic.h"
#include "mag3110_basic.h"
#include "mag3110_fun.h"
#include "mma8451q_generic.h"
//==============================================================
// I2C COMMUNICATION DATA
//--------------------------------------------------------------
volatile uint8 I2C2_Status = 0;         // volatile since it is changed by I2C callback functions
#define I2C2_BUF_LEN           16
#define I2C_DEVICE "ii2c3:"
uint8 I2C2_Buf[I2C2_BUF_LEN];

// sensor physical I2C addresses
#define MMA8x5x_I2C_ADDR        0x1C //0x1C for MMA8451,  0x1D for MMA8x5x
#define MAG3110_I2C_ADDR        0x0E


// MMA8x5x registers and constants
#define MMA8x5x_STATUS                  0x00
#define MMA8x5x_OUT_X_MSB               0x01
#define MMA8x5x_WHO_AM_I                0x0D
#define MMA8x5x_XYZ_DATA_CFG            0x0E
#define MMA8x5x_CTRL_REG1               0x2A
#define MMA8x5x_WHO_AM_I_VALUE          0x4A

// MAG3110 registers and constants
#define MAG3110_STATUS                  0x00
#define MAG3110_OUT_X_MSB               0x01
#define MAG3110_WHO_AM_I                0x07
#define MAG3110_CTRL_REG1               0x10
#define MAG3110_CTRL_REG2               0x11
#define MAG3110_WHO_AM_I_VALUE          0xC4

// MMA8x5x registers and constants
#define MMA8x5x_STATUS                  0x00
#define MMA8x5x_OUT_X_MSB               0x01
#define MMA8x5x_WHO_AM_I                0x0D
#define MMA8x5x_XYZ_DATA_CFG            0x0E
#define MMA8x5x_CTRL_REG1               0x2A
#define MMA8x5x_WHO_AM_I_VALUE          0x4A

// MAG3110 registers and constants
#define MAG3110_STATUS                  0x00
#define MAG3110_OUT_X_MSB               0x01
#define MAG3110_WHO_AM_I                0x07
#define MAG3110_CTRL_REG1               0x10
#define MAG3110_CTRL_REG2               0x11
#define MAG3110_WHO_AM_I_VALUE          0xC4

void *mma8451q_handle = NULL;
void *mag3110_handle = NULL;


//==============================================================
// initialize MMA8x5x accelerometer sensor
//--------------------------------------------------------------
static void MMA8x5x_Init(MQX_FILE_PTR fd)
{
      MMA8451Q_INIT_STRUCT    mma8451q_init_str = {
                                                   .SLAVE_ADDRESS        = MMA8451Q_ADDRESS_SA0_LOW,
                                                   .OUTPUT_DATA_RATE     = MMA8451Q_OUTPUT_DATA_RATE_800HZ,
                                                   .FULL_SCALE_RANGE     = MMA8451Q_FULL_SCALE_RANGE_2G,
                                                   .ACTIVE_POWER_SCHEME  = MMA8451Q_ACTIVE_POWER_SCHEME_NORMAL,
                                                   .BURST_READ_MODE      = MMA8451Q_BURST_READ_MODE_NORMAL,
                                                };

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

    mma8451q_set_slave_address(mma8451q_handle, MMA8x5x_I2C_ADDR);

    // write 0000 0000 = 0x00 to CTRL_REG1 to place MMA8x5x into standby
    // [7-1] = 0000 000
    // [0]: active=0
    if (mma8451q_write_single_reg(mma8451q_handle, MMA8x5x_CTRL_REG1, 0x00))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    // write 0000 0001 = 0x01 to XYZ_DATA_CFG register to set g range
    // [7-5]: reserved=000
    // [4]: HPF_OUT=0
    // [3-2]: reserved=00
    // [1-0]: FS=01 for +/-4g: 512 counts / g = 8192 counts / g after 4 bit left shift
    if (mma8451q_write_single_reg(mma8451q_handle, MMA8x5x_XYZ_DATA_CFG, 0x01))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    // write 0001 0001 = 0x11 to CTRL_REG1
    // [7-6]: aslp_rate=00
    // [5-3]: dr=010 for 200Hz data rate
    // [2]: unused=0
    // [1]: f_read=0 for normal 16 bit reads
    // [0]: active=1 to take the part out of standby and enable sampling
    if (mma8451q_write_single_reg(mma8451q_handle, MMA8x5x_CTRL_REG1, 0x11))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    }

    return;
}

//==============================================================
// initialize MAG3110 magnetometer sensor
//--------------------------------------------------------------
static void MAG3110_Init(MQX_FILE_PTR fd)
{
    MAG3110_INIT_STRUCT    mag3110_init_str = {
                                                  .SLAVE_ADDRESS        = MAG3110FCR1_ADDRESS,
                                                  .ADC_SAMPLE_RATE      = MAG3110_ADC_SAMPLE_RATE_80HZ,
                                                  .OVER_SAMPLE_RATIO    = MAG3110_OVER_SAMPLE_RATIO_16,
                                                  .BURST_READ_MODE      = MAG3110_BURST_READ_MODE_NORMAL,
                                                  .AUTO_MRST_MODE       = MAG3110_AUTO_MRST_ENABLE,
                                                  .DATA_CORRECTION_MODE = MAG3110_OUTPUT_CORRECT_ENABLE
                                              };
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


    // set up the MAG3110 I2C address
    if(mag3110_set_slave_address(mag3110_handle, MAG3110_I2C_ADDR))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    };

    // write 0000 0000 = 0x00 to CTRL_REG1 to place MMAG3110 into standby
    if(mag3110_set_operating_mode(mag3110_handle, MAG3110_OPERATING_MODE_STANDBY))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    };

    // enable degaussing
    if(mag3110_set_auto_mrst(mag3110_handle, MAG3110_AUTO_MRST_ENABLE))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    };

    // output correction, normal mode
    if(mag3110_set_output_correction(mag3110_handle, MAG3110_OUTPUT_CORRECT_ENABLE))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    };

    if(mag3110_reset_mag_sensor(mag3110_handle))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    };

    // write 0000 0001 = 0x01 to CTRL_REG1 to place MMAG3110 into standby
    if(mag3110_set_operating_mode(mag3110_handle, MAG3110_OPERATING_MODE_ACTIVE))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    };
    return;
}

static void MMA8x5x_ReadData(struct AccelSensor *pthisAccel)
{
    int16_t x, y, z;

       // set up the MMA8x5x I2C address
       if(mma8451q_set_slave_address(mma8451q_handle, MMA8x5x_I2C_ADDR))
    {
        ;
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    };

    if (mma8451q_get_acc_data(mma8451q_handle, &x, &y,& z))
            {
               ;
            }
            else
            {
                printf("ERROR\n");
                _task_block();
            }

    // place the 12 bytes read into the 16 bit accelerometer structure
    pthisAccel->iGpx = x;
    pthisAccel->iGpy = y;
    pthisAccel->iGpz = z;

    // finally check for -32768 in the accelerometer since
    // this value cannot be negated in a later HAL operation
    if (pthisAccel->iGpx == -32768) pthisAccel->iGpx++;
    if (pthisAccel->iGpy == -32768) pthisAccel->iGpy++;
    if (pthisAccel->iGpz == -32768) pthisAccel->iGpz++;

    return;
}

//==============================================================
// read MAG3110 accelerometer and magnetometer data over I2C
//--------------------------------------------------------------
static void MAG3110_ReadData(struct MagSensor *pthisMag)
{
    int16_t x,y,z;
    // set up the MAG3110 I2C address
    if(mag3110_set_slave_address(mag3110_handle, MAG3110_I2C_ADDR))
    {
       ;
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    };

    // read the 6 bytes of sequential sensor data
    if(mag3110_get_mag_data(mag3110_handle, &x, &y,& z))
    {
        ;
    }
    else
    {
        printf("ERROR\n");
        _task_block();
    };

    // place the 12 bytes read into the 16 bit magnetometer structure
    pthisMag->iBpx = x;
    pthisMag->iBpy = y;
    pthisMag->iBpz = z;

    // finally check for -32768 in the magnetometer data since
    // this value cannot be negated in a later HAL operation
    if (pthisMag->iBpx == -32768) pthisMag->iBpx++;
    if (pthisMag->iBpy == -32768) pthisMag->iBpy++;
    if (pthisMag->iBpz == -32768) pthisMag->iBpz++;

    return;
}


void SensorDrivers_Init()
{
    MQX_FILE_PTR fd;
    _mqx_int               param;

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
    if (I2C_OK == ioctl(fd, IO_IOCTL_I2C_SET_MASTER_MODE, NULL))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
    }

    // initialize the physical sensors over I2C
    MMA8x5x_Init(fd);
    MAG3110_Init(fd);

    return;
}

void fSixDOFSensorDrivers(struct AccelSensor *pthisAccel, struct MagSensor *pthisMag)
{
    // this function gets the real accelerometer and magnetometer sensor outputs
    // for random Euler angles (roll, pitch and yaw) under the NED (Aerospace),
    // Android and Win8 coordinate systems for the purpose of validating the eCompass
    // and magnetic calibration software independent of physical sensors.

    // it is vital that the x, y and z outputs from the real sensors are correctly
    // aligned with the NED, Android or Windows 8 coordinate system selected. the
    // raw x, y and z sensor data read from the accelerometer and magnetometer
    // sensors are in the sensor coordinate frame. the sensors frames of reference
    // will in general be rotated relative to each other and to the coordinate
    // system of the final product as a result of PCB layout decisions taken.

    // calculate the accelerometer output in units of g

    struct AccelSensor thisAccel;
    struct MagSensor thisMag;

    MMA8x5x_ReadData(&thisAccel);
    MAG3110_ReadData(&thisMag);


    pthisAccel->iGpx = thisAccel.iGpx;
    pthisAccel->iGpy = thisAccel.iGpy;
    pthisAccel->iGpz = -thisAccel.iGpz;

    // convert the accelerometer sensor reading into bit counts
    pthisAccel->fGpx = (float) (pthisAccel->iGpx / FCOUNTSPERG);
    pthisAccel->fGpy = (float) (pthisAccel->iGpy / FCOUNTSPERG);
    pthisAccel->fGpz = (float) (pthisAccel->iGpz / FCOUNTSPERG);

    // apply the forward soft iron matrix and hard iron vector
    pthisMag->iBpx = -thisMag.iBpx;
    pthisMag->iBpy = -thisMag.iBpy;
    pthisMag->iBpz = -thisMag.iBpz;
    // get magnetometer sensor reading iBpxyz in bit counts
    pthisMag->fBpx = (float) (pthisMag->iBpx / FCOUNTSPERUT);
    pthisMag->fBpy = (float) (pthisMag->iBpy / FCOUNTSPERUT);
    pthisMag->fBpz = (float) (pthisMag->iBpz / FCOUNTSPERUT);

    return;
}
