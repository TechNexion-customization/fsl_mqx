/**HEADER********************************************************************
*
* Copyright (c) 2014 Freescale Semiconductor;
* All Rights Reserved
*
***************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
**************************************************************************
*
* Comments:
*
*   This file contains read/write functions to access I2C MMA8451Q
*   using I2C driver.
*
*END************************************************************************/
#include <mqx.h>
#include <bsp.h>
#include <i2c.h>
#include "test.h"

#if BSPCFG_ENABLE_II2C3
    #define I2C_DEVICE "ii2c3:"
#elif BSPCFG_ENABLE_I2C3
    #define I2C_DEVICE "i2c3:"
#else
    #error This application requires BSPCFG_ENABLE_I2C3 or BSPCFG_ENABLE_II2C3 defined non-zero in user_config.h. Please recompile BSP with this option.
#endif

enum
{
    MMA8451Q_STATUS = 0x00,
    MMA8451Q_OUT_X_MSB,
    MMA8451Q_OUT_X_LSB,
    MMA8451Q_OUT_Y_MSB,
    MMA8451Q_OUT_Y_LSB,
    MMA8451Q_OUT_Z_MSB,
    MMA8451Q_OUT_Z_LSB,

    MMA8451Q_F_SETUP=0x09,
    MMA8451Q_TRIG_CFG,
    MMA8451Q_SYSMOD,
    MMA8451Q_INT_SOURCE,
    MMA8451Q_WHO_AM_I,
    MMA8451Q_XYZ_DATA_CFG,
    MMA8451Q_HP_FILTER_CUTOFF,

    MMA8451Q_PL_STATUS,
    MMA8451Q_PL_CFG,
    MMA8451Q_PL_COUNT,
    MMA8451Q_PL_BF_ZCOMP,
    MMA8451Q_PL_THS_REG,

    MMA8451Q_FF_MT_CFG,
    MMA8451Q_FF_MT_SRC,
    MMA8451Q_FF_MT_THS,
    MMA8451Q_FF_MT_COUNT,

    MMA8451Q_TRANSIENT_CFG=0x1d,
    MMA8451Q_TRANSIENT_SRC,
    MMA8451Q_TRANSIENT_THS,
    MMA8451Q_TRANSIENT_COUNT,

    MMA8451Q_PULSE_CFG,
    MMA8451Q_PULSE_SRC,
    MMA8451Q_PULSE_THSX,
    MMA8451Q_PULSE_THSY,
    MMA8451Q_PULSE_THSZ,
    MMA8451Q_PULSE_TMLT,
    MMA8451Q_PULSE_LTCY,
    MMA8451Q_PULSE_WIND,

    MMA8451Q_ASLP_COUNT,
    MMA8451Q_CTRL_REG1,
    MMA8451Q_CTRL_REG2,
    MMA8451Q_CTRL_REG3,
    MMA8451Q_CTRL_REG4,
    MMA8451Q_CTRL_REG5,

    MMA8451Q_OFF_X,
    MMA8451Q_OFF_Y,
    MMA8451Q_OFF_Z,
};

enum
{
    MODE_2G=0,
    MODE_4G,
    MODE_8G,
};

/* mma8450 status */
struct mma_status
{
    unsigned char  mode;
    unsigned char ctl_reg2;
    unsigned char ctl_reg1;
};

static struct mma_status mma_status =
{
    .mode = 0,
    .ctl_reg2 = 0,
    .ctl_reg1 = 0
};

/* The I2C MMA8451Q test addresses */
#define I2C_MMA8451Q_BUS_ADDRESS     0x1C     /* I2C bus address of MMA8451Q */
#define MMA8451Q_ID                  0x1A

#define BUFFER_SIZE 256

/* Function prototypes */
int i2c_write_MMA8451Q(MQX_FILE_PTR, uint8_t, unsigned char *, _mqx_int);
int i2c_read_MMA8451Q(MQX_FILE_PTR, uint8_t, unsigned char *, _mqx_int);
void report_abs(MQX_FILE_PTR);
int mma8451q_read_data(MQX_FILE_PTR ,short *, short *, short *);
int mma8451q_init_client(MQX_FILE_PTR);

 /*FUNCTION****************************************************************
*
* Function Name    : mma8451q_init_client
* Returned Value   : result
* Comments         :
*   Set the 2g/4g/8g mode, fast read bit clear, active mode on.
*
*END*********************************************************************/
int mma8451q_init_client
(
 /* [IN] The file pointer for the I2C channel */
    MQX_FILE_PTR fd
)
{
    int result;
    unsigned char x = 0;
    unsigned char *buffer;
    buffer = &x;

    mma_status.mode = MODE_2G;
    result = i2c_write_MMA8451Q(fd, MMA8451Q_XYZ_DATA_CFG, &mma_status.mode,1);
    if(result)
    {
        PRINT(" %d\t Test i2c_write_MMA8451Q ... \t  ERROR\n", _task_get_index_from_id(_task_get_id()));
    }

    mma_status.ctl_reg2=0x01;
    result =i2c_write_MMA8451Q(fd, MMA8451Q_CTRL_REG1,&mma_status.ctl_reg2,1);
    if(result)
    {
        PRINT(" %d\t Fast read clear and active mode ... \t  ERROR\n", _task_get_index_from_id(_task_get_id()));
    }

    result = i2c_read_MMA8451Q(fd, MMA8451Q_WHO_AM_I, buffer,1);
    if((result != I2C_OK) && ((*buffer) != MMA8451Q_ID))
    {
        PRINT(" %d\t Test i2c_read_MMA8451Q_byte ... \t  ERROR\n", _task_get_index_from_id(_task_get_id()));
    }

    return result;
}

 /*FUNCTION****************************************************************
*
* Function Name    : mma8451q_read_data
* Returned Value   : result
* Comments         :
*   Read sensor data from mma8451q
*
*END*********************************************************************/
int mma8451q_read_data(MQX_FILE_PTR fd,short *x, short *y, short *z)
{
    uint8_t  tmp_data[7];
    _mqx_int result;

    result = i2c_read_MMA8451Q(fd,MMA8451Q_OUT_X_MSB,tmp_data,7);
    if(result)
    {
        PRINT(" %d\t Test: Read 14-bit data x,y,z ... \t  ERROR\n", _task_get_index_from_id(_task_get_id()));
        _time_delay_ticks(4);
        _mqx_exit(1);
    }

    *x = ((tmp_data[0] << 8) & 0xff00) | tmp_data[1];
    *y = ((tmp_data[2] << 8) & 0xff00) | tmp_data[3];
    *z = ((tmp_data[4] << 8) & 0xff00) | tmp_data[5];
    *x = (short)(*x) >> 2;
    *y = (short)(*y) >> 2;
    *z = (short)(*z) >> 2;

    if(mma_status.mode == MODE_4G)
    {
        (*x) = (*x) << 1;
        (*y) = (*y) << 1;
        (*z) = (*z) << 1;
    }
    else if(mma_status.mode == MODE_8G)
    {
        (*x) = (*x) << 2;
        (*y) = (*y) << 2;
        (*z) = (*z) << 2;
    }

    return 0;
}

 /*FUNCTION****************************************************************
*
* Function Name    : report_abs
* Returned Value   :
* Comments         :
*   Check the new data is ready and output the result.
*
*END*********************************************************************/
void report_abs(MQX_FILE_PTR fd)
{
    short x,y,z;
    _mqx_int result;

    result=mma8451q_read_data(fd,&x, &y, &z);
    if(result)
    {
        PRINT(" %d\t Test: Read data done ... \t  ERROR\n", _task_get_index_from_id(_task_get_id()));
    }
}

/*FUNCTION****************************************************************
*
* Function Name    : i2c_write_MMA8451Q
* Returned Value   : void
* Comments         :
*   Writes the data buffer at address in  MMA8451Q
*
*END*********************************************************************/
int i2c_write_MMA8451Q
    (
        /* [IN] The file pointer for the I2C channel */
        MQX_FILE_PTR   fd,

        /* [IN] The address in MMA8451Q to write to */
        uint8_t        addr,

        /* [IN] The array of characters are to be written in MMA8451Q */
        unsigned char *buffer,

        /* [IN] Number of bytes in that buffer */
        _mqx_int       n
    )
{ /* Body */
    _mqx_int result = I2C_OK;

    /* Write register address*/
    result = fwrite(&addr, 1, 1, fd);

    /* Write of data */
    result = fwrite(buffer, 1, n, fd);

    /* Wait for completion */
    result = fflush(fd);

    /* Stop I2C transfer */
    result = ioctl(fd, IO_IOCTL_I2C_STOP, NULL);

    return result;
} /* Endbody */

/*FUNCTION****************************************************************
*
* Function Name    : i2c_read_MMA8451Q
* Returned Value   : void
* Comments         :
*   Reads the data buffer from address in  MMA8451Q
*END*********************************************************************/
int i2c_read_MMA8451Q
    (
        /* [IN] The file pointer for the I2C channel */
        MQX_FILE_PTR   fd,

        /* [IN] The address in MMA8451Q to read from */
        uint8_t        addr,

        /* [IN] The array of characters to be written into */
        unsigned char *buffer,

        /* [IN] Number of bytes to read */
        _mqx_int       n
    )
{ /* Body */
    _mqx_int result = I2C_OK;

   /* Write register address */
   result = fwrite (&addr, 1, 1, fd);

   /* Wait for completion */
   result = fflush(fd);

   /* Restart I2C transfer for reading */
   result = ioctl(fd, IO_IOCTL_I2C_REPEATED_START, NULL);

   /* M U S T read all data */
   result =  fread (buffer, 1, n, fd);

   /* Stop I2C transfer - initiate MMA8451Q write cycle . Requirement I2C033*/
   result = ioctl (fd, IO_IOCTL_I2C_STOP, NULL);

   return result;
} /* Endbody */

/* I2C task */
void i2c_task(uint32_t dummy)
{
    MQX_FILE_PTR          fd;
    _mqx_int              param, result;
    _mqx_int              c = 0;
    unsigned char        *buffer;

    /* Allocate receive buffer */
    buffer = _mem_alloc_zero(BUFFER_SIZE);
    if (buffer == NULL)
    {
        PRINT("\n %d\t ERROR getting receive buffer!\n", _task_get_index_from_id(_task_get_id()));
        _task_block();
    }

    /* Open the I2C driver */
    fd = fopen(I2C_DEVICE, NULL);
    if(fd == NULL)
    {
        PRINT("\n %d\t ERROR opening the I2C driver!\n", _task_get_index_from_id(_task_get_id()));
        _task_block();
    }

    param = I2C_MMA8451Q_BUS_ADDRESS;
    result = ioctl(fd, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param);
    if(result)
    {
        PRINT(" %d\t Set destination address ... \t  ERROR\n", _task_get_index_from_id(_task_get_id()));
    }

    /* Initialize the MMA8451Q chip and select the mode */
    result = mma8451q_init_client(fd);
    if(result)
    {
        PRINT(" %d\t Mma8451q_init_client ... \t  ERROR\n", _task_get_index_from_id(_task_get_id()));
    }

    while(1)
    {
        c++;
        report_abs(fd);
        if (c%100 == 0) {
           PRINT(" %d\t I2C task is running... \n", _task_get_index_from_id(_task_get_id()));
        }
    };
} /* Endbody */
/* EOF */

