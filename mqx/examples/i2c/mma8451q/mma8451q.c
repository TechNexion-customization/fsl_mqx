/**HEADER********************************************************************
*
* Copyright (c) 2013-2014 Freescale Semiconductor;
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
* FileName: mma8451q.c
* Version :
* Date    : Apr-10-2014
*
* Comments:
*
*   This file contains read/write functions to access I2C MMA8451Q
*   using I2C polled driver.
*
*END************************************************************************/
#include <mqx.h>
#include <bsp.h>
#include <i2c.h>
#include "mma8451q.h"

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

    printf("Set the mode :2G\n");
    printf("Test i2c_write_MMA8451Q ... ");
    mma_status.mode = MODE_2G;
    result = i2c_write_MMA8451Q(fd, MMA8451Q_XYZ_DATA_CFG, &mma_status.mode,1);
    if(result == I2C_OK)
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
    }

    printf("Fast read clear and active mode ... ");
    mma_status.ctl_reg2=0x01;
    result =i2c_write_MMA8451Q(fd, MMA8451Q_CTRL_REG1,&mma_status.ctl_reg2,1);
    if(result == I2C_OK)
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
    }

    printf("Test WHO_AM_I check\n");
    printf("Test i2c_read_MMA8451Q_byte ... ");
    result = i2c_read_MMA8451Q(fd, MMA8451Q_WHO_AM_I, buffer,1);
    if((result == I2C_OK) && ((*buffer)==MMA8451Q_ID))
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
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

    printf("Test: Read 14-bit data x,y,z ... ");
    result = i2c_read_MMA8451Q(fd,MMA8451Q_OUT_X_MSB,tmp_data,7);
    if(result == I2C_OK)
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
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
* Function Name    : mma8451q_change_mode
* Returned Value   :
* Comments         :
*   Change the current mode.
*
*END*********************************************************************/
int mma8451q_change_mode(MQX_FILE_PTR fd, int mode)
{
    int result;
    mma_status.mode = mode;
    result = i2c_write_MMA8451Q(fd,MMA8451Q_XYZ_DATA_CFG,&mma_status.mode,1);
    return result;
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
    unsigned char *buffer;
    unsigned char a = 0;
    _mqx_int result;

    buffer = &a;
    printf("Test: Read data done ... ");
    result=mma8451q_read_data(fd,&x, &y, &z);
    if(result == I2C_OK)
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
    }

    _time_delay_ticks(4);
    printf("Test: Check if the new data ready ... ");
    result = i2c_read_MMA8451Q(fd,MMA8451Q_STATUS,buffer,1);
    if(result == I2C_OK)
    {
        printf("OK\n");
    }
    else
    {
        printf("ERROR\n");
    }

    if(!(*buffer & 0x08))
    {
        _time_delay_ticks(4);
        _mqx_exit(1);
    };

    printf("\n output x:%d   y:%d   z:%d\n",x,y,z);
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
*Requirement
* I2C007,I2C008,I2C009,I2C010,I2C029,I2C030,I2C031,I2C033,I2C037
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
/* EOF */
