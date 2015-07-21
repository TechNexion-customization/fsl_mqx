/**HEADER*******************************************************************
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
* FileName: mag3110_basic.c
* Version :
* Date    : Apr-22-2014
*
* Comments:
*
*   MAG3110 basic IO function realization
*
*END************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "mqx.h"
#include "bsp.h"
#include "i2c.h"

#include "mag3110_reg.h"
#include "mag3110_prv.h"
#include "mag3110_basic.h"

// Initialize MAG3110 with parameter set in MAG3110 initialize structure
void * mag3110_init
(
    MAG3110_INIT_STRUCT   *mag3110_init_ptr,
    MQX_FILE_PTR           fd
)
{
    MAG3110_INFO_STRUCT *mag3110_info_ptr;

    /* Initialize internal data */
    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)_mem_alloc_system_zero((uint32_t)sizeof(MAG3110_INFO_STRUCT));
    if(mag3110_info_ptr == NULL)
        return NULL;

    if((mag3110_init_ptr == NULL) || (fd == NULL) ||
       ((mag3110_init_ptr->SLAVE_ADDRESS != MAG3110FCR1_ADDRESS) && (mag3110_init_ptr->SLAVE_ADDRESS != FXMS3110CDR1_ADDRESS)) ||
       (0 != (mag3110_init_ptr->BURST_READ_MODE & (~MAG3110_CTRL_REG1_FR_MASK))) ||
       (0 != (mag3110_init_ptr->ADC_SAMPLE_RATE & (~MAG3110_CTRL_REG1_DR_MASK))) ||
       (0 != (mag3110_init_ptr->OVER_SAMPLE_RATIO & (~MAG3110_CTRL_REG1_OS_MASK))) ||
       (0 != (mag3110_init_ptr->AUTO_MRST_MODE & (~MAG3110_CTRL_REG2_AUTO_MRST_EN_MASK))) ||
       (0 != (mag3110_init_ptr->DATA_CORRECTION_MODE & (~MAG3110_CTRL_REG2_RAW_MASK))))
    {
        /* Free info struct */
        _mem_free(mag3110_info_ptr);
        return NULL;
    }

    // initialize info structure
    mag3110_info_ptr->I2C_FILE_PTR    = fd;
    mag3110_info_ptr->SLAVE_ADDRESS   = mag3110_init_ptr->SLAVE_ADDRESS;
    mag3110_info_ptr->BURST_READ_MODE = mag3110_init_ptr->BURST_READ_MODE;
    // mag3110 will stay in standby mode after initialize
    mag3110_info_ptr->OPERATING_MODE  = MAG3110_OPERATING_MODE_STANDBY;

    if((mag3110_write_single_reg(mag3110_info_ptr, MAG3110_CTRL_REG2,
                                (mag3110_init_ptr->AUTO_MRST_MODE | mag3110_init_ptr->DATA_CORRECTION_MODE))) &&
       (mag3110_write_single_reg(mag3110_info_ptr, MAG3110_CTRL_REG1,
                                (mag3110_init_ptr->ADC_SAMPLE_RATE | mag3110_init_ptr->OVER_SAMPLE_RATIO |
                                 mag3110_init_ptr->BURST_READ_MODE))))
    {
        return (void *) mag3110_info_ptr;
    }
    else
    {
        /* Free info struct */
        _mem_free(mag3110_info_ptr);

        return NULL;
    }
}

// Deinit MAG3110 and make it working in power-saving mode.
bool mag3110_deinit
(
    void *mag3110_handle
)
{
    bool result;
    MAG3110_INFO_STRUCT *mag3110_info_ptr;

    if(mag3110_handle == NULL)
        return FALSE;

    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)mag3110_handle;

    /* Recover MAG3110 register to default value */
    if((mag3110_write_single_reg(mag3110_info_ptr, MAG3110_CTRL_REG1,0x00)) &&
       (mag3110_write_single_reg(mag3110_info_ptr, MAG3110_CTRL_REG2,0x00)) &&
       (mag3110_write_single_reg(mag3110_info_ptr, MAG3110_OFF_X_MSB,0x00)) &&
       (mag3110_write_single_reg(mag3110_info_ptr, MAG3110_OFF_X_LSB,0x00)) &&
       (mag3110_write_single_reg(mag3110_info_ptr, MAG3110_OFF_Y_MSB,0x00)) &&
       (mag3110_write_single_reg(mag3110_info_ptr, MAG3110_OFF_Y_LSB,0x00)) &&
       (mag3110_write_single_reg(mag3110_info_ptr, MAG3110_OFF_Z_MSB,0x00)) &&
       (mag3110_write_single_reg(mag3110_info_ptr, MAG3110_OFF_Z_LSB,0x00)))
    {
        result = TRUE;
    }
    else
    {
        result = FALSE;
    }

    /* Free info struct */
    _mem_free(mag3110_info_ptr);
    mag3110_handle = NULL;

    return result;
}

// Basic multi-Byte write function
bool mag3110_write_reg
(
    void       *mag3110_handle,
    uint8_t     addr,
    uint8_t    *buffer,
    uint16_t    n
)
{
    bool                 result;
    uint8_t              index, slave_address_old;
    _mqx_int             param;
    MAG3110_INFO_STRUCT *mag3110_info_ptr;

    if((mag3110_handle == NULL) || (buffer == NULL))
        return FALSE;

    if(n == 0)
        return TRUE;

    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)mag3110_handle;

    // Back up old i2c slave address
    if(I2C_OK == ioctl(mag3110_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_GET_DESTINATION_ADDRESS, &param))
    {
        slave_address_old = (uint8_t)(param & 0xFF);
    }
    else
    {
        return FALSE;
    }

    // Set MAG3110 slave address
    param = mag3110_info_ptr->SLAVE_ADDRESS;
    if(I2C_OK != ioctl(mag3110_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param))
        return FALSE;

    /* Write of data */
    if((1 == fwrite(&addr, 1, 1, mag3110_info_ptr->I2C_FILE_PTR)) &&
       (n == fwrite(buffer, 1, n, mag3110_info_ptr->I2C_FILE_PTR)) &&
       (MQX_OK == ioctl(mag3110_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_STOP, NULL)))
    {
        result = TRUE;
    }
    else
    {
        result = FALSE;
    }

    // If try to modify MAG3110_CTRL_REG1
    if(((addr <= MAG3110_CTRL_REG1) && ((addr + n - 1) >= MAG3110_CTRL_REG1)) && (result == TRUE))
    {
        index = (MAG3110_CTRL_REG1 - addr);
        if(mag3110_info_ptr->OPERATING_MODE == MAG3110_OPERATING_MODE_STANDBY)
        {
            mag3110_info_ptr->BURST_READ_MODE = (buffer[index] & MAG3110_CTRL_REG1_FR_MASK);
        }
        mag3110_info_ptr->OPERATING_MODE = (buffer[index] & MAG3110_CTRL_REG1_AC_MASK);
    }

    param = slave_address_old;
    if(I2C_OK == ioctl(mag3110_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param))
        return result;
    else
        return FALSE;
}

// Basic multi-Byte read function
bool mag3110_read_reg
(
    void       *mag3110_handle,
    uint8_t     addr,
    uint8_t    *buffer,
    uint16_t    n
)
{
    bool                 result;
    uint8_t              slave_address_old;
    _mqx_int             param;
    MAG3110_INFO_STRUCT *mag3110_info_ptr;

    if((mag3110_handle == NULL) || (buffer == NULL))
        return FALSE;

    if(n == 0)
        return TRUE;

    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)mag3110_handle;

    // Back up old i2c slave address
    if(I2C_OK == ioctl(mag3110_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_GET_DESTINATION_ADDRESS, &param))
    {
        slave_address_old = (uint8_t)(param & 0xFF);
    }
    else
    {
        return FALSE;
    }

    // Set MAG3110 slave address
    param = mag3110_info_ptr->SLAVE_ADDRESS;
    if(I2C_OK != ioctl(mag3110_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param))
        return FALSE;

    /* M U S T read all data */
    if((1 == fwrite(&addr, 1, 1, mag3110_info_ptr->I2C_FILE_PTR)) &&
       (MQX_OK == ioctl(mag3110_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_REPEATED_START, NULL)) &&
       (n == fread(buffer, 1, n, mag3110_info_ptr->I2C_FILE_PTR)) &&
       (MQX_OK == ioctl(mag3110_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_STOP, NULL)))
    {
        result = TRUE;
    }
    else
    {
        result = FALSE;
    }

    /* Restore old i2c address */
    param = slave_address_old;
    if(I2C_OK == ioctl(mag3110_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param))
        return result;
    else
        return FALSE;
}

// Basic single byte write function
bool mag3110_write_single_reg
(
    void      *mag3110_handle,
    uint8_t    addr,
    uint8_t    data
)
{
    return mag3110_write_reg(mag3110_handle, addr, &data, 1);
}

// Basic single byte read function
bool mag3110_read_single_reg
(
    void      *mag3110_handle,
    uint8_t    addr,
    uint8_t   *buffer
)
{
    return mag3110_read_reg(mag3110_handle, addr, buffer, 1);
}

// Mag3110 slave address configuration
bool mag3110_set_slave_address
(
    void      *mag3110_handle,
    uint8_t    slave_address
)
{
    MAG3110_INFO_STRUCT *mag3110_info_ptr;

    if(mag3110_handle == NULL)
        return FALSE;

    if((slave_address != MAG3110FCR1_ADDRESS) && (slave_address != FXMS3110CDR1_ADDRESS))
        return FALSE;

    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)mag3110_handle;

    mag3110_info_ptr->SLAVE_ADDRESS = slave_address;

    return TRUE;
}

// Mag3110 slave address configuration
bool mag3110_get_slave_address
(
    void      *mag3110_handle,
    uint8_t   *buffer
)
{
    MAG3110_INFO_STRUCT *mag3110_info_ptr;

    if((mag3110_handle == NULL) || (buffer == NULL))
        return FALSE;

    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)mag3110_handle;

    *buffer = mag3110_info_ptr->SLAVE_ADDRESS;

    return TRUE;
}
