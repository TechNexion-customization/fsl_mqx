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
* FileName: mma8451q_basic.c
* Version :
* Date    : May-5-2014
*
* Comments:
*
*   MMA8451Q basic IO function realization
*
*END************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "mqx.h"
#include "bsp.h"
#include "i2c.h"

#include "mma8451q_reg.h"
#include "mma8451q_prv.h"
#include "mma8451q_basic.h"

// Initialize MMA8451Q with parameter set in MMA8451Q initialize structure
void * mma8451q_init
(
    MMA8451Q_INIT_STRUCT   *mma8451q_init_ptr,
    MQX_FILE_PTR            fd
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    /* Initialize internal data */
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)_mem_alloc_system_zero((uint32_t)sizeof(MMA8451Q_INFO_STRUCT));
    if (mma8451q_info_ptr == NULL)
        return NULL;

    if ((mma8451q_init_ptr == NULL) || (fd == NULL) ||
        ((mma8451q_init_ptr->SLAVE_ADDRESS != MMA8451Q_ADDRESS_SA0_LOW) &&
         (mma8451q_init_ptr->SLAVE_ADDRESS != MMA8451Q_ADDRESS_SA0_HIGH)) ||
        (0 != (mma8451q_init_ptr->OUTPUT_DATA_RATE & (~MMA8451Q_CTRL_REG1_DR_MASK))) ||
        (0 != (mma8451q_init_ptr->FULL_SCALE_RANGE & (~MMA8451Q_XYZ_DATA_CFG_FS_MASK))) ||
        (0 != (mma8451q_init_ptr->ACTIVE_POWER_SCHEME & (~MMA8451Q_CTRL_REG2_MODS_MASK))) ||
        (0 != (mma8451q_init_ptr->BURST_READ_MODE & (~MMA8451Q_CTRL_REG1_F_READ_MASK))))
    {
        /* Free info struct */
        _mem_free(mma8451q_info_ptr);
        return NULL;
    }

    // initialize info structure
    mma8451q_info_ptr->I2C_FILE_PTR    = fd;
    mma8451q_info_ptr->SLAVE_ADDRESS   = mma8451q_init_ptr->SLAVE_ADDRESS;
    mma8451q_info_ptr->BURST_READ_MODE = mma8451q_init_ptr->BURST_READ_MODE;
    // mag3110 will stay in standby mode after initialize
    mma8451q_info_ptr->OPERATING_MODE  = MMA8451Q_OPERATING_MODE_STANDBY;

    if ((mma8451q_write_single_reg(mma8451q_info_ptr, MMA8451Q_XYZ_DATA_CFG,
                                  (mma8451q_init_ptr->FULL_SCALE_RANGE))) &&
        (mma8451q_write_single_reg(mma8451q_info_ptr, MMA8451Q_CTRL_REG2,
                                  (mma8451q_init_ptr->ACTIVE_POWER_SCHEME))) &&
        (mma8451q_write_single_reg(mma8451q_info_ptr, MMA8451Q_CTRL_REG1,
                                  (mma8451q_init_ptr->OUTPUT_DATA_RATE |
                                   mma8451q_init_ptr->BURST_READ_MODE ))))
    {
        return (void *) mma8451q_info_ptr;
    }
    else
    {
        /* Free info struct */
        _mem_free(mma8451q_info_ptr);
        return NULL;
    }
}

// Deinit MMA8451Q and make it working in power-saving mode.
bool mma8451q_deinit
(
    void *mma8451q_handle
)
{
    bool result;
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    if (mma8451q_handle == NULL)
        return FALSE;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    /* Recover MMA8451Q register to default value */
    if (mma8451q_write_single_reg(mma8451q_info_ptr, MMA8451Q_CTRL_REG2,0x40))
    {
        result = TRUE;
    }
    else
    {
        result = FALSE;
    }

    /* Wait for reset finish */
    _time_delay(5);

    /* Free info struct */
    _mem_free(mma8451q_info_ptr);
    mma8451q_handle = NULL;

    return result;
}

// Basic multi-Byte write function
bool mma8451q_write_reg
(
    void       *mma8451q_handle,
    uint8_t     addr,
    uint8_t    *buffer,
    uint16_t    n
)
{
    bool                  result;
    uint8_t               index, slave_address_old;
    _mqx_int              param;
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (n == 0)
        return TRUE;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    // Back up old i2c slave address
    if (I2C_OK == ioctl(mma8451q_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_GET_DESTINATION_ADDRESS, &param))
    {
        slave_address_old = (uint8_t)(param & 0xFF);
    }
    else
    {
        return FALSE;
    }

    // Set MMA8451Q slave address
    param = mma8451q_info_ptr->SLAVE_ADDRESS;
    if (I2C_OK != ioctl(mma8451q_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param))
        return FALSE;

    /* Write of data */
    if ((1 == fwrite(&addr, 1, 1, mma8451q_info_ptr->I2C_FILE_PTR)) &&
        (n == fwrite(buffer, 1, n, mma8451q_info_ptr->I2C_FILE_PTR)) &&
        (MQX_OK == ioctl(mma8451q_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_STOP, NULL)))
    {
        result = TRUE;
    }
    else
    {
        result = FALSE;
    }

    // If try to modify MMA8451Q_CTRL_REG1
    if (((addr <= MMA8451Q_CTRL_REG1) && ((addr + n - 1) >= MMA8451Q_CTRL_REG1)) && (result == TRUE))
    {
        index = (MMA8451Q_CTRL_REG1 - addr);
        if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_STANDBY)
        {
            mma8451q_info_ptr->BURST_READ_MODE = (buffer[index] & MMA8451Q_CTRL_REG1_F_READ_MASK);
        }
        mma8451q_info_ptr->OPERATING_MODE = (buffer[index] & MMA8451Q_CTRL_REG1_ACTIVE_MASK);
    }

    param = slave_address_old;
    if (I2C_OK == ioctl(mma8451q_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param))
        return result;
    else
        return FALSE;
}

// Basic multi-Byte read function
bool mma8451q_read_reg
(
    void       *mma8451q_handle,
    uint8_t     addr,
    uint8_t    *buffer,
    uint16_t    n
)
{
    bool                  result;
    uint8_t               slave_address_old;
    _mqx_int              param;
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (n == 0)
        return TRUE;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    // Back up old i2c slave address
    if (I2C_OK == ioctl(mma8451q_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_GET_DESTINATION_ADDRESS, &param))
    {
        slave_address_old = (uint8_t)(param & 0xFF);
    }
    else
    {
        return FALSE;
    }

    // Set MMA8451Q slave address
    param = mma8451q_info_ptr->SLAVE_ADDRESS;
    if (I2C_OK != ioctl(mma8451q_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param))
        return FALSE;

    /* M U S T read all data */
    if ((1 == fwrite(&addr, 1, 1, mma8451q_info_ptr->I2C_FILE_PTR)) &&
        (MQX_OK == ioctl(mma8451q_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_REPEATED_START, NULL)) &&
        (n == fread(buffer, 1, n, mma8451q_info_ptr->I2C_FILE_PTR)) &&
        (MQX_OK == ioctl(mma8451q_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_STOP, NULL)))
    {
        result = TRUE;
    }
    else
    {
        result = FALSE;
    }

    /* Restore old i2c address */
    param = slave_address_old;
    if (I2C_OK == ioctl(mma8451q_info_ptr->I2C_FILE_PTR, IO_IOCTL_I2C_SET_DESTINATION_ADDRESS, &param))
        return result;
    else
        return FALSE;
}

// Basic single byte write function
bool mma8451q_write_single_reg
(
    void      *mma8451q_handle,
    uint8_t    addr,
    uint8_t    data
)
{
    return mma8451q_write_reg(mma8451q_handle, addr, &data, 1);
}

// Basic single byte read function
bool mma8451q_read_single_reg
(
    void      *mma8451q_handle,
    uint8_t    addr,
    uint8_t   *buffer
)
{
    return mma8451q_read_reg(mma8451q_handle, addr, buffer, 1);
}

// Set MMA8451Q slave address configuration
bool mma8451q_set_slave_address
(
    void      *mma8451q_handle,
    uint8_t    slave_address
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    if ((mma8451q_handle == NULL) ||
        ((slave_address != MMA8451Q_ADDRESS_SA0_LOW) &&
        (slave_address != MMA8451Q_ADDRESS_SA0_HIGH)))
        return FALSE;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;
    mma8451q_info_ptr->SLAVE_ADDRESS = slave_address;

    return TRUE;
}

// Get MMA8451Q slave address configuration
bool mma8451q_get_slave_address
(
    void      *mma8451q_handle,
    uint8_t   *buffer
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;
    *buffer = mma8451q_info_ptr->SLAVE_ADDRESS;

    return TRUE;
}

// Useful function for higher level
bool mma8451q_modify_bitField
(
    void       *mma8451q_handle,
    uint8_t     reg_address,
    uint8_t     bit_field_mask,
    uint8_t     bit_field_value
)
{
    uint8_t               temp;

    if ((mma8451q_handle == NULL) ||
        (0 != (bit_field_value & (~bit_field_mask))))
        return FALSE;

    if (!mma8451q_read_single_reg(mma8451q_handle, reg_address, &temp))
        return FALSE;
    temp   &= (~bit_field_mask);
    temp   |= bit_field_value;
    return mma8451q_write_single_reg(mma8451q_handle, reg_address, temp);
}

// Useful function for higher level
bool mma8451q_get_bitField
(
    void       *mma8451q_handle,
    uint8_t     reg_address,
    uint8_t     bit_field_mask,
    uint8_t    *buffer
)
{
    uint8_t temp;

    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (mma8451q_read_single_reg(mma8451q_handle, reg_address, &temp))
    {
        *buffer = (temp & bit_field_mask);
        return TRUE;
    }
    else
        return FALSE;
}
