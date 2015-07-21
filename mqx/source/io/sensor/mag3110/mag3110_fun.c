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
* FileName: mag3110_fun.c
* Version :
* Date    : Apr-22-2014
*
* Comments:
*
*   MAG3110 functional level IO function realization
*
*END************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "mqx.h"

#include "mag3110_reg.h"
#include "mag3110_prv.h"
#include "mag3110_basic.h"
#include "mag3110_fun.h"

bool mag3110_get_dr_status
(
    void       *mag3110_handle,
    uint8_t    *buffer
)
{
    return mag3110_read_single_reg(mag3110_handle, MAG3110_DR_STATUS, buffer);
}

bool mag3110_get_device_id
(
    void       *mag3110_handle,
    uint8_t    *buffer
)
{
    return mag3110_read_single_reg(mag3110_handle, MAG3110_WHO_AM_I, buffer);
}

bool mag3110_get_system_mode
(
    void       *mag3110_handle,
    uint8_t    *buffer
)
{
    return mag3110_read_single_reg(mag3110_handle, MAG3110_SYSMOD, buffer);
}

bool mag3110_set_user_offset
(
    void       *mag3110_handle,
    int16_t     offset_x,
    int16_t     offset_y,
    int16_t     offset_z
)
{
    uint8_t tmp_data[6];

    if((mag3110_handle == NULL) ||
       ((offset_x > 10000) || (offset_x < -10000)) ||
       ((offset_y > 10000) || (offset_y < -10000)) ||
       ((offset_z > 10000) || (offset_z < -10000)))
        return FALSE;

    tmp_data[0] = (uint8_t)((((uint16_t)(offset_x << MAG3110_OFFSET_SHIFT)) >> 8) & 0xFF);
    tmp_data[1] = (uint8_t)((((uint16_t)(offset_x << MAG3110_OFFSET_SHIFT))) & 0xFF);
    tmp_data[2] = (uint8_t)((((uint16_t)(offset_y << MAG3110_OFFSET_SHIFT)) >> 8) & 0xFF);
    tmp_data[3] = (uint8_t)((((uint16_t)(offset_y << MAG3110_OFFSET_SHIFT))) & 0xFF);
    tmp_data[4] = (uint8_t)((((uint16_t)(offset_z << MAG3110_OFFSET_SHIFT)) >> 8) & 0xFF);
    tmp_data[5] = (uint8_t)((((uint16_t)(offset_z << MAG3110_OFFSET_SHIFT))) & 0xFF);

    return mag3110_write_reg(mag3110_handle, MAG3110_OFF_X_MSB, tmp_data, 6);
}

bool mag3110_get_user_offset
(
    void       *mag3110_handle,
    int16_t    *offset_x,
    int16_t    *offset_y,
    int16_t    *offset_z
)
{
    uint8_t tmp_data[6] = {0x00,0x00,0x00,0x00,0x00,0x00};

    if((mag3110_handle == NULL) || (offset_x == NULL) || (offset_y == NULL) || (offset_z == NULL))
        return FALSE;

    if(mag3110_read_reg(mag3110_handle, MAG3110_OFF_X_MSB, tmp_data, 6))
    {
        *offset_x = (((int16_t)((tmp_data[0] << 8) + tmp_data[1])) >> MAG3110_OFFSET_SHIFT);
        *offset_y = (((int16_t)((tmp_data[2] << 8) + tmp_data[3])) >> MAG3110_OFFSET_SHIFT);
        *offset_z = (((int16_t)((tmp_data[4] << 8) + tmp_data[5])) >> MAG3110_OFFSET_SHIFT);
        return TRUE;
    }
    else
        return FALSE;
}

bool mag3110_get_temperature
(
    void       *mag3110_handle,
    int8_t     *buffer
)
{
    uint8_t temp;

    if((mag3110_handle == NULL) || (buffer == NULL))
        return FALSE;

    if(mag3110_read_single_reg(mag3110_handle, MAG3110_DIE_TEMP, &temp))
    {
        *buffer = (int8_t)temp;
        return TRUE;
    }
    else
        return FALSE;
}

bool mag3110_get_mag_data
(
    void       *mag3110_handle,
    int16_t    *data_x,
    int16_t    *data_y,
    int16_t    *data_z
)
{
    MAG3110_INFO_STRUCT *mag3110_info_ptr;
    uint8_t              tmp_data[6] = {0x00,0x00,0x00,0x00,0x00,0x00};

    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)mag3110_handle;

    if((mag3110_handle == NULL) || (data_x == NULL) || (data_y == NULL) || (data_z == NULL))
        return FALSE;

    if(mag3110_info_ptr->BURST_READ_MODE == MAG3110_BURST_READ_MODE_NORMAL)
    {
        if(mag3110_read_reg(mag3110_handle, MAG3110_OUT_X_MSB, tmp_data, 6))
        {
            *data_x = (int16_t)((tmp_data[0] << 8) + tmp_data[1]);
            *data_y = (int16_t)((tmp_data[2] << 8) + tmp_data[3]);
            *data_z = (int16_t)((tmp_data[4] << 8) + tmp_data[5]);
            return TRUE;
        }
        else
            return FALSE;
    }
    else if(mag3110_info_ptr->BURST_READ_MODE == MAG3110_BURST_READ_MODE_FAST)
    {
        if(mag3110_read_reg(mag3110_handle, MAG3110_OUT_X_MSB, tmp_data, 3))
        {
            *data_x = (int8_t)tmp_data[0];
            *data_y = (int8_t)tmp_data[1];
            *data_z = (int8_t)tmp_data[2];
            return TRUE;
        }
        else
            return FALSE;
    }
    else
    {
        return FALSE;
    }
}

bool mag3110_set_adc_sample_rate
(
    void       *mag3110_handle,
    uint8_t     adc_sample_rate
)
{
    MAG3110_INFO_STRUCT *mag3110_info_ptr;
    uint8_t              temp;

    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)mag3110_handle;
    if((mag3110_handle == NULL) ||
       (mag3110_info_ptr->OPERATING_MODE == MAG3110_OPERATING_MODE_ACTIVE) ||
       (0 != (adc_sample_rate & (~MAG3110_CTRL_REG1_DR_MASK))))
        return FALSE;

    if(!mag3110_read_single_reg(mag3110_handle, MAG3110_CTRL_REG1, &temp))
        return FALSE;
    temp   &= (~MAG3110_CTRL_REG1_DR_MASK);
    temp   |= adc_sample_rate;
    return mag3110_write_single_reg(mag3110_handle, MAG3110_CTRL_REG1, temp);
}

bool mag3110_get_adc_sample_rate
(
    void       *mag3110_handle,
    uint8_t    *buffer
)
{
    uint8_t temp;

    if((mag3110_handle == NULL) && (buffer == NULL))
        return FALSE;

    if(mag3110_read_single_reg(mag3110_handle, MAG3110_CTRL_REG1, &temp))
    {
        *buffer = (temp & MAG3110_CTRL_REG1_DR_MASK);
        return TRUE;
    }
    else
        return FALSE;
}

bool mag3110_set_over_sample_ratio
(
    void       *mag3110_handle,
    uint8_t     over_sample_ratio
)
{
    MAG3110_INFO_STRUCT *mag3110_info_ptr;
    uint8_t              temp;

    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)mag3110_handle;
    if((mag3110_handle == NULL) ||
       (mag3110_info_ptr->OPERATING_MODE == MAG3110_OPERATING_MODE_ACTIVE) ||
       (0 != (over_sample_ratio & (~MAG3110_CTRL_REG1_OS_MASK))))
        return FALSE;

    if(!mag3110_read_single_reg(mag3110_handle, MAG3110_CTRL_REG1, &temp))
        return FALSE;
    temp   &= (~MAG3110_CTRL_REG1_OS_MASK);
    temp   |= over_sample_ratio;
    return mag3110_write_single_reg(mag3110_handle, MAG3110_CTRL_REG1, temp);
}

bool mag3110_get_over_sample_ratio
(
    void       *mag3110_handle,
    uint8_t    *buffer
)
{
    uint8_t temp;

    if((mag3110_handle == NULL) || (buffer == NULL))
        return FALSE;

    if(mag3110_read_single_reg(mag3110_handle, MAG3110_CTRL_REG1, &temp))
    {
        *buffer = (temp & MAG3110_CTRL_REG1_OS_MASK);
        return TRUE;
    }
    else
        return FALSE;
}

bool mag3110_set_burst_read_mode
(
    void       *mag3110_handle,
    uint8_t     read_mode
)
{
    MAG3110_INFO_STRUCT *mag3110_info_ptr;
    uint8_t              temp;

    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)mag3110_handle;
    if((mag3110_handle == NULL) ||
       (mag3110_info_ptr->OPERATING_MODE == MAG3110_OPERATING_MODE_ACTIVE) ||
       (0 != (read_mode & (~MAG3110_CTRL_REG1_FR_MASK))))
        return FALSE;

    if(mag3110_info_ptr->BURST_READ_MODE == read_mode)
        return TRUE;

    if(!mag3110_read_single_reg(mag3110_handle, MAG3110_CTRL_REG1, &temp))
        return FALSE;
    temp   &= (~MAG3110_CTRL_REG1_FR_MASK);
    temp   |= read_mode;
    return mag3110_write_single_reg(mag3110_handle, MAG3110_CTRL_REG1, temp);
}

bool mag3110_get_burst_read_mode
(
    void       *mag3110_handle,
    uint8_t    *buffer
)
{
    MAG3110_INFO_STRUCT *mag3110_info_ptr;

    if(mag3110_handle == NULL)
        return FALSE;
    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)mag3110_handle;
    *buffer = mag3110_info_ptr->BURST_READ_MODE;

    return TRUE;
}

bool mag3110_set_operating_mode
(
    void       *mag3110_handle,
    uint8_t     operating_mode
)
{
    MAG3110_INFO_STRUCT *mag3110_info_ptr;
    uint8_t              temp;

    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)mag3110_handle;
    if((mag3110_handle == NULL) || (0 != (operating_mode & (~MAG3110_CTRL_REG1_AC_MASK))))
        return FALSE;

    if(mag3110_info_ptr->OPERATING_MODE == operating_mode)
        return TRUE;

    if(!mag3110_read_single_reg(mag3110_handle, MAG3110_CTRL_REG1, &temp))
        return FALSE;
    temp   &= (~MAG3110_CTRL_REG1_AC_MASK);
    temp   |= operating_mode;
    if(mag3110_write_single_reg(mag3110_handle, MAG3110_CTRL_REG1, temp))
    {
        mag3110_info_ptr->OPERATING_MODE = operating_mode;
        return TRUE;
    }
    else
        return FALSE;
}

bool mag3110_get_operating_mode
(
    void       *mag3110_handle,
    uint8_t    *buffer
)
{
    MAG3110_INFO_STRUCT *mag3110_info_ptr;

    if((mag3110_handle == NULL) || (buffer == NULL))
        return FALSE;

    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)mag3110_handle;
    *buffer = mag3110_info_ptr->OPERATING_MODE;

    return TRUE;
}

bool mag3110_set_auto_mrst
(
    void       *mag3110_handle,
    uint8_t     auto_reset
)
{
    MAG3110_INFO_STRUCT *mag3110_info_ptr;
    uint8_t              temp;
    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)mag3110_handle;
    if((mag3110_handle == NULL) || (mag3110_info_ptr->OPERATING_MODE == MAG3110_OPERATING_MODE_ACTIVE) ||
       (0 != (auto_reset & (~MAG3110_CTRL_REG2_AUTO_MRST_EN_MASK))))
        return FALSE;

    if(!mag3110_read_single_reg(mag3110_handle, MAG3110_CTRL_REG2, &temp))
        return FALSE;
    temp   &= (~MAG3110_CTRL_REG2_AUTO_MRST_EN_MASK);
    temp   |= auto_reset;
    return mag3110_write_single_reg(mag3110_handle, MAG3110_CTRL_REG2, temp);
}

bool mag3110_set_output_correction
(
    void       *mag3110_handle,
    uint8_t     output_correction
)
{
    MAG3110_INFO_STRUCT *mag3110_info_ptr;
    uint8_t              temp;

    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)mag3110_handle;
    if((mag3110_handle == NULL) || (mag3110_info_ptr->OPERATING_MODE == MAG3110_OPERATING_MODE_ACTIVE)  ||
       (0 != (output_correction & (~MAG3110_CTRL_REG2_RAW_MASK))))
        return FALSE;

    if(!mag3110_read_single_reg(mag3110_handle, MAG3110_CTRL_REG2, &temp))
        return FALSE;
    temp   &= (~MAG3110_CTRL_REG2_RAW_MASK);
    temp   |= output_correction;
    return mag3110_write_single_reg(mag3110_handle, MAG3110_CTRL_REG2, temp);
}

bool mag3110_get_output_correction
(
    void       *mag3110_handle,
    uint8_t    *buffer
)
{
    uint8_t temp;

    if(mag3110_handle == NULL)
        return FALSE;

    if(mag3110_read_single_reg(mag3110_handle, MAG3110_CTRL_REG2, &temp))
    {
        *buffer = (temp & MAG3110_CTRL_REG2_RAW_MASK);
        return TRUE;
    }
    else
        return FALSE;
}

bool mag3110_trigger_measurement
(
    void       *mag3110_handle
)
{
    uint8_t temp;

    if(mag3110_handle == NULL)
        return FALSE;

    if(!mag3110_read_single_reg(mag3110_handle, MAG3110_CTRL_REG1, &temp))
        return FALSE;
    temp   |= MAG3110_CTRL_REG1_TM_MASK;
    return mag3110_write_single_reg(mag3110_handle, MAG3110_CTRL_REG1, temp);
}

bool mag3110_reset_mag_sensor
(
    void       *mag3110_handle
)
{
    MAG3110_INFO_STRUCT *mag3110_info_ptr;
    uint8_t              temp;

    mag3110_info_ptr = (MAG3110_INFO_STRUCT *)mag3110_handle;
    if((mag3110_handle == NULL) || (mag3110_info_ptr->OPERATING_MODE == MAG3110_OPERATING_MODE_ACTIVE))
        return FALSE;

    if(!mag3110_read_single_reg(mag3110_handle, MAG3110_CTRL_REG2, &temp))
        return FALSE;
    temp   |= MAG3110_CTRL_REG2_MAG_RST_MASK;
    return mag3110_write_single_reg(mag3110_handle, MAG3110_CTRL_REG2, temp);
}

bool mag3110_get_reset_status
(
    void       *mag3110_handle,
    uint8_t    *buffer
)
{
    uint8_t temp;

    if((mag3110_handle == NULL) || (buffer == NULL))
        return FALSE;

    if(mag3110_read_single_reg(mag3110_handle, MAG3110_CTRL_REG2, &temp))
    {
        *buffer = (temp & MAG3110_CTRL_REG2_MAG_RST_MASK);
        return TRUE;
    }
    else
        return FALSE;
}
