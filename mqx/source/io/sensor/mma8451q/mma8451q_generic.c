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
* FileName: mma8451q_generic.c
* Version :
* Date    : May-16-2014
*
* Comments:
*
*   MMA8451Q functional level IO generic function realization
*
*END************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "mqx.h"

#include "mma8451q_reg.h"
#include "mma8451q_prv.h"
#include "mma8451q_basic.h"
#include "mma8451q_generic.h"

bool mma8451q_get_dr_status
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_STATUS, buffer);
}

bool mma8451q_get_device_id
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_WHO_AM_I, buffer);
}

bool mma8451q_get_system_mode
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_SYSMOD,
                                 MMA8451Q_SYSMOD_SYSMOD_MASK, buffer);
}

bool mma8451q_set_output_data_rate
(
    void       *mma8451q_handle,
    uint8_t     output_rate
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_CTRL_REG1,
                                    MMA8451Q_CTRL_REG1_DR_MASK, output_rate);
}

bool mma8451q_get_output_data_rate
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_CTRL_REG1,
                                 MMA8451Q_CTRL_REG1_DR_MASK, buffer);
}

bool mma8451q_set_power_scheme
(
    void       *mma8451q_handle,
    uint8_t     power_scheme
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_CTRL_REG2,
                                    MMA8451Q_CTRL_REG2_MODS_MASK, power_scheme);
}

bool mma8451q_get_power_scheme
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_CTRL_REG2,
                                 MMA8451Q_CTRL_REG2_MODS_MASK, buffer);
}

bool mma8451q_set_full_scale_range
(
    void       *mma8451q_handle,
    uint8_t     full_scale
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_XYZ_DATA_CFG,
                                    MMA8451Q_XYZ_DATA_CFG_FS_MASK, full_scale);
}

bool mma8451q_get_full_scale_range
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_XYZ_DATA_CFG,
                                 MMA8451Q_XYZ_DATA_CFG_FS_MASK, buffer);
}

bool mma8451q_set_burst_read_mode
(
    void       *mma8451q_handle,
    uint8_t     read_mode
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_CTRL_REG1,
                                    MMA8451Q_CTRL_REG1_F_READ_MASK, read_mode);
}

bool mma8451q_get_burst_read_mode
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_CTRL_REG1,
                                 MMA8451Q_CTRL_REG1_F_READ_MASK, buffer);
}

bool mma8451q_set_user_offset
(
    void       *mma8451q_handle,
    int8_t      offset_x,
    int8_t      offset_y,
    int8_t      offset_z
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    uint8_t               tmp_data[3];

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;
    if ((mma8451q_handle == NULL) ||
        (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE))
        return FALSE;

    tmp_data[0] = (uint8_t)offset_x;
    tmp_data[1] = (uint8_t)offset_y;
    tmp_data[2] = (uint8_t)offset_z;

    return mma8451q_write_reg(mma8451q_handle, MMA8451Q_OFF_X, tmp_data, 3);
}

bool mma8451q_get_user_offset
(
    void       *mma8451q_handle,
    int8_t     *offset_x,
    int8_t     *offset_y,
    int8_t     *offset_z
)
{
    uint8_t tmp_data[3] = {0x00,0x00,0x00};

    if ((mma8451q_handle == NULL) || (offset_x == NULL) || (offset_y == NULL) || (offset_z == NULL))
        return FALSE;

    if (mma8451q_read_reg(mma8451q_handle, MMA8451Q_OFF_X, tmp_data, 3))
    {
        *offset_x = (int8_t)tmp_data[0];
        *offset_y = (int8_t)tmp_data[1];
        *offset_z = (int8_t)tmp_data[2];
        return TRUE;
    }
    else
        return FALSE;
}

bool mma8451q_get_acc_data
(
    void       *mma8451q_handle,
    int16_t    *data_x,
    int16_t    *data_y,
    int16_t    *data_z
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    uint8_t               tmp_data[6] = {0x00,0x00,0x00,0x00,0x00,0x00};

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if ((mma8451q_handle == NULL) || (data_x == NULL) || (data_y == NULL) || (data_z == NULL))
        return FALSE;

    if (mma8451q_info_ptr->BURST_READ_MODE == MMA8451Q_BURST_READ_MODE_NORMAL)
    {
        if (mma8451q_read_reg(mma8451q_handle, MMA8451Q_OUT_X_MSB, tmp_data, 6))
        {
            *data_x = (((int16_t)((tmp_data[0] << 8) + tmp_data[1])) >> MMA8451Q_OUT_DATA_SHIFT);
            *data_y = (((int16_t)((tmp_data[2] << 8) + tmp_data[3])) >> MMA8451Q_OUT_DATA_SHIFT);
            *data_z = (((int16_t)((tmp_data[4] << 8) + tmp_data[5])) >> MMA8451Q_OUT_DATA_SHIFT);
            return TRUE;
        }
        else
            return FALSE;
    }
    else if (mma8451q_info_ptr->BURST_READ_MODE == MMA8451Q_BURST_READ_MODE_FAST)
    {
        if (mma8451q_read_reg(mma8451q_handle, MMA8451Q_OUT_X_MSB, tmp_data, 3))
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

bool mma8451q_set_self_test_state
(
    void       *mma8451q_handle,
    uint8_t     st_enabled
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_CTRL_REG2,
                                    MMA8451Q_CTRL_REG2_ST_MASK, st_enabled);
}

bool mma8451q_get_self_test_state
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_CTRL_REG2,
                                 MMA8451Q_CTRL_REG2_ST_MASK, buffer);
}

bool mma8451q_reset_sensor
(
    void       *mma8451q_handle
)
{
    uint8_t temp;

    if (mma8451q_handle == NULL)
        return FALSE;

    if (!mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_CTRL_REG2, &temp))
        return FALSE;
    temp |= MMA8451Q_CTRL_REG2_RST_MASK;
    return mma8451q_write_single_reg(mma8451q_handle, MMA8451Q_CTRL_REG2, temp);
}

bool mma8451q_get_senor_reset_state
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_CTRL_REG2,
                                 MMA8451Q_CTRL_REG2_RST_MASK, buffer);
}

bool mma8451q_set_operating_mode
(
    void       *mma8451q_handle,
    uint8_t     operating_mode
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;
    if (mma8451q_info_ptr->OPERATING_MODE == operating_mode)
        return TRUE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_CTRL_REG1,
                                    MMA8451Q_CTRL_REG1_ACTIVE_MASK, operating_mode);
}

bool mma8451q_get_operating_mode
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;
    *buffer = mma8451q_info_ptr->OPERATING_MODE;

    return TRUE;
}

bool mma8451q_set_fifo_watermark
(
    void       *mma8451q_handle,
    uint8_t     watermark
)
{
    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_F_SETUP,
                                    MMA8451Q_F_SETUP_WMRK_MASK, watermark);
}

bool mma8451q_get_fifo_watermark
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_F_SETUP,
                                 MMA8451Q_F_SETUP_WMRK_MASK, buffer);
}

bool mma8451q_set_fifo_mode
(
    void       *mma8451q_handle,
    uint8_t     fifo_mode
)
{
    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_F_SETUP,
                                    MMA8451Q_F_SETUP_MODE_MASK, fifo_mode);
}

bool mma8451q_get_fifo_mode
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_F_SETUP,
                                 MMA8451Q_F_SETUP_MODE_MASK, buffer);
}

bool mma8451q_set_fifo_trigger_source
(
    void       *mma8451q_handle,
    uint8_t     trigger_source
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_TRIG_CFG,
                                    MMA9451Q_TRIG_CFG_MASK, trigger_source);
}

bool mma8451q_get_fifo_trigger_source
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_TRIG_CFG,
                                 MMA9451Q_TRIG_CFG_MASK, buffer);
}

bool mma8451q_set_fifo_gate
(
    void       *mma8451q_handle,
    uint8_t     fifo_gate
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_CTRL_REG3,
                                    MMA8451Q_CTRL_REG3_FIFO_GATE_MASK, fifo_gate);
}

bool mma8451q_get_fifo_gate
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_CTRL_REG3,
                                 MMA8451Q_CTRL_REG3_FIFO_GATE_MASK, buffer);
}

bool mma8451q_get_fifo_status
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    uint8_t temp;

    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_F_STATUS, &temp))
    {
        *buffer = temp;
        return TRUE;
    }
    else
        return FALSE;
}

bool mma8451q_get_fifo_count
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    uint8_t temp;

    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_F_STATUS, &temp))
    {
        *buffer = (temp & MMA8451Q_F_STATUS_CNT_MASK);
        return TRUE;
    }
    else
        return FALSE;
}

bool mma8451q_get_acc_from_fifo
(
    void       *mma8451q_handle,
    uint8_t    *buffer,
    uint8_t     n
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    uint8_t               fifo_cnt;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;
    if ((!mma8451q_get_fifo_count(mma8451q_handle, &fifo_cnt)) ||
        (n > fifo_cnt))
        return FALSE;

    fifo_cnt = 3 * n;

    if(mma8451q_info_ptr->BURST_READ_MODE == MMA8451Q_BURST_READ_MODE_NORMAL)
    {
        if(mma8451q_read_reg(mma8451q_handle, MMA8451Q_OUT_X_MSB, buffer, 2 * fifo_cnt))
        {
            return TRUE;
        }
        else
            return FALSE;
    }
    else if(mma8451q_info_ptr->BURST_READ_MODE == MMA8451Q_BURST_READ_MODE_FAST)
    {
        if(mma8451q_read_reg(mma8451q_handle, MMA8451Q_OUT_X_MSB, buffer, fifo_cnt))
        {
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

bool mma8451q_set_int_polarity
(
    void       *mma8451q_handle,
    uint8_t     polarity
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_CTRL_REG3,
                                    MMA8451Q_CTRL_REG3_IPOL_MASK, polarity);
}

bool mma8451q_get_int_polarity
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_CTRL_REG3,
                                 MMA8451Q_CTRL_REG3_IPOL_MASK, buffer);
}

bool mma8451q_set_int_output_mode
(
    void       *mma8451q_handle,
    uint8_t     output_mode
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_CTRL_REG3,
                                    MMA8451Q_CTRL_REG3_PP_OD_MASK, output_mode);
}

bool mma8451q_get_int_output_mode
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_CTRL_REG3,
                                 MMA8451Q_CTRL_REG3_PP_OD_MASK, buffer);
}

bool mma8451q_set_int_pin_route
(
    void       *mma8451q_handle,
    uint8_t     pin_route
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;
    if ((mma8451q_handle == NULL) ||
        (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE))
        return FALSE;

    return mma8451q_write_single_reg(mma8451q_handle, MMA8451Q_CTRL_REG5, pin_route);
}

bool mma8451q_get_int_pin_route
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    uint8_t temp;

    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_CTRL_REG5, &temp))
    {
        *buffer = temp;
        return TRUE;
    }
    else
        return FALSE;
}

bool mma8451q_set_int_state
(
    void       *mma8451q_handle,
    uint8_t     int_enabled
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;
    if ((mma8451q_handle == NULL) ||
        (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE) ||
        (0 != (int_enabled & (~MMA8451Q_CTRL_REG4_INT_EN_MASK))))
        return FALSE;

    return mma8451q_write_single_reg(mma8451q_handle, MMA8451Q_CTRL_REG4, int_enabled);
}

bool mma8451q_get_int_state
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    uint8_t temp;

    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_CTRL_REG4, &temp))
    {
        *buffer = (temp & MMA8451Q_CTRL_REG4_INT_EN_MASK);
        return TRUE;
    }
    else
        return FALSE;
}

bool mma8451q_get_int_source
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    uint8_t temp;

    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_INT_SOURCE, &temp))
    {
        *buffer = (temp & MMA8451Q_INT_SOURCE_MASK);
        return TRUE;
    }
    else
        return FALSE;
}

bool mma8451q_set_aslp_output_data_rate
(
    void       *mma8451q_handle,
    uint8_t     output_rate
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_CTRL_REG1,
                                    MMA8451Q_CTRL_REG1_ASLP_RATE_MASK, output_rate);
}

bool mma8451q_get_aslp_output_data_rate
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_CTRL_REG1,
                                 MMA8451Q_CTRL_REG1_ASLP_RATE_MASK, buffer);
}

bool mma8451q_set_aslp_power_scheme
(
    void       *mma8451q_handle,
    uint8_t     power_scheme
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_CTRL_REG2,
                                    MMA8451Q_CTRL_REG2_SMODS_MASK, power_scheme);
}

bool mma8451q_get_aslp_power_scheme
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_CTRL_REG2,
                                 MMA8451Q_CTRL_REG2_SMODS_MASK, buffer);
}

bool mma8451q_set_wake_up_bypass
(
    void       *mma8451q_handle,
    uint8_t     wake_up_bypass
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_CTRL_REG3,
                                    MMA8451Q_CTRL_REG3_WAKE_MASK, wake_up_bypass);
}

bool mma8451q_get_wake_up_bypass
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_CTRL_REG3,
                                 MMA8451Q_CTRL_REG3_WAKE_MASK, buffer);
}

bool mma8451q_set_aslp_count
(
    void       *mma8451q_handle,
    uint8_t     aslp_count
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;
    if ((mma8451q_handle == NULL) ||
        (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE))
        return FALSE;

    return mma8451q_write_single_reg(mma8451q_handle, MMA8451Q_ASLP_COUNT, aslp_count);
}

bool mma8451q_get_aslp_count
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_ASLP_COUNT, buffer))
    {
        return TRUE;
    }
    else
        return FALSE;
}

bool mma8451q_set_aslp_state
(
    void       *mma8451q_handle,
    uint8_t     aslp_enabled
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_CTRL_REG2,
                                    MMA8451Q_CTRL_REG2_SLPE_MASK, aslp_enabled);
}

bool mma8451q_get_aslp_state
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_CTRL_REG2,
                                 MMA8451Q_CTRL_REG2_SLPE_MASK, buffer);
}

bool mma8451q_set_low_noise_state
(
    void      *mma8451q_handle,
    uint8_t    lnoise_enabled
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_CTRL_REG1,
                                    MMA8451Q_CTRL_REG1_LNOISE_MASK, lnoise_enabled);
}

bool mma8451q_get_low_noise_state
(
    void      *mma8451q_handle,
    uint8_t   *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_CTRL_REG1,
                                 MMA8451Q_CTRL_REG1_LNOISE_MASK, buffer);
}

bool mma8451q_set_hpf_cutoff
(
    void      *mma8451q_handle,
    uint8_t    hpf_cutoff
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_HP_FILTER_CUTOFF,
                                    MMA8451Q_HP_FILTER_CUTOFF_SEL_MASK, hpf_cutoff);
}

bool mma8451q_get_hpf_cutoff
(
    void      *mma8451q_handle,
    uint8_t   *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_HP_FILTER_CUTOFF,
                                 MMA8451Q_HP_FILTER_CUTOFF_SEL_MASK, buffer);
}

bool mma8451q_set_hpf_state
(
    void      *mma8451q_handle,
    uint8_t    hpf_enabled
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_XYZ_DATA_CFG,
                                    MMA8451Q_XYZ_DATA_CFG_HPF_OUT_MASK, hpf_enabled);
}

bool mma8451q_get_hpf_state
(
    void      *mma8451q_handle,
    uint8_t   *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_XYZ_DATA_CFG,
                                 MMA8451Q_XYZ_DATA_CFG_HPF_OUT_MASK, buffer);
}
