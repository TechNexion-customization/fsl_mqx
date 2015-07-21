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
* FileName: mma8451q_pulse.c
* Version :
* Date    : May-16-2014
*
* Comments:
*
*   MMA8451Q pulse detection realization
*
*END************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "mqx.h"

#include "mma8451q_reg.h"
#include "mma8451q_prv.h"
#include "mma8451q_basic.h"
#include "mma8451q_pulse.h"

bool mma8451q_set_double_pulse_abort
(
    void       *mma8451q_handle,
    uint8_t     abort_sel
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_PULSE_CFG,
                                    MMA8451Q_PULSE_CFG_DPA_MASK, abort_sel);
}

bool mma8451q_get_double_pulse_abort
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_PULSE_CFG,
                                 MMA8451Q_PULSE_CFG_DPA_MASK, buffer);
}

bool mma8451q_set_pulse_threshold
(
    void       *mma8451q_handle,
    uint8_t     threshold,
    uint8_t     axis
)
{
    uint8_t               ths_address;
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    switch (axis)
    {
        case MMA8451Q_PULSE_AXIS_X:
            ths_address = MMA8451Q_PULSE_THSX;
            break;
        case MMA8451Q_PULSE_AXIS_Y:
            ths_address = MMA8451Q_PULSE_THSY;
            break;
        case MMA8451Q_PULSE_AXIS_Z:
            ths_address = MMA8451Q_PULSE_THSZ;
            break;
        default:
            return FALSE;
    }

    return mma8451q_modify_bitField(mma8451q_handle, ths_address,
                                    MMA8451Q_PULSE_THS_MASK, threshold);
}

bool mma8451q_get_pulse_threshold
(
    void       *mma8451q_handle,
    uint8_t    *buffer,
    uint8_t     axis
)
{
    uint8_t ths_address;

    switch (axis)
    {
        case MMA8451Q_PULSE_AXIS_X:
            ths_address = MMA8451Q_PULSE_THSX;
            break;
        case MMA8451Q_PULSE_AXIS_Y:
            ths_address = MMA8451Q_PULSE_THSY;
            break;
        case MMA8451Q_PULSE_AXIS_Z:
            ths_address = MMA8451Q_PULSE_THSZ;
            break;
        default:
            return FALSE;
    }

    return mma8451q_get_bitField(mma8451q_handle, ths_address,
                                 MMA8451Q_PULSE_THS_MASK, buffer);
}

bool mma8451q_set_pulse_time_limit
(
    void       *mma8451q_handle,
    uint8_t     time_limit
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if ((mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE) ||
        (mma8451q_handle == NULL))
        return FALSE;

    return mma8451q_write_single_reg(mma8451q_handle, MMA8451Q_PULSE_TMLT, time_limit);
}

bool mma8451q_get_pulse_time_limit
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_PULSE_TMLT, buffer))
    {
        return TRUE;
    }
    else
        return FALSE;
}

bool mma8451q_set_pulse_latency
(
    void       *mma8451q_handle,
    uint8_t     latency
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if ((mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE) ||
        (mma8451q_handle == NULL))
        return FALSE;

    return mma8451q_write_single_reg(mma8451q_handle, MMA8451Q_PULSE_LTCY, latency);
}

bool mma8451q_get_pulse_latency
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_PULSE_LTCY, buffer))
    {
        return TRUE;
    }
    else
        return FALSE;
}

bool mma8451q_set_pulse_time_window
(
    void       *mma8451q_handle,
    uint8_t     time_window
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if ((mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE) ||
        (mma8451q_handle == NULL))
        return FALSE;

    return mma8451q_write_single_reg(mma8451q_handle, MMA8451Q_PULSE_WIND, time_window);
}

bool mma8451q_get_pulse_time_window
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_PULSE_WIND, buffer))
    {
        return TRUE;
    }
    else
        return FALSE;
}

bool mma8451q_set_pulse_event_latch_state
(
    void       *mma8451q_handle,
    uint8_t     latch_state
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_PULSE_CFG,
                                    MMA8451Q_PULSE_CFG_ELE_MASK, latch_state);
}

bool mma8451q_get_pulse_event_latch_state
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_PULSE_CFG,
                                 MMA8451Q_PULSE_CFG_ELE_MASK, buffer);
}

bool mma8451q_set_pulse_hpf_state
(
    void       *mma8451q_handle,
    uint8_t     bypass_state
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_HP_FILTER_CUTOFF,
                                    MMA8451Q_HP_FILTER_CUTOFF_P_HPF_BYP_MASK, bypass_state);
}

bool mma8451q_get_pulse_hpf_state
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_HP_FILTER_CUTOFF,
                                 MMA8451Q_HP_FILTER_CUTOFF_P_HPF_BYP_MASK, buffer);
}

bool mma8451q_set_pulse_lpf_state
(
    void       *mma8451q_handle,
    uint8_t     bypass_state
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_HP_FILTER_CUTOFF,
                                    MMA8451Q_HP_FILTER_CUTOFF_P_LPF_EN_MASK, bypass_state);
}

bool mma8451q_get_pulse_lpf_state
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_HP_FILTER_CUTOFF,
                                 MMA8451Q_HP_FILTER_CUTOFF_P_LPF_EN_MASK, buffer);
}

bool mma8451q_set_pulse_detect_state
(
    void       *mma8451q_handle,
    uint8_t     tap_state
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_PULSE_CFG,
                                    MMA8451Q_PULSE_CFG_ZDPEFE_MASK |
                                    MMA8451Q_PULSE_CFG_ZSPEFE_MASK |
                                    MMA8451Q_PULSE_CFG_YDPEFE_MASK |
                                    MMA8451Q_PULSE_CFG_YSPEFE_MASK |
                                    MMA8451Q_PULSE_CFG_XDPEFE_MASK |
                                    MMA8451Q_PULSE_CFG_XSPEFE_MASK,
                                    tap_state);
}

bool mma8451q_get_pulse_detect_state
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_PULSE_CFG,
                                 MMA8451Q_PULSE_CFG_ZDPEFE_MASK |
                                 MMA8451Q_PULSE_CFG_ZSPEFE_MASK |
                                 MMA8451Q_PULSE_CFG_YDPEFE_MASK |
                                 MMA8451Q_PULSE_CFG_YSPEFE_MASK |
                                 MMA8451Q_PULSE_CFG_XDPEFE_MASK |
                                 MMA8451Q_PULSE_CFG_XSPEFE_MASK,
                                 buffer);
}

bool mma8451q_get_pulse_detect_status
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_PULSE_SRC, buffer))
    {
        return TRUE;
    }
    else
        return FALSE;
}
