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
* FileName: mma8451q_tran.c
* Version :
* Date    : May-16-2014
*
* Comments:
*
*   MMA8451Q transient detection realization
*
*END************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "mqx.h"

#include "mma8451q_reg.h"
#include "mma8451q_prv.h"
#include "mma8451q_basic.h"
#include "mma8451q_tran.h"

bool mma8451q_set_transient_db_cnt_mode
(
    void       *mma8451q_handle,
    uint8_t     cnt_mode
)
{
    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_TRANSIENT_THS,
                                    MMA8451Q_TRANSIENT_THS_DBCNTM_MASK, cnt_mode);
}

bool mma8451q_get_transient_db_cnt_mode
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_TRANSIENT_THS,
                                 MMA8451Q_TRANSIENT_THS_DBCNTM_MASK, buffer);
}

bool mma8451q_set_transient_db_cnt
(
    void       *mma8451q_handle,
    uint8_t     cnt_value
)
{
    if (mma8451q_handle == NULL)
        return FALSE;

    return mma8451q_write_single_reg(mma8451q_handle, MMA8451Q_TRANSIENT_COUNT, cnt_value);
}

bool mma8451q_get_transient_db_cnt
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_TRANSIENT_COUNT, buffer))
    {
        return TRUE;
    }
    else
        return FALSE;
}

bool mma8451q_set_transient_threshold
(
    void       *mma8451q_handle,
    uint8_t     threshold
)
{
    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_TRANSIENT_THS,
                                    MMA8451Q_TRANSIENT_THS_THS_MASK, threshold);
}

bool mma8451q_get_transient_threshold
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_TRANSIENT_THS,
                                 MMA8451Q_TRANSIENT_THS_THS_MASK, buffer);
}

bool mma8451q_set_transient_event_latch_state
(
    void       *mma8451q_handle,
    uint8_t     latch_state
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_TRANSIENT_CFG,
                                    MMA8451Q_TRANSIENT_CFG_ELE_MASK, latch_state);
}

bool mma8451q_get_transient_event_latch_state
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_TRANSIENT_CFG,
                                 MMA8451Q_TRANSIENT_CFG_ELE_MASK, buffer);
}

bool mma8451q_set_transient_bypass_hpf
(
    void       *mma8451q_handle,
    uint8_t     bypass_state
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_TRANSIENT_CFG,
                                    MMA8451Q_TRANSIENT_CFG_HPF_BYP_MASK, bypass_state);
}

bool mma8451q_get_transient_bypass_hpf
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_TRANSIENT_CFG,
                                 MMA8451Q_TRANSIENT_CFG_HPF_BYP_MASK, buffer);
}

bool mma8451q_set_transient_state
(
    void       *mma8451q_handle,
    uint8_t     enable_state
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;
    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;

    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_TRANSIENT_CFG,
                                    MMA8451Q_TRANSIENT_CFG_ZTEFE_MASK |
                                    MMA8451Q_TRANSIENT_CFG_YTEFE_MASK |
                                    MMA8451Q_TRANSIENT_CFG_XTEFE_MASK,
                                    enable_state);
}

bool mma8451q_get_transient_state
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_TRANSIENT_CFG,
                                 MMA8451Q_TRANSIENT_CFG_ZTEFE_MASK |
                                 MMA8451Q_TRANSIENT_CFG_YTEFE_MASK |
                                 MMA8451Q_TRANSIENT_CFG_XTEFE_MASK,
                                 buffer);
}

bool mma8451q_get_transient_status
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_TRANSIENT_SCR, buffer))
    {
        return TRUE;
    }
    else
        return FALSE;
}
