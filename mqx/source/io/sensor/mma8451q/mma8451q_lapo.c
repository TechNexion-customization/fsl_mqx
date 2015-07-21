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
* FileName: mma8451q_lapo.c
* Version :
* Date    : May-16-2014
*
* Comments:
*
*   MMA8451Q Portrait/Landscape detection realization
*
*END************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "mqx.h"

#include "mma8451q_reg.h"
#include "mma8451q_prv.h"
#include "mma8451q_basic.h"
#include "mma8451q_lapo.h"

bool mma8451q_set_lapo_db_cnt_mode
(
    void       *mma8451q_handle,
    uint8_t     cnt_mode
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;
    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_PL_CFG,
                                    MMA8451Q_PL_CFG_DBCNTM_MASK, cnt_mode);
}

bool mma8451q_get_lapo_db_cnt_mode
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_PL_CFG,
                                 MMA8451Q_PL_CFG_DBCNTM_MASK, buffer);
}

bool mma8451q_set_lapo_db_cnt
(
    void       *mma8451q_handle,
    uint8_t     cnt_value
)
{
    if (mma8451q_handle == NULL)
        return FALSE;

    return mma8451q_write_single_reg(mma8451q_handle, MMA8451Q_PL_COUNT, cnt_value);
}

bool mma8451q_get_lapo_db_cnt
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_PL_COUNT, buffer))
    {
        return TRUE;
    }
    else
        return FALSE;
}

bool mma8451q_set_back_front_threshold
(
    void       *mma8451q_handle,
    uint8_t     threshold
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;
    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_PL_BF_ZCOMP,
                                    MMA8451Q_PL_BF_ZCOMP_BKFR_MASK, threshold);
}

bool mma8451q_get_back_front_threshold
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_PL_BF_ZCOMP,
                                 MMA8451Q_PL_BF_ZCOMP_BKFR_MASK, buffer);
}

bool mma8451q_set_lapo_threshold
(
    void       *mma8451q_handle,
    uint8_t     threshold
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;
    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_P_L_THS_REG,
                                    MMA8451Q_P_L_THS_REG_THS_MASK, threshold);
}

bool mma8451q_get_lapo_threshold
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_P_L_THS_REG,
                                 MMA8451Q_P_L_THS_REG_THS_MASK, buffer);
}

bool mma8451q_set_z_lock_threshold
(
    void       *mma8451q_handle,
    uint8_t     threshold
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;
    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_PL_BF_ZCOMP,
                                    MMA8451Q_PL_BF_ZCOMP_ZLOCK_MASK, threshold);
}

bool mma8451q_get_z_lock_threshold
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_PL_BF_ZCOMP,
                                 MMA8451Q_PL_BF_ZCOMP_ZLOCK_MASK, buffer);
}

bool mma8451q_set_lapo_trip_hys
(
    void       *mma8451q_handle,
    uint8_t     hysteresis
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;
    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_P_L_THS_REG,
                                    MMA8451Q_P_L_THS_REG_HYS_MASK, hysteresis);
}

bool mma8451q_get_lapo_trip_hys
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_P_L_THS_REG,
                                 MMA8451Q_P_L_THS_REG_HYS_MASK, buffer);
}

bool mma8451q_set_lapo_state
(
    void       *mma8451q_handle,
    uint8_t     lapo_enabled
)
{
    MMA8451Q_INFO_STRUCT *mma8451q_info_ptr;

    mma8451q_info_ptr = (MMA8451Q_INFO_STRUCT *)mma8451q_handle;
    if (mma8451q_info_ptr->OPERATING_MODE == MMA8451Q_OPERATING_MODE_ACTIVE)
        return FALSE;

    return mma8451q_modify_bitField(mma8451q_handle, MMA8451Q_PL_CFG,
                                    MMA8451Q_PL_CFG_PL_EN_MASK, lapo_enabled);
}

bool mma8451q_get_lapo_state
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    return mma8451q_get_bitField(mma8451q_handle, MMA8451Q_PL_CFG,
                                 MMA8451Q_PL_CFG_PL_EN_MASK, buffer);
}

bool mma8451q_get_lapo_status
(
    void       *mma8451q_handle,
    uint8_t    *buffer
)
{
    if ((mma8451q_handle == NULL) || (buffer == NULL))
        return FALSE;

    if (mma8451q_read_single_reg(mma8451q_handle, MMA8451Q_PL_STATUS, buffer))
    {
        return TRUE;
    }
    else
        return FALSE;
}
