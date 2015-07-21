/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __FSL_FLEXCAN_PRV_H__
#define __FSL_FLEXCAN_PRV_H__

#include <mqx.h>
#include <psp.h>
#include <lwevent.h>
#include "fsl_flexcan_hal.h"
#include "fsl_flexcan_prv.h"
//! @addtogroup flexcan_driver
//! @{

//! @file

/////////////////////////////////////////////////////////////////////////////
// Definitions
/////////////////////////////////////////////////////////////////////////////

//! @brief FlexCAN bit rate and the related timing segments structure
typedef struct flexcan_dev_info {
    void* flexcan_clk;
    void* flexcan_ipg_clk;
    bool  int_mb;
    bool  int_fifo;
    uint32_t rx_mb_idx;
    LWEVENT_STRUCT event;
} flexcan_dev_info_t, *flexcan_dev_info_ptr;
#ifdef __cplusplus
extern "C" {
#endif
void *_bsp_get_flexcan_base_address(uint8_t dev_num);
uint32_t _bsp_get_flexcan_vector(uint8_t dev_num, uint8_t vector_type, uint32_t vector_index);
CLOCK_NAME _bsp_get_flexcan_clock_name(uint8_t dev_num);
CLOCK_NAME _bsp_get_flexcan_clock_ipg_name(uint8_t dev_num);
flexcan_dev_info_ptr _bsp_get_flexcan_dev_info(uint8_t dev_num);
uint32_t _bsp_flexcan_enter_stop_mode(uint8_t dev_num);
uint32_t _bsp_flexcan_exit_stop_mode(uint8_t dev_num);
#ifdef __cplusplus
}
#endif
#endif // __FSL_FLEXCAN_PRV_H__

