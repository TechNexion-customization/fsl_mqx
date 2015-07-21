#ifndef __mma8451q_prv_h__
#define __mma8451q_prv_h__
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
* FileName: mma8451q_prv.h
* Version :
* Date    : May-5-2014
*
* Comments:
*
*   This file includes the private definitions for the mma8451q
*   magnetometer drivers.
*
*END************************************************************************/

#include <stdint.h>
#include <stdbool.h>

#include "mqx.h"
#include "bsp.h"

typedef struct mma8451q_info_struct
{
    /* i2c device bus handle */
    MQX_FILE_PTR    I2C_FILE_PTR;

    /* MAG3110 I2C Slave address */
    uint8_t         SLAVE_ADDRESS;

    /* Actual burst read mode */
    uint8_t         BURST_READ_MODE;

    /* Actual working mode */
    uint8_t         OPERATING_MODE;

} MMA8451Q_INFO_STRUCT;

#endif
