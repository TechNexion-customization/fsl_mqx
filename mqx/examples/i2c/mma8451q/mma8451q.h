#ifndef __mma8451q_h__
#define __mma8451q_h__
/**HEADER*******************************************************************
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
* FileName: mma8451q.h
* Version : 3.8.6.0
* Date    : Apr-10-2014
*
* Comments:
*
*   This file contains definitions for the I2C master example.
*
*END************************************************************************/
/* register enum for mma8451 registers */
enum
{
    MMA8451Q_STATUS = 0x00,
    MMA8451Q_OUT_X_MSB,
    MMA8451Q_OUT_X_LSB,
    MMA8451Q_OUT_Y_MSB,
    MMA8451Q_OUT_Y_LSB,
    MMA8451Q_OUT_Z_MSB,
    MMA8451Q_OUT_Z_LSB,

    MMA8451Q_F_SETUP=0x09,
    MMA8451Q_TRIG_CFG,
    MMA8451Q_SYSMOD,
    MMA8451Q_INT_SOURCE,
    MMA8451Q_WHO_AM_I,
    MMA8451Q_XYZ_DATA_CFG,
    MMA8451Q_HP_FILTER_CUTOFF,

    MMA8451Q_PL_STATUS,
    MMA8451Q_PL_CFG,
    MMA8451Q_PL_COUNT,
    MMA8451Q_PL_BF_ZCOMP,
    MMA8451Q_PL_THS_REG,

    MMA8451Q_FF_MT_CFG,
    MMA8451Q_FF_MT_SRC,
    MMA8451Q_FF_MT_THS,
    MMA8451Q_FF_MT_COUNT,

    MMA8451Q_TRANSIENT_CFG=0x1d,
    MMA8451Q_TRANSIENT_SRC,
    MMA8451Q_TRANSIENT_THS,
    MMA8451Q_TRANSIENT_COUNT,

    MMA8451Q_PULSE_CFG,
    MMA8451Q_PULSE_SRC,
    MMA8451Q_PULSE_THSX,
    MMA8451Q_PULSE_THSY,
    MMA8451Q_PULSE_THSZ,
    MMA8451Q_PULSE_TMLT,
    MMA8451Q_PULSE_LTCY,
    MMA8451Q_PULSE_WIND,

    MMA8451Q_ASLP_COUNT,
    MMA8451Q_CTRL_REG1,
    MMA8451Q_CTRL_REG2,
    MMA8451Q_CTRL_REG3,
    MMA8451Q_CTRL_REG4,
    MMA8451Q_CTRL_REG5,

    MMA8451Q_OFF_X,
    MMA8451Q_OFF_Y,
    MMA8451Q_OFF_Z,
};

enum
{
    MODE_2G=0,
    MODE_4G,
    MODE_8G,
};

/* mma8450 status */
struct mma_status
{
    unsigned char  mode;
    unsigned char ctl_reg2;
    unsigned char ctl_reg1;
};

static struct mma_status mma_status =
{
    .mode = 0,
    .ctl_reg2 = 0,
    .ctl_reg1 = 0
};

#define MODE_CHANGE_DELAY_MS         100
/* The I2C MMA8451Q test addresses */
#define I2C_MMA8451Q_BUS_ADDRESS     0x1C     /* I2C bus address of MMA8451Q */
#define I2C_BUS_BAUDRATE             100000
#define MMA8451Q_ID                  0x1A

/* Function prototypes */
int i2c_write_MMA8451Q(MQX_FILE_PTR, uint8_t, unsigned char *, _mqx_int);
int i2c_read_MMA8451Q(MQX_FILE_PTR, uint8_t, unsigned char *, _mqx_int);
void report_abs(MQX_FILE_PTR);
int mma8451q_read_data(MQX_FILE_PTR ,short *, short *, short *);
int mma8451q_init_client(MQX_FILE_PTR);
int mma8451q_change_mode(MQX_FILE_PTR, int);

#endif
/* EOF */

