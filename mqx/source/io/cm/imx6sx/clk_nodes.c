/*HEADER**********************************************************************
*
* Copyright 2013-2014 Freescale Semiconductor, Inc.
*
* This software is owned or controlled by Freescale Semiconductor.
* Use of this software is governed by the Freescale MQX RTOS License
* distributed with this Material.
* See the MQX_RTOS_LICENSE file distributed for more details.
*
* Brief License Summary:
* This software is provided in source form for you to use free of charge,
* but it is not open source software. You are allowed to use this software
* but you cannot redistribute it or derivative works of it in source form.
* The software may be used only in connection with a product containing
* a Freescale microprocessor, microcontroller, or digital signal processor.
* See license agreement file for full license terms including other
* restrictions.
*****************************************************************************
*
* Comments:
*
*   This file implements each clock nodes and their manipulation
*   entry functions
*
*
*END************************************************************************/

#include "string.h"
#include "mqx.h"
#include "bsp.h"
#include "clk_nodes_impl.h"
#include <stddef.h>

static CCM_MemMapPtr s_reg_ccm = CCM_BASE_PTR;

/* ----------------------------------------------------------------------------
   -- CCM register field width define
   ---------------------------------------------------------------------------- */

#define CCM_CCR_OSCNT_WIDTH                      8
#define CCM_CCR_COSC_EN_WIDTH                    1
#define CCM_CCR_WB_COUNT_WIDTH                   3
#define CCM_CCR_REG_BYPASS_COUNT_WIDTH           6
#define CCM_CCR_RBC_EN_WIDTH                     1
#define CCM_CCDR_MMDC_MASK_WIDTH                 1
#define CCM_CSR_REF_EN_B_WIDTH                   1
#define CCM_CSR_COSC_READY_WIDTH                 1
#define CCM_CCSR_PLL3_SW_CLK_SEL_WIDTH           1
#define CCM_CCSR_PLL2_SW_CLK_SEL_WIDTH           1
#define CCM_CCSR_PLL1_SW_CLK_SEL_WIDTH           1
#define CCM_CCSR_STEP_SEL_WIDTH                  1
#define CCM_CACRR_ARM_PODF_WIDTH                 3
#define CCM_CBCDR_PERIPH2_CLK2_PODF_WIDTH        3
#define CCM_CBCDR_FABRIC_MMDC_PODF_WIDTH         3
#define CCM_CBCDR_OCRAM_CLK_SEL_WIDTH            1
#define CCM_CBCDR_OCRAM_ALT_CLK_SEL_WIDTH        1
#define CCM_CBCDR_IPG_PODF_WIDTH                 2
#define CCM_CBCDR_AHB_PODF_WIDTH                 3
#define CCM_CBCDR_OCRAM_PODF_WIDTH               3
#define CCM_CBCDR_PERIPH_CLK_SEL_WIDTH           1
#define CCM_CBCDR_PERIPH2_CLK_SEL_WIDTH          1
#define CCM_CBCDR_PERIPH_CLK2_PODF_WIDTH         3
#define CCM_CBCMR_GPU_CORE_SEL_WIDTH             2
#define CCM_CBCMR_GPU_AXI_SEL_WIDTH              2
#define CCM_CBCMR_PCIE_AXI_CLK_SEL_WIDTH         1
#define CCM_CBCMR_PERIPH_CLK2_SEL_WIDTH          2
#define CCM_CBCMR_PRE_PERIPH_CLK_SEL_WIDTH       2
#define CCM_CBCMR_PERIPH2_CLK2_SEL_WIDTH         1
#define CCM_CBCMR_PRE_PERIPH2_CLK_SEL_WIDTH      2
#define CCM_CBCMR_LCDIF1_PODF_WIDTH              3
#define CCM_CBCMR_GPU_CORE_PODF_WIDTH            3
#define CCM_CBCMR_GPU_AXI_PODF_WIDTH             3
#define CCM_CSCMR1_PERCLK_PODF_WIDTH             6
#define CCM_CSCMR1_PERCLK_CLK_SEL_WIDTH          1
#define CCM_CSCMR1_QSPI1_CLK_SEL_WIDTH           3
#define CCM_CSCMR1_SSI1_CLK_SEL_WIDTH            2
#define CCM_CSCMR1_SSI2_CLK_SEL_WIDTH            2
#define CCM_CSCMR1_SSI3_CLK_SEL_WIDTH            2
#define CCM_CSCMR1_USDHC1_CLK_SEL_WIDTH          1
#define CCM_CSCMR1_USDHC2_CLK_SEL_WIDTH          1
#define CCM_CSCMR1_USDHC3_CLK_SEL_WIDTH          1
#define CCM_CSCMR1_USDHC4_CLK_SEL_WIDTH          1
#define CCM_CSCMR1_LCDIF2_PODF_WIDTH             3
#define CCM_CSCMR1_ACLK_ELM_SLOW_PODF_WIDTH      3
#define CCM_CSCMR1_QSPI1_PODF_WIDTH              3
#define CCM_CSCMR1_ACLK_ELM_SLOW_SEL_WIDTH       2
#define CCM_CSCMR2_CAN_PODF_WIDTH                6
#define CCM_CSCMR2_CAN_CLK_SEL_WIDTH             2
#define CCM_CSCMR2_LDB_DI0_IPU_DIV_WIDTH         1
#define CCM_CSCMR2_LDB_DI1_IPU_DIV_WIDTH         1
#define CCM_CSCMR2_ESAI_CLK_SEL_WIDTH            2
#define CCM_CSCMR2_VID_CLK_SEL_WIDTH             3
#define CCM_CSCMR2_VID_PRE_PODF_WIDTH            2
#define CCM_CSCMR2_VID_PODF_WIDTH                3
#define CCM_CSCDR1_UART_PODF_WIDTH               6
#define CCM_CSCDR1_UART_CLK_SEL_WIDTH            1
#define CCM_CSCDR1_USDHC1_PODF_WIDTH             3
#define CCM_CSCDR1_USDHC2_PODF_WIDTH             3
#define CCM_CSCDR1_USDHC3_PODF_WIDTH             3
#define CCM_CSCDR1_USDHC4_PODF_WIDTH             3
#define CCM_CS1CDR_SSI1_PODF_WIDTH               6
#define CCM_CS1CDR_SSI1_CLK_PRED_WIDTH           3
#define CCM_CS1CDR_ESAI_CLK_PRED_WIDTH           3
#define CCM_CS1CDR_SSI3_PODF_WIDTH               6
#define CCM_CS1CDR_SSI3_CLK_PRED_WIDTH           3
#define CCM_CS1CDR_ESAI_PODF_WIDTH               3
#define CCM_CS2CDR_SSI2_PODF_WIDTH               6
#define CCM_CS2CDR_SSI2_CLK_PRED_WIDTH           3
#define CCM_CS2CDR_LDB_DI0_CLK_SEL_WIDTH         3
#define CCM_CS2CDR_LDB_DI1_CLK_SEL_WIDTH         3
#define CCM_CS2CDR_QSPI2_CLK_SEL_WIDTH           3
#define CCM_CS2CDR_QSPI2_CLK_PRED_WIDTH          3
#define CCM_CS2CDR_QSPI2_PODF_WIDTH              6
#define CCM_CDCDR_AUDIO_CLK_SEL_WIDTH            2
#define CCM_CDCDR_AUDIO_PODF_WIDTH               3
#define CCM_CDCDR_AUDIO_CLK_PRED_WIDTH           3
#define CCM_CDCDR_SPDIF0_CLK_SEL_WIDTH           2
#define CCM_CDCDR_SPDIF0_PODF_WIDTH              3
#define CCM_CDCDR_SPDIF0_CLK_PRED_WIDTH          3
#define CCM_CHSCCDR_M4_CLK_SEL_WIDTH             3
#define CCM_CHSCCDR_M4_PODF_WIDTH                3
#define CCM_CHSCCDR_M4_PRE_CLK_SEL_WIDTH         3
#define CCM_CHSCCDR_ENET_CLK_SEL_WIDTH           3
#define CCM_CHSCCDR_ENET_PRE_CLK_SEL_WIDTH       3
#define CCM_CSCDR2_LCDIF2_CLK_SEL_WIDTH          3
#define CCM_CSCDR2_LCDIF2_CLK_PRED_WIDTH         3
#define CCM_CSCDR2_LCDIF2_PRE_CLK_SEL_WIDTH      3
#define CCM_CSCDR2_LCDIF1_CLK_SEL_WIDTH          3
#define CCM_CSCDR2_LCDIF1_CLK_PRED_WIDTH         3
#define CCM_CSCDR2_LCDIF1_PRE_CLK_SEL_WIDTH      3
#define CCM_CSCDR2_ECSPI_CLK_SEL_WIDTH           1
#define CCM_CSCDR2_ECSPI_PODF_WIDTH              6
#define CCM_CCOSR_CKO1_SEL_WIDTH                 4
#define CCM_CCOSR_CKO1_DIV_WIDTH                 3
#define CCM_CCOSR_CKO1_EN_WIDTH                  1
#define CCM_CCOSR_CKO2_SEL_WIDTH                 5
#define CCM_CCOSR_CKO2_DIV_WIDTH                 3
#define CCM_CCOSR_CKO2_EN_WIDTH                  1
#define CCM_CCGR0_CG0_WIDTH                      2
#define CCM_CCGR0_CG1_WIDTH                      2
#define CCM_CCGR0_CG2_WIDTH                      2
#define CCM_CCGR0_CG3_WIDTH                      2
#define CCM_CCGR0_CG4_WIDTH                      2
#define CCM_CCGR0_CG5_WIDTH                      2
#define CCM_CCGR0_CG6_WIDTH                      2
#define CCM_CCGR0_CG7_WIDTH                      2
#define CCM_CCGR0_CG8_WIDTH                      2
#define CCM_CCGR0_CG9_WIDTH                      2
#define CCM_CCGR0_CG10_WIDTH                     2
#define CCM_CCGR0_CG11_WIDTH                     2
#define CCM_CCGR0_CG12_WIDTH                     2
#define CCM_CCGR0_CG13_WIDTH                     2
#define CCM_CCGR0_CG14_WIDTH                     2
#define CCM_CCGR0_CG15_WIDTH                     2
#define CCM_CCGR1_CG0_WIDTH                      2
#define CCM_CCGR1_CG1_WIDTH                      2
#define CCM_CCGR1_CG2_WIDTH                      2
#define CCM_CCGR1_CG3_WIDTH                      2
#define CCM_CCGR1_CG4_WIDTH                      2
#define CCM_CCGR1_CG5_WIDTH                      2
#define CCM_CCGR1_CG6_WIDTH                      2
#define CCM_CCGR1_CG7_WIDTH                      2
#define CCM_CCGR1_CG8_WIDTH                      2
#define CCM_CCGR1_CG9_WIDTH                      2
#define CCM_CCGR1_CG10_WIDTH                     2
#define CCM_CCGR1_CG11_WIDTH                     2
#define CCM_CCGR1_CG12_WIDTH                     2
#define CCM_CCGR1_CG13_WIDTH                     2
#define CCM_CCGR1_CG14_WIDTH                     2
#define CCM_CCGR1_CG15_WIDTH                     2
#define CCM_CCGR2_CG0_WIDTH                      2
#define CCM_CCGR2_CG1_WIDTH                      2
#define CCM_CCGR2_CG2_WIDTH                      2
#define CCM_CCGR2_CG3_WIDTH                      2
#define CCM_CCGR2_CG4_WIDTH                      2
#define CCM_CCGR2_CG5_WIDTH                      2
#define CCM_CCGR2_CG6_WIDTH                      2
#define CCM_CCGR2_CG7_WIDTH                      2
#define CCM_CCGR2_CG8_WIDTH                      2
#define CCM_CCGR2_CG9_WIDTH                      2
#define CCM_CCGR2_CG10_WIDTH                     2
#define CCM_CCGR2_CG11_WIDTH                     2
#define CCM_CCGR2_CG12_WIDTH                     2
#define CCM_CCGR2_CG13_WIDTH                     2
#define CCM_CCGR2_CG14_WIDTH                     2
#define CCM_CCGR2_CG15_WIDTH                     2
#define CCM_CCGR3_CG0_WIDTH                      2
#define CCM_CCGR3_CG1_WIDTH                      2
#define CCM_CCGR3_CG2_WIDTH                      2
#define CCM_CCGR3_CG3_WIDTH                      2
#define CCM_CCGR3_CG4_WIDTH                      2
#define CCM_CCGR3_CG5_WIDTH                      2
#define CCM_CCGR3_CG6_WIDTH                      2
#define CCM_CCGR3_CG7_WIDTH                      2
#define CCM_CCGR3_CG8_WIDTH                      2
#define CCM_CCGR3_CG9_WIDTH                      2
#define CCM_CCGR3_CG10_WIDTH                     2
#define CCM_CCGR3_CG11_WIDTH                     2
#define CCM_CCGR3_CG12_WIDTH                     2
#define CCM_CCGR3_CG13_WIDTH                     2
#define CCM_CCGR3_CG14_WIDTH                     2
#define CCM_CCGR3_CG15_WIDTH                     2
#define CCM_CCGR4_CG0_WIDTH                      2
#define CCM_CCGR4_CG1_WIDTH                      2
#define CCM_CCGR4_CG2_WIDTH                      2
#define CCM_CCGR4_CG3_WIDTH                      2
#define CCM_CCGR4_CG4_WIDTH                      2
#define CCM_CCGR4_CG5_WIDTH                      2
#define CCM_CCGR4_CG6_WIDTH                      2
#define CCM_CCGR4_CG7_WIDTH                      2
#define CCM_CCGR4_CG8_WIDTH                      2
#define CCM_CCGR4_CG9_WIDTH                      2
#define CCM_CCGR4_CG10_WIDTH                     2
#define CCM_CCGR4_CG11_WIDTH                     2
#define CCM_CCGR4_CG12_WIDTH                     2
#define CCM_CCGR4_CG13_WIDTH                     2
#define CCM_CCGR4_CG14_WIDTH                     2
#define CCM_CCGR4_CG15_WIDTH                     2
#define CCM_CCGR5_CG0_WIDTH                      2
#define CCM_CCGR5_CG1_WIDTH                      2
#define CCM_CCGR5_CG2_WIDTH                      2
#define CCM_CCGR5_CG3_WIDTH                      2
#define CCM_CCGR5_CG4_WIDTH                      2
#define CCM_CCGR5_CG5_WIDTH                      2
#define CCM_CCGR5_CG6_WIDTH                      2
#define CCM_CCGR5_CG7_WIDTH                      2
#define CCM_CCGR5_CG8_WIDTH                      2
#define CCM_CCGR5_CG9_WIDTH                      2
#define CCM_CCGR5_CG10_WIDTH                     2
#define CCM_CCGR5_CG11_WIDTH                     2
#define CCM_CCGR5_CG12_WIDTH                     2
#define CCM_CCGR5_CG13_WIDTH                     2
#define CCM_CCGR5_CG14_WIDTH                     2
#define CCM_CCGR5_CG15_WIDTH                     2
#define CCM_CCGR6_CG0_WIDTH                      2
#define CCM_CCGR6_CG1_WIDTH                      2
#define CCM_CCGR6_CG2_WIDTH                      2
#define CCM_CCGR6_CG3_WIDTH                      2
#define CCM_CCGR6_CG4_WIDTH                      2
#define CCM_CCGR6_CG5_WIDTH                      2
#define CCM_CCGR6_CG6_WIDTH                      2
#define CCM_CCGR6_CG7_WIDTH                      2
#define CCM_CCGR6_CG8_WIDTH                      2
#define CCM_CCGR6_CG9_WIDTH                        2
#define CCM_CCGR6_CG10_WIDTH                       2
#define CCM_CCGR6_CG11_WIDTH                       2
#define CCM_CCGR6_CG12_WIDTH                       2
#define CCM_CCGR6_CG13_WIDTH                       2
#define CCM_CCGR6_CG14_WIDTH                       2
#define CCM_CCGR6_CG15_WIDTH                       2
#define CCM_CMEOR_MOD_EN_OV_GPT_WIDTH              1
#define CCM_CMEOR_MOD_EN_OV_EPIT_WIDTH             1
#define CCM_CMEOR_MOD_EN_USDHC_WIDTH               1
#define CCM_CMEOR_MOD_EN_GPU_WIDTH                 1
#define CCM_CMEOR_MOD_EN_OV_CAN2_CPI_WIDTH         1
#define CCM_CMEOR_MOD_EN_OV_CAN1_CPI_WIDTH         1
#define CCM_ANALOG_PLL_ARM_DIV_SEL_WIDTH           7
#define CCM_ANALOG_PLL_ARM_POWER_DOWN_WIDTH        1
#define CCM_ANALOG_PLL_ARM_ENABLE_WIDTH            1
#define CCM_ANALOG_PLL_ARM_BYPASSSRC_WIDTH         2
#define CCM_ANALOG_PLL_ARM_BYPASS_WIDTH            1
#define CCM_ANALOG_PLL_ARM_LVDS_SEL_WIDTH          1
#define CCM_ANALOG_PLL_ARM_LVDS_24M_SEL_WIDTH      1
#define CCM_ANALOG_PLL_ARM_PLL_SEL_WIDTH           1
#define CCM_ANALOG_PLL_ARM_LOCK_WIDTH              1
#define CCM_ANALOG_PLL_USBn_DIV_SEL_WIDTH          2
#define CCM_ANALOG_PLL_USBn_EN_USB_CLK_WIDTH       1
#define CCM_ANALOG_PLL_USBn_POWER_WIDTH            1
#define CCM_ANALOG_PLL_USBn_ENABLE_WIDTH           1
#define CCM_ANALOG_PLL_USBn_BYPASSSRC_WIDTH        2
#define CCM_ANALOG_PLL_USBn_BYPASS_WIDTH           1
#define CCM_ANALOG_PLL_USBn_LOCK_WIDTH             1
#define CCM_ANALOG_PLL_SYS_DIV_SEL_WIDTH           1
#define CCM_ANALOG_PLL_SYS_POWER_DOWN_WIDTH        1
#define CCM_ANALOG_PLL_SYS_ENABLE_WIDTH            1
#define CCM_ANALOG_PLL_SYS_BYPASSSRC_WIDTH         2
#define CCM_ANALOG_PLL_SYS_BYPASS_WIDTH            1
#define CCM_ANALOG_PLL_SYS_PFD_OFFSET_EN_WIDTH     1
#define CCM_ANALOG_PLL_SYS_LOCK_WIDTH              1
#define CCM_ANALOG_PLL_AUDIO_DIV_SEL_WIDTH         7
#define CCM_ANALOG_PLL_AUDIO_POWER_DOWN_WIDTH      1
#define CCM_ANALOG_PLL_AUDIO_ENABLE_WIDTH          1
#define CCM_ANALOG_PLL_AUDIO_BYPASSSRC_WIDTH       2
#define CCM_ANALOG_PLL_AUDIO_BYPASS_WIDTH          1
#define CCM_ANALOG_PLL_AUDIO_PFD_OFFSET_EN_WIDTH   1
#define CCM_ANALOG_PLL_AUDIO_POST_DIV_SEL_WIDTH    2
#define CCM_ANALOG_PLL_AUDIO_SSC_EN_WIDTH          1
#define CCM_ANALOG_PLL_AUDIO_LOCK_WIDTH            1
#define CCM_ANALOG_PLL_VIDEO_DIV_SEL_WIDTH         7
#define CCM_ANALOG_PLL_VIDEO_POWER_DOWN_WIDTH      1
#define CCM_ANALOG_PLL_VIDEO_ENABLE_WIDTH          1
#define CCM_ANALOG_PLL_VIDEO_BYPASSSRC_WIDTH       2
#define CCM_ANALOG_PLL_VIDEO_BYPASS_WIDTH          1
#define CCM_ANALOG_PLL_VIDEO_PFD_OFFSET_EN_WIDTH   1
#define CCM_ANALOG_PLL_VIDEO_POST_DIV_SEL_WIDTH    2
#define CCM_ANALOG_PLL_VIDEO_SSC_EN_WIDTH          1
#define CCM_ANALOG_PLL_VIDEO_LOCK_WIDTH            1
#define CCM_ANALOG_PFD_480_PFD0_FRAC_WIDTH         6
#define CCM_ANALOG_PFD_480_PFD0_STB_WIDTH          1
#define CCM_ANALOG_PFD_480_PFD0_CLKGATE_WIDTH      1
#define CCM_ANALOG_PFD_480_PFD1_FRAC_WIDTH         6
#define CCM_ANALOG_PFD_480_PFD1_STB_WIDTH          1
#define CCM_ANALOG_PFD_480_PFD1_CLKGATE_WIDTH      1
#define CCM_ANALOG_PFD_480_PFD2_FRAC_WIDTH         6
#define CCM_ANALOG_PFD_480_PFD2_STB_WIDTH          1
#define CCM_ANALOG_PFD_480_PFD2_CLKGATE_WIDTH      1
#define CCM_ANALOG_PFD_480_PFD3_FRAC_WIDTH         6
#define CCM_ANALOG_PFD_480_PFD3_STB_WIDTH          1
#define CCM_ANALOG_PFD_480_PFD3_CLKGATE_WIDTH      1
#define CCM_ANALOG_PFD_528_PFD0_FRAC_WIDTH         6
#define CCM_ANALOG_PFD_528_PFD0_STB_WIDTH          1
#define CCM_ANALOG_PFD_528_PFD0_CLKGATE_WIDTH      1
#define CCM_ANALOG_PFD_528_PFD1_FRAC_WIDTH         6
#define CCM_ANALOG_PFD_528_PFD1_STB_WIDTH          1
#define CCM_ANALOG_PFD_528_PFD1_CLKGATE_WIDTH      1
#define CCM_ANALOG_PFD_528_PFD2_FRAC_WIDTH         6
#define CCM_ANALOG_PFD_528_PFD2_STB_WIDTH          1
#define CCM_ANALOG_PFD_528_PFD2_CLKGATE_WIDTH      1
#define CCM_ANALOG_PFD_528_PFD3_FRAC_WIDTH         6
#define CCM_ANALOG_PFD_528_PFD3_STB_WIDTH          1
#define CCM_ANALOG_PFD_528_PFD3_CLKGATE_WIDTH      1
#define CCM_ANALOG_MISC0_REFTOP_PWD_WIDTH          1
#define CCM_ANALOG_MISC0_REFTOP_SELFBIASOFF_WIDTH  1
#define CCM_ANALOG_MISC0_REFTOP_VBGADJ_WIDTH       3
#define CCM_ANALOG_MISC0_REFTOP_VBGUP_WIDTH        1
#define CCM_ANALOG_MISC0_STOP_MODE_CONFIG_WIDTH    2
#define CCM_ANALOG_MISC0_RTC_RINGOSC_EN_WIDTH      1
#define CCM_ANALOG_MISC0_OSC_I_WIDTH               2
#define CCM_ANALOG_MISC0_OSC_XTALOK_WIDTH          1
#define CCM_ANALOG_MISC0_OSC_XTALOK_EN_WIDTH       1
#define CCM_ANALOG_MISC0_CLKGATE_CTRL_WIDTH        1
#define CCM_ANALOG_MISC0_CLKGATE_DELAY_WIDTH       3
#define CCM_ANALOG_MISC0_RTC_XTAL_SRC_WIDTH        1
#define CCM_ANALOG_MISC0_XTAL_24M_PWD_WIDTH        1
#define CCM_ANALOG_MISC0_VID_PLL_PREDIV_WIDTH      1
#define CCM_ANALOG_MISC1_LVDS1_CLK_SEL_WIDTH       5
#define CCM_ANALOG_MISC1_LVDS2_CLK_SEL_WIDTH       5
#define CCM_ANALOG_MISC1_LVDSCLK1_OBEN_WIDTH       1
#define CCM_ANALOG_MISC1_LVDSCLK2_OBEN_WIDTH       1
#define CCM_ANALOG_MISC1_LVDSCLK1_IBEN_WIDTH       1
#define CCM_ANALOG_MISC1_LVDSCLK2_IBEN_WIDTH       1
#define CCM_ANALOG_MISC1_PFD_480_AUTOGATE_EN_WIDTH 1
#define CCM_ANALOG_MISC1_PFD_528_AUTOGATE_EN_WIDTH 1
#define CCM_ANALOG_MISC1_IRQ_TEMPPANIC_WIDTH       1
#define CCM_ANALOG_MISC1_IRQ_TEMPLOW_WIDTH         1
#define CCM_ANALOG_MISC1_IRQ_TEMPHIGH_WIDTH        1
#define CCM_ANALOG_MISC1_IRQ_ANA_BO_WIDTH          1
#define CCM_ANALOG_MISC1_IRQ_DIG_BO_WIDTH          1
#define CCM_ANALOG_MISC2_REG0_BO_OFFSET_WIDTH      3
#define CCM_ANALOG_MISC2_REG0_BO_STATUS_WIDTH      1
#define CCM_ANALOG_MISC2_REG0_ENABLE_BO_WIDTH      1
#define CCM_ANALOG_MISC2_PLL3_DISABLE_WIDTH        1
#define CCM_ANALOG_MISC2_REG1_BO_OFFSET_WIDTH      3
#define CCM_ANALOG_MISC2_REG1_BO_STATUS_WIDTH      1
#define CCM_ANALOG_MISC2_REG1_ENABLE_BO_WIDTH      1
#define CCM_ANALOG_MISC2_AUDIO_DIV_LSB_WIDTH       1
#define CCM_ANALOG_MISC2_REG2_BO_OFFSET_WIDTH      3
#define CCM_ANALOG_MISC2_REG2_BO_STATUS_WIDTH      1
#define CCM_ANALOG_MISC2_REG2_ENABLE_BO_WIDTH      1
#define CCM_ANALOG_MISC2_REG2_OK_WIDTH             1
#define CCM_ANALOG_MISC2_AUDIO_DIV_MSB_WIDTH       1
#define CCM_ANALOG_MISC2_REG0_STEP_TIME_WIDTH      2
#define CCM_ANALOG_MISC2_REG1_STEP_TIME_WIDTH      2
#define CCM_ANALOG_MISC2_REG2_STEP_TIME_WIDTH      2
#define CCM_ANALOG_MISC2_VIDEO_DIV_WIDTH           2

#define INS_BITFIELD(reg, shift, width, val) (*(reg) = (*(reg) & ~(((1 << (width)) - 1) << (shift))) | ((val) << (shift)))
#define EXTRACT_BITFIELD(reg, shift, width) ((*(reg) & (((1 << (width)) - 1) << (shift))) >> (shift))

#define TIMEOUT 0x10000

#define WAIT(exp, timeout) do { \
    uint8_t val; \
    do { \
        val = exp; \
    } while (!val); \
} while (0)

#define CHECK_PARENT_IDENTICAL  2
#define CHECK_PARENT_INVALID    1
#define CHECK_PARENT_PASS       0

#define CG_OFF_ALL              0U
#define CG_ON_RUN               1U
#define CG_ON_ALL               2U
#define CG_ON_ALL_EXCEPT_STOP   3U

#define FREQ_OSC                24000000
#define FREQ_CKIL               32768

/*
 * Clock nodes implementation
 * TODO : The pseudo code don't include the register manipulation in read
 *        world, these should be added in the function series
 *      - open_clk_XXX
 *          physically open the clock node
 *      - close_clk_XXX
 *          physically close the clock node
 *      - check_parent_XXX
 *          check if the parent valid, if it is valid, check
 *          the old parent and new parent's gating status
 *      - get_parent_XXX
 *          check current physical parent
 *      - set_parent_XXX
 *          physical set altenative clock path
 */

/*
 * Begin of clock nodes implementation
 */
static CLK_NODE_T clk_osc;
static CLK_NODE_T clk_pll2;
static CLK_NODE_T clk_pll2_pfd0;
static CLK_NODE_T clk_pll2_pfd2;
static CLK_NODE_T clk_pll2_pfd0_div;
static CLK_NODE_T clk_pll3;
static CLK_NODE_T clk_pll3_pfd1;
static CLK_NODE_T clk_pll3_pfd2;
static CLK_NODE_T clk_pll3_pfd3;
static CLK_NODE_T clk_pll3_80;
static CLK_NODE_T clk_pll3_60;
static CLK_NODE_T clk_pll4;
static CLK_NODE_T clk_pll5;
static CLK_NODE_T clk_ahb;
static CLK_NODE_T clk_ipg;
static CLK_NODE_T clk_perclk;

/* CLK NODE CKIL */
static FIX_FREQ_DATA ckil_freq = {
    .freq_val = FREQ_CKIL,
};

static CLK_NODE_T clk_ckil = {
    .name = CLK_CKIL,
    .gate_type = GATE_NONE,
    .freq_type = FREQ_FIX,
    .parent_type = PARENT_NONE,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "CKIL",
    .freq_data = (void*)&ckil_freq,
};

/* CLK NODE OSC */
static OSC_GATE_DATA osc_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CSR),
    .enable_shift = CCM_CCR_COSC_EN_SHIFT,
    .enable_width = CCM_CCR_COSC_EN_WIDTH,
    .ready_shift = CCM_CSR_cosc_ready_SHIFT,
    .ready_width = CCM_CSR_COSC_READY_WIDTH,
};

static FIX_FREQ_DATA osc_freq = {
    .freq_val = FREQ_OSC,
};

static CLK_NODE_T clk_osc = {
    .name = CLK_OSC,
    .gate_type = GATE_OSC,
    .freq_type = FREQ_FIX,
    .parent_type = PARENT_NONE,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "OSC",
    .gate_data = (void*)&osc_gate,
    .freq_data = (void*)&osc_freq,
};

/* CLK NODE PLL2 */
static FIX_PARENT_DATA pll_parent = {
    .parent = CLK_OSC,
};

static PLL_GATE_DATA pll2_gate = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PLL_SYS) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .enable_shift = CCM_ANALOG_PLL_SYS_ENABLE_SHIFT,
    .enable_width = CCM_ANALOG_PLL_SYS_ENABLE_WIDTH,
    .pwr_shift = CCM_ANALOG_PLL_SYS_POWERDOWN_SHIFT,
    .pwr_width = CCM_ANALOG_PLL_SYS_POWER_DOWN_WIDTH,
    .lock_shift = CCM_ANALOG_PLL_SYS_LOCK_SHIFT,
    .lock_width = CCM_ANALOG_PLL_SYS_LOCK_WIDTH,
    .bypass_shift = CCM_ANALOG_PLL_SYS_BYPASS_SHIFT,
    .bypass_width = CCM_ANALOG_PLL_SYS_BYPASS_WIDTH,
};

static PLL_A_FREQ_DATA pll2_freq = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PLL_SYS) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .shift = CCM_ANALOG_PLL_SYS_DIV_SELECT_MASK,
    .width = CCM_ANALOG_PLL_SYS_DIV_SEL_WIDTH,
};

static CLK_NODE_T clk_pll2 = {
    .name = CLK_PLL2,
    .gate_type = GATE_PLL,
    .freq_type = FREQ_PLL_A,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PLL2",
    .parent_data = (void*)&pll_parent,
    .gate_data = (void*)&pll2_gate,
    .freq_data = (void*)&pll2_freq,
};

/* CLK NODE PLL2_PFD0 */
static FIX_PARENT_DATA pll2_childs_parent = {
    .parent = CLK_PLL2,
};

static PFD_GATE_DATA pll2_pfd0_gate = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PFD_528) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .gate_shift = CCM_ANALOG_PFD_528_PFD0_CLKGATE_SHIFT,
    .gate_width = CCM_ANALOG_PFD_528_PFD0_CLKGATE_WIDTH,
    .stable_shift = CCM_ANALOG_PFD_528_PFD0_STABLE_SHIFT,
    .stable_width = CCM_ANALOG_PFD_528_PFD0_STB_WIDTH,
};

static PFD_FREQ_DATA pll2_pfd0_freq = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PFD_528) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .shift = CCM_ANALOG_PFD_528_PFD0_FRAC_SHIFT,
    .width = CCM_ANALOG_PFD_528_PFD0_FRAC_WIDTH,
};

static CLK_NODE_T clk_pll2_pfd0 = {
    .name = CLK_PLL2_PFD0,
    .gate_type = GATE_PLL_PFD,
    .freq_type = FREQ_PLL_PFD,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PLL2_PFD0",
    .parent_data = (void*)&pll2_childs_parent,
    .gate_data = (void*)&pll2_pfd0_gate,
    .freq_data = (void*)&pll2_pfd0_freq,
};
/* CLK NODE PLL2_PFD2 */
static PFD_GATE_DATA pll2_pfd2_gate = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PFD_528) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .gate_shift = CCM_ANALOG_PFD_528_PFD2_CLKGATE_SHIFT,
    .gate_width = CCM_ANALOG_PFD_528_PFD2_CLKGATE_WIDTH,
    .stable_shift = CCM_ANALOG_PFD_528_PFD2_STABLE_SHIFT,
    .stable_width = CCM_ANALOG_PFD_528_PFD2_STB_WIDTH,
};

static PFD_FREQ_DATA pll2_pfd2_freq = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PFD_528) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .shift = CCM_ANALOG_PFD_528_PFD2_FRAC_SHIFT,
    .width = CCM_ANALOG_PFD_528_PFD2_FRAC_WIDTH,
};

static CLK_NODE_T clk_pll2_pfd2 = {
    .name = CLK_PLL2_PFD2,
    .gate_type = GATE_PLL_PFD,
    .freq_type = FREQ_PLL_PFD,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PLL2_PFD2",
    .parent_data = (void*)&pll2_childs_parent,
    .gate_data = (void*)&pll2_pfd2_gate,
    .freq_data = (void*)&pll2_pfd2_freq,
};

/* CLK NODE PLL2_PFD0_DIV */
static FIX_PARENT_DATA pll2_pfd0_div_parent = {
    .parent = CLK_PLL2_PFD0,
};

static FIX_DIV_FREQ_DATA pll2_pfd0_div_freq = {
    .div_val = 2,
};

static CLK_NODE_T clk_pll2_pfd0_div = {
    .name = CLK_PLL2_PFD0_DIV,
    .gate_type = GATE_NONE,
    .freq_type = FREQ_FIX_DIV,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PLL2_PFD0_DIV",
    .parent_data = (void*)&pll2_pfd0_div_parent,
    .freq_data = (void*)&pll2_pfd0_div_freq,
};

/* CLK NODE PLL3 */
static PLL_GATE_DATA pll3_gate = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PLL_USB1) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .enable_shift = CCM_ANALOG_PLL_USB1_ENABLE_SHIFT,
    .enable_width = CCM_ANALOG_PLL_USBn_ENABLE_WIDTH,
    .pwr_shift = CCM_ANALOG_PLL_USB1_POWER_SHIFT, /*Note it is USB PLL*/
    .pwr_width = CCM_ANALOG_PLL_USBn_POWER_WIDTH,
    .lock_shift = CCM_ANALOG_PLL_USB1_LOCK_SHIFT,
    .lock_width = CCM_ANALOG_PLL_USBn_LOCK_WIDTH,
    .bypass_shift = CCM_ANALOG_PLL_USB1_BYPASS_SHIFT,
    .bypass_width = CCM_ANALOG_PLL_USBn_BYPASS_WIDTH,
};

static PLL_A_FREQ_DATA pll3_freq = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PLL_USB1) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .shift = CCM_ANALOG_PLL_USB1_DIV_SELECT_SHIFT,
    .width = CCM_ANALOG_PLL_USBn_DIV_SEL_WIDTH,
};

static CLK_NODE_T clk_pll3 = {
    .name = CLK_PLL3,
    .gate_type = GATE_PLL_USB,
    .freq_type = FREQ_PLL_A,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PLL3",
    .parent_data = (void*)&pll_parent,
    .gate_data = (void*)&pll3_gate,
    .freq_data = (void*)&pll3_freq,
};

/* CLK NODE PLL3_PFD1 */
static FIX_PARENT_DATA pll3_childs_parent = {
    .parent = CLK_PLL3,
};

static PFD_GATE_DATA pll3_pfd1_gate = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PFD_480) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .gate_shift = CCM_ANALOG_PFD_480_PFD1_CLKGATE_SHIFT,
    .gate_width = CCM_ANALOG_PFD_480_PFD1_CLKGATE_WIDTH,
    .stable_shift = CCM_ANALOG_PFD_480_PFD1_STABLE_SHIFT,
    .stable_width = CCM_ANALOG_PFD_480_PFD1_STB_WIDTH,
};

static PFD_FREQ_DATA pll3_pfd1_freq = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PFD_480) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .shift = CCM_ANALOG_PFD_480_PFD1_FRAC_SHIFT,
    .width = CCM_ANALOG_PFD_480_PFD1_FRAC_WIDTH,
};

static CLK_NODE_T clk_pll3_pfd1 = {
    .name = CLK_PLL3_PFD1,
    .gate_type = GATE_PLL_PFD,
    .freq_type = FREQ_PLL_PFD,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PLL3_PFD1",
    .parent_data = (void*)&pll3_childs_parent,
    .gate_data = (void*)&pll3_pfd1_gate,
    .freq_data = (void*)&pll3_pfd1_freq,
};

/* CLK NODE PLL3_PFD2 */
static PFD_GATE_DATA pll3_pfd2_gate = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PFD_480) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .gate_shift = CCM_ANALOG_PFD_480_PFD2_CLKGATE_SHIFT,
    .gate_width = CCM_ANALOG_PFD_480_PFD2_CLKGATE_WIDTH,
    .stable_shift = CCM_ANALOG_PFD_480_PFD2_STABLE_SHIFT,
    .stable_width = CCM_ANALOG_PFD_480_PFD2_STB_WIDTH,
};

static PFD_FREQ_DATA pll3_pfd2_freq = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PFD_480) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .shift = CCM_ANALOG_PFD_480_PFD2_FRAC_SHIFT,
    .width = CCM_ANALOG_PFD_480_PFD2_FRAC_WIDTH,
};

static CLK_NODE_T clk_pll3_pfd2 = {
    .name = CLK_PLL3_PFD2,
    .gate_type = GATE_PLL_PFD,
    .freq_type = FREQ_PLL_PFD,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PLL3_PFD2",
    .parent_data = (void*)&pll3_childs_parent,
    .gate_data = (void*)&pll3_pfd2_gate,
    .freq_data = (void*)&pll3_pfd2_freq,
};

/* CLK NODE PLL3_PFD3 */
static PFD_GATE_DATA pll3_pfd3_gate = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PFD_480) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .gate_shift = CCM_ANALOG_PFD_480_PFD3_CLKGATE_SHIFT,
    .gate_width = CCM_ANALOG_PFD_480_PFD3_CLKGATE_WIDTH,
    .stable_shift = CCM_ANALOG_PFD_480_PFD3_STABLE_SHIFT,
    .stable_width = CCM_ANALOG_PFD_480_PFD3_STB_WIDTH,
};

static PFD_FREQ_DATA pll3_pfd3_freq = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PFD_480) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .shift = CCM_ANALOG_PFD_480_PFD3_FRAC_SHIFT,
    .width = CCM_ANALOG_PFD_480_PFD3_FRAC_WIDTH,
};
static CLK_NODE_T clk_pll3_pfd3 = {
    .name = CLK_PLL3_PFD3,
    .gate_type = GATE_PLL_PFD,
    .freq_type = FREQ_PLL_PFD,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PLL3_PFD3",
    .parent_data = (void*)&pll3_childs_parent,
    .gate_data = (void*)&pll3_pfd3_gate,
    .freq_data = (void*)&pll3_pfd3_freq,
};

/* CLK NODE PLL3_80 */
static FIX_DIV_FREQ_DATA pll3_80_freq = {
    .div_val = 6,
};

static CLK_NODE_T clk_pll3_80 = {
    .name = CLK_PLL3_80,
    .gate_type = GATE_NONE,
    .freq_type = FREQ_FIX_DIV,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PLL3_80",
    .parent_data = (void*)&pll3_childs_parent,
    .freq_data = (void*)&pll3_80_freq,
};

/* CLK NODE PLL3_60 */
static FIX_DIV_FREQ_DATA pll3_60_freq = {
    .div_val = 8,
};

static CLK_NODE_T clk_pll3_60 = {
    .name = CLK_PLL3_60,
    .gate_type = GATE_NONE,
    .freq_type = FREQ_FIX_DIV,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PLL3_60",
    .parent_data = (void*)&pll3_childs_parent,
    .freq_data = (void*)&pll3_60_freq,
};

/* CLK NODE PLL4 */
static PLL_GATE_DATA pll4_gate = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PLL_AUDIO) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .enable_shift = CCM_ANALOG_PLL_AUDIO_ENABLE_SHIFT,
    .enable_width = CCM_ANALOG_PLL_AUDIO_ENABLE_WIDTH,
    .pwr_shift = CCM_ANALOG_PLL_AUDIO_POWERDOWN_SHIFT,
    .pwr_width = CCM_ANALOG_PLL_AUDIO_POWER_DOWN_WIDTH,
    .lock_shift = CCM_ANALOG_PLL_AUDIO_LOCK_SHIFT,
    .lock_width = CCM_ANALOG_PLL_AUDIO_LOCK_WIDTH,
    .bypass_shift = CCM_ANALOG_PLL_AUDIO_BYPASS_SHIFT,
    .bypass_width = CCM_ANALOG_PLL_AUDIO_BYPASS_WIDTH,
};

static PLL_B_FREQ_DATA pll4_freq = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PLL_AUDIO) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .shift = CCM_ANALOG_PLL_AUDIO_DIV_SELECT_SHIFT,
    .width = CCM_ANALOG_PLL_AUDIO_DIV_SEL_WIDTH,
    .num_reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PLL_AUDIO_NUM) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .denom_reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PLL_AUDIO_DENOM) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
};

static CLK_NODE_T clk_pll4 = {
    .name = CLK_PLL4,
    .gate_type = GATE_PLL,
    .freq_type = FREQ_PLL_B,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PLL4",
    .parent_data = (void*)&pll_parent,
    .gate_data = (void*)&pll4_gate,
    .freq_data = (void*)&pll4_freq,
};

/* CLK NODE PLL5 */
static PLL_GATE_DATA pll5_gate = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PLL_AUDIO) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .enable_shift = CCM_ANALOG_PLL_VIDEO_ENABLE_SHIFT,
    .enable_width = CCM_ANALOG_PLL_VIDEO_ENABLE_WIDTH,
    .pwr_shift = CCM_ANALOG_PLL_VIDEO_POWERDOWN_SHIFT,
    .pwr_width = CCM_ANALOG_PLL_VIDEO_POWER_DOWN_WIDTH,
    .lock_shift = CCM_ANALOG_PLL_VIDEO_LOCK_SHIFT,
    .lock_width = CCM_ANALOG_PLL_VIDEO_LOCK_WIDTH,
    .bypass_shift = CCM_ANALOG_PLL_VIDEO_BYPASS_SHIFT,
    .bypass_width = CCM_ANALOG_PLL_VIDEO_BYPASS_WIDTH,
};

static PLL_B_FREQ_DATA pll5_freq = {
    .reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PLL_VIDEO) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .shift = CCM_ANALOG_PLL_VIDEO_DIV_SELECT_SHIFT,
    .width = CCM_ANALOG_PLL_VIDEO_DIV_SEL_WIDTH,
    .num_reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PLL_VIDEO_NUM) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
    .denom_reg_offset = (offsetof(struct CCM_ANALOG_MemMap, PLL_VIDEO_DENOM) + ((uint32_t)CCM_ANALOG_BASE_PTR - (uint32_t)CCM_BASE_PTR)),
};

static CLK_NODE_T clk_pll5 = {
    .name = CLK_PLL5,
    .gate_type = GATE_PLL,
    .freq_type = FREQ_PLL_B,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PLL5",
    .parent_data = (void*)&pll_parent,
    .gate_data = (void*)&pll5_gate,
    .freq_data = (void*)&pll5_freq,
};

/* CLK NODE PERIPH */
static CLOCK_NAME alt_parent_pre_periph[] = {
    CLK_PLL2,
    CLK_PLL2_PFD2,
    CLK_PLL2_PFD0,
    CLK_PLL2_PFD0_DIV
};

static SINGLE_SEL_PARENT_DATA pre_periph_parent = {
    .reg_offset = offsetof(struct CCM_MemMap, CBCMR),
    .shift = CCM_CBCMR_pre_periph_clk_sel_SHIFT,
    .width = CCM_CBCMR_PRE_PERIPH_CLK_SEL_WIDTH,
    .alt_parent = alt_parent_pre_periph,
    .alt_parent_nr = sizeof(alt_parent_pre_periph) / sizeof(CLOCK_NAME),
};

static CLK_NODE_T clk_pre_periph = {
    .name = CLK_PRE_PERIPH,
    .gate_type = GATE_NONE,
    .freq_type = FREQ_INHERIT,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PRE_PERIPH",
    .parent_data = (void*)&pre_periph_parent,
};

/* CLK NODE PERIPH2 */
static CLOCK_NAME alt_parent_periph2[] = {
    CLK_PLL3,
    CLK_OSC,
    CLK_PLL2,
    CLK_MAX
};

static SINGLE_SEL_PARENT_DATA periph2_parent = {
    .reg_offset = offsetof(struct CCM_MemMap, CBCMR),
    .shift = CCM_CBCMR_periph_clk2_sel_SHIFT,
    .width = CCM_CBCMR_PERIPH_CLK2_SEL_WIDTH,
    .alt_parent = alt_parent_periph2,
    .alt_parent_nr = sizeof(alt_parent_periph2) / sizeof(CLOCK_NAME),
};

static SINGLE_DIV_FREQ_DATA periph2_freq = {
    .reg_offset = offsetof(struct CCM_MemMap, CBCDR),
    .shift = CCM_CBCDR_periph_clk2_podf_SHIFT,
    .width = CCM_CBCDR_PERIPH_CLK2_PODF_WIDTH,
};

static CLK_NODE_T clk_periph2 = {
    .name = CLK_PERIPH2,
    .gate_type = GATE_NONE,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PERIPH2",
    .freq_data = (void*)&periph2_freq,
    .parent_data = (void*)&periph2_parent,
};

/* CLK NODE PERIPH */
static CLOCK_NAME alt_parent_periph[] = {
    CLK_PRE_PERIPH,
    CLK_PERIPH2,
};

static SINGLE_SEL_PARENT_DATA periph_parent = {
    .reg_offset = offsetof(struct CCM_MemMap, CBCDR),
    .shift = CCM_CBCDR_periph_clk_sel_SHIFT,
    .width = CCM_CBCDR_PERIPH_CLK_SEL_WIDTH,
    .alt_parent = alt_parent_periph,
    .alt_parent_nr = sizeof(alt_parent_periph) / sizeof(CLOCK_NAME),
};

static CLK_NODE_T clk_periph = {
    .name = CLK_PERIPH,
    .gate_type = GATE_NONE,
    .freq_type = FREQ_INHERIT,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PERIPH",
    .parent_data = (void*)&periph_parent,
};

/* CLK NODE AHB */
static FIX_PARENT_DATA ahb_parent = {
    .parent = CLK_PERIPH,
};

static SINGLE_DIV_FREQ_DATA ahb_freq = {
    .reg_offset = offsetof(struct CCM_MemMap, CBCDR),
    .shift = CCM_CBCDR_ahb_podf_SHIFT,
    .width = CCM_CBCDR_AHB_PODF_WIDTH,
};

static CLK_NODE_T clk_ahb = {
    .name = CLK_AHB,
    .gate_type = GATE_NONE,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "AHB",
    .freq_data = (void*)&ahb_freq,
    .parent_data = (void*)&ahb_parent,
};

/* CLK NODE IPG */
static FIX_PARENT_DATA ipg_parent = {
    .parent = CLK_AHB,
};

static SINGLE_DIV_FREQ_DATA ipg_freq = {
    .reg_offset = offsetof(struct CCM_MemMap, CBCDR),
    .shift = CCM_CBCDR_ipg_podf_SHIFT,
    .width = CCM_CBCDR_IPG_PODF_WIDTH,
};

static CLK_NODE_T clk_ipg = {
    .name = CLK_IPG,
    .gate_type = GATE_NONE,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "IPG",
    .parent_data = (void*)&ipg_parent,
    .freq_data = (void*)&ipg_freq,
};

/* CLK NODE PERCLK */
static CLOCK_NAME alt_parent_perclk[] = {
    CLK_IPG,
    CLK_OSC,
};

static SINGLE_SEL_PARENT_DATA perclk_parent = {
    .reg_offset = offsetof(struct CCM_MemMap, CSCMR1),
    .shift = CCM_CSCMR1_perclk_clk_sel_SHIFT,
    .width = CCM_CSCMR1_PERCLK_CLK_SEL_WIDTH,
    .alt_parent = alt_parent_perclk,
    .alt_parent_nr = sizeof(alt_parent_perclk) / sizeof(CLOCK_NAME),
};

static SINGLE_DIV_FREQ_DATA perclk_freq = {
    .reg_offset = offsetof(struct CCM_MemMap, CSCMR1),
    .shift = CCM_CSCMR1_perclk_podf_SHIFT,
    .width = CCM_CSCMR1_PERCLK_PODF_WIDTH,
};

static CLK_NODE_T clk_perclk = {
    .name = CLK_PERCLK,
    .gate_type = GATE_NONE,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "PERCLK",
    .parent_data = (void*)&perclk_parent,
    .freq_data = (void*)&perclk_freq,
};

/* CLK NODE M4 */
static CLOCK_NAME alt_parent_m4[] = {
    CLK_PLL2,
    CLK_PLL3,
    CLK_OSC,
    CLK_PLL2_PFD0,
    CLK_PLL2_PFD2,
    CLK_PLL3_PFD3,
    CLK_MAX,
    CLK_MAX,
};

static SINGLE_SEL_PARENT_DATA m4_parent = {
    .reg_offset = offsetof(struct CCM_MemMap, CHSCCDR),
    .shift = CCM_CHSCCDR_m4_pre_clk_sel_SHIFT,
    .width = CCM_CHSCCDR_M4_PRE_CLK_SEL_WIDTH,
    .alt_parent = alt_parent_m4,
    .alt_parent_nr = sizeof(alt_parent_m4) / sizeof(CLOCK_NAME),
};

static SINGLE_GATE_DATA m4_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR3),
    .reg_shift = CCM_CCGR3_CG1_SHIFT,
    .reg_width = CCM_CCGR3_CG1_WIDTH,
};

static SINGLE_DIV_FREQ_DATA m4_freq = {
    .reg_offset = offsetof(struct CCM_MemMap, CHSCCDR),
    .shift = CCM_CHSCCDR_m4_podf_SHIFT,
    .width = CCM_CHSCCDR_M4_PODF_WIDTH,
};

static CLK_NODE_T clk_m4 = {
    .name = CLK_M4,
    /*
     * M4 has 2 levels of clock parent selection, for simplicity, 
     * we assume the 2nd level CHSCCDR[m4_clk_sel] is 0, which select
     * pre-muxed clock
     */
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "M4",
    .parent_data = (void*)&m4_parent,
    .gate_data = (void*)&m4_gate,
    .freq_data = (void*)&m4_freq,
};

/* CLK NODE ALT_OCRAM */
static CLOCK_NAME alt_parent_alt_ocram[] = {
    CLK_PLL2_PFD2,
    CLK_PLL3_PFD1,
};

static SINGLE_SEL_PARENT_DATA alt_ocram_parent = {
    .reg_offset = offsetof(struct CCM_MemMap, CBCDR),
    .shift = CCM_CBCDR_ocram_alt_clk_sel_SHIFT,
    .width = CCM_CBCDR_OCRAM_ALT_CLK_SEL_WIDTH,
    .alt_parent = alt_parent_alt_ocram,
    .alt_parent_nr = sizeof(alt_parent_alt_ocram) / sizeof(CLOCK_NAME),
};

static CLK_NODE_T clk_alt_ocram = {
    .name = CLK_ALT_OCRAM,
    .gate_type = GATE_NONE,
    .freq_type = FREQ_INHERIT,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "ALT_OCRAM",
    .parent_data = (void*)&alt_ocram_parent,
};

/* CLK NODE OCRAM */
static CLOCK_NAME alt_parent_ocram[] = {
    CLK_PERIPH,
    CLK_ALT_OCRAM,
};

static SINGLE_SEL_PARENT_DATA ocram_parent = {
    .reg_offset = offsetof(struct CCM_MemMap, CBCDR),
    .shift = CCM_CBCDR_ocram_clk_sel_SHIFT,
    .width = CCM_CBCDR_OCRAM_CLK_SEL_WIDTH,
    .alt_parent = alt_parent_ocram,
    .alt_parent_nr = sizeof(alt_parent_ocram) / sizeof(CLOCK_NAME),
};

static SINGLE_GATE_DATA ocram_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR3),
    .reg_shift = CCM_CCGR3_CG14_SHIFT,
    .reg_width = CCM_CCGR3_CG14_WIDTH,
};

static SINGLE_DIV_FREQ_DATA ocram_freq = {
    .reg_offset = offsetof(struct CCM_MemMap, CBCDR),
    .shift = CCM_CBCDR_ocram_podf_SHIFT,
    .width = CCM_CBCDR_OCRAM_PODF_WIDTH,
};

static CLK_NODE_T clk_ocram = {
    .name = CLK_OCRAM,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "OCRAM",
    .gate_data = (void*)&ocram_gate,
    .freq_data = (void*)&ocram_freq,
    .parent_data = (void*)&ocram_parent,
};

/* CLK NODE FLEXCAN1 */
static CLOCK_NAME alt_parent_can[] = {
    CLK_PLL3_60,
    CLK_OSC,
    CLK_PLL3_80,
    CLK_MAX,
};

static SINGLE_SEL_PARENT_DATA can_parent = {
    .reg_offset = offsetof(struct CCM_MemMap, CSCMR2),
    .shift = CCM_CSCMR2_can_clk_sel_SHIFT,
    .width = CCM_CSCMR2_CAN_CLK_SEL_WIDTH,
    .alt_parent = alt_parent_can,
    .alt_parent_nr = sizeof(alt_parent_can) / sizeof(CLOCK_NAME),
};

static SINGLE_GATE_DATA flexcan1_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR0),
    .reg_shift = CCM_CCGR0_CG8_SHIFT,
    .reg_width = CCM_CCGR0_CG8_WIDTH,
};

static SINGLE_DIV_FREQ_DATA can_freq = {
    .reg_offset = offsetof(struct CCM_MemMap, CSCMR2),
    .shift = CCM_CSCMR2_can_clk_podf_SHIFT,
    .width = CCM_CSCMR2_CAN_PODF_WIDTH,
};

static CLK_NODE_T clk_flexcan1 = {
    .name = CLK_FLEXCAN1,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "FLEXCAN1",
    .parent_data = (void*)&can_parent,
    .gate_data = (void*)&flexcan1_gate,
    .freq_data = (void*)&can_freq,
};

/* CLK NODE FLEXCAN1_IPG */
static FIX_PARENT_DATA ipg_childs_parent = {
    .parent = CLK_IPG,
};

static SINGLE_GATE_DATA flexcan1_ipg_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR0),
    .reg_shift = CCM_CCGR0_CG7_SHIFT,
    .reg_width = CCM_CCGR0_CG7_WIDTH,
};

static CLK_NODE_T clk_flexcan1_ipg = {
    .name = CLK_FLEXCAN1_IPG,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_INHERIT,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "FLEXCAN1_IPG",
    .parent_data = (void*)&ipg_childs_parent,
    .gate_data = (void*)&flexcan1_ipg_gate,
};

/* CLK NODE FLEXCAN2 */
static SINGLE_GATE_DATA flexcan2_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR0),
    .reg_shift = CCM_CCGR0_CG10_SHIFT,
    .reg_width = CCM_CCGR0_CG10_WIDTH,
};

static CLK_NODE_T clk_flexcan2 = {
    .name = CLK_FLEXCAN2,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "FLEXCAN2",
    .parent_data = (void*)&can_parent,
    .gate_data = (void*)&flexcan2_gate,
    .freq_data = (void*)&can_freq,
};

/* CLK NODE FLEXCAN2_IPG */
static SINGLE_GATE_DATA flexcan2_ipg_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR0),
    .reg_shift = CCM_CCGR0_CG9_SHIFT,
    .reg_width = CCM_CCGR0_CG9_WIDTH,
};

static CLK_NODE_T clk_flexcan2_ipg = {
    .name = CLK_FLEXCAN2_IPG,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_INHERIT,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "FLEXCAN2_IPG",
    .parent_data = (void*)&ipg_childs_parent,
    .gate_data = (void*)&flexcan2_ipg_gate,
};

/* CLK NODE CANFD */
static SINGLE_GATE_DATA canfd_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR1),
    .reg_shift = CCM_CCGR1_CG15_SHIFT,
    .reg_width = CCM_CCGR1_CG15_WIDTH,
};

static CLK_NODE_T clk_canfd = {
    .name = CLK_CANFD,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "CANFD",
    .parent_data = (void*)&can_parent,
    .gate_data = (void*)&canfd_gate,
    .freq_data = (void*)&can_freq,
};

/* CLK NODE ECSPI */
static CLOCK_NAME alt_parent_ecspi[] = {
    CLK_PLL3_60,
    CLK_OSC,
};

static SINGLE_SEL_PARENT_DATA ecspi_parent = {
    .reg_offset = offsetof(struct CCM_MemMap, CSCDR2),
    .shift = CCM_CSCDR2_ecspi_clk_sel_SHIFT,
    .width = CCM_CSCDR2_ECSPI_CLK_SEL_WIDTH,
    .alt_parent = alt_parent_ecspi,
    .alt_parent_nr = sizeof(alt_parent_ecspi) / sizeof(CLOCK_NAME),
};

static SINGLE_GATE_DATA ecspi1_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR1),
    .reg_shift = CCM_CCGR1_CG0_SHIFT,
    .reg_width = CCM_CCGR1_CG0_WIDTH,
};

static SINGLE_GATE_DATA ecspi2_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR1),
    .reg_shift = CCM_CCGR1_CG1_SHIFT,
    .reg_width = CCM_CCGR1_CG1_WIDTH,
};

static SINGLE_GATE_DATA ecspi3_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR1),
    .reg_shift = CCM_CCGR1_CG2_SHIFT,
    .reg_width = CCM_CCGR1_CG2_WIDTH,
};

static SINGLE_GATE_DATA ecspi4_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR1),
    .reg_shift = CCM_CCGR1_CG3_SHIFT,
    .reg_width = CCM_CCGR1_CG3_WIDTH,
};

static SINGLE_GATE_DATA ecspi5_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR1),
    .reg_shift = CCM_CCGR1_CG4_SHIFT,
    .reg_width = CCM_CCGR1_CG4_WIDTH,
};

static SINGLE_DIV_FREQ_DATA ecspi_freq = {
    .reg_offset = offsetof(struct CCM_MemMap, CSCDR2),
    .shift = CCM_CSCDR2_ecspi_clk_podf_SHIFT,
    .width = CCM_CSCDR2_ECSPI_PODF_WIDTH,
};

static CLK_NODE_T clk_ecspi1 = {
    .name = CLK_ECSPI1,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "ECSPI1",
    .parent_data = (void*)&ecspi_parent,
    .gate_data = (void*)&ecspi1_gate,
    .freq_data = (void*)&ecspi_freq,
};

static CLK_NODE_T clk_ecspi2 = {
    .name = CLK_ECSPI2,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "ECSPI2",
    .parent_data = (void*)&ecspi_parent,
    .gate_data = (void*)&ecspi2_gate,
    .freq_data = (void*)&ecspi_freq,
};

static CLK_NODE_T clk_ecspi3 = {
    .name = CLK_ECSPI3,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "ECSPI3",
    .parent_data = (void*)&ecspi_parent,
    .gate_data = (void*)&ecspi3_gate,
    .freq_data = (void*)&ecspi_freq,
};

static CLK_NODE_T clk_ecspi4 = {
    .name = CLK_ECSPI4,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "ECSPI4",
    .parent_data = (void*)&ecspi_parent,
    .gate_data = (void*)&ecspi4_gate,
    .freq_data = (void*)&ecspi_freq,
};

static CLK_NODE_T clk_ecspi5 = {
    .name = CLK_ECSPI5,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "ECSPI5",
    .parent_data = (void*)&ecspi_parent,
    .gate_data = (void*)&ecspi5_gate,
    .freq_data = (void*)&ecspi_freq,
};

/* CLK NODE QSPI1 */
static CLOCK_NAME alt_parent_qspi1[] = {
    CLK_PLL3,
    CLK_PLL2_PFD0,
    CLK_PLL2_PFD2,
    CLK_PLL2,
    CLK_PLL3_PFD3,
    CLK_PLL3_PFD2,
    CLK_MAX,
    CLK_MAX
};

static SINGLE_SEL_PARENT_DATA qspi1_parent = {
    .reg_offset = offsetof(struct CCM_MemMap, CSCMR1),
    .shift = CCM_CSCMR1_qspi1_sel_SHIFT,
    .width = CCM_CSCMR1_QSPI1_CLK_SEL_WIDTH,
    .alt_parent = alt_parent_qspi1,
    .alt_parent_nr = sizeof(alt_parent_qspi1) / sizeof(CLOCK_NAME),
};

static SINGLE_GATE_DATA qspi1_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR3),
    .reg_shift = CCM_CCGR3_CG7_SHIFT,
    .reg_width = CCM_CCGR3_CG7_WIDTH,
};

static SINGLE_DIV_FREQ_DATA qspi1_freq = {
    .reg_offset = offsetof(struct CCM_MemMap, CSCMR1),
    .shift = CCM_CSCMR1_qspi1_podf_SHIFT,
    .width = CCM_CSCMR1_QSPI1_PODF_WIDTH,
};

static CLK_NODE_T clk_qspi1 = {
    .name = CLK_QSPI1,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "QSPI1",
    .parent_data = (void*)&qspi1_parent,
    .gate_data = (void*)&qspi1_gate,
    .freq_data = (void*)&qspi1_freq,
};

/* CLK NODE QSPI2 */
static CLOCK_NAME alt_parent_qspi2[] = {
    CLK_PLL2_PFD0,
    CLK_PLL2,
    CLK_PLL3,
    CLK_PLL2_PFD2,
    CLK_PLL3_PFD3,
    CLK_MAX,
    CLK_MAX,
    CLK_MAX
};

static SINGLE_SEL_PARENT_DATA qspi2_parent = {
    .reg_offset = offsetof(struct CCM_MemMap, CS2CDR),
    .shift = CCM_CS2CDR_qspi2_clk_sel_SHIFT,
    .width = CCM_CS2CDR_QSPI2_CLK_SEL_WIDTH,
    .alt_parent = alt_parent_qspi2,
    .alt_parent_nr = sizeof(alt_parent_qspi2) / sizeof(CLOCK_NAME),
};

static SINGLE_GATE_DATA qspi2_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR4),
    .reg_shift = CCM_CCGR4_CG5_SHIFT,
    .reg_width = CCM_CCGR4_CG5_WIDTH,
};

static DUAL_DIV_FREQ_DATA qspi2_freq = {
    .reg_offset = offsetof(struct CCM_MemMap, CS2CDR),
    .pred_shift = CCM_CS2CDR_qspi2_clk_pred_SHIFT,
    .pred_width = CCM_CS2CDR_QSPI2_CLK_PRED_WIDTH,
    .podf_shift = CCM_CS2CDR_qspi2_clk_podf_SHIFT,
    .podf_width = CCM_CS2CDR_QSPI2_PODF_WIDTH,
};

static CLK_NODE_T clk_qspi2 = {
    .name = CLK_QSPI2,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_DUAL_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "QSPI2",
    .parent_data = (void*)&qspi2_parent,
    .gate_data = (void*)&qspi2_gate,
    .freq_data = (void*)&qspi2_freq,
};
/* CLK NODE SSI */
static CLOCK_NAME alt_parent_ssi[] = {
    CLK_PLL3_PFD2,
    CLK_PLL5,
    CLK_PLL4,
    CLK_MAX,
};

static SINGLE_SEL_PARENT_DATA ssi1_parent = {
    .reg_offset = offsetof(struct CCM_MemMap, CSCMR1),
    .shift = CCM_CSCMR1_ssi1_clk_sel_SHIFT,
    .width = CCM_CSCMR1_SSI1_CLK_SEL_WIDTH,
    .alt_parent = alt_parent_ssi,
    .alt_parent_nr = sizeof(alt_parent_ssi) / sizeof(CLOCK_NAME),
};

static SINGLE_GATE_DATA ssi1_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR5),
    .reg_shift = CCM_CCGR5_CG9_SHIFT,
    .reg_width = CCM_CCGR5_CG9_WIDTH,
};

static DUAL_DIV_FREQ_DATA ssi1_freq = {
    .reg_offset = offsetof(struct CCM_MemMap, CS1CDR),
    .pred_shift = CCM_CS1CDR_ssi1_clk_pred_SHIFT,
    .pred_width = CCM_CS1CDR_SSI1_CLK_PRED_WIDTH,
    .podf_shift = CCM_CS1CDR_ssi1_clk_podf_SHIFT,
    .podf_width = CCM_CS1CDR_SSI1_PODF_WIDTH,
};

static CLK_NODE_T clk_ssi1 = {
    .name = CLK_SSI1,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_DUAL_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "SSI1",
    .parent_data = (void*)&ssi1_parent,
    .gate_data = (void*)&ssi1_gate,
    .freq_data = (void*)&ssi1_freq,
};

static SINGLE_SEL_PARENT_DATA ssi2_parent = {
    .reg_offset = offsetof(struct CCM_MemMap, CSCMR1),
    .shift = CCM_CSCMR1_ssi2_clk_sel_SHIFT,
    .width = CCM_CSCMR1_SSI2_CLK_SEL_WIDTH,
    .alt_parent = alt_parent_ssi,
    .alt_parent_nr = sizeof(alt_parent_ssi) / sizeof(CLOCK_NAME),
};

static SINGLE_GATE_DATA ssi2_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR5),
    .reg_shift = CCM_CCGR5_CG10_SHIFT,
    .reg_width = CCM_CCGR5_CG10_WIDTH,
};

static DUAL_DIV_FREQ_DATA ssi2_freq = {
    .reg_offset = offsetof(struct CCM_MemMap, CS2CDR),
    .pred_shift = CCM_CS2CDR_ssi2_clk_pred_SHIFT,
    .pred_width = CCM_CS2CDR_SSI2_CLK_PRED_WIDTH,
    .podf_shift = CCM_CS2CDR_ssi2_clk_podf_SHIFT,
    .podf_width = CCM_CS2CDR_SSI2_PODF_WIDTH,
};

static CLK_NODE_T clk_ssi2 = {
    .name = CLK_SSI2,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_DUAL_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "SSI2",
    .parent_data = (void*)&ssi2_parent,
    .gate_data = (void*)&ssi2_gate,
    .freq_data = (void*)&ssi2_freq,
};

static SINGLE_SEL_PARENT_DATA ssi3_parent = {
    .reg_offset = offsetof(struct CCM_MemMap, CSCMR1),
    .shift = CCM_CSCMR1_ssi3_clk_sel_SHIFT,
    .width = CCM_CSCMR1_SSI3_CLK_SEL_WIDTH,
    .alt_parent = alt_parent_ssi,
    .alt_parent_nr = sizeof(alt_parent_ssi) / sizeof(CLOCK_NAME),
};

static SINGLE_GATE_DATA ssi3_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR5),
    .reg_shift = CCM_CCGR5_CG11_SHIFT,
    .reg_width = CCM_CCGR5_CG11_WIDTH,
};

static DUAL_DIV_FREQ_DATA ssi3_freq = {
    .reg_offset = offsetof(struct CCM_MemMap, CS1CDR),
    .pred_shift = CCM_CS1CDR_ssi3_clk_pred_SHIFT,
    .pred_width = CCM_CS1CDR_SSI3_CLK_PRED_WIDTH,
    .podf_shift = CCM_CS1CDR_ssi1_clk_podf_SHIFT,
    .podf_width = CCM_CS1CDR_SSI3_PODF_WIDTH,
};

static CLK_NODE_T clk_ssi3 = {
    .name = CLK_SSI3,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_DUAL_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "SSI3",
    .parent_data = (void*)&ssi3_parent,
    .gate_data = (void*)&ssi3_gate,
    .freq_data = (void*)&ssi3_freq,
};

/* CLK NODE UART */
static CLOCK_NAME alt_parent_uart[] = {
    CLK_PLL3_80,
    CLK_OSC,
};

static SINGLE_SEL_PARENT_DATA uart_parent = {
    .reg_offset = offsetof(struct CCM_MemMap, CSCDR1),
    .shift = CCM_CSCDR1_uart_clk_sel_SHIFT,
    .width = CCM_CSCDR1_UART_CLK_SEL_WIDTH,
    .alt_parent = alt_parent_uart,
    .alt_parent_nr = sizeof(alt_parent_uart) / sizeof(CLOCK_NAME),
};

static SINGLE_GATE_DATA uart_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR5),
    .reg_shift = CCM_CCGR5_CG13_SHIFT,
    .reg_width = CCM_CCGR5_CG13_WIDTH,
};

static SINGLE_DIV_FREQ_DATA uart_freq = {
    .reg_offset = offsetof(struct CCM_MemMap, CSCDR1),
    .shift = CCM_CSCDR1_uart_clk_podf_SHIFT,
    .width = CCM_CSCDR1_UART_PODF_WIDTH,
};

static CLK_NODE_T clk_uart = {
    .name = CLK_UART,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_SINGLE_DIV,
    .parent_type = PARENT_ONE_SEL,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "UART",
    .parent_data = (void*)&uart_parent,
    .gate_data = (void*)&uart_gate,
    .freq_data = (void*)&uart_freq,
};

/* CLK NODE UART_IPG */
static SINGLE_GATE_DATA uart_ipg_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR5),
    .reg_shift = CCM_CCGR5_CG12_SHIFT,
    .reg_width = CCM_CCGR5_CG12_WIDTH,
};

static CLK_NODE_T clk_uart_ipg = {
    .name = CLK_UART_IPG,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_INHERIT,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "UART_IPG",
    .parent_data = (void*)&ipg_childs_parent,
    .gate_data = (void*)&uart_ipg_gate,
};

/* CLK NODE CLK_I2C1 */
static FIX_PARENT_DATA perclk_childs_parent = {
    .parent = CLK_PERCLK,
};

static SINGLE_GATE_DATA i2c1_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR2),
    .reg_shift = CCM_CCGR2_CG3_SHIFT,
    .reg_width = CCM_CCGR2_CG3_WIDTH,
};

static CLK_NODE_T clk_i2c1 = {
    .name = CLK_I2C1,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_INHERIT,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "I2C1",
    .parent_data = (void*)&perclk_childs_parent,
    .gate_data = (void*)&i2c1_gate,
};

/* CLK NODE CLK_I2C2 */
static SINGLE_GATE_DATA i2c2_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR2),
    .reg_shift = CCM_CCGR2_CG4_SHIFT,
    .reg_width = CCM_CCGR2_CG4_WIDTH,
};

static CLK_NODE_T clk_i2c2 = {
    .name = CLK_I2C2,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_INHERIT,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "I2C2",
    .parent_data = (void*)&perclk_childs_parent,
    .gate_data = (void*)&i2c2_gate,
};
/* CLK NODE CLK_I2C3 */
static SINGLE_GATE_DATA i2c3_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR2),
    .reg_shift = CCM_CCGR2_CG5_SHIFT,
    .reg_width = CCM_CCGR2_CG5_WIDTH,
};

static CLK_NODE_T clk_i2c3 = {
    .name = CLK_I2C3,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_INHERIT,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "I2C3",
    .parent_data = (void*)&perclk_childs_parent,
    .gate_data = (void*)&i2c3_gate,
};
/* CLK NODE CLK_I2C4 */
static SINGLE_GATE_DATA i2c4_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR6),
    .reg_shift = CCM_CCGR6_CG12_SHIFT,
    .reg_width = CCM_CCGR6_CG12_WIDTH,
};

static CLK_NODE_T clk_i2c4 = {
    .name = CLK_I2C4,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_INHERIT,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "I2C4",
    .parent_data = (void*)&perclk_childs_parent,
    .gate_data = (void*)&i2c4_gate,
};
/* CLK NODE CLK_EPIT1 */
static SINGLE_GATE_DATA epit1_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR1),
    .reg_shift = CCM_CCGR1_CG6_SHIFT,
    .reg_width = CCM_CCGR1_CG6_WIDTH,
};

static CLK_NODE_T clk_epit1 = {
    .name = CLK_EPIT1,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_INHERIT,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "EPIT1",
    .parent_data = (void*)&perclk_childs_parent,
    .gate_data = (void*)&epit1_gate,
};

/* CLK NODE CLK_EPIT2 */
static SINGLE_GATE_DATA epit2_gate = {
    .reg_offset = offsetof(struct CCM_MemMap, CCGR1),
    .reg_shift = CCM_CCGR1_CG7_SHIFT,
    .reg_width = CCM_CCGR1_CG7_WIDTH,
};

static CLK_NODE_T clk_epit2 = {
    .name = CLK_EPIT2,
    .gate_type = GATE_SINGLE_CG,
    .freq_type = FREQ_INHERIT,
    .parent_type = PARENT_FIXED,
    .active = 0,
    .is_always_open = 0,
    .freq_valid = 0,
    .name_str = "EPIT2",
    .parent_data = (void*)&perclk_childs_parent,
    .gate_data = (void*)&epit2_gate,
};

/*
 * End of clock nodes implementation
 */

/*
 * Clock Array
 */
static const P_CLK_NODE_T clk_table[] = {
    &clk_ckil,
    &clk_osc,
    &clk_pll2,
    &clk_pll2_pfd0,
    &clk_pll2_pfd2,
    &clk_pll2_pfd0_div,
    &clk_pll3,
    &clk_pll3_pfd1,
    &clk_pll3_pfd2,
    &clk_pll3_pfd3,
    &clk_pll3_80,
    &clk_pll3_60,
    &clk_pll4,
    &clk_pll5,
    &clk_pre_periph,
    &clk_periph2,
    &clk_periph,
    &clk_ahb,
    &clk_ipg,
    &clk_perclk,
    // &clk_m4,
    &clk_alt_ocram,
    &clk_ocram,
    &clk_flexcan1,
    &clk_flexcan1_ipg,
    &clk_flexcan2,
    &clk_flexcan2_ipg,
    &clk_canfd,
    &clk_ecspi1,
    &clk_ecspi2,
    &clk_ecspi3,
    &clk_ecspi4,
    &clk_ecspi5,
    &clk_qspi1,
    &clk_qspi2,
    &clk_ssi1,
    &clk_ssi2,
    &clk_ssi3,
    &clk_uart,
    &clk_uart_ipg,
    &clk_i2c1,
    &clk_i2c2,
    &clk_i2c3,
    &clk_i2c4,
    &clk_epit1,
    &clk_epit2,
};

/*
 * nodes that not in the clock tree, this is specific for the M4 clock node,
 * because of the glitch problem, m4 will not manipulate this node by itself,
 * only get information such as freq
 */
static const P_CLK_NODE_T individual_nodes[] = {
    &clk_m4,
};

/*
 * This array marks the clocks that should never be gated off.
 * For SDB board, QSPI2 is used, for AI board, QSPI1 is used
 */
static CLOCK_NAME ini_enable_clock_node_name[] = {
    CLK_CKIL,
    CLK_PERCLK,
    CLK_M4,
    CLK_OCRAM,
#if BSP_IMX6SX_SDB_M4
    CLK_QSPI2,
#elif BSP_IMX6SX_AI_M4
    CLK_QSPI1,
#endif
};

/*
 * Before Share memory clock management or similar mechanism is implemented, 
 * For Linux / MQX operational, all the share nodes should be kept open, so
 * we need to acturally make all the share nodes init-opened
 */
#if BSPCFG_CM_LINUX_PEER_WALKAROUND
CLOCK_NAME walkaournd_enable_clock_node_name[] = {
    // All other nodes
    CLK_OSC,
    CLK_PLL2,
    CLK_PLL2_PFD0,
    CLK_PLL2_PFD2,
    CLK_PLL2_PFD0_DIV,
    CLK_PLL3,
    CLK_PLL3_PFD1,
    CLK_PLL3_PFD2,
    CLK_PLL3_PFD3,
    CLK_PLL3_80,
    CLK_PLL3_60,
    CLK_PLL4,
    CLK_PLL5,
    CLK_PRE_PERIPH,
    CLK_PERIPH2,
    CLK_PERIPH,
    CLK_AHB,
    CLK_IPG,
    CLK_ALT_OCRAM,
    CLK_FLEXCAN1,
    CLK_FLEXCAN1_IPG,
    CLK_FLEXCAN2,
    CLK_FLEXCAN2_IPG,
    CLK_CANFD,
    CLK_ECSPI1,
    CLK_ECSPI2,
    CLK_ECSPI3,
    CLK_ECSPI4,
    CLK_ECSPI5,
    CLK_QSPI1,
    CLK_SSI1,
    CLK_SSI2,
    CLK_SSI3,
    CLK_UART,
    CLK_UART_IPG,
    CLK_I2C1,
    CLK_I2C2,
    CLK_I2C3,
    CLK_I2C4,
    CLK_EPIT1,
    CLK_EPIT2,
};
#endif

void clk_initialize_pre_enable_node(void)
{
    uint8_t cnt = sizeof(ini_enable_clock_node_name) / sizeof(CLOCK_NAME);
    uint8_t i;
    P_CLK_NODE_T p_clk;
    for (i=0; i!=cnt; i++) {
        p_clk = (P_CLK_NODE_T)clock_get(ini_enable_clock_node_name[i]);
        p_clk->enable_cnt = 1;
    }

#if BSPCFG_CM_LINUX_PEER_WALKAROUND
    /*
     * Make sure all the clock nodes will not be physically closed
     * This should be disabled when running clkapi demo
     */
    cnt = sizeof(walkaournd_enable_clock_node_name) / sizeof(CLOCK_NAME);
    for (i=0; i!=cnt; i++) {
        p_clk = (P_CLK_NODE_T)clock_get(walkaournd_enable_clock_node_name[i]);
        p_clk->is_always_open = 1;
    }
#endif
}

/*
 * Gating related operation implementation
 */
static uint32_t operate_clk_single_cg(P_CLK_NODE_T p_clk, uint8_t operation)
{
    uint32_t result = OPERATION_SUCCESS;
    P_ONE_GATE_DATA gate_data = (P_ONE_GATE_DATA)p_clk->gate_data;
    volatile uint32_t* config_reg = (uint32_t*)((uint32_t)s_reg_ccm + gate_data->reg_offset);
    uint32_t shift = gate_data->reg_shift;
    uint32_t width = gate_data->reg_width;
    uint32_t cg;

    switch (operation) {
        case CLK_GATE_ENABLE:
            INS_BITFIELD(config_reg, shift, width, CG_ON_ALL_EXCEPT_STOP);
            break;
        case CLK_GATE_DISABLE:
            INS_BITFIELD(config_reg, shift, width, CG_OFF_ALL);
            break;
        case CLK_GATE_QUERY:
            cg = EXTRACT_BITFIELD(config_reg, shift, width);
            result = (cg == CG_OFF_ALL) ? 0 : 1;
            break;
        default:
            break;
    }

    return result;
}

static uint32_t operate_clk_pll(P_CLK_NODE_T p_clk, uint8_t operation)
{
    uint32_t result = OPERATION_SUCCESS;
    P_PLL_GATE_DATA gate_data = (P_PLL_GATE_DATA)p_clk->gate_data;
    volatile uint32_t* config_reg = (uint32_t*)((uint32_t)s_reg_ccm + gate_data->reg_offset);
    uint32_t pwr_shift = gate_data->pwr_shift;
    uint32_t pwr_width = gate_data->pwr_width;
    uint32_t enable_shift = gate_data->enable_shift;
    uint32_t enable_width = gate_data->enable_width;
    uint32_t lock_shift = gate_data->lock_shift;
    uint32_t lock_width = gate_data->lock_width;
    uint32_t bypass_shift = gate_data->bypass_shift;
    uint32_t bypass_width = gate_data->bypass_width;
    uint32_t pwr_val = 0;
    uint32_t enable, lock, powerdown;

    switch (operation) {
        case CLK_GATE_ENABLE:
            if (p_clk->gate_type == GATE_PLL)
                pwr_val = 0; /*to powerdown field*/
            else if (p_clk->gate_type == GATE_PLL_USB)
                pwr_val = 1; /*to power field*/

            INS_BITFIELD(config_reg, bypass_shift, bypass_width, 0);
            INS_BITFIELD(config_reg, pwr_shift, pwr_width, pwr_val);
            WAIT(EXTRACT_BITFIELD(config_reg, lock_shift, lock_width), TIMEOUT);
            INS_BITFIELD(config_reg, enable_shift, enable_width, 1);
            break;
        case CLK_GATE_DISABLE:
            if (p_clk->gate_type == GATE_PLL)
                pwr_val = 1; /*to powerdown field*/
            else if (p_clk->gate_type == GATE_PLL_USB)
                pwr_val = 0; /*to power field*/

            INS_BITFIELD(config_reg, enable_shift, enable_width, 0);
            INS_BITFIELD(config_reg, pwr_shift, pwr_width, pwr_val);
            INS_BITFIELD(config_reg, bypass_shift, bypass_width, 1);
            break;
        case CLK_GATE_QUERY:
            enable = EXTRACT_BITFIELD(config_reg, enable_shift, enable_width);
            lock = EXTRACT_BITFIELD(config_reg, lock_shift, lock_width);
            powerdown = EXTRACT_BITFIELD(config_reg, pwr_shift, pwr_width);
            if (p_clk->gate_type == GATE_PLL_USB)
                powerdown = !powerdown; /*USB PLL has "POWER" instead of "POWERDOWN" field*/

            if (!powerdown && lock && enable)
                result = 1;
            else
                result = 0;
            break;
        default:
            break;
    }

    return result;
}

static uint32_t operate_clk_pll_pfd(P_CLK_NODE_T p_clk, uint8_t operation)
{
    uint32_t result = OPERATION_SUCCESS;
    P_PFD_GATE_DATA gate_data = (P_PFD_GATE_DATA)p_clk->gate_data;
    volatile uint32_t* config_reg = (uint32_t*)((uint32_t)s_reg_ccm + gate_data->reg_offset);
    uint32_t gate_shift = gate_data->gate_shift;
    uint32_t gate_width = gate_data->gate_width;
    uint32_t stable_shift = gate_data->stable_shift;
    uint32_t stable_width = gate_data->stable_width;
    uint32_t stable, gate;

    switch (operation) {
        case CLK_GATE_ENABLE:
            INS_BITFIELD(config_reg, gate_shift, gate_width, 0);
            break;
        case CLK_GATE_DISABLE:
            INS_BITFIELD(config_reg, gate_shift, gate_width, 1);
            break;
        case CLK_GATE_QUERY:
            stable = EXTRACT_BITFIELD(config_reg, stable_shift, stable_width);
            gate = EXTRACT_BITFIELD(config_reg, gate_shift, gate_width);
            if (stable && !gate)
                result = 1;
            else
                result = 0;
            break;
        default:
            break;
    }

    return result;
}

static uint32_t operate_clk_osc(P_CLK_NODE_T p_clk, uint8_t operation)
{
    uint32_t result = OPERATION_SUCCESS;
    P_OSC_GATE_DATA gate_data = (P_OSC_GATE_DATA)p_clk->gate_data;
    volatile uint32_t* config_reg = (uint32_t*)((uint32_t)s_reg_ccm + gate_data->reg_offset);
    uint32_t enable_shift = gate_data->enable_shift;
    uint32_t enable_width = gate_data->enable_width;
    uint32_t ready_shift = gate_data->ready_shift;
    uint32_t ready_width = gate_data->ready_width;

    switch (operation) {
        case CLK_GATE_ENABLE:
            INS_BITFIELD(config_reg, enable_shift, enable_width, 1);
            WAIT(EXTRACT_BITFIELD(config_reg, ready_shift, ready_width), TIMEOUT);
            break;
        case CLK_GATE_DISABLE:
            INS_BITFIELD(config_reg, enable_shift, enable_width, 0);
            break;
        case CLK_GATE_QUERY:
            result = EXTRACT_BITFIELD(config_reg, ready_shift, ready_width);
            break;
        default:
            break;
    }

    return result;
}

/*
 * Parent related Implementation
 */
P_CLK_NODE_T get_parent_fixed_parent(P_CLK_NODE_T p_clk)
{
    P_FIX_PARENT_DATA data = (P_FIX_PARENT_DATA)p_clk->parent_data;
    return clk_find_node(data->parent);
}

P_CLK_NODE_T get_parent_one_sel(P_CLK_NODE_T p_clk)
{
    P_SINGLE_SEL_PARENT_DATA data = (P_SINGLE_SEL_PARENT_DATA)p_clk->parent_data;
    uint32_t* config_reg = (uint32_t*)((uint32_t)s_reg_ccm + data->reg_offset);
    uint32_t shift = data->shift;
    uint32_t width = data->width;
    uint32_t sel = EXTRACT_BITFIELD(config_reg, shift, width);
    return clk_find_node(data->alt_parent[sel]);
}

/*
 * Freq related implementation
 */
static uint32_t calc_freq_fix(P_CLK_NODE_T p_clk, uint32_t parent_freq)
{
    P_FIX_FREQ_DATA freq_data = (P_FIX_FREQ_DATA)p_clk->freq_data;
    return freq_data->freq_val;
}

static uint32_t calc_freq_fix_div(P_CLK_NODE_T p_clk, uint32_t parent_freq)
{
    P_FIX_DIV_FREQ_DATA freq_data = (P_FIX_DIV_FREQ_DATA)p_clk->freq_data;
    return parent_freq / freq_data->div_val;
}

static uint32_t calc_freq_pll_a(P_CLK_NODE_T p_clk, uint32_t parent_freq)
{
    uint32_t freq = 0;
    P_PLL_A_FREQ_DATA freq_data = (P_PLL_A_FREQ_DATA)p_clk->freq_data;
    volatile uint32_t* config_reg = (uint32_t*)((uint32_t)s_reg_ccm + freq_data->reg_offset);
    uint32_t shift = freq_data->shift;
    uint32_t width = freq_data->width;
    uint32_t div_sel = EXTRACT_BITFIELD(config_reg, shift, width);
    if (div_sel == 0)
        freq = parent_freq * 20;
    else if (div_sel == 1)
        freq = parent_freq * 22;
    return freq;
}

static uint32_t calc_freq_pll_b(P_CLK_NODE_T p_clk, uint32_t parent_freq)
{
    uint32_t freq;
    P_PLL_B_FREQ_DATA freq_data = (P_PLL_B_FREQ_DATA)p_clk->freq_data;
    volatile uint32_t* config_reg = (uint32_t*)((uint32_t)s_reg_ccm + freq_data->reg_offset);
    uint32_t shift = freq_data->shift;
    uint32_t width = freq_data->width;
    uint32_t div = EXTRACT_BITFIELD(config_reg, shift, width);
    volatile uint32_t* num_reg = (uint32_t*)((uint32_t)s_reg_ccm + freq_data->num_reg_offset);
    volatile uint32_t* denom_reg = (uint32_t*)((uint32_t)s_reg_ccm + freq_data->denom_reg_offset);
    uint32_t num = *num_reg;
    uint32_t denom = *denom_reg;
    freq = parent_freq * div + (uint64_t)(parent_freq * num) / denom;
    return freq;
}

static uint32_t calc_freq_pll_pfd(P_CLK_NODE_T p_clk, uint32_t parent_freq)
{
    uint32_t freq;
    P_PFD_FREQ_DATA freq_data = (P_PFD_FREQ_DATA)p_clk->freq_data;
    volatile uint32_t* config_reg = (uint32_t*)((uint32_t)s_reg_ccm + freq_data->reg_offset);
    uint32_t shift = freq_data->shift;
    uint32_t width = freq_data->width;
    uint32_t frac = EXTRACT_BITFIELD(config_reg, shift, width);
    freq = (uint64_t)parent_freq * 18 / frac;
    return freq;
}

static uint32_t calc_freq_single_div(P_CLK_NODE_T p_clk, uint32_t parent_freq)
{
    P_SINGLE_DIV_FREQ_DATA freq_data = (P_SINGLE_DIV_FREQ_DATA)p_clk->freq_data;
    volatile uint32_t* config_reg = (uint32_t*)((uint32_t)s_reg_ccm + freq_data->reg_offset);
    uint32_t shift = freq_data->shift;
    uint32_t width = freq_data->width;
    uint32_t podf = EXTRACT_BITFIELD(config_reg, shift, width) + 1;
    return parent_freq / podf;
}

static uint32_t calc_freq_dual_div(P_CLK_NODE_T p_clk, uint32_t parent_freq)
{
    P_DUAL_DIV_FREQ_DATA freq_data = (P_DUAL_DIV_FREQ_DATA)p_clk->freq_data;
    volatile uint32_t* config_reg = (uint32_t*)((uint32_t)s_reg_ccm + freq_data->reg_offset);
    uint32_t pred_shift = freq_data->pred_shift;
    uint32_t pred_width = freq_data->pred_width;
    uint32_t podf_shift = freq_data->podf_shift;
    uint32_t podf_width = freq_data->podf_width;
    uint32_t pred = EXTRACT_BITFIELD(config_reg, pred_shift, pred_width) + 1;
    uint32_t podf = EXTRACT_BITFIELD(config_reg, podf_shift, podf_width) + 1;
    return parent_freq / (pred * podf);
}


/*
 * General Function : Gating Related, open / close clock node, query clock node status
 */
static void mark_ungated_child(P_CLK_NODE_T p_clk, uint8_t active)
{
    // Open all ungated child nodes
    P_CLK_NODE_T child, p;

    child = p_clk->child_ungated;
    p = NULL;

    if (child != NULL) {
        for (p = child; p != NULL; p = p->sibling) {
            mark_ungated_child(p, active);
            if (p->active != active)
            {
                p->active = active;
            }
        }
    }
}

static void open_clk_preproc(P_CLK_NODE_T p_clk)
{
    clk_inc_descendant(p_clk->parent);
}

static void open_clk_postproc(P_CLK_NODE_T p_clk)
{                                                
    p_clk->active = 1;
    // Open all ungated child nodes
    mark_ungated_child(p_clk, 1);
}

static void close_clk_preproc(P_CLK_NODE_T p_clk)
{
}

static void close_clk_postproc(P_CLK_NODE_T p_clk)
{
    p_clk->active = 0;
    clk_dec_descendant(p_clk->parent);
    // Close all ungated child nodes
    mark_ungated_child(p_clk, 0);
}

static void physically_open_clk(P_CLK_NODE_T p_clk)
{
    if (p_clk->gate_type != GATE_NONE) { /*No gate so no register operation is needed*/
        /*
         * has a new logic
         */
        switch (p_clk->gate_type) {
            case GATE_SINGLE_CG:
                operate_clk_single_cg(p_clk, CLK_GATE_ENABLE);
                break;
            case GATE_PLL:
            case GATE_PLL_USB:
                operate_clk_pll(p_clk, CLK_GATE_ENABLE);
                break;
            case GATE_PLL_PFD:
                operate_clk_pll_pfd(p_clk, CLK_GATE_ENABLE);
                break;
            case GATE_OSC:
                operate_clk_osc(p_clk, CLK_GATE_ENABLE);
                break;
            default:
                break;
        }
    }    
}

uint8_t clk_open(P_CLK_NODE_T p_clk)
{
    /* only dump when clock node is not uart itself and uart is active */
    if (p_clk != &clk_uart && p_clk != &clk_uart_ipg && clk_uart.active && clk_uart_ipg.active)
        clk_detail_message("\t[clk_drv] : will enable <%s>\n", p_clk->name_str);
    open_clk_preproc(p_clk);
#if CFG_ENABLE_SHARE_CM
    /*
     * The semaphore ensures
     *    1. all access to share memory is protected
     *    2. all access to CCM register is protected
     * During this inteval, interrupt is disabled
     */
    clk_get_semaphore();
    clk_update_sharemem_enable(p_clk, 1);
    switch (clk_get_sharemem_peer_enable(p_clk)) {
        case SHM_NO_SHARE_NODE:
        case SHM_LINUX_PEER_OFF:
        case SHM_LINUX_NOT_READY:
            // physically open the clock
            physically_open_clk(p_clk);
            break;
        case SHM_LINUX_PEER_ON:
        default:
            // need do nothing
            break;
    }
    clk_release_semaphore();
#else
    physically_open_clk(p_clk);
#endif
    open_clk_postproc(p_clk);
    return 1;
}

void physically_close_clk(P_CLK_NODE_T p_clk)
{
    if (p_clk->gate_type != GATE_NONE) {
        /*
         * has a new logic
         */
        switch (p_clk->gate_type) {
            case GATE_SINGLE_CG:
                operate_clk_single_cg(p_clk, CLK_GATE_DISABLE);
                break;
            case GATE_PLL:
            case GATE_PLL_USB:
                operate_clk_pll(p_clk, CLK_GATE_DISABLE);
                break;
            case GATE_PLL_PFD:
                operate_clk_pll_pfd(p_clk, CLK_GATE_DISABLE);
                break;
            case GATE_OSC:
                operate_clk_osc(p_clk, CLK_GATE_DISABLE);
                break;
            default:
                break;
        }
    }
}

uint8_t clk_close(P_CLK_NODE_T p_clk)
{
    /* only dump when clock node is not uart itself and uart is active */
    if (p_clk != &clk_uart && p_clk != &clk_uart_ipg && clk_uart.active && clk_uart_ipg.active)
        clk_detail_message("\t[clk_drv] : will disable <%s>\n", p_clk->name_str);
    close_clk_preproc(p_clk);
#if CFG_ENABLE_SHARE_CM
    /*
     * The semaphore ensures
     *    1. all access to share memory is protected
     *    2. all access to CCM register is protected
     * During this inteval, interrupt is disabled
     */
    clk_get_semaphore();
    clk_update_sharemem_enable(p_clk, 0);
    switch (clk_get_sharemem_peer_enable(p_clk)) {
        case SHM_NO_SHARE_NODE:
        case SHM_LINUX_PEER_OFF:
            // physically close the clock
            physically_close_clk(p_clk);
            break;
        case SHM_LINUX_PEER_ON:
        case SHM_LINUX_NOT_READY:
        default:
            // need do nothing
            break;
    }
    clk_release_semaphore();
#else
    physically_close_clk(p_clk);
#endif
    close_clk_postproc(p_clk);
    return 1;
}

uint8_t is_physically_open(P_CLK_NODE_T p_clk)
{
    uint8_t result = 0;

    switch (p_clk->gate_type) {
        case GATE_NONE:
            if (p_clk->parent == NULL)
                /*root gate*/
                result = 1;
            else
                /*Has a parent and no gate*/
                result = is_physically_open(p_clk->parent); // Is there possibility that p_clk->parent is NULL
            break;
        case GATE_SINGLE_CG: /*Has a prent and one gate*/
            result = operate_clk_single_cg(p_clk, CLK_GATE_QUERY);
            break;
        case GATE_PLL:
        case GATE_PLL_USB:
            result = operate_clk_pll(p_clk, CLK_GATE_QUERY);
            break;
        case GATE_PLL_PFD:
            result = operate_clk_pll_pfd(p_clk, CLK_GATE_QUERY);
            break;
        case GATE_OSC:
            result = operate_clk_osc(p_clk, CLK_GATE_QUERY);
            break;
        default:
            break;
    }
    return result;
}

/*
 * query peer status
 *  return 1 means peer is ready
 *  return 0 means peer is not ready
 */
uint32_t clk_query_peer_status(void)
{
#if CFG_ENABLE_SHARE_CM
    return clk_check_sharemem_peer_magic();
#else
    return LINUX_STATUS_NOT_READY;
#endif
}

/*
 * General function : Parent detemination
 */
P_CLK_NODE_T get_parent(P_CLK_NODE_T p_clk)
{
    P_CLK_NODE_T p_parent;
    switch (p_clk->parent_type) {
        case PARENT_NONE:
            p_parent = NULL;
            break;
        case PARENT_FIXED:
            p_parent = get_parent_fixed_parent(p_clk);
            break;
        case PARENT_ONE_SEL:
            p_parent = get_parent_one_sel(p_clk);
            break;
        case PARENT_TWO_SEL:
        default:
            /*no 2 selector in this soc*/
            p_parent = NULL;
            break;
    }
    return p_parent;
}

/*
 * General function : Freq Calculation
 */
void clk_init_node_freq(P_CLK_NODE_T p_clk)
{
    P_CLK_NODE_T parent = NULL;
    uint32_t parent_freq = 0;

    if (p_clk->freq_valid)
        return;
    else {
        parent = p_clk->parent;
        if (parent != NULL) {
            // get freq of parent node
            if (!parent->freq_valid) {
                clk_init_node_freq(parent);
            }
            parent_freq = parent->freq;
        }
        switch (p_clk->freq_type) {
            case FREQ_INHERIT:
                p_clk->freq = parent_freq;
                break;
            case FREQ_FIX:
                p_clk->freq = calc_freq_fix(p_clk, parent_freq);
                break;
            case FREQ_FIX_DIV:
                p_clk->freq = calc_freq_fix_div(p_clk, parent_freq);
                break;
            case FREQ_PLL_A:
                p_clk->freq = calc_freq_pll_a(p_clk, parent_freq);
                break;
            case FREQ_PLL_B:
                p_clk->freq = calc_freq_pll_b(p_clk, parent_freq);
                break;
            case FREQ_PLL_PFD:
                p_clk->freq = calc_freq_pll_pfd(p_clk, parent_freq);
                break;
            case FREQ_SINGLE_DIV:
                p_clk->freq = calc_freq_single_div(p_clk, parent_freq);
                break;
            case FREQ_DUAL_DIV:
                p_clk->freq = calc_freq_dual_div(p_clk, parent_freq);
                break;
            default:
                break;
        }
        p_clk->freq_valid = 1;
    }
}

/*
 * Tool functions
 */
void clk_inc_descendant(P_CLK_NODE_T p_clk)
{
    if (p_clk != NULL) {
        p_clk->descendant_cnt++;
        if (!p_clk->active) {
            clk_open(p_clk);
        }
    }
}

void clk_dec_descendant(P_CLK_NODE_T p_clk)
{
    if (p_clk != NULL) {
        p_clk->descendant_cnt--;
        if ((p_clk->enable_cnt == 0) && (p_clk->descendant_cnt == 0) && (!p_clk->is_always_open)) {
            clk_close(p_clk);
        }
    }
}

int clk_get_nodes_nr()
{
    return sizeof(clk_table) / sizeof(P_CLK_NODE_T);
}

int clk_get_individual_nodes_nr()
{
    return sizeof(individual_nodes) / sizeof(P_CLK_NODE_T);
}

P_CLK_NODE_T clk_find_node(CLOCK_NAME clk_name)
{
    P_CLK_NODE_T entry;
    int i;
    int node_nr = clk_get_nodes_nr();

    for (i=0; i!=node_nr; i++) {
        entry = clk_table[i];
        if (clk_name == entry->name) {
            return entry;
        }
    }

    node_nr = clk_get_individual_nodes_nr();
    for (i=0; i!=node_nr; i++) {
        entry = individual_nodes[i];
        if (clk_name == entry->name) {
            return entry;
        }
    }

    return NULL;
}

P_CLK_NODE_T clk_get_entry(int i)
{
    if ((i < 0) || (i > clk_get_nodes_nr())) {
        return NULL;
    }

    return clk_table[i];
}

P_CLK_NODE_T clk_get_individual_entry(int i)
{
    if ((i < 0) || (i > clk_get_individual_nodes_nr())) {
        return NULL;
    }

    return individual_nodes[i];
}

/*
 * Depreciated functions : to be handled later
 */
P_CLK_NODE_T clk_set_parent(P_CLK_NODE_T p_clk, P_CLK_NODE_T p_parent)
{
    uint8_t result;
    P_SINGLE_SEL_PARENT_DATA parent_data;
    uint8_t i, nr, target_index;
    P_CLK_NODE_T old_parent = NULL;
    uint32_t* config_reg;
    uint32_t shift, width;
    uint32_t old_freq, new_freq;

    /*
     * 1. Check if the operation is applicable
     *      - p_clk have alt parents
     *      - p_clk current parent is not p_parent
     *      - p_parent is in p_clk alt parents
     */
    if (p_clk->parent == p_parent)
        result = CHECK_PARENT_IDENTICAL;
    else
        switch (p_clk->parent_type) {
            case PARENT_NONE:
            case PARENT_FIXED:
                result = CHECK_PARENT_INVALID;
                break;
            case PARENT_ONE_SEL:
                parent_data = (P_SINGLE_SEL_PARENT_DATA)p_clk->parent_data;
                nr = parent_data->alt_parent_nr;
                for (i=0; i!=nr; i++)
                    if (parent_data->alt_parent[i] == p_parent->name) {
                        target_index = i;
                        result = CHECK_PARENT_PASS;
                        break;
                    }
                if (i == nr)
                    result = CHECK_PARENT_INVALID;
                break;
            default:
                result = CHECK_PARENT_INVALID;
                break;
        }

    if (result == CHECK_PARENT_PASS) {
        /*Begin Change Parent*/
        old_parent = p_clk->parent;
        // Update Data Structure
        clk_remove_child(old_parent, p_clk);
        clk_add_child(p_parent, p_clk);
        p_clk->parent = p_parent;
        // Prepare the new node for on-the-fly change
        if (p_clk->active)
            clk_inc_descendant(p_parent);
        
        // register configuration
        switch (p_clk->parent_type) {
            case PARENT_ONE_SEL:
                parent_data = (P_SINGLE_SEL_PARENT_DATA)p_clk->parent_data;
                config_reg = (uint32_t*)((uint32_t)s_reg_ccm + parent_data->reg_offset);
                shift = parent_data->shift;
                width = parent_data->width;
                INS_BITFIELD(config_reg, shift, width, target_index);
                break;
            default:
                break;
        }

        // Decrement the original
        if (p_clk->active)
            clk_dec_descendant(old_parent);

        // Freq Change
        old_freq = old_parent->freq;
        new_freq = p_parent->freq;
        p_clk->freq = ((uint64_t)p_clk->freq * new_freq + old_freq / 2) / old_freq;
        clk_update_child_freq(p_clk, old_freq, new_freq);

        // Return Old Parent
        return old_parent;
    } else {
        printf("Fault : result is %d\n", result);
        return NULL;
    }
}

void clk_update_child_freq(P_CLK_NODE_T p_clk, uint8_t old_val, uint8_t new_val)
{
    P_CLK_NODE_T child = p_clk->child_gated;
    P_CLK_NODE_T p = NULL;

    /*First Round for gated child*/
    if (child != NULL) {
        for (p = child; p != NULL; p = p->sibling) {
            p->freq = ((uint64_t)p->freq * new_val + old_val / 2) / old_val;
            clk_update_child_freq(p, old_val, new_val);
        }
    }

    /*Second Round for Ungated child*/
    child = p_clk->child_ungated;
    p = NULL;
    if (child != NULL) {
        for (p = child; p != NULL; p = p->sibling) {
            p->freq = ((uint64_t)p->freq * new_val + old_val / 2) / old_val;
            clk_update_child_freq(p, old_val, new_val);
        }
    }
}

static void clk_add_child_gated(P_CLK_NODE_T parent, P_CLK_NODE_T child)
{
    P_CLK_NODE_T sibling, prev_node;
    if (parent != NULL) {
        if (parent->child_gated == NULL)
            parent->child_gated = child;
        else {
            prev_node = parent->child_gated;
            sibling = prev_node->sibling;
            while (sibling != NULL) {
                prev_node = sibling;
                sibling = sibling->sibling;
            }
            prev_node->sibling = child;
        }
    }
}

static void clk_add_child_ungated(P_CLK_NODE_T parent, P_CLK_NODE_T child)
{
    P_CLK_NODE_T sibling, prev_node;
    if (parent != NULL) {
        if (parent->child_ungated == NULL)
            parent->child_ungated = child;
        else {
            prev_node = parent->child_ungated;
            sibling = prev_node->sibling;
            while (sibling != NULL) {
                prev_node = sibling;
                sibling = sibling->sibling;
            }
            prev_node->sibling = child;
        }
    }
}

void clk_add_child(P_CLK_NODE_T parent, P_CLK_NODE_T child)
{
    if (child->gate_type != GATE_NONE) {
        clk_add_child_gated(parent, child);      
    }
    else {
        clk_add_child_ungated(parent, child);
    }
}

static void clk_remove_child_gated(P_CLK_NODE_T parent, P_CLK_NODE_T child)
{
    P_CLK_NODE_T sibling, prev_node;
    if (parent != NULL) {
        if (parent->child_gated == child) {
            parent->child_gated = child->sibling;
            child->sibling = NULL;
        } else {
            prev_node = parent->child_gated;
            sibling = parent->child_gated->sibling;
            while (sibling != NULL) {
                if (sibling == child)
                    break;
                prev_node = sibling;
                sibling = prev_node->sibling;
            }
            prev_node->sibling = child->sibling;
            child->sibling = NULL;
        }
    }
}

static void clk_remove_child_ungated(P_CLK_NODE_T parent, P_CLK_NODE_T child)
{
    P_CLK_NODE_T sibling, prev_node;
    if (parent != NULL) {
        if (parent->child_ungated == child) {
            parent->child_ungated = child->sibling;
            child->sibling = NULL;
        } else {
            prev_node = parent->child_ungated;
            sibling = parent->child_ungated->sibling;
            while (sibling != NULL) {
                if (sibling == child)
                    break;
                prev_node = sibling;
                sibling = prev_node->sibling;
            }
            prev_node->sibling = child->sibling;
            child->sibling = NULL;
        }
    }
}

void clk_remove_child(P_CLK_NODE_T parent, P_CLK_NODE_T child)
{
    if (child->gate_type != GATE_NONE) {
        clk_remove_child_gated(parent, child);      
    }
    else {
        clk_remove_child_ungated(parent, child);
    }
}
