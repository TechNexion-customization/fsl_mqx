/*HEADER**********************************************************************
*
* Copyright 2014 Freescale Semiconductor, Inc.
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
*   This file contains board-specific pin initialization functions.
*
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_adc_io_init
* Returned Value   : 0 for success, -1 for failure
* Comments         :
*    This function performs BSP-specific initialization related to ADC
*
*END*----------------------------------------------------------------------*/

_mqx_int _bsp_adc_io_init
(
     /* [IN] number of ADC device on which to perform hardware initialization */
    _mqx_uint adc_num
)
{
    int32_t status = IO_ERROR;
    /* The clock of ADC on i.mx6sx is always enabled since it roots from
     * ipg_clk_root directly. */

    if (adc_num < ADC_NUM_DEVICES)
    {
        status = MQX_OK;
    }
    return status;
}


/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_adc_channel_io_init
* Returned Value   : 0 for success, -1 for failure
* Comments         :
*    This function performs BSP-specific initialization related to ADC
*
*END*----------------------------------------------------------------------*/
_mqx_int _bsp_adc_channel_io_init
(
     /* [IN] number of ADC device on which to perform hardware initialization */
    _mqx_uint input
)
{
    uint32_t device, channel;
    int32_t status = IO_ERROR;

    // decode device & channel
    device  = ADC_DEVICE(input);
    channel = ADC_CHANNEL(input);

    // Only channel 0-3 are supported for i.mx6sx.
    // - gpio mux is not required for these channels
    if (device == 0 && (channel >= 0 && channel <= 3))
    {
        status = MQX_OK;
    }

    // Only channel 0-3 are supported for i.mx6sx
    // - gpio mux is not required for these channels
    if (device == 1 && (channel >= 0 && channel <= 3))
    {
        status = MQX_OK;
    }

    return status;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_gpio_io_init
* Returned Value   : MQX_OK or -1
* Comments         :
*    This function performs BSP-specific initialization related to GPIO
*
*END*----------------------------------------------------------------------*/

_mqx_int _bsp_gpio_io_init
(
    void
)
{
    // enable clock gate for all ports
    CCM_CCGR2 |=
        CCM_CCGR2_CG7(0x3)  |  // iomuxc
        CCM_CCGR2_CG8(0x3)  |  // ipmux 1
        CCM_CCGR2_CG9(0x3)  |  // ipmux 2
        CCM_CCGR2_CG10(0x3);   // ipmux 3

    return MQX_OK;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_serial_io_init
* Returned Value   : MQX_OK for success, -1 for failure
* Comments         :
*    This function performs BSP-specific initialization related to serial
*
*END*----------------------------------------------------------------------*/

_mqx_int _bsp_serial_io_init
(
    /* [IN] Serial device number */
    _mqx_uint dev_num,

    /* [IN] Required functionality */
    _mqx_uint flags
)
{
    int32_t status = IO_ERROR;

    switch (dev_num)
    {
        case 1:
            /* Enable pin mux */
            if (flags & IO_PERIPHERAL_PIN_MUX_ENABLE)
            {
                IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO04 = IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO04_MUX_MODE(0);
                IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO05 = IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO05_MUX_MODE(0);
                IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO04 = IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO04_PKE_MASK | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO04_PUE_MASK | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO04_PUS(2)   | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO04_SPEED(2) | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO04_DSE(6)   | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO04_SRE_MASK | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO04_HYS_MASK;
                IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO05 = IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO05_PKE_MASK | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO05_PUE_MASK | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO05_PUS(2)   | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO05_SPEED(2) | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO05_DSE(6)   | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO05_SRE_MASK | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO05_HYS_MASK;
                IOMUXC_UART1_IPP_UART_RXD_MUX_SELECT_INPUT = 1;
            }
            /* Disable pin mux to default gpio */
            if (flags & IO_PERIPHERAL_PIN_MUX_DISABLE)
            {
                IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO04 = IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO04_MUX_MODE(5);
                IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO05 = IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO05_MUX_MODE(5);
            }

            status = MQX_OK;
            break;
        case 2:
            /* Enable pin mux */
            if (flags & IO_PERIPHERAL_PIN_MUX_ENABLE)
            {
                IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO06 = IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO06_MUX_MODE(0);
                IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO07 = IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO07_MUX_MODE(0);
                IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO06 = IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO06_PKE_MASK | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO06_PUE_MASK | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO06_PUS(2)   | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO06_SPEED(2) | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO06_DSE(6)   | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO06_SRE_MASK | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO06_HYS_MASK;
                IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO07 = IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO07_PKE_MASK | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO07_PUE_MASK | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO07_PUS(2)   | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO07_SPEED(2) | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO07_DSE(6)   | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO07_SRE_MASK | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO07_HYS_MASK;
                IOMUXC_UART2_IPP_UART_RXD_MUX_SELECT_INPUT = 1;
            }
            /* Disable pin mux to the default gpio */
            if (flags & IO_PERIPHERAL_PIN_MUX_DISABLE)
            {
                IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO06 = IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO06_MUX_MODE(5);
                IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO07 = IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO07_MUX_MODE(5);
            }

            status = MQX_OK;
            break;
    }

    return status;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_flexcan_io_init
* Returned Value   : MQX_OK for success, -1 for failure
* Comments         :
*    This function performs BSP-specific initialization related to FLEXCAN
*
*END*----------------------------------------------------------------------*/
_mqx_int _bsp_flexcan_io_init
(
    /* [IN] FlexCan device number */
    _mqx_uint dev_num
)
{
    int32_t status = IO_ERROR;
    volatile GPIO_MemMapPtr gpio4_pdir;
    void   *addr;

    switch (dev_num)
    {
        case 1:
#if 0
            /* This configure is for old imx6sx-sdb board */
            // CAN1_2_EN drive HIGH
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA1 = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA1_MUX_MODE(5);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DATA1 = 0x1B0B0;
            addr = (void *) GPIO4_BASE_PTR;
            gpio4_pdir = addr;
            gpio4_pdir->GDIR |= 0X02000000;
            gpio4_pdir->DR |= 0X02000000;
            // CAN1_2_STBY_B DRIVE HIGH
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3 = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3_MUX_MODE(5);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DATA3 = 0x1B0B0;
            addr = (void *) GPIO4_BASE_PTR;
            gpio4_pdir = addr;
            gpio4_pdir->GDIR |= 0X08000000;
            gpio4_pdir->DR |= 0X08000000;
#else
            /* This configure is for new imx6sx-sdb board */
            // CAN1_2_STBY_B DRIVE LOW
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3 = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3_MUX_MODE(5);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DATA3 = 0x1B0B0;
            addr = (void *) GPIO4_BASE_PTR;
            gpio4_pdir = addr;
            gpio4_pdir->GDIR |= 0X08000000;
            gpio4_pdir->DR &= ~0X08000000;
#endif
            // CAN1_TX
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DQS = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DQS_MUX_MODE(1);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DQS = 0x1B0B0;

            // CAN1_RX
            IOMUXC_SW_MUX_CTL_PAD_QSPI1A_SS1_B = IOMUXC_SW_MUX_CTL_PAD_QSPI1A_SS1_B_MUX_MODE(1);
            IOMUXC_CAN1_IPP_IND_CANRX_SELECT_INPUT = 0x2;
            IOMUXC_SW_PAD_CTL_PAD_QSPI1A_SS1_B = 0x1B0B0;

            status = MQX_OK;
            break;
        case 2:
#if 0
            /* This configure is for old imx6sx-sdb board */
            // CAN1_2_EN drive HIGH
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA1 = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA1_MUX_MODE(5);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DATA1 = 0x1B0B0;
            addr = (void *) GPIO4_BASE_PTR;
            gpio4_pdir = addr;
            gpio4_pdir->GDIR |= 0X02000000;
            gpio4_pdir->DR |= 0X02000000;
            // CAN1_2_STBY_B DRIVE HIGH
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3 = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3_MUX_MODE(5);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DATA3 = 0x1B0B0;
            addr = (void *) GPIO4_BASE_PTR;
            gpio4_pdir = addr;
            gpio4_pdir->GDIR |= 0X08000000;
            gpio4_pdir->DR |= 0X08000000;
#else
            /* This configure is for new imx6sx-sdb board */
            // CAN1_2_STBY_B DRIVE LOW
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3 = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3_MUX_MODE(5);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DATA3 = 0x1B0B0;
            addr = (void *) GPIO4_BASE_PTR;
            gpio4_pdir = addr;
            gpio4_pdir->GDIR |= 0X08000000;
            gpio4_pdir->DR &= ~0X08000000;
#endif
            // CAN2_TX
            IOMUXC_SW_MUX_CTL_PAD_QSPI1A_DQS = IOMUXC_SW_MUX_CTL_PAD_QSPI1A_DQS_MUX_MODE(1);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1A_DQS = 0x1B0B0;

            // CAN2_RX
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_SS1_B = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_SS1_B_MUX_MODE(1);
            IOMUXC_CAN2_IPP_IND_CANRX_SELECT_INPUT = 0x2;
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_SS1_B = 0x1B0B0;
            status = MQX_OK;
            break;
    }

    return status;
}

#if 0
/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_mcan_io_init
* Returned Value   : MQX_OK for success, -1 for failure
* Comments         :
*    This function performs BSP-specific initialization related to M_CAN
*
*END*----------------------------------------------------------------------*/
_mqx_int _bsp_mcan_io_init
(
    /* [IN] M_Can device number */
    _mqx_uint dev_num
)
{
    int32_t status = IO_ERROR;
    volatile GPIO_MemMapPtr gpio4_pdir;
    void   *addr;

    switch (dev_num)
    {
        case 1:
#if 0
            /* This configure is for old imx6sx-sdb board */
            // CAN1_2_EN drive HIGH
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA1 = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA1_MUX_MODE(5);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DATA1 = 0x1B0B0;
            addr = (void *) GPIO4_BASE_PTR;
            gpio4_pdir = addr;
            gpio4_pdir->GDIR |= 0X02000000;
            gpio4_pdir->DR |= 0X02000000;
            // CAN1_2_STBY_B DRIVE HIGH
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3 = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3_MUX_MODE(5);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DATA3 = 0x1B0B0;
            addr = (void *) GPIO4_BASE_PTR;
            gpio4_pdir = addr;
            gpio4_pdir->GDIR |= 0X08000000;
            gpio4_pdir->DR |= 0X08000000;
#else
            /* This configure is for new imx6sx-sdb board */
            // CAN1_2_STBY_B DRIVE LOW
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3 = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3_MUX_MODE(5);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DATA3 = 0x1B0B0;
            addr = (void *) GPIO4_BASE_PTR;
            gpio4_pdir = addr;
            gpio4_pdir->GDIR |= 0X08000000;
            gpio4_pdir->DR &= ~0X08000000;
#endif
            // M_CAN1_TX
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DQS = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DQS_MUX_MODE(2);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DQS = 0x1B0B0;

            // M_CAN1_RX
            IOMUXC_SW_MUX_CTL_PAD_QSPI1A_SS1_B = IOMUXC_SW_MUX_CTL_PAD_QSPI1A_SS1_B_MUX_MODE(2);
            IOMUXC_CANFD_IPD_M_CAN_0_RX_SELECT_INPUT = 0x2;
            IOMUXC_SW_PAD_CTL_PAD_QSPI1A_SS1_B = 0x1B0B0;

            status = MQX_OK;
            break;
        case 2:
#if 0
            /* This configure is for old imx6sx-sdb board */
            // CAN1_2_EN drive HIGH
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA1 = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA1_MUX_MODE(5);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DATA1 = 0x1B0B0;
            addr = (void *) GPIO4_BASE_PTR;
            gpio4_pdir = addr;
            gpio4_pdir->GDIR |= 0X02000000;
            gpio4_pdir->DR |= 0X02000000;
            // CAN1_2_STBY_B DRIVE HIGH
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3 = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3_MUX_MODE(5);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DATA3 = 0x1B0B0;
            addr = (void *) GPIO4_BASE_PTR;
            gpio4_pdir = addr;
            gpio4_pdir->GDIR |= 0X08000000;
            gpio4_pdir->DR |= 0X08000000;
#else
            /* This configure is for new imx6sx-sdb board */
            // CAN1_2_STBY_B DRIVE LOW
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3 = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DATA3_MUX_MODE(5);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DATA3 = 0x1B0B0;
            addr = (void *) GPIO4_BASE_PTR;
            gpio4_pdir = addr;
            gpio4_pdir->GDIR |= 0X08000000;
            gpio4_pdir->DR &= ~0X08000000;
#endif
            // M_CAN2_TX
            IOMUXC_SW_MUX_CTL_PAD_QSPI1A_DQS = IOMUXC_SW_MUX_CTL_PAD_QSPI1A_DQS_MUX_MODE(2);
            IOMUXC_SW_PAD_CTL_PAD_QSPI1A_DQS = 0x1B0B0;

            // M_CAN2_RX
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_SS1_B = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_SS1_B_MUX_MODE(2);
            IOMUXC_CANFD_IPD_M_CAN_1_RX_SELECT_INPUT = 0x2;
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_SS1_B = 0x1B0B0;

            status = MQX_OK;
            break;
    }

    return status;
}
#endif

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_ecspi_io_init
* Returned Value   : MQX_OK for success, -1 for failure
* Comments         :
*    This function performs BSP-specific initialization related to ecspi
*
*END*----------------------------------------------------------------------*/

_mqx_int _bsp_ecspi_io_init
(
    /* [IN] ECSPI device number */
    _mqx_uint dev_num,

    /* [IN] Required functionality */
    _mqx_uint flags
)
{
    int32_t status = IO_ERROR;

    switch (dev_num)
    {
        case 4:
            if (flags & IO_PERIPHERAL_PIN_MUX_ENABLE)
            {
                /* daisy chain selection */
                IOMUXC_ECSPI4_IPP_CSPI_CLK_IN_SELECT_INPUT = 1;
                IOMUXC_ECSPI4_IPP_IND_MISO_SELECT_INPUT = 1;
                IOMUXC_ECSPI4_IPP_IND_MOSI_SELECT_INPUT = 1;

                /* iomux */
                IOMUXC_SW_MUX_CTL_PAD_SD2_CLK = IOMUXC_SW_MUX_CTL_PAD_SD2_CLK_MUX_MODE(3);    /* ECSPI SLK  */
                IOMUXC_SW_MUX_CTL_PAD_SD2_CMD = IOMUXC_SW_MUX_CTL_PAD_SD2_CMD_MUX_MODE(3);    /* ECSPI MOSI */
                IOMUXC_SW_MUX_CTL_PAD_SD2_DATA0 = IOMUXC_SW_MUX_CTL_PAD_SD2_DATA0_MUX_MODE(6);  /* ECSPI SS3  */
                IOMUXC_SW_MUX_CTL_PAD_SD2_DATA3 = IOMUXC_SW_MUX_CTL_PAD_SD2_DATA3_MUX_MODE(3);  /* ECSPI MISO */

                /* pad control */
                IOMUXC_SW_PAD_CTL_PAD_SD2_CLK = IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_PKE_MASK | \
                                                IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_PUE_MASK | \
                                                IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_PUS(0)   | \
                                                IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_SPEED(2) | \
                                                IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_DSE(6)   | \
                                                IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_SRE_MASK | \
                                                IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_HYS_MASK;
                IOMUXC_SW_PAD_CTL_PAD_SD2_CMD = IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_SPEED(2) | \
                                                IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_DSE(6)   | \
                                                IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_SRE_MASK | \
                                                IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_HYS_MASK;
                IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0 = IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_SPEED(2) | \
                                                  IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_DSE(6)   | \
                                                  IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_SRE_MASK | \
                                                  IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_HYS_MASK;
                IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3 = IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_SPEED(2) | \
                                                  IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_DSE(6)   | \
                                                  IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_SRE_MASK | \
                                                  IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_HYS_MASK;
            }

            status = MQX_OK;
            break;

        case 5:
            if (flags & IO_PERIPHERAL_PIN_MUX_ENABLE)
            {
                /* daisy chain selection */
                IOMUXC_ECSPI5_IPP_CSPI_CLK_IN_SELECT_INPUT = 1;
                IOMUXC_ECSPI5_IPP_IND_MISO_SELECT_INPUT = 1;
                IOMUXC_ECSPI5_IPP_IND_MOSI_SELECT_INPUT = 1;
                IOMUXC_ECSPI5_IPP_IND_SS_B_SELECT_INPUT_0 = 1;

                /* iomux */
                IOMUXC_SW_MUX_CTL_PAD_QSPI1A_SS1_B = IOMUXC_SW_MUX_CTL_PAD_QSPI1A_SS1_B_MUX_MODE(3);
                IOMUXC_SW_MUX_CTL_PAD_QSPI1A_DQS = IOMUXC_SW_MUX_CTL_PAD_QSPI1A_DQS_MUX_MODE(3);
                IOMUXC_SW_MUX_CTL_PAD_QSPI1B_SS1_B = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_SS1_B_MUX_MODE(3);
                IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DQS = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DQS_MUX_MODE(3);

                /* pad control */
                IOMUXC_SW_PAD_CTL_PAD_QSPI1A_SS1_B = IOMUXC_SW_PAD_CTL_PAD_QSPI1A_SS1_B_SPEED(2) | \
                                                     IOMUXC_SW_PAD_CTL_PAD_QSPI1A_SS1_B_DSE(6)   | \
                                                     IOMUXC_SW_PAD_CTL_PAD_QSPI1A_SS1_B_SRE_MASK | \
                                                     IOMUXC_SW_PAD_CTL_PAD_QSPI1A_SS1_B_HYS_MASK;
                IOMUXC_SW_PAD_CTL_PAD_QSPI1A_DQS = IOMUXC_SW_PAD_CTL_PAD_QSPI1A_DQS_SPEED(2) | \
                                                   IOMUXC_SW_PAD_CTL_PAD_QSPI1A_DQS_DSE(6)   | \
                                                   IOMUXC_SW_PAD_CTL_PAD_QSPI1A_DQS_SRE_MASK | \
                                                   IOMUXC_SW_PAD_CTL_PAD_QSPI1A_DQS_HYS_MASK;
                IOMUXC_SW_PAD_CTL_PAD_QSPI1B_SS1_B = IOMUXC_SW_PAD_CTL_PAD_QSPI1B_SS1_B_PKE_MASK | \
                                                     IOMUXC_SW_PAD_CTL_PAD_QSPI1B_SS1_B_PUE_MASK | \
                                                     IOMUXC_SW_PAD_CTL_PAD_QSPI1B_SS1_B_PUS(0)   | \
                                                     IOMUXC_SW_PAD_CTL_PAD_QSPI1B_SS1_B_SPEED(2) | \
                                                     IOMUXC_SW_PAD_CTL_PAD_QSPI1B_SS1_B_DSE(6)   | \
                                                     IOMUXC_SW_PAD_CTL_PAD_QSPI1B_SS1_B_SRE_MASK | \
                                                     IOMUXC_SW_PAD_CTL_PAD_QSPI1B_SS1_B_HYS_MASK;
                IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DQS = IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DQS_SPEED(2) | \
                                                   IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DQS_DSE(6)   | \
                                                   IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DQS_SRE_MASK | \
                                                   IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DQS_HYS_MASK;
            }

            status = MQX_OK;
            break;
    }

    return status;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_ecspi_slave_io_init
* Returned Value   : MQX_OK for success, -1 for failure
* Comments         :
*    This function performs BSP-specific initialization related to ecspi slave.
*
*END*----------------------------------------------------------------------*/

_mqx_int _bsp_ecspi_slave_io_init
(
    /* [IN] ECSPI device number */
    _mqx_uint dev_num
)
{
    int32_t status = IO_ERROR;

    switch (dev_num)
    {
        case 4:
            /* daisy chain selection */
            IOMUXC_ECSPI4_IPP_CSPI_CLK_IN_SELECT_INPUT = 1;
            IOMUXC_ECSPI4_IPP_IND_MISO_SELECT_INPUT = 1;
            IOMUXC_ECSPI4_IPP_IND_MOSI_SELECT_INPUT = 1;

            /* iomux */
            IOMUXC_SW_MUX_CTL_PAD_SD2_CLK = IOMUXC_SW_MUX_CTL_PAD_SD2_CLK_MUX_MODE(3);    /* ECSPI SLK  */
            IOMUXC_SW_MUX_CTL_PAD_SD2_CMD = IOMUXC_SW_MUX_CTL_PAD_SD2_CMD_MUX_MODE(3);    /* ECSPI MOSI */
            IOMUXC_SW_MUX_CTL_PAD_SD2_DATA0 = IOMUXC_SW_MUX_CTL_PAD_SD2_DATA0_MUX_MODE(6);  /* ECSPI SS3  */
            IOMUXC_SW_MUX_CTL_PAD_SD2_DATA3 = IOMUXC_SW_MUX_CTL_PAD_SD2_DATA3_MUX_MODE(3);  /* ECSPI MISO */

            /* pad control */
            IOMUXC_SW_PAD_CTL_PAD_SD2_CLK = IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_SPEED(2) | \
                                            IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_DSE(6)   | \
                                            IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_SRE_MASK | \
                                            IOMUXC_SW_PAD_CTL_PAD_SD2_CLK_HYS_MASK;
            IOMUXC_SW_PAD_CTL_PAD_SD2_CMD = IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_SPEED(2) | \
                                            IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_DSE(6)   | \
                                            IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_SRE_MASK | \
                                            IOMUXC_SW_PAD_CTL_PAD_SD2_CMD_HYS_MASK;
            IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0 = IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_SPEED(2) | \
                                              IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_DSE(6)   | \
                                              IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_SRE_MASK | \
                                              IOMUXC_SW_PAD_CTL_PAD_SD2_DATA0_HYS_MASK;
            IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3 = IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_SPEED(2) | \
                                              IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_DSE(6)   | \
                                              IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_SRE_MASK | \
                                              IOMUXC_SW_PAD_CTL_PAD_SD2_DATA3_HYS_MASK;

            status = MQX_OK;
            break;
        case 5:
            /* daisy chain selection */
            IOMUXC_ECSPI5_IPP_CSPI_CLK_IN_SELECT_INPUT = 1;
            IOMUXC_ECSPI5_IPP_IND_MISO_SELECT_INPUT = 1;
            IOMUXC_ECSPI5_IPP_IND_MOSI_SELECT_INPUT = 1;
            IOMUXC_ECSPI5_IPP_IND_SS_B_SELECT_INPUT_0 = 1;

            /* iomux */
            IOMUXC_SW_MUX_CTL_PAD_QSPI1A_SS1_B = IOMUXC_SW_MUX_CTL_PAD_QSPI1A_SS1_B_MUX_MODE(3);
            IOMUXC_SW_MUX_CTL_PAD_QSPI1A_DQS = IOMUXC_SW_MUX_CTL_PAD_QSPI1A_DQS_MUX_MODE(3);
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_SS1_B = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_SS1_B_MUX_MODE(3);
            IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DQS = IOMUXC_SW_MUX_CTL_PAD_QSPI1B_DQS_MUX_MODE(3);

            /* pad control */
            IOMUXC_SW_PAD_CTL_PAD_QSPI1A_SS1_B = IOMUXC_SW_PAD_CTL_PAD_QSPI1A_SS1_B_SPEED(2) | \
                                                 IOMUXC_SW_PAD_CTL_PAD_QSPI1A_SS1_B_DSE(6)   | \
                                                 IOMUXC_SW_PAD_CTL_PAD_QSPI1A_SS1_B_SRE_MASK | \
                                                 IOMUXC_SW_PAD_CTL_PAD_QSPI1A_SS1_B_HYS_MASK;
            IOMUXC_SW_PAD_CTL_PAD_QSPI1A_DQS = IOMUXC_SW_PAD_CTL_PAD_QSPI1A_DQS_SPEED(2) | \
                                               IOMUXC_SW_PAD_CTL_PAD_QSPI1A_DQS_DSE(6)   | \
                                               IOMUXC_SW_PAD_CTL_PAD_QSPI1A_DQS_SRE_MASK | \
                                               IOMUXC_SW_PAD_CTL_PAD_QSPI1A_DQS_HYS_MASK;
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_SS1_B = IOMUXC_SW_PAD_CTL_PAD_QSPI1B_SS1_B_SPEED(2) | \
                                                 IOMUXC_SW_PAD_CTL_PAD_QSPI1B_SS1_B_DSE(6)   | \
                                                 IOMUXC_SW_PAD_CTL_PAD_QSPI1B_SS1_B_SRE_MASK | \
                                                 IOMUXC_SW_PAD_CTL_PAD_QSPI1B_SS1_B_HYS_MASK;
            IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DQS = IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DQS_SPEED(2) | \
                                               IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DQS_DSE(6)   | \
                                               IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DQS_SRE_MASK | \
                                               IOMUXC_SW_PAD_CTL_PAD_QSPI1B_DQS_HYS_MASK;

            status = MQX_OK;
            break;
    }
    return status;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _bsp_i2c_io_init
* Returned Value   : MQX_OK or -1
* Comments         :
*    This function performs BSP-specific initialization related to I2C
*
*END*----------------------------------------------------------------------*/

_mqx_int _bsp_i2c_io_init
(
    /* [IN] i2c device number */
    _mqx_uint dev_num,
    /* [IN] Required functionality */
    _mqx_uint flags
)
{
    switch (dev_num)
    {
        case 1:
            if (flags & IO_PERIPHERAL_PIN_MUX_ENABLE)
            {
                /* SCL & SDA */
                IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO00 = IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO00_MUX_MODE(0) | IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO00_SION_MASK;
                IOMUXC_I2C1_IPP_SCL_IN_SELECT_INPUT = 1;
                IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO01 = IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO01_MUX_MODE(0) | IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO01_SION_MASK;
                IOMUXC_I2C1_IPP_SDA_IN_SELECT_INPUT = 1;
                IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO00 = IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO00_PKE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO00_PUE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO00_PUS(2)    | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO00_SPEED(2)  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO00_DSE(6)    | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO00_ODE_SHIFT | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO00_SRE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO00_HYS_MASK;
                IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO01 = IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO01_PKE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO01_PUE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO01_PUS(2)    | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO01_SPEED(2)  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO01_DSE(6)    | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO01_ODE_SHIFT | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO01_SRE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO01_HYS_MASK;
            }
            break;
        case 2:
            if (flags & IO_PERIPHERAL_PIN_MUX_ENABLE)
            {
                /* SCL & SDA */
                IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO02 = IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO02_MUX_MODE(0) | IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO02_SION_MASK;
                IOMUXC_I2C2_IPP_SCL_IN_SELECT_INPUT = 1;
                IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO03 = IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO03_MUX_MODE(0) | IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO03_SION_MASK;
                IOMUXC_I2C2_IPP_SDA_IN_SELECT_INPUT = 1;
                IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO02 = IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO02_PKE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO02_PUE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO02_PUS(2)    | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO02_SPEED(2)  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO02_DSE(6)    | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO02_ODE_SHIFT | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO02_SRE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO02_HYS_MASK;
                IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO03 = IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO03_PKE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO03_PUE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO03_PUS(2)    | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO03_SPEED(2)  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO03_DSE(6)    | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO03_ODE_SHIFT | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO03_SRE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO03_HYS_MASK;
            }
            break;
        case 3:
            if (flags & IO_PERIPHERAL_PIN_MUX_ENABLE)
            {
                /* SCL & SDA */
                IOMUXC_SW_MUX_CTL_PAD_KEY_COL4 = IOMUXC_SW_MUX_CTL_PAD_KEY_COL4_MUX_MODE(2) | IOMUXC_SW_MUX_CTL_PAD_KEY_COL4_SION_MASK;
                IOMUXC_I2C3_IPP_SCL_IN_SELECT_INPUT = 2;
                IOMUXC_SW_MUX_CTL_PAD_KEY_ROW4 = IOMUXC_SW_MUX_CTL_PAD_KEY_ROW4_MUX_MODE(2) | IOMUXC_SW_MUX_CTL_PAD_KEY_ROW4_SION_MASK;
                IOMUXC_I2C3_IPP_SDA_IN_SELECT_INPUT = 2;
                IOMUXC_SW_PAD_CTL_PAD_KEY_COL4 = IOMUXC_SW_PAD_CTL_PAD_KEY_COL4_PKE_MASK  | \
                                                 IOMUXC_SW_PAD_CTL_PAD_KEY_COL4_PUE_MASK  | \
                                                 IOMUXC_SW_PAD_CTL_PAD_KEY_COL4_PUS(2)    | \
                                                 IOMUXC_SW_PAD_CTL_PAD_KEY_COL4_SPEED(2)  | \
                                                 IOMUXC_SW_PAD_CTL_PAD_KEY_COL4_DSE(6)    | \
                                                 IOMUXC_SW_PAD_CTL_PAD_KEY_COL4_ODE_SHIFT | \
                                                 IOMUXC_SW_PAD_CTL_PAD_KEY_COL4_SRE_MASK  | \
                                                 IOMUXC_SW_PAD_CTL_PAD_KEY_COL4_HYS_MASK;
                IOMUXC_SW_PAD_CTL_PAD_KEY_ROW4 = IOMUXC_SW_PAD_CTL_PAD_KEY_ROW4_PKE_MASK  | \
                                                 IOMUXC_SW_PAD_CTL_PAD_KEY_ROW4_PUE_MASK  | \
                                                 IOMUXC_SW_PAD_CTL_PAD_KEY_ROW4_PUS(2)    | \
                                                 IOMUXC_SW_PAD_CTL_PAD_KEY_ROW4_SPEED(2)  | \
                                                 IOMUXC_SW_PAD_CTL_PAD_KEY_ROW4_DSE(6)    | \
                                                 IOMUXC_SW_PAD_CTL_PAD_KEY_ROW4_ODE_SHIFT | \
                                                 IOMUXC_SW_PAD_CTL_PAD_KEY_ROW4_SRE_MASK  | \
                                                 IOMUXC_SW_PAD_CTL_PAD_KEY_ROW4_HYS_MASK;
            }
            break;
        case 4:
            if (flags & IO_PERIPHERAL_PIN_MUX_ENABLE)
            {
                /* SCL & SDA */
                IOMUXC_SW_MUX_CTL_PAD_CSI_DATA06 = IOMUXC_SW_MUX_CTL_PAD_CSI_DATA06_MUX_MODE(2) | IOMUXC_SW_MUX_CTL_PAD_CSI_DATA06_SION_MASK;
                IOMUXC_I2C4_IPP_SCL_IN_SELECT_INPUT = 2;
                IOMUXC_SW_MUX_CTL_PAD_CSI_DATA07 = IOMUXC_SW_MUX_CTL_PAD_CSI_DATA07_MUX_MODE(2) | IOMUXC_SW_MUX_CTL_PAD_CSI_DATA07_SION_MASK;
                IOMUXC_I2C4_IPP_SDA_IN_SELECT_INPUT = 2;
                IOMUXC_SW_PAD_CTL_PAD_CSI_DATA06 = IOMUXC_SW_PAD_CTL_PAD_CSI_DATA06_PKE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_CSI_DATA06_PUE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_CSI_DATA06_PUS(2)    | \
                                                   IOMUXC_SW_PAD_CTL_PAD_CSI_DATA06_SPEED(2)  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_CSI_DATA06_DSE(6)    | \
                                                   IOMUXC_SW_PAD_CTL_PAD_CSI_DATA06_ODE_SHIFT | \
                                                   IOMUXC_SW_PAD_CTL_PAD_CSI_DATA06_SRE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_CSI_DATA06_HYS_MASK;
                IOMUXC_SW_PAD_CTL_PAD_CSI_DATA07 = IOMUXC_SW_PAD_CTL_PAD_CSI_DATA07_PKE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_CSI_DATA07_PUE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_CSI_DATA07_PUS(2)    | \
                                                   IOMUXC_SW_PAD_CTL_PAD_CSI_DATA07_SPEED(2)  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_CSI_DATA07_DSE(6)    | \
                                                   IOMUXC_SW_PAD_CTL_PAD_CSI_DATA07_ODE_SHIFT | \
                                                   IOMUXC_SW_PAD_CTL_PAD_CSI_DATA07_SRE_MASK  | \
                                                   IOMUXC_SW_PAD_CTL_PAD_CSI_DATA07_HYS_MASK;
            }
            break;
        default:
            /* Do nothing if bad dev_num was selected */
            return IO_ERROR;
    }
    return MQX_OK;
}

/* EOF */
