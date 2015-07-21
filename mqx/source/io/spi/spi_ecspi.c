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
*   The file contains low level SPI driver functions for ECSPI module
*
*
*END************************************************************************/


#include <mqx.h>
#include <bsp.h>
#include <io_prv.h>

#include "spi.h"
#include "spi_prv.h"

#include "spi_ecspi.h"
#include "spi_ecspi_prv.h"

/* SPI low level driver interface functions */
static _mqx_int _ecspi_init(const void  *init_data_ptr, void **io_info_ptr_ptr);
static _mqx_int _ecspi_deinit(void *io_info_ptr);
static _mqx_int _ecspi_setparam(void *io_info_ptr, SPI_PARAM_STRUCT_PTR params);
static _mqx_int _ecspi_tx_rx(void *io_info_ptr, uint8_t *txbuf, uint8_t *rxbuf, uint32_t len);
static _mqx_int _ecspi_cs_deassert(void *io_info_ptr);
static _mqx_int _ecspi_ioctl(void *io_info_ptr, SPI_PARAM_STRUCT_PTR params, uint32_t cmd, uint32_t *param_ptr);

const SPI_DEVIF_STRUCT _spi_ecspi_devif = {
    _ecspi_init,
    _ecspi_deinit,
    _ecspi_setparam,
    _ecspi_tx_rx,
    _ecspi_cs_deassert,
    _ecspi_ioctl
};


/* Forward declarations */
static void _ecspi_isr(void *parameter);

static void _ecspi_init_low(ECSPI_MemMapPtr ecspi_ptr)
{
    /* reset internal logic */
    ecspi_ptr->CONREG = 0;
    ecspi_ptr->CONREG = 1;
}

static void _ecspi_deinit_low(ECSPI_MemMapPtr ecspi_ptr)
{
    /* disable ecspi */
    ecspi_ptr->CONREG = 0;
}

static uint32_t _ecspi_get_closest_divider(uint32_t clock_speed, uint32_t baud, uint32_t div)
{
    if ((clock_speed - baud * div) < (baud * (div + 1) - clock_speed))
        return div - 1; /* pre_divider value is one less than the real divider */
    return div;
}

static uint32_t _ecspi_find_baudrate(uint32_t clock_speed, uint32_t baud, ECSPI_INFO_STRUCT_PTR info)
{
    uint32_t div, pre_div;
    uint32_t post_baud; /* baud rate after post divider */
    uint32_t pre_baud; /* baud rate before pre divider */

    if (clock_speed <= baud) {
        if (info) {
            info->PRE_DIV = 0;
            info->POST_DIV = 0;
        }
        return clock_speed;
    }

    div = clock_speed / baud;
    if (div < 16) /* pre_divider is enough */
    {
        pre_div = _ecspi_get_closest_divider(clock_speed, baud, div);
        if (info) {
            info->PRE_DIV = pre_div;
            info->POST_DIV = 0;
        }
        return clock_speed / (pre_div + 1);
    }

    pre_baud = baud * 16;
    for (div = 1; div < 16; div++) {
        post_baud = clock_speed >> div;
        if (post_baud < pre_baud)
            break;
    }

    if (div == 16) { /* divider is not enough, set the biggest ones */
        if (info) {
            info->PRE_DIV = 15;
            info->POST_DIV = 15;
        }
        return post_baud / 16;
    }

    /* find the closest one */
    pre_div = _ecspi_get_closest_divider(post_baud, baud, post_baud / baud);
    if (info) {
        info->PRE_DIV = pre_div;
        info->POST_DIV = div;
    }

    return post_baud / (pre_div + 1);
}

static void _ecspi_rx(ECSPI_INFO_STRUCT_PTR info, ECSPI_MemMapPtr ecspi_ptr)
{
    uint32_t data;
    uint32_t bytes;
    uint32_t i;

    while (info->TX_BYTES > 0 && (ecspi_ptr->STATREG & ECSPI_STATREG_RR_MASK)) {
        data = ecspi_ptr->RXDATA;

        bytes = info->TX_BYTES & 0x3; /* first get unaligned part received */
        bytes = bytes ? bytes : 4; /* if aligned, then must be 4 */

        if (info->RX_BUF) /* not half-duplex tx */
        {
            for (i = bytes; i > 0; i--)
            {
                *(info->RX_BUF+i-1) = data & 0xFF;
                data >>= 8;
            }
            info->RX_BUF += bytes;
        }

        info->TX_BYTES -= bytes;
    }
}

static void _ecspi_tx(ECSPI_INFO_STRUCT_PTR info, ECSPI_MemMapPtr ecspi_ptr)
{
    uint32_t data;
    uint32_t bytes;
    uint32_t i;

    while (info->BYTES_LEFT > 0 && ((ecspi_ptr->STATREG & ECSPI_STATREG_TF_MASK) == 0)) {
        bytes = info->BYTES_LEFT & 0x3; /* first get unaligned part trasmitted */
        bytes = bytes ? bytes : 4; /* if aligned, then must be 4 */

        if (!info->TX_BUF) { /* half-duplex rx */
            data = info->DUMMY_PATTERN;
        }
        else
        {
            data = 0;
            for (i = 0; i < bytes; i++)
                data = (data << 8) | *info->TX_BUF++;
        }

        info->BYTES_LEFT -= bytes;
        info->TX_BYTES += bytes;

        ecspi_ptr->TXDATA = data;
    }

    /* start transmission */
    ecspi_ptr->CONREG |= (1 << ECSPI_CONREG_XCH_SHIFT);
}

static uint32_t _ecspi_tx_rx_burst(ECSPI_INFO_STRUCT_PTR info, ECSPI_MemMapPtr ecspi_ptr,
        uint8_t *tx, uint8_t *rx, uint32_t tx_rx_bytes)
{
    uint32_t reg;

    /* update frame bits to real size */
    reg = ecspi_ptr->CONREG & ~ECSPI_CONREG_BURST_LENGTH_MASK;
    reg |= ECSPI_CONREG_BURST_LENGTH(tx_rx_bytes * 8 - 1);
    ecspi_ptr->CONREG = reg;

    info->RX_BUF = rx;
    info->TX_BUF = tx;
    info->BYTES_LEFT = tx_rx_bytes;
    info->TX_BYTES = 0;

    /* fill the fifo */
    _ecspi_tx(info, ecspi_ptr);
    /* enable tx empty interrupt */
    ecspi_ptr->INTREG = ECSPI_INTREG_TEEN_MASK;

    /* wait burst done */
    _lwsem_wait(&info->EVENT_IO_FINISHED);

    return tx_rx_bytes;
}

/*FUNCTION****************************************************************
*
* Function Name    : _ecspi_init
* Returned Value   : MQX error code
* Comments         :
*    This function initializes the SPI driver
*
*END*********************************************************************/
static _mqx_int _ecspi_init
    (
        /* [IN] The initialization information for the device being opened */
        const void                *init_data_ptr,

        /* [OUT] The address to store device specific information */
        void                      **io_info_ptr_ptr
    )
{
    ECSPI_INIT_STRUCT_PTR ecspi_init_ptr = (ECSPI_INIT_STRUCT_PTR)init_data_ptr;
    ECSPI_INFO_STRUCT_PTR ecspi_info_ptr;
    ECSPI_MemMapPtr       ecspi_ptr;
    CLOCK_NAME            clk_name;
    /* Check channel */
    ecspi_ptr = _bsp_get_ecspi_base_address (ecspi_init_ptr->CHANNEL);
    if (NULL == ecspi_ptr)
    {
        return SPI_ERROR_CHANNEL_INVALID;
    }

    clk_name = _bsp_get_ecspi_clock_name(ecspi_init_ptr->CHANNEL);
    if  (clk_name == CLK_MAX)
    {
        return SPI_ERROR_CHANNEL_INVALID;
    }

    /* Initialize internal data */
    ecspi_info_ptr = (ECSPI_INFO_STRUCT_PTR)_mem_alloc_system_zero((uint32_t)sizeof(ECSPI_INFO_STRUCT));
    if (ecspi_info_ptr == NULL)
    {
        return MQX_OUT_OF_MEMORY;
    }
    _mem_set_type(ecspi_info_ptr, MEM_TYPE_IO_SPI_INFO_STRUCT);

    ecspi_info_ptr->ECSPI_PTR = ecspi_ptr;
    ecspi_info_ptr->ECSPI_CLOCK = clock_get(clk_name);

    if (_bsp_ecspi_io_init (ecspi_init_ptr->CHANNEL, IO_PERIPHERAL_PIN_MUX_ENABLE) == IO_ERROR)
    {
        _mem_free(ecspi_info_ptr);
        return SPI_ERROR_CHANNEL_INVALID;
    }

    clock_enable(ecspi_info_ptr->ECSPI_CLOCK);

    _ecspi_init_low(ecspi_info_ptr->ECSPI_PTR);

    _lwsem_create(&ecspi_info_ptr->EVENT_IO_FINISHED, 0);

    /* Install ISRs */
    ecspi_info_ptr->VECTOR = _bsp_get_ecspi_vector(ecspi_init_ptr->CHANNEL);
    _int_install_isr(ecspi_info_ptr->VECTOR, _ecspi_isr, ecspi_info_ptr);
    _bsp_int_init(ecspi_info_ptr->VECTOR, BSP_ECSPI_INT_LEVEL, 0, TRUE);

    clock_disable(ecspi_info_ptr->ECSPI_CLOCK);

    *io_info_ptr_ptr = (void *)ecspi_info_ptr;

    return SPI_OK;
}


/*FUNCTION****************************************************************
*
* Function Name    : _ecspi_deinit
* Returned Value   : MQX error code
* Comments         :
*    This function de-initializes the SPI module
*
*END*********************************************************************/
static _mqx_int _ecspi_deinit
    (
        /* [IN] the address of the device specific information */
        void                          *io_info_ptr
    )
{
    ECSPI_INFO_STRUCT_PTR ecspi_info_ptr = (ECSPI_INFO_STRUCT_PTR)io_info_ptr;

    if (NULL == ecspi_info_ptr)
    {
        return SPI_ERROR_DEINIT_FAILED;
    }

    clock_enable(ecspi_info_ptr->ECSPI_CLOCK);

    _ecspi_deinit_low(ecspi_info_ptr->ECSPI_PTR);

    /* Uninstall interrupt service routines */
    /* Disable interrupt on vector */
    _bsp_int_disable(ecspi_info_ptr->VECTOR);
    /* Install default isr routine */
    _int_install_isr(ecspi_info_ptr->VECTOR, _int_get_default_isr(), NULL);

    _lwsem_destroy(&ecspi_info_ptr->EVENT_IO_FINISHED);

    clock_disable(ecspi_info_ptr->ECSPI_CLOCK);

    _mem_free(ecspi_info_ptr);

    return SPI_OK;
}


/*FUNCTION****************************************************************
*
* Function Name    : _ecspi_setparam
* Returned Value   :
* Comments         :
*    Set parameters for following transfers.
*
*END*********************************************************************/
static _mqx_int _ecspi_setparam
   (
        /* [IN] Device specific context structure */
        void                          *io_info_ptr,

        /* [IN] Parameters to set */
        SPI_PARAM_STRUCT_PTR           params
   )
{
    ECSPI_INFO_STRUCT_PTR   ecspi_info_ptr = (ECSPI_INFO_STRUCT_PTR)io_info_ptr;
    ECSPI_MemMapPtr         ecspi_ptr = ecspi_info_ptr->ECSPI_PTR;
    BSP_CLOCK_CONFIGURATION clock_config;
    uint32_t reg;
    uint32_t clock_speed;
    uint32_t cs = params->CS;

    /* Transfer mode */
    if ((params->ATTR & SPI_ATTR_TRANSFER_MODE_MASK) != SPI_ATTR_MASTER_MODE)
        return SPI_ERROR_TRANSFER_MODE_INVALID;

    if (params->FRAMESIZE < 1 || params->FRAMESIZE > 0x1000)
        return SPI_ERROR_FRAMESIZE_INVALID;

    if (cs > 3)
        return SPI_ERROR_CHANNEL_INVALID;

    clock_enable(ecspi_info_ptr->ECSPI_CLOCK);
    reg = ecspi_ptr->CONREG & ~(
             ECSPI_CONREG_PRE_DIVIDER_MASK | ECSPI_CONREG_POST_DIVIDER_MASK |
             ECSPI_CONREG_CHANNEL_SELECT_MASK | ECSPI_CONREG_BURST_LENGTH_MASK);

    /* Set master mode */
    reg |= (1 << (ECSPI_CONREG_CHANNEL_MODE_SHIFT + cs));

    clock_config = _bsp_get_clock_configuration();
    /* Check the parameter against most recent values to avoid time consuming baudrate finding routine */
    if ((ecspi_info_ptr->CLOCK_CONFIG != clock_config) ||
        (ecspi_info_ptr->BAUDRATE != params->BAUDRATE))
    {
        ecspi_info_ptr->CLOCK_CONFIG = clock_config;
        ecspi_info_ptr->BAUDRATE = params->BAUDRATE;

        /* Find configuration of prescalers best matching the desired value */
        clock_speed = clock_get_freq(ecspi_info_ptr->ECSPI_CLOCK);
        _ecspi_find_baudrate(clock_speed, ecspi_info_ptr->BAUDRATE, ecspi_info_ptr);
    }

    /* Set up dividers */
    reg |= ECSPI_CONREG_PRE_DIVIDER(ecspi_info_ptr->PRE_DIV) |
           ECSPI_CONREG_POST_DIVIDER(ecspi_info_ptr->POST_DIV);

    /* Channel select */
    reg |= ECSPI_CONREG_CHANNEL_SELECT(cs);

    /* Write to the hardware */
    ecspi_ptr->CONREG = reg;

    reg = ecspi_ptr->CONFIGREG;

    /* use default active low SS */
    reg &= ~(1 << (ECSPI_CONFIGREG_SS_POL_SHIFT + cs));

    if (params->MODE & SPI_CPOL_MASK)
        reg |= (1 << (ECSPI_CONFIGREG_SCLK_POL_SHIFT + cs));
    else
        reg &= ~(1 << (ECSPI_CONFIGREG_SCLK_POL_SHIFT + cs));

    if (params->MODE & SPI_CPHA_MASK)
        reg |= (1 << (ECSPI_CONFIGREG_SCLK_PHA_SHIFT + cs));
    else
        reg &= ~(1 << (ECSPI_CONFIGREG_SCLK_PHA_SHIFT + cs));

    ecspi_ptr->CONFIGREG = reg;

    ecspi_info_ptr->FRAMESIZE = params->FRAMESIZE;
    ecspi_info_ptr->DUMMY_PATTERN = params->DUMMY_PATTERN;
    ecspi_info_ptr->ATTR = params->ATTR;

    ecspi_ptr->INTREG = 0; /* disable interrupts */

    clock_disable(ecspi_info_ptr->ECSPI_CLOCK);

    return SPI_OK;
}


/*FUNCTION****************************************************************
*
* Function Name    : _ecspi_isr
* Returned Value   : SPI interrupt routine
* Comments         :
*   State machine transferring data between buffers and ECSPI FIFO.
*
*END*********************************************************************/
static void _ecspi_isr
    (
        /* [IN] The address of the device specific information */
        void                     *parameter
    )
{
    ECSPI_INFO_STRUCT_PTR         ecspi_info_ptr = parameter;
    ECSPI_MemMapPtr               ecspi_ptr = ecspi_info_ptr->ECSPI_PTR;

    _ecspi_rx(ecspi_info_ptr, ecspi_ptr);

    if (ecspi_info_ptr->BYTES_LEFT) {
        _ecspi_tx(ecspi_info_ptr, ecspi_ptr);
        return;
    }

    if (ecspi_info_ptr->TX_BYTES) {
        /* No data left to push, but still waiting for rx data,
         * enable receive data available interrupt.
         */
        ecspi_ptr->INTREG = ECSPI_INTREG_RREN_MASK;
        return;
    }

    ecspi_ptr->INTREG = 0;
    /* clear interrupt status */
    ecspi_ptr->STATREG = ECSPI_STATREG_RO_MASK | ECSPI_STATREG_TC_MASK;

    _lwsem_post(&ecspi_info_ptr->EVENT_IO_FINISHED);
}

/*FUNCTION****************************************************************
*
* Function Name    : _ecspi_tx_rx
* Returned Value   : number of bytes transferred
* Comments         :
*   Actual transmit and receive function.
*   Overrun prevention used, no need to update statistics in this function
*
*END*********************************************************************/
static _mqx_int _ecspi_tx_rx
    (
        /* [IN] Device specific context structure */
        void                        *io_info_ptr,

        /* [IN] Data to transmit */
        uint8_t                     *txbuf,

        /* [OUT] Received data */
        uint8_t                     *rxbuf,

        /* [IN] Length of transfer in bytes */
        uint32_t                     len
    )
{
    ECSPI_INFO_STRUCT_PTR ecspi_info_ptr = (ECSPI_INFO_STRUCT_PTR)io_info_ptr;
    ECSPI_MemMapPtr       ecspi_ptr = ecspi_info_ptr->ECSPI_PTR;
    uint32_t burst_bytes;
    uint32_t tx_rx_bytes;
    uint8_t *tx, *rx;
    uint32_t bytes = len;

    clock_enable(ecspi_info_ptr->ECSPI_CLOCK);

    burst_bytes = (ecspi_info_ptr->FRAMESIZE + 7) / 8;

    tx = txbuf;
    rx = rxbuf;
    while (bytes > 0)
    {
        tx_rx_bytes = bytes >= burst_bytes ? burst_bytes : bytes;

        if (_ecspi_tx_rx_burst(ecspi_info_ptr, ecspi_ptr, tx, rx, tx_rx_bytes) != tx_rx_bytes)
            break; /* something wrong */

        bytes -= tx_rx_bytes;
        if (rx)
            rx += tx_rx_bytes;
        if (tx)
            tx += tx_rx_bytes;
    }

    clock_disable(ecspi_info_ptr->ECSPI_CLOCK);

    return len - bytes;
}


/*FUNCTION****************************************************************
*
* Function Name    : _ecspi_cs_deassert
* Returned Value   :
* Comments         :
*   Deactivates chip select signals.
*
*END*********************************************************************/
static _mqx_int _ecspi_cs_deassert
    (
        /* [IN] The address of the device registers */
        void                     *io_info_ptr
    )
{
    /* nothing to do */
    return MQX_OK;
}


/*FUNCTION****************************************************************
*
* Function Name    : _ecspi_ioctl
* Returned Value   : MQX error code
* Comments         :
*    This function performs miscellaneous services for
*    the SPI I/O device.
*
*END*********************************************************************/
static _mqx_int _ecspi_ioctl
    (
        /* [IN] The address of the device specific information */
        void                         *io_info_ptr,

        /* [IN] SPI transfer parameters */
        SPI_PARAM_STRUCT_PTR          params,

        /* [IN] The command to perform */
        uint32_t                      cmd,

        /* [IN] Parameters for the command */
        uint32_t                     *param_ptr
    )
{
    ECSPI_INFO_STRUCT_PTR             ecspi_info_ptr = (ECSPI_INFO_STRUCT_PTR)io_info_ptr;
    uint32_t                          result = SPI_OK;

    uint32_t clock_speed;

    clock_enable(ecspi_info_ptr->ECSPI_CLOCK);

    switch (cmd)
    {
        case IO_IOCTL_SPI_GET_BAUD:
            clock_speed = clock_get_freq(ecspi_info_ptr->ECSPI_CLOCK);
            *((uint32_t *)param_ptr) = _ecspi_find_baudrate(clock_speed, *((uint32_t *)param_ptr), NULL);
            break;

        default:
            result = IO_ERROR_INVALID_IOCTL_CMD;
            break;
    }

    clock_disable(ecspi_info_ptr->ECSPI_CLOCK);

    return result;
}

/* EOF */
