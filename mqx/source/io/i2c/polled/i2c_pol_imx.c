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
*   This file contains low level functions for the I2C polled device driver
*   for I.MX family.
*
*
*END************************************************************************/


#include <mqx.h>
#include <bsp.h>
#include <io_prv.h>
#include <fio_prv.h>
#include "i2c.h"
#include "i2c_pol_prv.h"
#include "i2c_imx_prv.h"

extern uint32_t _imx_i2c_polled_init(const void *, void **, char *);
extern uint32_t _imx_i2c_polled_deinit(void *, void *);
extern uint32_t _imx_i2c_polled_rx(IO_I2C_POLLED_DEVICE_STRUCT_PTR, unsigned char *, int32_t);
extern uint32_t _imx_i2c_polled_tx(IO_I2C_POLLED_DEVICE_STRUCT_PTR, unsigned char *, int32_t);
extern uint32_t _imx_i2c_polled_ioctl(void *, uint32_t, uint32_t *);
static uint16_t _imx_i2c_find_baudrate_divider(uint32_t clock, uint32_t baudrate);
static uint32_t _imx_i2c_polled_rx_tx(I2C_MemMapPtr, IMX_I2C_INFO_STRUCT_PTR, unsigned char *, uint32_t);


static const uint32_t I2C_CLK_DIV[][2] =
{
    {22, 0x20}, {24, 0x21}, {26, 0x22}, {28, 0x23},
    {30, 0x00}, {32, 0x24}, {36, 0x25}, {40, 0x26},
    {42, 0x03}, {44, 0x27}, {48, 0x28}, {52, 0x05},
    {56, 0x29}, {60, 0x06}, {64, 0x2A}, {72, 0x2B},
    {80, 0x2C}, {88, 0x09}, {96, 0x2D}, {104, 0x0A},
    {112, 0x2E}, {128, 0x2F}, {144, 0x0C}, {160, 0x30},
    {192, 0x31}, {224, 0x32}, {240, 0x0F}, {256, 0x33},
    {288, 0x10}, {320, 0x34}, {384, 0x35}, {448, 0x36},
    {480, 0x13}, {512, 0x37}, {576, 0x14}, {640, 0x38},
    {768, 0x39}, {896, 0x3A}, {960, 0x17}, {1024, 0x3B},
    {1152, 0x18}, {1280, 0x3C}, {1536, 0x3D}, {1792, 0x3E},
    {1920, 0x1B}, {2048, 0x3F}, {2304, 0x1C}, {2560, 0x1D},
    {3072, 0x1E}, {3840, 0x1F},
};

/*FUNCTION****************************************************************
*
* Function Name    : _imx_i2c_find_baudrate_divider
* Returned Value   : uint32_t divider register setting
* Comments         :
*    Find closest setting of divider register for given baudrate.
*
*END*********************************************************************/

uint16_t _imx_i2c_find_baudrate_divider
    (
        /* [IN] Module input clock in Hz */
        uint32_t clock,

        /* [IN] Desired baudrate in Hz */
        uint32_t baudrate
    )
{
    uint32_t div;
    uint8_t clk_div_index = 0;

    div = clock / baudrate;

    if (div < I2C_CLK_DIV[0][0])
    {
        clk_div_index = 0;
    }
    else if (div > I2C_CLK_DIV[sizeof(I2C_CLK_DIV)/sizeof(I2C_CLK_DIV[0]) - 1][0])
    {
        clk_div_index = sizeof(I2C_CLK_DIV)/sizeof(I2C_CLK_DIV[0]) - 1;
    }
    else
    {
        while (I2C_CLK_DIV[clk_div_index][0] < div)
            clk_div_index++;
    }

    return I2C_CLK_DIV[clk_div_index][1];
}


/*FUNCTION****************************************************************
*
* Function Name    : _imx_i2c_polled_install
* Returned Value   : uint32_t a task error code or MQX_OK
* Comments         :
*    Install an polled I2C device.
*
*END*********************************************************************/

uint32_t _imx_i2c_polled_install
    (
        /* [IN] A string that identifies the device for fopen */
        char *identifier,

        /* [IN] The I/O init data pointer */
        IMX_I2C_INIT_STRUCT_CPTR init_data_ptr
    )
{ /* Body */

    return _io_i2c_polled_install(identifier,
        (_mqx_uint (_CODE_PTR_)(void *, void **, char *))_imx_i2c_polled_init,
        (_mqx_uint (_CODE_PTR_)(void *, void *))_imx_i2c_polled_deinit,
        (_mqx_int (_CODE_PTR_)(void *, char *, int32_t))_imx_i2c_polled_rx,
        (_mqx_int (_CODE_PTR_)(void *, char *, int32_t))_imx_i2c_polled_tx,
        (_mqx_int (_CODE_PTR_)(void *, uint32_t, uint32_t *))_imx_i2c_polled_ioctl,
        (void *)init_data_ptr);

} /* Endbody */


/*FUNCTION****************************************************************
*
* Function Name    : _imx_i2c_polled_init
* Returned Value   : MQX error code
* Comments         :
*    This function initializes an I2C device.
*
*END*********************************************************************/

uint32_t _imx_i2c_polled_init
    (
        /* [IN] Initialization information for the device being opened */
        const void  *io_init_ptr,

        /* [OUT] Address to store device specific information */
        void       **io_info_ptr_ptr,

        /* [IN] The rest of the name of the device opened */
        char        *open_name_ptr
    )
{ /* Body */
    IMX_I2C_INIT_STRUCT_PTR i2c_init_ptr = (IMX_I2C_INIT_STRUCT_PTR)io_init_ptr;
    IMX_I2C_INFO_STRUCT_PTR i2c_info_ptr;
    I2C_MemMapPtr           i2c_ptr;

    /* Check channel */
    i2c_ptr = _bsp_get_i2c_base_address(i2c_init_ptr->CHANNEL);
    if (NULL == i2c_ptr)
    {
        return I2C_ERROR_CHANNEL_INVALID;
    }

    /* Enable peripheral clock & IOMUX */
    if (_bsp_i2c_io_init(i2c_init_ptr->CHANNEL,
                         IO_PERIPHERAL_CLOCK_ENABLE | IO_PERIPHERAL_PIN_MUX_ENABLE) == IO_ERROR)
    {
        return I2C_ERROR_CHANNEL_INVALID;
    }

    /* Initialize internal data */
    i2c_info_ptr = (IMX_I2C_INFO_STRUCT_PTR)_mem_alloc_system_zero((uint32_t)sizeof(IMX_I2C_INFO_STRUCT));
    if (NULL == i2c_info_ptr)
    {
        return MQX_OUT_OF_MEMORY;
    }
    _mem_set_type ((void *)i2c_info_ptr, MEM_TYPE_IO_I2C_INFO_STRUCT);

    *io_info_ptr_ptr      = (void *)i2c_info_ptr;
    i2c_info_ptr->INIT    = *i2c_init_ptr;
    i2c_info_ptr->I2C_PTR = i2c_ptr;
    i2c_info_ptr->MODE    = i2c_init_ptr->MODE;
    i2c_info_ptr->STATE   = I2C_STATE_READY;

    /* Disable and clear I2C before initializing it */
    i2c_ptr->I2CR = 0x00;

    /* Clear out all I2C events */
    i2c_ptr->I2SR = 0x00;

    /* Set the frequency divider for the nearest found baud rate */
    i2c_ptr->IFDR = (I2C_IFDR_IC_MASK & _imx_i2c_find_baudrate_divider(BSP_I2C_CLOCK, i2c_init_ptr->BAUD_RATE)) << I2C_IFDR_IC_SHIFT;

    /* Enable I2C */
    i2c_ptr->I2CR |= I2C_I2CR_IEN_MASK;

    /* Set the station address for SLAVE receive operations */
    i2c_ptr->IADR = (i2c_init_ptr->STATION_ADDRESS) << 1;

    return I2C_OK;
} /* Endbody */


/*FUNCTION****************************************************************
*
* Function Name    : _imx_i2c_polled_deinit
* Returned Value   : MQX error code
* Comments         :
*    This function de-initializes an I2C device.
*
*END*********************************************************************/

uint32_t _imx_i2c_polled_deinit
    (
        /* [IN] Initialization information for the device being opened */
        void * io_init_ptr,

        /* [IN] the address of the device specific information */
        void * io_info_ptr
    )
{ /* Body */
    IMX_I2C_INIT_STRUCT_PTR i2c_init_ptr = (IMX_I2C_INIT_STRUCT_PTR)io_init_ptr;
    IMX_I2C_INFO_STRUCT_PTR i2c_info_ptr = (IMX_I2C_INFO_STRUCT_PTR)io_info_ptr;
    I2C_MemMapPtr i2c_ptr;

    if ((NULL == i2c_init_ptr) || (NULL == i2c_info_ptr))
    {
        return I2C_ERROR_INVALID_PARAMETER;
    }

    i2c_ptr = i2c_info_ptr->I2C_PTR;
    if (i2c_ptr->I2SR & I2C_I2SR_IBB_MASK)
    {
        return I2C_ERROR_DEVICE_BUSY;
    }

    /* Disable the I2C */
    i2c_ptr->I2CR = 0x00;

    /* Clear the I2C events */
    i2c_ptr->I2SR = 0x00;

    /* Free info struct */
    _mem_free(i2c_info_ptr);
    io_info_ptr = NULL;

    /* Disable peripheral clock */
    _bsp_i2c_io_init(i2c_init_ptr->CHANNEL, IO_PERIPHERAL_CLOCK_DISABLE);

    return MQX_OK;
} /* Endbody */


/*FUNCTION****************************************************************
*
* Function Name    : _imx_i2c_polled_ioctl
* Returned Value   : MQX error code.
* Comments         :
*    This function performs miscellaneous services for
*    the I2C I/O device.
*
*END*********************************************************************/

uint32_t _imx_i2c_polled_ioctl
    (
        /* [IN] the address of the device specific information */
        void       *io_info_ptr,

        /* [IN] The command to perform */
        uint32_t    cmd,

        /* [IN] Parameters for the command */
        uint32_t   *param_ptr
    )
{ /* Body */
    IMX_I2C_INFO_STRUCT_PTR i2c_info_ptr = (IMX_I2C_INFO_STRUCT_PTR)io_info_ptr;
    I2C_MemMapPtr           i2c_ptr = i2c_info_ptr->I2C_PTR;
    uint32_t                result = MQX_OK;

    volatile uint16_t       tmp;
    uint32_t                div, index;

    switch (cmd)
    {
        case IO_IOCTL_FLUSH_OUTPUT:
            if ((I2C_OPERATION_STARTED == (i2c_info_ptr->OPERATION & (I2C_OPERATION_READ | I2C_OPERATION_STARTED))))
            {
                while (I2C_I2SR_IBB_MASK == ((tmp=i2c_ptr->I2SR) & (I2C_I2SR_IBB_MASK | I2C_I2SR_ICF_MASK)))
                {}
            }
            else
            {
                tmp=i2c_ptr->I2SR; /* initialize tmp in case we are not in write; tmp is used below */
            }

            if ((i2c_info_ptr->OPERATION & I2C_OPERATION_STARTED) && (0 == (tmp & I2C_I2SR_IBB_MASK)))
            {
                i2c_info_ptr->OPERATION = 0;
                i2c_info_ptr->RX_REQUEST = 0;
                i2c_info_ptr->STATE = I2C_STATE_FINISHED;
            }
            if (NULL != param_ptr)
            {
                *param_ptr = tmp & I2C_I2SR_RXAK_MASK;
            }
            break;
        case IO_IOCTL_I2C_REPEATED_START:
            result = I2C_ERROR_DEVICE_BUSY;
            if (i2c_ptr->I2CR & I2C_I2CR_MSTA_MASK)
            {
                if (I2C_STATE_TRANSMIT == i2c_info_ptr->STATE)
                {
                    i2c_info_ptr->STATE = I2C_STATE_REPEATED_START;
                    result = MQX_OK;
                }
            }
            break;
        case IO_IOCTL_I2C_STOP:
            i2c_ptr->I2CR &= (~(I2C_I2CR_MTX_MASK | I2C_I2CR_TXAK_MASK));
            i2c_ptr->I2SR &= (~(I2C_I2SR_IAL_MASK | I2C_I2SR_IIF_MASK));
            i2c_info_ptr->RX_REQUEST = 0;
            i2c_info_ptr->OPERATION = 0;
            i2c_info_ptr->STATE = I2C_STATE_READY;

            if (i2c_ptr->I2CR & I2C_I2CR_MSTA_MASK)
            {
                /* As master, send STOP condition */
                i2c_ptr->I2CR &= (~ I2C_I2CR_MSTA_MASK);
            }

            /* Wait for bus idle */
            tmp = i2c_ptr->I2SR;
            while (tmp & I2C_I2SR_IBB_MASK)
            {
                if (tmp & I2C_I2SR_IIF_MASK)
                {
                  i2c_ptr->I2SR &= (~I2C_I2SR_IIF_MASK);
                  tmp = (uint8_t)(i2c_ptr->I2DR & I2C_I2DR_DATA_MASK);
                }
                tmp = i2c_ptr->I2SR;
            }
            break;
        case IO_IOCTL_I2C_SET_RX_REQUEST:
            if (NULL == param_ptr)
            {
                result = I2C_ERROR_INVALID_PARAMETER;
            }
            break;
        case IO_IOCTL_I2C_DISABLE_DEVICE:
            i2c_ptr->I2CR &= (~ I2C_I2CR_IEN_MASK);
            break;
        case IO_IOCTL_I2C_ENABLE_DEVICE:
            i2c_ptr->I2CR |= I2C_I2CR_IEN_MASK;
            break;
        case IO_IOCTL_I2C_SET_MASTER_MODE:
            if (I2C_STATE_READY != i2c_info_ptr->STATE)
            {
                result = I2C_ERROR_DEVICE_BUSY;
            }
            else
            {
                i2c_info_ptr->MODE = I2C_MODE_MASTER;
            }
            break;
        case IO_IOCTL_I2C_SET_SLAVE_MODE:
            if (I2C_STATE_READY != i2c_info_ptr->STATE)
            {
                result = I2C_ERROR_DEVICE_BUSY;
            }
            else
            {
                i2c_info_ptr->MODE = I2C_MODE_SLAVE;
            }
            break;
        case IO_IOCTL_I2C_GET_MODE:
            if (NULL == param_ptr)
            {
                result = I2C_ERROR_INVALID_PARAMETER;
            }
            else
            {
                *param_ptr = i2c_info_ptr->MODE;
            }
            break;
        case IO_IOCTL_I2C_SET_BAUD:
            if (NULL == param_ptr)
            {
                result = I2C_ERROR_INVALID_PARAMETER;
            }
            else if (i2c_ptr->I2SR & I2C_I2SR_IBB_MASK)
            {
                result = I2C_ERROR_DEVICE_BUSY;
            }
            else
            {
                i2c_ptr->IFDR = (I2C_IFDR_IC_MASK & _imx_i2c_find_baudrate_divider(BSP_I2C_CLOCK, *param_ptr)) << I2C_IFDR_IC_SHIFT;
            }
            break;
        case IO_IOCTL_I2C_GET_BAUD:
            if (NULL == param_ptr)
            {
                result = I2C_ERROR_INVALID_PARAMETER;
            }
            else
            {
                tmp = (i2c_ptr->IFDR >> I2C_IFDR_IC_SHIFT) & I2C_IFDR_IC_MASK;
                for(index = 0; index < sizeof(I2C_CLK_DIV)/sizeof(I2C_CLK_DIV[0]); index++)
                {
                    if(I2C_CLK_DIV[index][1] == tmp)
                    {
                        break;
                    }
                }
                div = I2C_CLK_DIV[index][0];
                *param_ptr = (uint32_t)((BSP_I2C_CLOCK) / (div));
            }
            break;
        case IO_IOCTL_I2C_SET_DESTINATION_ADDRESS:
            if (NULL == param_ptr)
            {
                result = I2C_ERROR_INVALID_PARAMETER;
            }
            else
            {
                i2c_info_ptr->ADDRESSEE = *param_ptr;
            }
            break;
        case IO_IOCTL_I2C_GET_DESTINATION_ADDRESS:
            if (NULL == param_ptr)
            {
                result = I2C_ERROR_INVALID_PARAMETER;
            }
            else
            {
                *param_ptr = i2c_info_ptr->ADDRESSEE;
            }
            break;
        case IO_IOCTL_I2C_SET_STATION_ADDRESS:
            if (NULL == param_ptr)
            {
                result = I2C_ERROR_INVALID_PARAMETER;
            }
            else
            {
                i2c_ptr->IADR = I2C_IADR_ADR(*param_ptr);
            }
            break;
        case IO_IOCTL_I2C_GET_STATION_ADDRESS:
            if (NULL == param_ptr)
            {
                result = I2C_ERROR_INVALID_PARAMETER;
            }
            else
            {
                *param_ptr = ( i2c_ptr->IADR & I2C_IADR_ADR_MASK ) >> I2C_IADR_ADR_SHIFT;
            }
            break;
        case IO_IOCTL_I2C_GET_STATE:
            if (NULL == param_ptr)
            {
                result = I2C_ERROR_INVALID_PARAMETER;
            }
            else
            {
                *param_ptr = i2c_info_ptr->STATE;
            }
            break;
        case IO_IOCTL_I2C_GET_STATISTICS:
            if (NULL == param_ptr)
            {
               result = I2C_ERROR_INVALID_PARAMETER;
            }
            else
            {
               *((I2C_STATISTICS_STRUCT_PTR)param_ptr) = i2c_info_ptr->STATISTICS;
            }
            break;
        case IO_IOCTL_I2C_CLEAR_STATISTICS:
            i2c_info_ptr->STATISTICS.INTERRUPTS = 0;
            i2c_info_ptr->STATISTICS.RX_PACKETS = 0;
            i2c_info_ptr->STATISTICS.TX_PACKETS = 0;
            i2c_info_ptr->STATISTICS.TX_LOST_ARBITRATIONS = 0;
            i2c_info_ptr->STATISTICS.TX_ADDRESSED_AS_SLAVE = 0;
            i2c_info_ptr->STATISTICS.TX_NAKS = 0;
            break;
        case IO_IOCTL_I2C_GET_BUS_AVAILABILITY:
            if (NULL == param_ptr)
            {
                result = I2C_ERROR_INVALID_PARAMETER;
            }
            else
            {
                if (i2c_ptr->I2SR & I2C_I2SR_IBB_MASK)
                {
                    *param_ptr = I2C_BUS_BUSY;
                }
                else
                {
                    *param_ptr = I2C_BUS_IDLE;
                }
            }
            break;
        default:
            break;
   }
   return result;
} /* Endbody */


/*FUNCTION****************************************************************
*
* Function Name    : _imx_i2c_polled_rx
* Returned Value   : number of bytes read
* Comments         :
*   Returns the number of bytes received.
*   Reads the data into provided array when data is available.
*
*END*********************************************************************/

uint32_t _imx_i2c_polled_rx
    (
        /* [IN] the address of the device specific information */
        IO_I2C_POLLED_DEVICE_STRUCT_PTR pol_io_dev_ptr,

       /* [IN] The array to copy data into */
       unsigned char                   *buffer,

        /* [IN] number of bytes to read */
        int32_t                         length
    )
{ /* Body */
    IMX_I2C_INFO_STRUCT_PTR i2c_info_ptr;
    I2C_MemMapPtr           i2c_ptr;

    i2c_info_ptr = pol_io_dev_ptr->DEV_INFO_PTR;
    i2c_ptr = i2c_info_ptr->I2C_PTR;

    if (length == 0)
        return 0;

    /* If beginning of transmission, set state and send address (master only) */
    i2c_info_ptr->OPERATION |= I2C_OPERATION_READ;

    if (I2C_MODE_MASTER == i2c_info_ptr->MODE)
    {
        if ((I2C_STATE_READY == i2c_info_ptr->STATE) || (I2C_STATE_REPEATED_START == i2c_info_ptr->STATE))
        {
            i2c_ptr->I2CR |= I2C_I2CR_MTX_MASK;
            i2c_ptr->I2SR &= (~ I2C_I2SR_IIF_MASK);
            if (I2C_STATE_REPEATED_START == i2c_info_ptr->STATE)
            {
                i2c_ptr->I2CR |= I2C_I2CR_RSTA_MASK;
            }
            else
            {
                 i2c_ptr->I2CR |= I2C_I2CR_MSTA_MASK;
            }
            i2c_info_ptr->OPERATION |= I2C_OPERATION_STARTED;
            i2c_ptr->I2DR = (i2c_info_ptr->ADDRESSEE << 1) | I2C_OPERATION_READ;
            i2c_info_ptr->STATISTICS.TX_PACKETS++;

            /* Wait for IIF be set */
            while (0 == (i2c_ptr->I2SR & I2C_I2SR_IIF_MASK))
            {}

            /* Clear IIF */
            i2c_ptr->I2SR &= (~I2C_I2SR_IIF_MASK);
        }
        else
            return 0;
    }

    return _imx_i2c_polled_rx_tx(i2c_ptr, i2c_info_ptr, buffer, length);
} /* Endbody */


/*FUNCTION****************************************************************
*
* Function Name    : _imx_i2c_polled_tx
* Returned Value   : number of bytes transmitted
* Comments         :
*   Writes the provided data buffer and loops until transmission complete.
*
*END*********************************************************************/

uint32_t _imx_i2c_polled_tx
    (
        /* [IN] the address of the device specific information */
        IO_I2C_POLLED_DEVICE_STRUCT_PTR pol_io_dev_ptr,

        /* [IN] The array characters are to be read from */
        unsigned char                  *buffer,

        /* [IN] number of bytes to output */
        int32_t                         length
    )
{ /* Body */
    IMX_I2C_INFO_STRUCT_PTR i2c_info_ptr;
    I2C_MemMapPtr           i2c_ptr;

    i2c_info_ptr = pol_io_dev_ptr->DEV_INFO_PTR;
    i2c_ptr = i2c_info_ptr->I2C_PTR;

    /* If beginning of transmission, set state and send address (master only) */
    i2c_info_ptr->OPERATION &= (~ I2C_OPERATION_READ);

    if (I2C_MODE_MASTER == i2c_info_ptr->MODE)
    {
        /* Address cycle */
        if ((I2C_STATE_READY == i2c_info_ptr->STATE) || (I2C_STATE_REPEATED_START == i2c_info_ptr->STATE))
        {
            // set i2c work as tx mode
            i2c_ptr->I2CR |= I2C_I2CR_MTX_MASK;
            i2c_ptr->I2SR &= (~ I2C_I2SR_IIF_MASK);
            if (I2C_STATE_REPEATED_START == i2c_info_ptr->STATE)
            {
               // Send Repeat Start Signal
               i2c_ptr->I2CR |= I2C_I2CR_RSTA_MASK;
            }
            else
            {
                // Send Start Signal
                i2c_ptr->I2CR |= I2C_I2CR_MSTA_MASK;
            }
            i2c_info_ptr->OPERATION |= I2C_OPERATION_STARTED;
            i2c_ptr->I2DR = (i2c_info_ptr->ADDRESSEE << 1) | I2C_OPERATION_WRITE;
            i2c_info_ptr->STATISTICS.TX_PACKETS++;

            /* Wait for IIF be set */
            while (0 == (i2c_ptr->I2SR & I2C_I2SR_IIF_MASK))
            {}

            /* Clear IIF */
            i2c_ptr->I2SR &= (~I2C_I2SR_IIF_MASK);
        }
    }
    return _imx_i2c_polled_rx_tx(i2c_ptr, i2c_info_ptr, buffer, length);
} /* Endbody */


/*FUNCTION****************************************************************
*
* Function Name    : _imx_i2c_polled_rx_tx
* Returned Value   : number of bytes processed
* Comments         :
*   Actual data transfer on I2C bus.
*
*END*********************************************************************/

uint32_t _imx_i2c_polled_rx_tx
    (
        /* [IN] I2C register structure */
        I2C_MemMapPtr           i2c_ptr,

        /* [IN] I2C state structure */
        IMX_I2C_INFO_STRUCT_PTR i2c_info_ptr,

        /* [IN] The buffer for IO operation */
        unsigned char          *buffer,

        /* [IN] Number of bytes in buffer */
        uint32_t                length
    )
{ /* Body */
    uint32_t count = 0;
    uint16_t  tmp;

    while(1)
    {
        /* Master */
        if (i2c_ptr->I2CR & I2C_I2CR_MSTA_MASK)
        {
            /* Master Transmit Byte Finish*/
            if (i2c_ptr->I2CR & I2C_I2CR_MTX_MASK)
            {
                /* Byte Not Ack by Slave */
                if (i2c_ptr->I2SR & I2C_I2SR_RXAK_MASK)
                {
                    i2c_info_ptr->STATE = I2C_STATE_FINISHED;
                    i2c_info_ptr->STATISTICS.TX_NAKS++;
                    return count;
                }
                /* Byte Ack by Slave */
                else
                {
                    /* End of address cycle? */
                    if ((I2C_STATE_READY == i2c_info_ptr->STATE) || (I2C_STATE_REPEATED_START == i2c_info_ptr->STATE))
                    {
                        count = 0;

                        /* Transmit operation */
                        if (0 == (I2C_OPERATION_READ & i2c_info_ptr->OPERATION))
                        {
                            i2c_info_ptr->STATE = I2C_STATE_TRANSMIT;
                            if (length != 0)
                            {
                                /*Transmit first byte*/
                                i2c_info_ptr->STATISTICS.TX_PACKETS++;
                                /* transmit first data */
                                i2c_ptr->I2DR = *buffer++;
                                count++;
                            }
                            else
                                return 0;
                        }
                        /* Receive operation */
                        else
                        {
                            /* Change to receive state */
                            i2c_info_ptr->STATE = I2C_STATE_RECEIVE;
                            i2c_ptr->I2CR &= (~ I2C_I2CR_MTX_MASK);

                            if (1 == length)
                            {
                                /* Send Nack */
                                i2c_ptr->I2CR |= I2C_I2CR_TXAK_MASK;
                            }
                            else
                            {
                                /* Send ack */
                                i2c_ptr->I2CR &= (~ I2C_I2CR_TXAK_MASK);
                            }
                            /* dummy read to clock in 1st byte */
                            tmp = i2c_ptr->I2DR;
                        }
                    }
                    /* Normal i2c transmit */
                    else
                    {
                        /* Anything more to transmit? */
                        if (count < length)
                        {
                            i2c_info_ptr->STATISTICS.TX_PACKETS++;
                            /* transmit data */
                            i2c_ptr->I2DR = *buffer++;
                            count++;
                        }
                        else
                        {
                            /* Transmit finished */
                            return count;
                        }
                    }
                }
            }
            /* Master Receive Byte Finish */
            else
            {
                /* 2nd last byte to read */
                if (2 == (length - count))
                {
                    /* Send Nack */
                    i2c_ptr->I2CR |= I2C_I2CR_TXAK_MASK;
                }
                else
                {
                    /* Send ack */
                    i2c_ptr->I2CR &= (~ I2C_I2CR_TXAK_MASK);
                }

                if(1 == (length - count))
                {
                    i2c_ptr->I2CR |= I2C_I2CR_MTX_MASK;  /* no more reading */
                }

                tmp = i2c_ptr->I2DR;   /* receive data */
                *buffer++ = (tmp & I2C_I2DR_DATA_MASK);
                i2c_info_ptr->STATISTICS.RX_PACKETS++;
                count++;
                if(0 == (length - count))
                {
                    return count;
                }
            }
        }
        /* Slave */
        else
        {
            /* Master arbitration lost */
            if (i2c_ptr->I2SR & I2C_I2SR_IAL_MASK)
            {
                i2c_ptr->I2SR &= (~ I2C_I2SR_IAL_MASK);
                i2c_info_ptr->STATE = I2C_STATE_LOST_ARBITRATION;
                i2c_info_ptr->STATISTICS.TX_LOST_ARBITRATIONS++;
            }

            /* Addressed as slave */
            if ((I2C_MODE_MASTER == i2c_info_ptr->MODE) && (i2c_ptr->I2SR & I2C_I2SR_IAAS_MASK))
            {
                i2c_info_ptr->STATISTICS.TX_ADDRESSED_AS_SLAVE++;
            }

            /* Master arbitration lost */
            return count;
        }
        while (0 == (i2c_ptr->I2SR & I2C_I2SR_IIF_MASK))
        {}
        /* Clear IIF */
        i2c_ptr->I2SR &= (~I2C_I2SR_IIF_MASK);
    }
} /* Endbody */
