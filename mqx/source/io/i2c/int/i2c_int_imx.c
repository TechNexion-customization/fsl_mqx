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
*   This file contains low level functions for the I2C interrupt device driver
*   for I.MX family.
*
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>
#include <io_prv.h>
#include <fio_prv.h>
#include "i2c.h"
#include "i2c_int_prv.h"
#include "i2c_imx_prv.h"

extern uint32_t _imx_i2c_int_init(IO_I2C_INT_DEVICE_STRUCT_PTR, char *);
extern uint32_t _imx_i2c_int_deinit(IO_I2C_INT_DEVICE_STRUCT_PTR, IMX_I2C_INFO_STRUCT_PTR);
extern uint32_t _imx_i2c_int_rx(IO_I2C_INT_DEVICE_STRUCT_PTR, unsigned char *, uint32_t);
extern uint32_t _imx_i2c_int_tx(IO_I2C_INT_DEVICE_STRUCT_PTR, unsigned char *, uint32_t);
extern uint32_t _imx_i2c_int_ioctl(void *, uint32_t, uint32_t *);
static void     _imx_i2c_isr(void *);

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

static uint16_t _imx_i2c_find_baudrate_divider
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
* Function Name    : _imx_i2c_int_install
* Returned Value   : MQX error code
* Comments         :
*    Install an I2C device.
*
*END*********************************************************************/

uint32_t _imx_i2c_int_install
    (
        /* [IN] A string that identifies the device for fopen */
        char *identifier,

        /* [IN] The I/O init data pointer */
        IMX_I2C_INIT_STRUCT_CPTR init_data_ptr
    )
{ /* Body */

    return _io_i2c_int_install(identifier,
        (_mqx_uint (_CODE_PTR_)(void *, char *))_imx_i2c_int_init,
        (_mqx_uint (_CODE_PTR_)(void *, void *))_imx_i2c_int_deinit,
        (_mqx_int  (_CODE_PTR_)(void *, char *, _mqx_int))_imx_i2c_int_rx,
        (_mqx_int  (_CODE_PTR_)(void *, char *, _mqx_int))_imx_i2c_int_tx,
        (_mqx_int  (_CODE_PTR_)(void *, _mqx_uint, _mqx_uint_ptr))_imx_i2c_int_ioctl,
        (void *)init_data_ptr);

} /* Endbody */


/*FUNCTION****************************************************************
*
* Function Name    : _imx_i2c_int_init
* Returned Value   : MQX error code
* Comments         :
*    This function initializes an I2C device.
*
*END*********************************************************************/

uint32_t _imx_i2c_int_init
    (
        /* [IN] Initialization information for the device being opened */
        IO_I2C_INT_DEVICE_STRUCT_PTR int_io_dev_ptr,

        /* [IN] The rest of the name of the device opened */
        char                        *open_name_ptr
    )
{ /* Body */
    I2C_MemMapPtr            i2c_ptr;
    IMX_I2C_INFO_STRUCT_PTR  i2c_info_ptr;
    IMX_I2C_INIT_STRUCT_PTR  i2c_init_ptr;
    uint32_t                 vector;
    uint32_t                 clock;
    CLOCK_NAME               clock_name;

    i2c_init_ptr = (IMX_I2C_INIT_STRUCT_PTR)(int_io_dev_ptr->DEV_INIT_DATA_PTR);

    i2c_ptr      = _bsp_get_i2c_base_address(i2c_init_ptr->CHANNEL);
    if (i2c_ptr == NULL)
    {
       return I2C_ERROR_CHANNEL_INVALID;
    }

    clock_name   = _bsp_get_i2c_clock_name(i2c_init_ptr->CHANNEL);
    if (clock_name == CLK_MAX) {
       /* clock name equals to CLK_MAX if no clock name is found */
       return I2C_ERROR_CHANNEL_INVALID;
    }

    /* Initialize internal data */
    i2c_info_ptr = (IMX_I2C_INFO_STRUCT_PTR)_mem_alloc_system_zero((uint32_t)sizeof(IMX_I2C_INFO_STRUCT));
    if (NULL == i2c_info_ptr)
    {
        return MQX_OUT_OF_MEMORY;
    }
    _mem_set_type ((void *)i2c_info_ptr, MEM_TYPE_IO_I2C_INFO_STRUCT);

    i2c_info_ptr->INIT    = *i2c_init_ptr;
    i2c_info_ptr->I2C_PTR = i2c_ptr;
    i2c_info_ptr->MODE    = i2c_init_ptr->MODE;
    i2c_info_ptr->STATE   = I2C_STATE_READY;

    i2c_info_ptr->CLOCK_I2C      = clock_get(clock_name);

    /* Enable peripheral IOMUX */
    if (_bsp_i2c_io_init(i2c_init_ptr->CHANNEL,
                         IO_PERIPHERAL_PIN_MUX_ENABLE) == IO_ERROR)
    {
        _mem_free(i2c_info_ptr);
        return I2C_ERROR_CHANNEL_INVALID;
    }

    clock_enable(i2c_info_ptr->CLOCK_I2C);

    /* Disable and clear I2C before initializing it */
    i2c_ptr->I2CR = 0x00;

    /* Clear out all I2C events */
    i2c_ptr->I2SR = 0x00;

    /* Set the frequency divider for the nearest found baud rate */
    clock = clock_get_freq(i2c_info_ptr->CLOCK_I2C);
    i2c_ptr->IFDR = (I2C_IFDR_IC_MASK & _imx_i2c_find_baudrate_divider(clock, i2c_init_ptr->BAUD_RATE)) << I2C_IFDR_IC_SHIFT;

    /* Enable I2C */
    i2c_ptr->I2CR |= I2C_I2CR_IEN_MASK;

    /* Set the station address for SLAVE receive operations */
    i2c_ptr->IADR = (i2c_init_ptr->STATION_ADDRESS) << 1;
    vector = _bsp_get_i2c_vector(i2c_init_ptr->CHANNEL);
    if (0 == vector)
    {
        clock_disable(i2c_info_ptr->CLOCK_I2C);
        _mem_free(i2c_info_ptr);
        return I2C_ERROR_CHANNEL_INVALID;
    }
    i2c_info_ptr->VECTOR = vector;

    _lwsem_create(&i2c_info_ptr->LWSEM, 0);

    /* Install new vectors and backup the old ones */
    i2c_info_ptr->OLD_ISR_DATA = _int_get_isr_data(vector);
    i2c_info_ptr->OLD_ISR      = _int_install_isr(vector, _imx_i2c_isr, (void *)i2c_info_ptr);
    _bsp_int_init(vector, i2c_init_ptr->LEVEL, i2c_init_ptr->SUBLEVEL, TRUE);

    /* Enable I2C interrupts */
    i2c_ptr->I2CR |= I2C_I2CR_IIEN_MASK;

    clock_disable(i2c_info_ptr->CLOCK_I2C);

    int_io_dev_ptr->DEV_INFO_PTR = (void*)i2c_info_ptr;

    return I2C_OK;
} /* Endbody */


/*FUNCTION****************************************************************
*
* Function Name    : _imx_i2c_int_deinit
* Returned Value   : MQX error code
* Comments         :
*    This function de-initializes an I2C device.
*
*END*********************************************************************/

uint32_t _imx_i2c_int_deinit
    (
        /* [IN] the initialization information for the device being opened */
        IO_I2C_INT_DEVICE_STRUCT_PTR int_io_dev_ptr,

        /* [IN] the address of the device specific information */
        IMX_I2C_INFO_STRUCT_PTR      io_info_ptr
    )
{ /* Body */
    I2C_MemMapPtr i2c_ptr;

    if ((NULL == io_info_ptr) || (NULL == int_io_dev_ptr))
    {
        return I2C_ERROR_INVALID_PARAMETER;
    }

    clock_enable(io_info_ptr->CLOCK_I2C);

    i2c_ptr = io_info_ptr->I2C_PTR;

    if (i2c_ptr->I2SR & I2C_I2SR_IBB_MASK)
    {
        clock_disable(io_info_ptr->CLOCK_I2C);
        return I2C_ERROR_DEVICE_BUSY;
    }

    /* Disable the I2C */
    i2c_ptr->I2CR = 0x00;

    /* Clear the I2C events */
    i2c_ptr->I2SR = 0x00;

    /* Disable I2C interrupts */
    _bsp_int_disable(io_info_ptr->VECTOR);

    /* Install original vectors */
    _int_install_isr(io_info_ptr->VECTOR, io_info_ptr->OLD_ISR, io_info_ptr->OLD_ISR_DATA);

    _lwsem_destroy(&io_info_ptr->LWSEM);

    clock_disable(io_info_ptr->CLOCK_I2C);
    /* Free info struct */
    _mem_free(io_info_ptr);
    io_info_ptr = NULL;

    return I2C_OK;
} /* Endbody */

/*FUNCTION****************************************************************
*
* Function Name    : _imx_i2c_int_rx
* Returned Value   : number of bytes read
* Comments         :
*   Returns the number of bytes received.
*   Reads the data into provided array when data is available.
*
*END*********************************************************************/

uint32_t _imx_i2c_int_rx
    (
        /* [IN] the address of the device specific information */
        IO_I2C_INT_DEVICE_STRUCT_PTR int_io_dev_ptr,

        /* [IN] The array to copy data into */
        unsigned char               *buffer,

        /* [IN] number of bytes to read */
        uint32_t                     length
    )
{   /* Body */
    IMX_I2C_INFO_STRUCT_PTR io_info_ptr;
    I2C_MemMapPtr           i2c_ptr;

    io_info_ptr = int_io_dev_ptr->DEV_INFO_PTR;
    i2c_ptr = io_info_ptr->I2C_PTR;

    if (length == 0)
        return 0;

    clock_enable(io_info_ptr->CLOCK_I2C);

    /* Critical section + avoiding spurious interrupt */
    _int_disable();
    _bsp_int_disable(io_info_ptr->VECTOR);
    _int_enable();

    /* If beginning of transmission, set state and send address (master only) */
    io_info_ptr->OPERATION     |= I2C_OPERATION_READ;
    io_info_ptr->RX_BUFFER      = buffer;
    io_info_ptr->RX_REQUEST     = length;
    io_info_ptr->RX_BUFFER_SIZE = length;
    io_info_ptr->RX_INDEX       = 0;

    if (I2C_MODE_MASTER == io_info_ptr->MODE)
    {
        /* Address cycle */
        if ((I2C_STATE_READY == io_info_ptr->STATE) || (I2C_STATE_REPEATED_START == io_info_ptr->STATE))
        {
            // set i2c work as tx mode
            i2c_ptr->I2CR |= I2C_I2CR_MTX_MASK;
            i2c_ptr->I2SR &= (~ I2C_I2SR_IIF_MASK);
            if (I2C_STATE_REPEATED_START == io_info_ptr->STATE)
            {
                // Send Repeat Start Signal
                i2c_ptr->I2CR |= I2C_I2CR_RSTA_MASK;
            }
            else
            {
                if (i2c_ptr->I2SR & I2C_I2SR_IBB_MASK)
                {
                    clock_disable(io_info_ptr->CLOCK_I2C);
                    return 0;
                }
                // Send Start Signal
                i2c_ptr->I2CR |= I2C_I2CR_MSTA_MASK;
            }
            io_info_ptr->OPERATION |= I2C_OPERATION_STARTED;
            i2c_ptr->I2DR = (io_info_ptr->ADDRESSEE << 1) | I2C_OPERATION_READ;
            io_info_ptr->STATISTICS.TX_PACKETS++;
        }
        else
        {
          clock_disable(io_info_ptr->CLOCK_I2C);
          return 0;
        }
    }

    /* Interrupt enable - end of critical section */
    _bsp_int_enable(io_info_ptr->VECTOR);

    /* Wait for rx complite */
    _lwsem_wait(&io_info_ptr->LWSEM);

    io_info_ptr->RX_BUFFER = NULL;

    clock_disable(io_info_ptr->CLOCK_I2C);
    return io_info_ptr->RX_INDEX;
} /* Endbody */


/*FUNCTION****************************************************************
*
* Function Name    : _imx_i2c_int_tx
* Returned Value   : number of bytes written
* Comments         :
*   Returns the number of bytes written.
*   Writes the data provided into transmition buffer if available.
*
*END*********************************************************************/

uint32_t _imx_i2c_int_tx
    (
        /* [IN] the address of the device specific information */
        IO_I2C_INT_DEVICE_STRUCT_PTR int_io_dev_ptr,

        /* [IN] The array characters are to be read from */
        unsigned char               *buffer,

        /* [IN] number of bytes to output */
        uint32_t                     length
    )
{   /* Body */
    IMX_I2C_INFO_STRUCT_PTR io_info_ptr;
    I2C_MemMapPtr           i2c_ptr;

    io_info_ptr  = int_io_dev_ptr->DEV_INFO_PTR;
    i2c_ptr      = io_info_ptr->I2C_PTR;

    clock_enable(io_info_ptr->CLOCK_I2C);

    /* Critical section + avoiding spurious interrupt */
    _int_disable();
    _bsp_int_disable(io_info_ptr->VECTOR);
    _int_enable();

    /* If beginning of transmission, set state and send address (master only) */
    io_info_ptr->OPERATION     &= (~ I2C_OPERATION_READ);
    io_info_ptr->TX_BUFFER      = buffer;
    io_info_ptr->TX_BUFFER_SIZE = length;
    io_info_ptr->TX_INDEX       = 0;

    if (I2C_MODE_MASTER == io_info_ptr->MODE)
    {
        /* Address cycle */
        if ((I2C_STATE_READY == io_info_ptr->STATE) || (I2C_STATE_REPEATED_START == io_info_ptr->STATE))
        {
            // set i2c work as tx mode
            i2c_ptr->I2CR |= I2C_I2CR_MTX_MASK;
            i2c_ptr->I2SR &= (~ I2C_I2SR_IIF_MASK);
            if (I2C_STATE_REPEATED_START == io_info_ptr->STATE)
            {
                // Send Repeat Start Signal
                i2c_ptr->I2CR |= I2C_I2CR_RSTA_MASK;
            }
            else
            {
                if (i2c_ptr->I2SR & I2C_I2SR_IBB_MASK)
                {
                    clock_disable(io_info_ptr->CLOCK_I2C);
                    return 0;
                }
                // Send Start Signal
                i2c_ptr->I2CR |= I2C_I2CR_MSTA_MASK;
            }
            io_info_ptr->OPERATION |= I2C_OPERATION_STARTED;
            i2c_ptr->I2DR = (io_info_ptr->ADDRESSEE << 1) | I2C_OPERATION_WRITE;
            io_info_ptr->STATISTICS.TX_PACKETS++;
        }
        /* Transmit cycle */
        else if (I2C_STATE_TRANSMIT == io_info_ptr->STATE)
        {
            if (length != 0)
            {
                /* send first byte */
                i2c_ptr->I2DR = io_info_ptr->TX_BUFFER[io_info_ptr->TX_INDEX++];   /*  transmit data */
                io_info_ptr->STATISTICS.TX_PACKETS++;
            }
            else
            {
                clock_disable(io_info_ptr->CLOCK_I2C);
                return 0;
            }
        }
    }
    /* Interrupt enable - end of critical section */
    _bsp_int_enable(io_info_ptr->VECTOR);

    /* Wait for tx complite */
    _lwsem_wait(&io_info_ptr->LWSEM);

    io_info_ptr->TX_BUFFER = NULL;

    clock_disable(io_info_ptr->CLOCK_I2C);
    return io_info_ptr->TX_INDEX;

} /* Endbody */


/*FUNCTION****************************************************************
*
* Function Name    :_imx_i2c_isr
* Returned Value   : none
*
*END*********************************************************************/
static void _imx_i2c_isr
    (
        void *parameter
    )
{   /* Body */
    IMX_I2C_INFO_STRUCT_PTR io_info_ptr = (IMX_I2C_INFO_STRUCT_PTR)parameter;
    I2C_MemMapPtr           i2c_ptr = io_info_ptr->I2C_PTR;
    uint16_t                i2csr;
    volatile uint16_t       tmp;

    /* Clear interrupt flag */
    i2c_ptr->I2SR &= (~ I2C_I2SR_IIF_MASK);
    io_info_ptr->STATISTICS.INTERRUPTS++;

    i2csr = i2c_ptr->I2SR;

    /* Master */
    if (i2c_ptr->I2CR & I2C_I2CR_MSTA_MASK)
    {
        /* Master Transmit */
        if (i2c_ptr->I2CR & I2C_I2CR_MTX_MASK)
        {
            /* Not ack */
            if (i2csr & I2C_I2SR_RXAK_MASK)
            {
                io_info_ptr->STATE = I2C_STATE_FINISHED;
                io_info_ptr->STATISTICS.TX_NAKS++;
                _bsp_int_disable(io_info_ptr->VECTOR);
                _lwsem_post(&io_info_ptr->LWSEM);
            }
            /* Ack */
            else
            {
                /* End of address cycle? */
                if ((I2C_STATE_READY == io_info_ptr->STATE) || (I2C_STATE_REPEATED_START == io_info_ptr->STATE))
                {
                    /* Transmit operation */
                    if (0 == (I2C_OPERATION_READ & io_info_ptr->OPERATION))
                    {
                        io_info_ptr->STATE = I2C_STATE_TRANSMIT;
                        if (0 == io_info_ptr->TX_BUFFER_SIZE)
                        {
                            _bsp_int_disable(io_info_ptr->VECTOR);
                            _lwsem_post(&io_info_ptr->LWSEM);
                        }
                        else
                        {
                            /*Transmit first byte*/
                            i2c_ptr->I2DR = io_info_ptr->TX_BUFFER[io_info_ptr->TX_INDEX++];
                            io_info_ptr->STATISTICS.TX_PACKETS++;
                        }
                    }
                    /* Receive operation */
                    else
                    {
                        /* Change to receive state */
                        io_info_ptr->STATE = I2C_STATE_RECEIVE;
                        i2c_ptr->I2CR &= (~ I2C_I2CR_MTX_MASK);

                        if (1 == io_info_ptr->RX_REQUEST)
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
                    /* Anything to transmit? */
                    if (io_info_ptr->TX_INDEX < io_info_ptr->TX_BUFFER_SIZE)
                    {
                        i2c_ptr->I2DR = io_info_ptr->TX_BUFFER[io_info_ptr->TX_INDEX++];   /*  transmit data */
                        io_info_ptr->STATISTICS.TX_PACKETS++;
                    }
                    else
                    {
                        /* Transmit finish */
                        _bsp_int_disable(io_info_ptr->VECTOR);
                        _lwsem_post(&io_info_ptr->LWSEM);
                    }
                }
            }
        }
        /* Master Receive */
        else
        {
            /* Buffer full */
            if (io_info_ptr->RX_INDEX >= io_info_ptr->RX_BUFFER_SIZE)
            {
                _bsp_int_disable(io_info_ptr->VECTOR);
                _lwsem_post(&io_info_ptr->LWSEM);
            }
            /* Buffer not full */
            else
            {
                /* 2nd last byte to read */
                if (2 == io_info_ptr->RX_REQUEST)
                {
                    i2c_ptr->I2CR |= I2C_I2CR_TXAK_MASK;
                }
                else
                {
                    i2c_ptr->I2CR &= (~ I2C_I2CR_TXAK_MASK);
                }

                if(1 == io_info_ptr->RX_REQUEST)
                {
                    i2c_ptr->I2CR |= I2C_I2CR_MTX_MASK;  /* no more reading */
                }

                tmp = i2c_ptr->I2DR;   /* receive data */
                io_info_ptr->RX_BUFFER[io_info_ptr->RX_INDEX++] = (tmp & I2C_I2DR_DATA_MASK);
                io_info_ptr->RX_REQUEST--;
                io_info_ptr->STATISTICS.RX_PACKETS++;
                if(0 == io_info_ptr->RX_REQUEST)
                {
                    _bsp_int_disable(io_info_ptr->VECTOR);
                    _lwsem_post(&io_info_ptr->LWSEM);
                }
            }
        }
    }
    /* Slave */
    else
    {
        /* Master arbitration lost */
        if (i2csr & I2C_I2SR_IAL_MASK)
        {
            i2c_ptr->I2SR &= (~ I2C_I2SR_IAL_MASK);
            io_info_ptr->STATE = I2C_STATE_LOST_ARBITRATION;
            io_info_ptr->STATISTICS.TX_LOST_ARBITRATIONS++;
        }

        /* Addressed as slave */
        if ((i2csr & I2C_I2SR_IAAS_MASK) && (I2C_MODE_MASTER == io_info_ptr->MODE))
        {
            io_info_ptr->STATISTICS.TX_ADDRESSED_AS_SLAVE++;
        }

        _bsp_int_disable(io_info_ptr->VECTOR);
        _lwsem_post(&io_info_ptr->LWSEM);
    }
} /* Endbody */

/*FUNCTION****************************************************************
*
* Function Name    : _imx_i2c_int_ioctl
* Returned Value   : MQX error code.
* Comments         :
*    This function performs miscellaneous services for
*    the I2C I/O device.
*
*END*********************************************************************/

uint32_t _imx_i2c_int_ioctl(void *io_info_ptr, uint32_t cmd, uint32_t *param_ptr) {

    IMX_I2C_INFO_STRUCT_PTR i2c_info_ptr = (IMX_I2C_INFO_STRUCT_PTR)io_info_ptr;
    I2C_MemMapPtr           i2c_ptr = i2c_info_ptr->I2C_PTR;
    uint32_t                result = MQX_OK;
    MQX_TICK_STRUCT         tick_old, tick_new;
    volatile uint16_t       tmp;
    uint32_t                div, index;

    clock_enable(i2c_info_ptr->CLOCK_I2C);
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

            _time_get_elapsed_ticks(&tick_old);
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

                _time_get_elapsed_ticks(&tick_new);
                /* if polling time great than 10ms then break */
                if (50 < _time_diff_milliseconds(&tick_new, &tick_old, NULL))
                    break;
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
                i2c_ptr->IFDR = (I2C_IFDR_IC_MASK & _imx_i2c_find_baudrate_divider(clock_get_freq(i2c_info_ptr->CLOCK_I2C), *param_ptr)) << I2C_IFDR_IC_SHIFT;
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
                *param_ptr = (uint32_t)((clock_get_freq(i2c_info_ptr->CLOCK_I2C)) / (div));
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
    clock_disable(i2c_info_ptr->CLOCK_I2C);
    return result;
}

/* EOF */
