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
*   This file contains functions for ECSPI slave driver.
*
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>
#include <stdint.h>

#include "spi_slave_ecspi.h"
#include "spi_slave_ecspi_prv.h"

/* Forward declarations */
void ecspi_slave_irq_handler(void *parameter);

/*FUNCTION**********************************************************************
*
* Function Name    : spi_slave_init
* Returned Value   : SPI status code
* Comments         : 
*    Initializes the ECSPI module
*    Turns on the clock to the module. 
*    Enables the device and enables interrupts.
*
* param1: Pointer to driver info structure.
*
*END***************************************************************************/
_mqx_int ecspi_slave_init(ECSPI_SLAVE_INFO_STRUCT_PTR info_ptr)
{

    ECSPI_SLAVE_INFO_STRUCT_PTR ecspi_info_ptr = (ECSPI_SLAVE_INFO_STRUCT_PTR)info_ptr;
    ECSPI_MemMapPtr             ecspi_reg_ptr;
    CLOCK_NAME                  clk_name;
    uint32_t cs = ecspi_info_ptr->CS;
    uint32_t reg;    /* Temporary register value */
    uint32_t vector; /* Interrupt vector supported by the peripheral */
    uint32_t result; /* Status code */
    uint32_t tx_data = 0;

/******************************************************************************/
    /* Map MemMpam Struct */
    ecspi_reg_ptr = _bsp_get_ecspi_base_address (ecspi_info_ptr->INSTANCE);
    if (NULL == ecspi_reg_ptr)
    {
        return SPI_ERROR_CHANNEL_INVALID;
    }

    clk_name = _bsp_get_ecspi_clock_name(ecspi_info_ptr->INSTANCE);
    if (clk_name == CLK_MAX)
    {
        return SPI_ERROR_CHANNEL_INVALID;
    }

    /* Check framesize */
    if (ecspi_info_ptr->FRAME_SIZE < 1 || ecspi_info_ptr->FRAME_SIZE > 0x1000)
    {
        return SPI_ERROR_FRAMESIZE_INVALID;
    }

    ecspi_info_ptr->ECSPI_CLOCK = clock_get(clk_name);
    clock_enable(ecspi_info_ptr->ECSPI_CLOCK);

    /* Disable ECSPI interrupts */
    ecspi_reg_ptr->INTREG = 0;

    /* Save register access pointer into info structure */
    ecspi_info_ptr->ECSPI_REG_PTR = ecspi_reg_ptr;

    /* Dissable ecspi block */
    ecspi_reg_ptr->CONREG = 0;

    /* Enable ECSPI block */
    ecspi_reg_ptr->CONREG = (1 << (ECSPI_CONREG_EN_SHIFT));

    /* enable clock, set mux, set pad */
    result = _bsp_ecspi_slave_io_init(ecspi_info_ptr->INSTANCE);
    if (result != MQX_OK)
    {
        clock_disable(ecspi_info_ptr->ECSPI_CLOCK);
        ecspi_info_ptr->ECSPI_CLOCK = NULL;
        return result;
    }

    /************************* Set CONTROL REGISTER ***************************/
    reg = 0;

    /* Set burst length mode */
    reg |= ECSPI_CONREG_BURST_LENGTH(ecspi_info_ptr->FRAME_SIZE-1);

    /* Set cs */
    reg |= ECSPI_CONREG_CHANNEL_SELECT(cs);

    /* Enable ECSPI block */
    reg |= (1 << (ECSPI_CONREG_EN_SHIFT));

    /* Write to the CONREG */
    ecspi_reg_ptr->CONREG = reg;

    /*********************** SET CONFIGURATION REGISTER ***********************/
    reg = 0;

    /* Set SS polarity (SS_POL) */
    if (ecspi_info_ptr->SS_POL){
        reg |= (1 << (ECSPI_CONFIGREG_SS_POL_SHIFT + cs));
    }

    /* Set end of SPI burst(SS_CTL)*/
    if (ecspi_info_ptr->SS_CTL){
        reg |= (1 << (ECSPI_CONFIGREG_SS_CTL_SHIFT + cs));
    }

    /* Set Clock polarity */
    if (ecspi_info_ptr->MODE & SPI_CPOL_MASK)
    {
        reg |= (1 << (ECSPI_CONFIGREG_SCLK_POL_SHIFT + cs));
    }

    /* Set Clock phase */
    if (ecspi_info_ptr->MODE & SPI_CPHA_MASK)
    {
        reg |= (1 << (ECSPI_CONFIGREG_SCLK_PHA_SHIFT + cs));
    }

    ecspi_reg_ptr->CONFIGREG = reg;

    /**************************************************************************/
    /* Initial callback call. Get data from application for first transfer. */
    ecspi_info_ptr->CALLBACK(NULL, &tx_data, NULL);

    /* Move first data to tx data register to be ready for first transmition. */
    ecspi_reg_ptr->TXDATA = tx_data;

    /* Install ISR */
    vector = _bsp_get_ecspi_vector(ecspi_info_ptr->INSTANCE);

    _int_install_isr(vector, ecspi_slave_irq_handler, ecspi_info_ptr);

    /* Enable ISR and set priority */
    result = _bsp_int_init(vector, BSP_ECSPI_INT_LEVEL, 0, TRUE);
    if (result != MQX_OK)
    {
        clock_disable(ecspi_info_ptr->ECSPI_CLOCK);
        ecspi_info_ptr->ECSPI_CLOCK = NULL;
        return result;
    }

    /* Clear ECSPI status register */
    ecspi_reg_ptr->STATREG = 0xC0;

    /* Enable RXFIFO Ready Interrupt.*/
    ecspi_reg_ptr->INTREG |= (ECSPI_INTREG_RREN_MASK);

    return SPI_OK;
}


/*FUNCTION**********************************************************************
*
* Function Name    : spi_slave_shutdown
* Returned Value   : SPI status code
* Comments         : 
*    Deinitializes the device.
*    Clears the control register and turns off the clock to the module.
*
* param1: Pointer to driver info structure.
*
*END***************************************************************************/
_mqx_uint ecspi_slave_shutdown(ECSPI_SLAVE_INFO_STRUCT_PTR info_ptr)
{
    ECSPI_SLAVE_INFO_STRUCT_PTR ecspi_info_ptr = (ECSPI_SLAVE_INFO_STRUCT_PTR)info_ptr;
    ECSPI_MemMapPtr              ecspi_reg_ptr = ecspi_info_ptr->ECSPI_REG_PTR;
    uint32_t vector; 
    uint32_t result;

    if (NULL == ecspi_info_ptr || NULL == ecspi_info_ptr->ECSPI_CLOCK)
    {
        return SPI_ERROR_DEINIT_FAILED;
    }

    vector = _bsp_get_ecspi_vector(ecspi_info_ptr->INSTANCE);

    /* Disable interrupt on vector */
    result = _bsp_int_disable(vector);
    if (result != MQX_OK)
    {
        return result;
    }
    /* Install default isr routine */
    _int_install_isr(vector, _int_get_default_isr(), NULL);

    /* Disable ecspi block */
    ecspi_reg_ptr->CONREG = 0;

    clock_disable(ecspi_info_ptr->ECSPI_CLOCK);
    ecspi_info_ptr->ECSPI_CLOCK = NULL;

    return SPI_OK;
}

/*FUNCTION**********************************************************************
*
* Function Name    : spi_slave_irq_handler
* Returned Value   : none
* Comments         : 
*    ECSPI slave IRQ handler.
*    Copy tx buffer to tx data register (will be send in next step) and received
*    data to rx buffer. 
*
* param1: Handler to active ECSPI peripheral.
*
*END***************************************************************************/
void ecspi_slave_irq_handler(void *parameter)
{
    ECSPI_SLAVE_INFO_STRUCT_PTR         ecspi_info_ptr = (ECSPI_SLAVE_INFO_STRUCT_PTR)parameter;
    ECSPI_MemMapPtr                     ecspi_reg_ptr = ecspi_info_ptr->ECSPI_REG_PTR;
    uint32_t rx_data = 0;
    uint32_t tx_data = 0;

    /* Are one word or more in RX FIFO */
    if (ecspi_reg_ptr->STATREG & ECSPI_STATREG_RR_MASK) {
      
        /* Read byte from rx data register */
        rx_data = ecspi_reg_ptr->RXDATA;
        
        /* Propagate received data to application and get tx data via callback */
        ecspi_info_ptr->CALLBACK((void *)ecspi_info_ptr->APP_DATA_PTR, &tx_data, &rx_data);
        
        /* Store tx data to tx data register for next transmition */
        ecspi_reg_ptr->TXDATA = tx_data;
       
        /* Clear status flags */
        ecspi_reg_ptr->STATREG = 0xC0;
    }
}
