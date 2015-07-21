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
*   This file contains the GPIO standard functions used on boards
*
*
*END************************************************************************/

#include "mqx.h"
#include "bsp.h"
#include "lwgpio_igpio.h"
#include "lwgpio.h"

#define IOMUX_MUX_MODE_ALT0           (0)
#define IOMUX_MUX_MODE_ALT1           (1)
#define IOMUX_MUX_MODE_ALT2           (2)
#define IOMUX_MUX_MODE_ALT3           (3)
#define IOMUX_MUX_MODE_ALT4           (4)
#define IOMUX_MUX_MODE_ALT5           (5)
#define IOMUX_MUX_MODE_ALT6           (6)

#define IOMUX_PAD_CTL_PUS_100K_DOWN   (0 << 14)
#define IOMUX_PAD_CTL_PUS_47K_UP      (1 << 14)
#define IOMUX_PAD_CTL_PUS_100K_UP     (2 << 14)
#define IOMUX_PAD_CTL_PUS_22K_UP      (3 << 14)

#define IOMUX_PAD_CTL_PUE             (1 << 13)
#define IOMUX_PAD_CTL_PKE             (1 << 12)
#define IOMUX_PAD_CTL_ODE             (1 << 11)

#define IOMUX_PAD_CTL_SPEED_LOW       (0 << 6)
#define IOMUX_PAD_CTL_SPEED_MED_0     (1 << 6)
#define IOMUX_PAD_CTL_SPEED_MED_1     (2 << 6)
#define IOMUX_PAD_CTL_SPEED_HIGH      (3 << 6)

#define IOMUX_PAD_CTL_DSE_HIZ         (0 << 3)
#define IOMUX_PAD_CTL_DSE_260_OHM     (1 << 3)
#define IOMUX_PAD_CTL_DSE_130_OHM     (2 << 3)
#define IOMUX_PAD_CTL_DSE_87_OHM      (3 << 3)
#define IOMUX_PAD_CTL_DSE_65_OHM      (4 << 3)
#define IOMUX_PAD_CTL_DSE_52_OHM      (5 << 3)
#define IOMUX_PAD_CTL_DSE_43_OHM      (6 << 3)
#define IOMUX_PAD_CTL_DSE_37_OHM      (7 << 3)

#define IOMUX_PAD_CTL_SRE_FAST        (1 << 0)
#define IOMUX_PAD_CTL_SRE_SLOW        (0 << 0)

static volatile uint32_t *iomuxc_pad_reg_arr[] = { 
    &IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO00, /* GPIO1: 0-25 */
    &IOMUXC_SW_PAD_CTL_PAD_ENET1_COL,  /* GPIO2: 0-19 */
    &IOMUXC_SW_PAD_CTL_PAD_LCD1_CLK,   /* GPIO3: 0-28 */
    &IOMUXC_SW_PAD_CTL_PAD_NAND_ALE,   /* GPIO4: 0-31 */
    &IOMUXC_SW_PAD_CTL_PAD_RGMII1_RD0, /* GPIO5: 0-23 */
    &IOMUXC_SW_PAD_CTL_PAD_SD1_CLK,    /* GPIO6: 0-11 */
    &IOMUXC_SW_PAD_CTL_PAD_SD3_CLK,    /* GPIO7: 0-9 */
    &IOMUXC_SW_PAD_CTL_PAD_SD4_CLK,    /* GPIO6: 12-22 */
    &IOMUXC_SW_PAD_CTL_PAD_USB_H_DATA  /* GPIO7: 10-11 */
};

static volatile uint32_t *iomuxc_mux_reg_arr[] = { 
    &IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO00, /* GPIO1: 0-25 */
    &IOMUXC_SW_MUX_CTL_PAD_ENET1_COL,  /* GPIO2: 0-19 */
    &IOMUXC_SW_MUX_CTL_PAD_LCD1_CLK,   /* GPIO3: 0-28 */
    &IOMUXC_SW_MUX_CTL_PAD_NAND_ALE,   /* GPIO4: 0-31 */
    &IOMUXC_SW_MUX_CTL_PAD_RGMII1_RD0, /* GPIO5: 0-23 */
    &IOMUXC_SW_MUX_CTL_PAD_SD1_CLK,    /* GPIO6: 0-11 */
    &IOMUXC_SW_MUX_CTL_PAD_SD3_CLK,    /* GPIO7: 0-9 */
    &IOMUXC_SW_MUX_CTL_PAD_SD4_CLK,    /* GPIO6: 12-22 */
    &IOMUXC_SW_MUX_CTL_PAD_USB_H_DATA  /* GPIO7: 10-11 */
};

static const GPIO_MemMapPtr gpio_map_arr[] = GPIO_BASE_PTRS;

static const uint32_t gpio_pad_count_arr[] = {
    26, 20, 29, 32, 24, 23, 12
};

#define LWGPIO_PIN_FROM_ID(id)         (((id) & LWGPIO_PIN_MASK) >> LWGPIO_PIN_SHIFT)
#define LWGPIO_PORT_FROM_ID(id)        (((id) & LWGPIO_PORT_MASK) >> LWGPIO_PORT_SHIFT)

/*FUNCTION*****************************************************************
* 
* Function Name    : lwgpio_init
* Returned Value   : TRUE if succesfull, FALSE otherwise
* Comments         :
*    Decodes ID to HW specific struct and then performs pin initialization
*
*END*********************************************************************/
bool lwgpio_init
(
    /* Pointer to LWGPIO internal structure to be filled in */
    LWGPIO_STRUCT_PTR handle,
    /* Pin ID, bitmask integer value */
    LWGPIO_PIN_ID     id,
    /* Direction to be set within initialization */
    LWGPIO_DIR        dir,
    /* Value to be set within initialization */
    LWGPIO_VALUE      value
)
{ /* Body */
    uint32_t port_idx, pin_idx, iomuxc_idx = 0;
    
    port_idx = LWGPIO_PORT_FROM_ID(id) - 1; /* port id starts from 1 */
    pin_idx = LWGPIO_PIN_FROM_ID(id);

    /* parameter check */
    if (port_idx >= ELEMENTS_OF(gpio_pad_count_arr) ||
            pin_idx >= gpio_pad_count_arr[port_idx])
        return FALSE;

    handle->flags = id;
    handle->gpio_ptr = gpio_map_arr[port_idx];
    handle->pinmask = 1 << pin_idx;

    /* fix iomuxc index */
    if (port_idx < 5)
        iomuxc_idx = port_idx;
    else if (port_idx == 5) {
        if (pin_idx < 12)
            iomuxc_idx = port_idx;
        else {
            iomuxc_idx = port_idx + 2;
            pin_idx -= 12;
        }
    }
    else if (port_idx == 6) {
        if (pin_idx < 10)
            iomuxc_idx = port_idx;
        else {
            iomuxc_idx = port_idx + 2;
            pin_idx -= 10;
        }
    }

    handle->iomuxc_pad_reg = iomuxc_pad_reg_arr[iomuxc_idx] + pin_idx;
    handle->iomuxc_mux_reg = iomuxc_mux_reg_arr[iomuxc_idx] + pin_idx;
    
    /* Set value prior to set to output */
    if (value != LWGPIO_VALUE_NOCHANGE) {
        /* Note: there is no check for values not defined as LWGPIO_VALUE enum */
        lwgpio_set_value(handle, value);
    }

    if (dir != LWGPIO_DIR_NOCHANGE) {
        /* Note: there is no check for values not defined as LWGPIO_DIR enum */
        lwgpio_set_direction(handle, dir);
    }

    return TRUE;
}

/*FUNCTION*****************************************************************
* 
* Function Name    : lwgpio_set_attribute
* Returned Value   : TRUE if successful, FALSE otherwise
* Comments         :
*    Sets attributes
*
*END*********************************************************************/
bool lwgpio_set_attribute
(
    /* Pin handle to get function from */
    LWGPIO_STRUCT_PTR  handle,
    /* PORT attribute */
    uint32_t attribute_id,
    /* Attribute value */
    uint32_t value
)
{
    uint32_t temp;

    switch (attribute_id) 
    {
        case(LWGPIO_ATTR_PULL_UP):
        {
            if (value == LWGPIO_AVAL_ENABLE)
            {
                temp = *handle->iomuxc_pad_reg;

                /* select 47k pull up resistor */
                temp &= ~IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO00_PUE_MASK;
                temp |= IOMUX_PAD_CTL_PUS_47K_UP;

                /* select pull mode, enable pull mode */
                temp |= IOMUX_PAD_CTL_PUE | IOMUX_PAD_CTL_PKE;

                *handle->iomuxc_pad_reg = temp;
            }
            else
            {
                *handle->iomuxc_pad_reg &= ~IOMUX_PAD_CTL_PKE;
            }
            break;
        }
        case(LWGPIO_ATTR_PULL_DOWN):
        {
            if (value == LWGPIO_AVAL_ENABLE)
            {
                temp = *handle->iomuxc_pad_reg;

                /* select 100k pull down resistor */
                temp &= ~IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO00_PUS_MASK;
		temp |= IOMUX_PAD_CTL_PUS_100K_DOWN;

                /* select pull mode, enable pull mode */
                temp |= IOMUX_PAD_CTL_PUE | IOMUX_PAD_CTL_PKE;

                *handle->iomuxc_pad_reg = temp;
            } 
            else 
            {
                *handle->iomuxc_pad_reg &= ~IOMUX_PAD_CTL_PKE;
            }
            break;
        }
        case(LWGPIO_ATTR_SLEW_RATE):
        {
            if (value == LWGPIO_AVAL_SLEW_RATE_SLOW)
            {
                *handle->iomuxc_pad_reg &= ~IOMUX_PAD_CTL_SRE_FAST;
            }
            else
            {
                *handle->iomuxc_pad_reg |= IOMUX_PAD_CTL_SRE_FAST;
            }
            break;
        }
        case(LWGPIO_ATTR_OPEN_DRAIN):
        {
            if (value == LWGPIO_AVAL_ENABLE)
            {
                *handle->iomuxc_pad_reg |= IOMUX_PAD_CTL_ODE;
            }
            else
            {
                *handle->iomuxc_pad_reg &= ~IOMUX_PAD_CTL_ODE;
            }
            break;
        }
        case(LWGPIO_ATTR_DRIVE_STRENGTH):
        {
            temp = *handle->iomuxc_pad_reg;

            /* setup drive strenth */
            temp &= ~IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO00_DSE_MASK;
            temp |= IOMUXC_SW_PAD_CTL_PAD_GPIO1_IO00_DSE(value);

            *handle->iomuxc_pad_reg = temp;
            break;
        }
        default:
            return FALSE;
    }
    return TRUE;
}

/*FUNCTION*****************************************************************
* 
* Function Name    : lwgpio_set_functionality
* Returned Value   : void
* Comments         :
*    Sets functionality (peripheral mode) of the pin
*
*END*********************************************************************/
void lwgpio_set_functionality
(
    /* Pin handle to set function on */
    LWGPIO_STRUCT_PTR  handle, 
    /* Function to be set (integer value) */
    uint32_t           function
)
{
    uint32_t temp;

    temp = *handle->iomuxc_mux_reg;

    temp &= ~IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO00_MUX_MODE_MASK;
    temp |= IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO00_MUX_MODE(function);

    *handle->iomuxc_mux_reg = temp;
}

/*FUNCTION*****************************************************************
* 
* Function Name    : lwgpio_get_functionality
* Returned Value   : void
* Comments         :
*    Gets functionality (peripheral mode) of the pin
*
*END*********************************************************************/
uint32_t lwgpio_get_functionality
(
    /* Pin handle to get function from */
    LWGPIO_STRUCT_PTR  handle
)
{
    return (*handle->iomuxc_mux_reg & IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO00_MUX_MODE_MASK) >> IOMUXC_SW_MUX_CTL_PAD_GPIO1_IO00_MUX_MODE_SHIFT;
}

/*FUNCTION*****************************************************************
* 
* Function Name    : lwgpio_set_direction
* Returned Value   : void
* Comments         :
*    Sets direction of the pin
*
*END*********************************************************************/
void lwgpio_set_direction
(
    /* Pin handle to set direction on */
    LWGPIO_STRUCT_PTR  handle, 
    /* Direction to be set */
    LWGPIO_DIR         dir
)
{
    uint32_t port_id = LWGPIO_PORT_FROM_ID(handle->flags);

    /* RDC SEMA42 lock */
    _bsp_lock_lwgpio(port_id);

    if (dir == LWGPIO_DIR_INPUT) {
        handle->gpio_ptr->GDIR &= ~handle->pinmask;
    }
    else {
        handle->gpio_ptr->GDIR |= handle->pinmask;
    }

    /* RDC SEMA42 unlock */
    _bsp_unlock_lwgpio(port_id);
}

/*FUNCTION*****************************************************************
* 
* Function Name    : lwgpio_set_value
* Returned Value   : void
* Comments         :
*    Sets value (output latch) of the pin
*
*END*********************************************************************/
void lwgpio_set_value
(
    /* Pin handle to set value on */
    LWGPIO_STRUCT_PTR  handle, 
    /* Value to be set */
    LWGPIO_VALUE       out_value
)
{
    uint32_t port_id = LWGPIO_PORT_FROM_ID(handle->flags);

    /* RDC SEMA42 lock */
    _bsp_lock_lwgpio(port_id);

    if (out_value == LWGPIO_VALUE_LOW) {
        handle->gpio_ptr->DR &= ~(handle->pinmask);
    }
    else if (out_value == LWGPIO_VALUE_HIGH) {
        handle->gpio_ptr->DR |= handle->pinmask;
    }

    /* RDC SEMA42 unlock */
    _bsp_unlock_lwgpio(port_id);
}

/*FUNCTION*****************************************************************
* 
* Function Name    : lwgpio_toggle_value
* Returned Value   : void
* Comments         :
*    Toggles value of output latch of the pin
*
*END*********************************************************************/
void lwgpio_toggle_value
(
    /* Pin handle to toggle value on */
    LWGPIO_STRUCT_PTR  handle
)
{
    uint32_t port_id = LWGPIO_PORT_FROM_ID(handle->flags);

    /* RDC SEMA42 lock */
    _bsp_lock_lwgpio(port_id);

    if (handle->gpio_ptr->DR & handle->pinmask) {
        handle->gpio_ptr->DR &= ~(handle->pinmask);
    }
    else {
        handle->gpio_ptr->DR |= handle->pinmask;
    }

    /* RDC SEMA42 unlock */
    _bsp_unlock_lwgpio(port_id);
}

/*FUNCTION*****************************************************************
* 
* Function Name    : lwgpio_get_value
* Returned Value   : LWGPIO_VALUE of pin status
* Comments         :
*    Returns value (output latch or read data) of the pin
*
*END*********************************************************************/
LWGPIO_VALUE lwgpio_get_value
(
    /* Pin handle to get value from */
    LWGPIO_STRUCT_PTR  handle
)
{
    LWGPIO_VALUE val;
    uint32_t port_id = LWGPIO_PORT_FROM_ID(handle->flags);

    /* RDC SEMA42 lock */
    _bsp_lock_lwgpio(port_id);

    val = (handle->gpio_ptr->DR & handle->pinmask) ? LWGPIO_VALUE_HIGH : LWGPIO_VALUE_LOW;

    /* RDC SEMA42 unlock */
    _bsp_unlock_lwgpio(port_id);

    return val;
}

/*FUNCTION*****************************************************************
* 
* Function Name    : lwgpio_get_raw
* Returned Value   : LWGPIO_VALUE of pin status
* Comments         :
*    Returns read level of the pin
*
*END*********************************************************************/
LWGPIO_VALUE lwgpio_get_raw
(
    /* Pin handle to get value from */
    LWGPIO_STRUCT_PTR  handle
)
{
    LWGPIO_VALUE val;
    uint32_t port_id = LWGPIO_PORT_FROM_ID(handle->flags);

    /* RDC SEMA42 lock */
    _bsp_lock_lwgpio(port_id);

    val = (handle->gpio_ptr->PSR & handle->pinmask) ? LWGPIO_VALUE_HIGH : LWGPIO_VALUE_LOW;

    /* RDC SEMA42 unlock */
    _bsp_unlock_lwgpio(port_id);

    return val;
}

/*FUNCTION*****************************************************************
* 
* Function Name    : lwgpio_int_init
* Returned Value   : TRUE if succesfull, FALSE otherwise
* Comments         :
*    Initializes pin to generate interrupt
*
*END*********************************************************************/
bool lwgpio_int_init
(
    /* Pin handle to initialize interrupt on */
    LWGPIO_STRUCT_PTR handle,
    /* Interrupt mode */
    LWGPIO_INT_MODE   mode
)
{ /* Body */
    uint32_t pin_idx = LWGPIO_PIN_FROM_ID(handle->flags);
    volatile uint32_t *icr;
    uint32_t mask, shift;
    uint32_t port_id = LWGPIO_PORT_FROM_ID(handle->flags);

    if (pin_idx < 16) {
        icr = &handle->gpio_ptr->ICR1;
    }
    else {
        icr = &handle->gpio_ptr->ICR2;
	pin_idx -= 16;
    }
    shift = pin_idx * 2;
    mask = ~(3 << shift);

    /* RDC SEMA42 lock */
    _bsp_lock_lwgpio(port_id);

    if (mode & LWGPIO_INT_MODE_RISING) {
        *icr = (*icr & mask) | (GPIO_ICR1_ICR0(2) << shift);
    }
    else if (mode & LWGPIO_INT_MODE_FALLING) {
        *icr = (*icr & mask) | (GPIO_ICR1_ICR0(3) << shift);
    }
    else if (mode & LWGPIO_INT_MODE_HIGH) {
        *icr = (*icr & mask) | (GPIO_ICR1_ICR0(1) << shift);
    }
    else if (mode & LWGPIO_INT_MODE_LOW) {
        *icr = (*icr & mask) | (GPIO_ICR1_ICR0(0) << shift);
    }

    /* RDC SEMA42 unlock */
    _bsp_unlock_lwgpio(port_id);

    return TRUE;
}

/*FUNCTION*****************************************************************
* 
* Function Name    : lwgpio_int_enable
* Returned Value   : void
* Comments         :
*    Enables / disables interrupts for specified pin
*
*END*********************************************************************/
void lwgpio_int_enable
(
    /* Pin handle to enable interrupt on */
    LWGPIO_STRUCT_PTR handle,
    /* Enable or disable interrupt? TRUE = enable */
    bool              ena
)
{ /* Body */
    uint32_t port_id = LWGPIO_PORT_FROM_ID(handle->flags);

    /* RDC SEMA42 lock */
    _bsp_lock_lwgpio(port_id);

    if (ena) {
        /* enable pin interrupts */
        handle->gpio_ptr->IMR |= handle->pinmask;
    }
    else {
        /* disable pin interrupts */
        handle->gpio_ptr->IMR &= ~handle->pinmask;
    }

    /* RDC SEMA42 unlock */
    _bsp_unlock_lwgpio(port_id);
}

/*FUNCTION*****************************************************************
* 
* Function Name    : lwgpio_int_get_flag
* Returned Value   : TRUE if interrupt flag is set
* Comments         :
*    Checks if there is pending interrupt flag for specified pin
*
*END*********************************************************************/
bool lwgpio_int_get_flag
(
    /* Pin handle to get interrupt flag on */
    LWGPIO_STRUCT_PTR handle
)
{ /* Body */
    bool flag;
    uint32_t port_id = LWGPIO_PORT_FROM_ID(handle->flags);

    /* RDC SEMA42 lock */
    _bsp_lock_lwgpio(port_id);

    flag = (handle->gpio_ptr->ISR & handle->pinmask) ? TRUE : FALSE;

    /* RDC SEMA42 unlock */
    _bsp_unlock_lwgpio(port_id);

    return flag;
}

/*FUNCTION*****************************************************************
* 
* Function Name    : lwgpio_int_clear_flag
* Returned Value   : void
* Comments         :
*    Clears pending interrupt flag on peripheral
*
*END*********************************************************************/
void lwgpio_int_clear_flag
(
    /* Pin handle to clear interrupt flag on */
    LWGPIO_STRUCT_PTR handle
)
{ /* Body */
    uint32_t port_id = LWGPIO_PORT_FROM_ID(handle->flags);

    /* RDC SEMA42 lock */
    _bsp_lock_lwgpio(port_id);

    handle->gpio_ptr->ISR = handle->pinmask;

    /* RDC SEMA42 unlock */
    _bsp_unlock_lwgpio(port_id);
}

/*FUNCTION****************************************************************
* 
* Function Name    : _bsp_get_gpio_base_address
* Returned Value   : pointer to base of GPIO registers
* Comments         :
*    This function returns base address of GPIO related register space.
*
*END*********************************************************************/
uint32_t lwgpio_int_get_vector
(
    /* Pin handle to get vector of */
    LWGPIO_STRUCT_PTR  handle
)
{
    uint32_t port_id, pin_id;
    uint32_t vec;

    port_id = LWGPIO_PORT_FROM_ID(handle->flags);
    pin_id = LWGPIO_PIN_FROM_ID(handle->flags);

#if PSP_CPU_IMX6SX_M4
    /* return CM4 VECTOR */
    if (port_id == 1) {
        if (pin_id < 8)
            vec = INT_GPIO1_INT0 - pin_id;
	else
	    vec = INT_GPIO1_Combined_0_15 + ((pin_id < 16) ? 0 : 1);
    }
    else {
        vec = INT_GPIO2_Combined_0_15 + (port_id - 2) * 2 + ((pin_id < 16) ? 0 : 1);
    }
#else
    #error "Unsupported core"
#endif
    return vec;
}

/* EOF */
