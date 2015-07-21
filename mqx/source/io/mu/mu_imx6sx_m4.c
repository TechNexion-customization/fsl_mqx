
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
* See license agreement file for full license terms including other restrictions.
*****************************************************************************
*
* Comments:
*
*   This file contains MCC Message Unit driver implementation of MQX on i.mx6sx M4 core
*
*
*END************************************************************************/

#include "mqx.h"
#include "bsp.h"
#include "mu.h"

static MU_CALLBACK rx_callback[4];
static MU_CALLBACK tx_callback[4];
static MU_CALLBACK gp_callback[4];
static void *rx_callback_arg[4];
static void *tx_callback_arg[4];
static void *gp_callback_arg[4];

static void mu_isr(void *data)
{
    uint32_t sr;
    int i;
    
    sr = MUB_BSR;
    for (i = 0; i < 4; i++)
    {
        if (rx_callback[i] && (sr & ((MU_BSR_RF0_MASK) >> i)))
            rx_callback[i](rx_callback_arg[i]);
        if (tx_callback[i] && (sr & ((MU_BSR_TE0_MASK) >> i)))
            tx_callback[i](tx_callback_arg[i]);
        if (gp_callback[i] && (sr & ((MU_BSR_GIP0_MASK) << i)))
            gp_callback[i](gp_callback_arg[i]);
    }
}

void mu_initialize(void)
{
    static uint8_t ini_flag = 0;

    if (ini_flag == 0) {
        MUB_BCR = 0;

        _int_install_isr(INT_MU_M4, mu_isr, NULL);
        _bsp_int_init(INT_MU_M4, MU_INT_PRIORITY, 0, TRUE);
        _bsp_int_enable(INT_MU_M4);
        ini_flag = 1;
    }
}

void mu_finalize(void)
{
    MUB_BCR = 0;

    _bsp_int_disable(INT_MU_M4);
    _int_install_isr(INT_MU_M4, _int_get_default_isr(), NULL);
}

uint32_t mu_rx(int channel)
{
    switch(channel)
    {
        case 0:
            return MUB_BRR0;
        case 1:
            return MUB_BRR1;
        case 2:
            return MUB_BRR2;
        case 3:
            return MUB_BRR3;
        default:
            return 0;
    }
}

uint32_t mu_rx_available(int channel)
{
    return MUB_BSR & ((MU_BSR_RF0_MASK) >> channel);
}

void mu_tx(int channel, uint32_t data)
{
    switch(channel)
    {
        case 0:
            MUB_BTR0 = data;
            break;
        case 1:
            MUB_BTR1 = data;
            break;
        case 2:
            MUB_BTR2 = data;
            break;
        case 3:
            MUB_BTR3 = data;
            break;
        default:
            break;
    }
}

uint32_t mu_tx_available(int channel)
{
    return MUB_BSR & ((MU_BSR_TE0_MASK) >> channel);
}

void mu_disable_rx_isr(int channel)
{
    MUB_BCR &= ~((MU_BCR_RIE0_MASK) >> channel);
    rx_callback[channel] = NULL;
}

void mu_enable_rx_isr(int channel, MU_CALLBACK callback, void *data)
{
    _int_disable();
    rx_callback[channel] = callback;
    rx_callback_arg[channel] = data;
    _int_enable();

    MUB_BCR |= ((MU_BCR_RIE0_MASK) >> channel);
    // writelocalbuf("MU_BCR is 0x%x\n", MU_BCR);
}

void mu_disable_tx_isr(int channel)
{
    MUB_BCR &= ~((MU_BCR_TIE0_MASK) >> channel);
    tx_callback[channel] = NULL;
}

void mu_set_gp(int index)
{
    MUB_BCR |= MU_BCR_GIR0_MASK << index;
}

void mu_clear_gp(int index)
{
    MUB_BSR = (MU_BSR_GIP0_MASK << index);
}

void mu_enable_tx_isr(int channel, MU_CALLBACK callback, void *data)
{
    _int_disable();
    tx_callback[channel] = callback;
    tx_callback_arg[channel] = data;
    _int_enable();

    MUB_BCR |= ((MU_BCR_TIE0_MASK) >> channel);
}

void mu_disable_gp_isr(int index)
{
    MUB_BCR &= ~((MU_BCR_GIE0_MASK) << index);
    gp_callback[index] = NULL;
}

void mu_enable_gp_isr(int index, MU_CALLBACK callback, void *data)
{
    _int_disable();
    gp_callback[index] = callback;
    gp_callback_arg[index] = data;
    _int_enable();

    MUB_BCR |= ((MU_BCR_GIE0_MASK) << index);
}
