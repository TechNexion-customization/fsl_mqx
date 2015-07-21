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
*   This file contains flash boot code to initialize chip selects,
*   disable the watchdog timer and initialize the PLL.
*
*
*END************************************************************************/

#include "mqx.h"
#include "bsp.h"
#include "bsp_prv.h"

typedef struct {
    uint32_t start;
    uint32_t end;
} rdc_memory_region;

/* RDC configuration */
static uint8_t rdc_peripheral_m4[] = {
    RDC_PDAP_UART2_ID,
    RDC_PDAP_I2C3_ID,
    RDC_PDAP_ECSPI4_ID,
    RDC_PDAP_ECSPI5_ID,
    RDC_PDAP_ADC1_ID,
    RDC_PDAP_ADC2_ID,
    RDC_PDAP_CAN1_ID,
    RDC_PDAP_CAN2_ID,
    RDC_PDAP_EPIT1_ID,
    RDC_PDAP_EPIT2_ID,
    RDC_PDAP_WDOG3_ID
};

static uint8_t mmdc_idx, qspi1_idx, qspi2_idx, ocram_idx;

static rdc_memory_region rdc_memory_m4[] = {
    { (uint32_t)__VECTOR_TABLE_ROM_START, (uint32_t)__ROM_END }
};

static volatile uint8_t *rdc_get_sema42(uint32_t pdap)
{
    RDC_MemMapPtr           rdc       = RDC_BASE_PTR;
    RDC_SEMAPHORE_MemMapPtr rdc_semi1 = RDC_SEMAPHORE1_BASE_PTR;
    RDC_SEMAPHORE_MemMapPtr rdc_semi2 = RDC_SEMAPHORE2_BASE_PTR;

    volatile uint8_t *sema42 = NULL;

    /* lock required */
    if (pdap < ELEMENTS_OF(rdc->PDAP) && (rdc->PDAP[pdap] & 0x40000000))
    {
        if (pdap < sizeof(rdc_semi1->GATE)/sizeof(rdc_semi1->GATE[0]))
            sema42 = &rdc_semi1->GATE[pdap];
        else
            sema42 = &rdc_semi2->GATE[pdap - ELEMENTS_OF(rdc_semi1->GATE)];
    }

    return sema42;
}

static void rdc_init_memory(uint32_t start, uint32_t end, uint32_t value)
{
    RDC_MemMapPtr rdc = RDC_BASE_PTR;

    if (start >= 0x80000000)
    {  /* MMDC */
        rdc->MR[RDC_MR_MMDC_0_ID + mmdc_idx].MRSA = start & 0xFFFFF000;
        rdc->MR[RDC_MR_MMDC_0_ID + mmdc_idx].MREA = (end + 0xFFF) & 0xFFFFF000;
        rdc->MR[RDC_MR_MMDC_0_ID + mmdc_idx].MRC = value;
        mmdc_idx++;
    }
    else if (start >= 0x70000000)
    {  /* QSPI2 */
        rdc->MR[RDC_MR_QSPI2_0_ID + qspi2_idx].MRSA = start & 0xFFFFF000;
        rdc->MR[RDC_MR_QSPI2_0_ID + qspi2_idx].MREA = (end + 0xFFF) & 0xFFFFF000;
        rdc->MR[RDC_MR_QSPI2_0_ID + qspi2_idx].MRC = value;
        qspi2_idx++;
    }
    else if (start >= 0x60000000)
    {  /* QSPI1 */
        rdc->MR[RDC_MR_QSPI1_0_ID + qspi1_idx].MRSA = start & 0xFFFFF000;
        rdc->MR[RDC_MR_QSPI1_0_ID + qspi1_idx].MREA = (end + 0xFFF) & 0xFFFFF000;
        rdc->MR[RDC_MR_QSPI1_0_ID + qspi1_idx].MRC = value;
        qspi1_idx++;
    }
    else if (start >= 0x00900000 && start < 0x00920000)
    {  /* OCRAM */
        rdc->MR[RDC_MR_OCRAM_0_ID + ocram_idx].MRSA = start & 0xFFFFFF80;
        rdc->MR[RDC_MR_OCRAM_0_ID + ocram_idx].MREA = (end + 0x7F) & 0xFFFFFF80;
        rdc->MR[RDC_MR_OCRAM_0_ID + ocram_idx].MRC = value;
        ocram_idx++;
    }
}

/* clock initialization */
static void clocks_init(void)
{
    // select can clock is derived from OSC clock(24M)
    CCM_CSCMR2 &= ~0x300;
    CCM_CSCMR2 &= ~0xFC;
    CCM_CSCMR2 |= 0x100;

    //select uart clock is derived from OSC clock(24M)
    CCM_CSCDR1 &= ~0x3F;
    CCM_CSCDR1 |= 0x40;

    //select i2c and epit clock is derived from OSC clock(24M)
    CCM_CSCMR1 &= ~0x3F;
    CCM_CSCMR1 |= 0x40;

    //select spi colck is derived from OSC clock (24M)
    CCM_CSCDR2 &= ~0x01F80000;
    CCM_CSCDR2 |= 0x00040000;
}

/* memory initialization */
static void mem_init(void)
{
    _ICACHE_ENABLE(0);
    _DCACHE_ENABLE(0);
}

/*FUNCTION*---------------------------------------------------------------------
*
* Function Name    : init_hardware
* Returned Value   : void
* Comments         :
*   Initialize device.
*
*END*-------------------------------------------------------------------------*/

void init_hardware(void)
{
    /*
     * Walkaround : For some boards, the reset button will not clear
     * the share memory content. MQX will do a manual share memory
     * clean up in the very first begining
     */
    memset(BSP_SHARED_IRAM_CM_START, 0, sizeof(uint64_t));

    /* init clock */
    clocks_init();

    /* init memory */
    mem_init();

    /* disable watchdog */
    _bsp_watchdog_disable_powerdown();

    /* gpio init */
    _bsp_gpio_io_init();
}

/*FUNCTION*---------------------------------------------------------------------
*
* Function Name    : _bsp_watchdog_disable
* Returned Value   : void
* Comments         :
*   Disable watchdog powerdown timer
*
*END*-------------------------------------------------------------------------*/

void _bsp_watchdog_disable_powerdown(void)
{
    WDOG_MemMapPtr wdog = WDOG3_BASE_PTR;

    /* disable power down watchdog */
    wdog->WMCR = 0;
}

/*FUNCTION*---------------------------------------------------------------------
*
* Function Name    : _bsp_soft_reset
* Returned Value   : void
* Comments         :
*   Reset the M4 core
*
*END*-------------------------------------------------------------------------*/

void _bsp_soft_reset(void)
{
    SRC_MemMapPtr src = SRC_BASE_PTR;

    src->SCR = src->SCR | 0x1008; /* reset M4 platform and core */
}

/*FUNCTION*---------------------------------------------------------------------
*
* Function Name    : _bsp_watchdog_start
* Returned Value   : void
* Comments         :
*   Start watchdog for M4 core
*
*END*-------------------------------------------------------------------------*/

void _bsp_watchdog_start(uint8_t timeout)
{
    WDOG_MemMapPtr wdog = WDOG3_BASE_PTR;

    wdog->WCR = (wdog->WCR & 0xFF) | ((uint16_t)timeout << 8) | 4;
}

/*FUNCTION*---------------------------------------------------------------------
*
* Function Name    : _bsp_watchdog_service
* Returned Value   : void
* Comments         :
*   Feed the watchdog for M4 core
*
*END*-------------------------------------------------------------------------*/

void _bsp_watchdog_service(void)
{
    WDOG_MemMapPtr wdog = WDOG3_BASE_PTR;

    wdog->WSR = 0x5555;
    wdog->WSR = 0xAAAA;
}

/*FUNCTION*---------------------------------------------------------------------
*
* Function Name    : _bsp_rdc_init
* Returned Value   : void
* Comments         :
*   Initialize RDC and lock the values
*
*END*-------------------------------------------------------------------------*/

void _bsp_rdc_init(void)
{
    RDC_MemMapPtr rdc = RDC_BASE_PTR;
    uint32_t i;

    /* first move M4 core to RDC domain 1, and keep others unchanged */
    rdc->MDA[RDC_MDA_M4_CORE_ID] = 0x00000001;

    for (i = 0; i < ELEMENTS_OF(rdc_peripheral_m4); i++)
    {
        /* assign proprietary peripherals to RDC domain 1 only */
        rdc->PDAP[rdc_peripheral_m4[i]] = 0x0000000C;
    }

    mmdc_idx = qspi1_idx = qspi2_idx = ocram_idx = 0;
    for (i = 0; i < ELEMENTS_OF(rdc_memory_m4); i++)
    {
        /* assign proprietary memory to RDC domain 1 only */
        rdc_init_memory(rdc_memory_m4[i].start, rdc_memory_m4[i].end, 0x4000000C);
    }
}

/*FUNCTION*---------------------------------------------------------------------
*
* Function Name    : _bsp_rdc_sema42_lock
* Returned Value   : void
* Comments         :
*   Lock the SEMA42 to access the peripheral
*
*END*-------------------------------------------------------------------------*/

void _bsp_rdc_sema42_lock(uint32_t pdap)
{
    volatile uint8_t *sema42;

    sema42 = rdc_get_sema42(pdap);
    if (sema42)
    {
        do {
            *sema42 = RDC_SEMAPHORE_GATE_GTFSM(6);
        } while ((*sema42 & RDC_SEMAPHORE_GATE_GTFSM_MASK) != RDC_SEMAPHORE_GATE_GTFSM(6));
    }
}

/*FUNCTION*---------------------------------------------------------------------
*
* Function Name    : _bsp_rdc_sema42_unlock
* Returned Value   : void
* Comments         :
*   Unlock the SEMA42 to access the peripheral
*
*END*-------------------------------------------------------------------------*/

void _bsp_rdc_sema42_unlock(uint32_t pdap)
{
    volatile uint8_t *sema42;

    sema42 = rdc_get_sema42(pdap);
    if (sema42 && (*sema42 & RDC_SEMAPHORE_GATE_GTFSM_MASK) == RDC_SEMAPHORE_GATE_GTFSM(6))
        *sema42 = 0;
}
