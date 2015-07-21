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
*   This file contains the source for the low power example program.
*
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h> 
#include <fio.h>
#include <mqx_inc.h>
#include "mu.h"

static uint32_t mcore_lpm_state = STATUS_NORMAL_RUN;

static void (*runInRAM)(void) = NULL;

#if defined(__CC_ARM)   /*Patch for DS-5*/
static void ram_wfi(void) __attribute__((section("RAM_FUNC")));
static void ram_wfi_end(void) __attribute__((section("RAM_FUNC_END")));
#else
static void ram_wfi(void);
static void ram_wfi_end(void);
#endif
/*-----------------------------------------------------
 * 
 * Func Name    : init_ram_function
 * Comments     :
 *
 *-----------------------------------------------------*/
static char* init_ram_function(char* func_start, char* func_end)
{
    char *ram_code_ptr;
    _mem_size ram_function_start;

    ram_function_start = (_mem_size)func_start & ~3;
    ram_code_ptr = _mem_alloc_system_align((char*)func_end - (char*)ram_function_start, 4);
    _mem_copy((char*)ram_function_start, ram_code_ptr, (char*)func_end - (char*)ram_function_start);
    ram_code_ptr = (char *)((_mem_size)ram_code_ptr | ((_mem_size)func_start & 3));
    
    return ram_code_ptr;
}

/*-----------------------------------------------------
 * 
 * Func Name    : _io_mcore_lpm_init
 * Comments     :
 *
 *-----------------------------------------------------*/
void _io_mcore_lpm_init(void)
{
    // register MU
    // Here we do not need to enable the interrupt, because the MU message waiting
    // is implemented in the assembly code

    // Prepare the function running in TCM
    runInRAM = (void(*)(void))init_ram_function((char*)ram_wfi, (char*)ram_wfi_end);
}

/*-----------------------------------------------------
 * 
 * Func Name    : _io_mcore_lpm_get_status
 * Comments     :
 *
 *-----------------------------------------------------*/
uint32_t _io_mcore_lpm_get_status(void)
{
    return mcore_lpm_state;
}

/*-----------------------------------------------------
 * 
 * Func Name    : _io_mcore_lpm_set_status
 * Comments     :
 *
 *-----------------------------------------------------*/
void _io_mcore_lpm_set_status(uint32_t status)
{
    mcore_lpm_state = status;
}

/*-----------------------------------------------------
 * 
 * Func Name    : _io_mcore_lpm_register_peer_wakeup
 * Comments     :
 *
 *-----------------------------------------------------*/
uint32_t _io_mcore_lpm_register_peer_wakeup(uint32_t int_no, uint32_t enable)
{
    uint32_t peer_int_no; 
    uint32_t msg; 
    if (clock_query_peer_status() == LINUX_STATUS_NOT_READY)
        return WAKEUP_REGISTER_FAILURE;
    else {
        peer_int_no = int_no + NVIC_GIC_OFFSET;
        msg = REGISTER_PEER_WAKEUP_MASK | (peer_int_no << MSG_FMT_INT_NO_OFFSET) | enable;

        /* wait until the channel is available */
        while (!mu_tx_available(LPM_MCORE_MU_CHANNEL));
        /* transmit the message */
        mu_tx(LPM_MCORE_MU_CHANNEL, msg);
        return WAKEUP_REGISTER_SUCCESS;
    }
}


/*-----------------------------------------------------
 * 
 * Func Name    : _io_mcore_lpm_wfi
 * Comments     :
 *
 *-----------------------------------------------------*/
void _io_mcore_lpm_wfi(void)
{
    // Before run WFI in TCM, ARM Core interrupts are disabled
    // After resume from WFI, ARM Core interrupts are enabled
    // the interrupt should be disabled to make core status query and
    // set atomic
    if (LINUX_STATUS_READY != clock_query_peer_status())
        _ASM_WFI();
    else {
        __disable_interrupt();
        if ((runInRAM != NULL) && (STATUS_LOWPOWER_RUN == _io_mcore_lpm_get_status())) {
            runInRAM();
        } else {
            _ASM_WFI();
        }
        __enable_interrupt();
    }
}

/*
 * The following functions should be copied to TCM in runtime, no subroutines
 * can be called inside ram_wfi. So instead of calling mu_tx, mu_tx_available, direct
 * register operation is performed
 */

/*-----------------------------------------------------
 * 
 * Func Name    : ram_wfi
 * Comments     :
 *
 *-----------------------------------------------------*/
static void ram_wfi(void)
{
    VCORTEX_NVIC_STRUCT_PTR nvic = (VCORTEX_NVIC_STRUCT_PTR)&(((CORTEX_SCS_STRUCT_PTR)CORTEX_PRI_PERIPH_IN_BASE)->NVIC);
    uint8_t mu_int_index;
    uint8_t enable_index, enable_offset, priority_index, priority_offset;
    uint32_t mu_bcr, nvic_mu_enable, nvic_mu_priority;
    uint8_t su_priority = CORTEX_PRIOR(BSP_DEFAULT_MQX_HARDWARE_INTERRUPT_LEVEL_MAX - 1);
    KERNEL_DATA_STRUCT_PTR kernel_data;

    _GET_KERNEL_DATA(kernel_data);

    mu_int_index = INT_MU_M4 - 16;
    enable_index = mu_int_index >> 5;
    enable_offset = mu_int_index & 0x1f;
    priority_index = mu_int_index >> 2;
    priority_offset = (mu_int_index & 0x3) << 3;

    /*
     * 1. Save NVIC MU related register content
     */
    nvic_mu_enable = nvic->ENABLE[enable_index];
    nvic_mu_priority = nvic->PRIORITY[priority_index];
    mu_bcr = MUB_BCR;


    /*
     * 2. Disable all NVIC interrupts except for MU
     */

    /*
     * change BASEPRI to disable all interrupts
     */
    _INT_DISABLE_CODE();

    /*
     * change MU interrupt settings
     */
    nvic->PRIORITY[priority_index] = (nvic_mu_priority & ~(0xFF << priority_offset)) | (su_priority << priority_offset);
    nvic->ENABLE[enable_index] = nvic_mu_enable | (1 << enable_offset);
    MUB_BCR = MU_BCR_RIE0_MASK >> LPM_MCORE_MU_CHANNEL;

    /*
     * Manuually clear pending
     */
    nvic->CLR[mu_int_index >> 5] = 1 << (mu_int_index & 0x1f);

    /*
     * 3. Send LPRUN Request
     */

    /*
     * wait for the previous MU message being read
     */
    while ((MUB_BSR & (MU_BSR_TE0_MASK >> LPM_MCORE_MU_CHANNEL)) == 0);

    /* Inform peer about the low power status */
#if (LPM_MCORE_MU_CHANNEL == 0)
    MUB_BTR0 = STATUS_LPRUN;
#elif (LPM_MCORE_MU_CHANNEL == 1)
    MUB_BTR1 = STATUS_LPRUN;
#elif (LPM_MCORE_MU_CHANNEL == 2)
    MUB_BTR2 = STATUS_LPRUN;
#elif (LPM_MCORE_MU_CHANNEL == 3)
    MUB_BTR3 = STATUS_LPRUN;
#endif

    /* Waiting for B-side event pending clear*/
    while ((MUB_BSR & MU_BSR_EP_MASK) != 0);

    /*
     * 4. WFI which will be only wake up by A9 MU, after wakeup clock
     * has been changed and QSPI, DDR are not available A9 will send
     * a MU to indicate this done
     */
    _ASM_WFI();

    /* 
     * 5. Confirms that A9 send FREQ_READY msg
     */
    for (;;) {
        if ((MUB_BSR & (MU_BSR_RF0_MASK >> LPM_MCORE_MU_CHANNEL)) != 0)
#if (LPM_MCORE_MU_CHANNEL == 0)
            if (MUB_BRR0 == PEER_FREQ_CHANGE_READY)
#elif (LPM_MCORE_MU_CHANNEL == 1)
            if (MUB_BRR1 == PEER_FREQ_CHANGE_READY)
#elif (LPM_MCORE_MU_CHANNEL == 2)
            if (MUB_BRR2 == PEER_FREQ_CHANGE_READY)
#elif (LPM_MCORE_MU_CHANNEL == 3)
            if (MUB_BRR3 == PEER_FREQ_CHANGE_READY)
#endif
                break;
    }

    /*
     * Manuually clear pending
     */
    nvic->CLR[mu_int_index >> 5] = 1 << (mu_int_index & 0x1f);

    /*
     * 6. restore interrupts
     */

    /*
     * restore MU interrupt settings
     */
    MUB_BCR = mu_bcr;
    nvic->ENABLE[enable_index] = nvic_mu_enable;
    nvic->PRIORITY[priority_index] = nvic_mu_priority;

    /*
     * restore BASEPRI
     */
    _INT_ENABLE_CODE();

    /* 
     * 7. WFI, which will be waken up by enabled interrupts
     */
    _ASM_WFI();

    /*
     * 8. disable all interrupts except for MU
     */

    /*
     * change BASEPRI to disable all interrupts
     */
    _INT_DISABLE_CODE();

    /*
     * change MU interrupt settings
     */
    nvic->PRIORITY[priority_index] = (nvic_mu_priority & ~(0xFF << priority_offset)) | (su_priority << priority_offset);
    nvic->ENABLE[enable_index] = nvic_mu_enable | (1 << enable_offset);
    MUB_BCR = MU_BCR_RIE0_MASK >> LPM_MCORE_MU_CHANNEL;

    /*
     * Manuually clear pending
     */
    nvic->CLR[mu_int_index >> 5] = 1 << (mu_int_index & 0x1f);

    /*
     * 9. Wake up, send HI_RUN msg to A9
     */
    while ((MUB_BSR & (MU_BSR_TE0_MASK >> LPM_MCORE_MU_CHANNEL)) == 0);
    /* Inform peer about the high power status */
#if (LPM_MCORE_MU_CHANNEL == 0)
    MUB_BTR0 = STATUS_HIRUN;
#elif (LPM_MCORE_MU_CHANNEL == 1)
    MUB_BTR1 = STATUS_HIRUN;
#elif (LPM_MCORE_MU_CHANNEL == 2)
    MUB_BTR2 = STATUS_HIRUN;
#elif (LPM_MCORE_MU_CHANNEL == 3)
    MUB_BTR3 = STATUS_HIRUN;
#endif

    /* Waiting for B-side event pending clear*/
    while ((MUB_BSR & MU_BSR_EP_MASK) != 0);
    /*
     * 10. WFI which can only be waken up by MU
     */
    _ASM_WFI();

    /* 
     * 11. confirms A9 send RESPONSE_HI msg
     */
    for (;;) {
        if ((MUB_BSR & (MU_BSR_RF0_MASK >> LPM_MCORE_MU_CHANNEL)) != 0) {
#if (LPM_MCORE_MU_CHANNEL == 0)
            if (MUB_BRR0 == PEER_RESPONSE_HI_OK)
#elif (LPM_MCORE_MU_CHANNEL == 1)
            if (MUB_BRR1 == PEER_RESPONSE_HI_OK)
#elif (LPM_MCORE_MU_CHANNEL == 2)
            if (MUB_BRR2 == PEER_RESPONSE_HI_OK)
#elif (LPM_MCORE_MU_CHANNEL == 3)
            if (MUB_BRR3 == PEER_RESPONSE_HI_OK)
#endif
                break;
        }
    }

    /*
     * Manuually clear pending
     */
    nvic->CLR[mu_int_index >> 5] = 1 << (mu_int_index & 0x1f);

    /*
     * 12. restore interrupts
     */

    /*
     * restore MU related registers
     */
    MUB_BCR = mu_bcr;
    nvic->ENABLE[enable_index] = nvic_mu_enable;
    nvic->PRIORITY[priority_index] = nvic_mu_priority;

    /*
     * restore BASEPRI
     */
    _INT_ENABLE_CODE();
}

/*-----------------------------------------------------
 * 
 * Func Name    : ram_wfi_end
 * Comments     : A trick to get the end of ram_wfi
 *
 *-----------------------------------------------------*/
static void ram_wfi_end(void)
{
    return;
}
/* EOF */

