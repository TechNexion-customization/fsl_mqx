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
*   This file implements share memroy clock management
*
*
*END************************************************************************/

#include "string.h"
#include "mqx.h"
#include "bsp.h"
#include "clk_nodes_impl.h"

#if CFG_ENABLE_SHARE_CM
/*Here we assume every clock node is shared*/
static CLOCK_NAME shared_clock_node_name[] = {
    // CLK_CKIL,
    // CLK_OSC,
    CLK_PLL2,
    CLK_PLL2_PFD0,
    CLK_PLL2_PFD2,
    // CLK_PLL2_PFD2_DIV,
    CLK_PLL3,
    CLK_PLL3_PFD1,
    CLK_PLL3_PFD2,
    CLK_PLL3_PFD3,
    // CLK_PLL3_80,
    // CLK_PLL3_60,
    CLK_PLL4,
    CLK_PLL5,
    // CLK_AHB,
    // CLK_IPG,
    // CLK_PERCLK,
    // CLK_M4,
    CLK_OCRAM,
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
    CLK_QSPI2,
    CLK_SSI1,
    CLK_SSI2,
    CLK_SSI3,
    CLK_UART,
    CLK_UART_IPG,
    CLK_PERIPH2,
    CLK_ALT_OCRAM,
    CLK_I2C1,
    CLK_I2C2,
    CLK_I2C3,
    CLK_I2C4,
    CLK_EPIT1,
    CLK_EPIT2,
};

static CORE_MUTEX_PTR cm_ptr;

static SHARE_CLK_MGMT_STRUCT* clk_share_mem;

static void invalidate_cache(void)
{
    SHM_DCACHE_INVALIDATE_MLINES(&clk_share_mem->linux_ready_magic, sizeof(SHARE_CLK_MGMT_STRUCT)); 
}

static void flush_cache(void)
{
    SHM_DCACHE_FLUSH_MLINES(&clk_share_mem->linux_ready_magic, sizeof(SHARE_CLK_MGMT_STRUCT));
}

int clk_init_semaphore(unsigned int sem_num)
{
    /* Create a core mutex */
    cm_ptr = _core_mutex_create(0, sem_num, MQX_TASK_QUEUE_FIFO);

    if(NULL == cm_ptr) {
        return CLK_ERR_SEMAPHORE;
    }
    else {
        return CLK_SUCCESS;
    }
}

uint8_t clk_get_sharemem_peer_enable(P_CLK_NODE_T p_clk)
{
    struct share_node_struct* p_share_node;
    p_share_node = p_clk->share_node_pointer;
    if (p_share_node == NULL) {
        // This is not a share node, which means we can ignore peer status
        // Another case is its own share memory is not initialized yet. It
        // happens during initialization, in which we can also directly
        // physically open the node
        return SHM_NO_SHARE_NODE;
    }

    invalidate_cache();
    if (clk_share_mem->linux_ready_magic != SHAREMEM_READY_MAGIC) {
        // Should assume peer is on
        return SHM_LINUX_NOT_READY;
    }
    else {
        return p_share_node->linux_enable ? SHM_LINUX_PEER_ON : SHM_LINUX_PEER_OFF;
    }
}

void clk_update_sharemem_enable(P_CLK_NODE_T p_clk, uint8_t enable)
{
    struct share_node_struct* p_share_node;
    p_share_node = p_clk->share_node_pointer;
    if (p_share_node != NULL) {
        invalidate_cache();
        p_share_node->mqx_enable = enable;
        flush_cache();
    }
}

int clk_get_semaphore(void)
{
    _int_disable();
    /*Wait until the semaphore is retrieved*/
    while (_core_mutex_trylock(cm_ptr) != COREMUTEX_LOCKED);
    return CLK_SUCCESS;
}

int clk_release_semaphore(void)
{
    _core_mutex_unlock(cm_ptr);
    _int_enable();
    return CLK_SUCCESS;
}

void clk_init_share_mem(void)
{
    uint8_t share_cnt = sizeof(shared_clock_node_name) / sizeof(CLOCK_NAME);
    uint8_t i;
    P_CLK_NODE_T p_clk;
    struct share_node_struct* p_cur;
    clk_share_mem = (SHARE_CLK_MGMT_STRUCT *)SHARE_CLK_MGMT_BASE_ADDRESS;

    /*
     * Share memory lock
     */
    clk_get_semaphore();

    invalidate_cache();

    for (i=0; i!=share_cnt; i++) {
        // Name and Parent Name
        p_cur = &clk_share_mem->share_node[i];
        p_clk = (P_CLK_NODE_T)clock_get(shared_clock_node_name[i]);
        p_cur->p_clk = p_clk; 
        p_cur->p_parent = p_clk->parent;
        
        // Enablement
        p_cur->mqx_enable = p_clk->active;

        // Make a Link
        p_clk->share_node_pointer = p_cur;
    }


    // Mark MQX Init OK
    clk_share_mem->mqx_ready_magic = SHAREMEM_READY_MAGIC;

    flush_cache();

    /*
     * Share memory unlock
     */
    clk_release_semaphore();

}

uint32_t clk_check_sharemem_peer_magic(void)
{
    static uint32_t result = LINUX_STATUS_NOT_READY;

    if (result != LINUX_STATUS_READY) {
        /*
         * check share memory region to see if linux has fill the magic number
         */
        clk_get_semaphore();
        invalidate_cache();
        result = (clk_share_mem->linux_ready_magic == SHAREMEM_READY_MAGIC) ?  LINUX_STATUS_READY : LINUX_STATUS_NOT_READY;
        clk_release_semaphore();
    }
    return result;
}

/*
 * "clk_init_share_mem_linux" is only for debugging purpose
 */
void clk_init_share_mem_linux(void)
{
    uint8_t share_cnt = sizeof(shared_clock_node_name) / sizeof(CLOCK_NAME);
    clk_share_mem = (SHARE_CLK_MGMT_STRUCT *)SHARE_CLK_MGMT_BASE_ADDRESS;
    uint8_t i;
    struct share_node_struct* p_cur;

    clk_get_semaphore();
    clk_share_mem->linux_ready_magic = SHAREMEM_READY_MAGIC;
    for (i=0; i!=share_cnt; i++) {
        p_cur = &clk_share_mem->share_node[i];
        p_cur->linux_enable = 0;
    }

    clk_release_semaphore();
}
#endif

