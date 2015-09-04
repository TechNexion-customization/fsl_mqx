/*HEADER**********************************************************************
*
* Copyright 2013-2014 Freescale Semiconductor, Inc.
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
*   Header file for vybrid clock nodes implementation
*
*
*END************************************************************************/

#ifndef __bsp_clk_shared_mgmt_h__
#define __bsp_clk_shared_mgmt_h__

#define SHM_LINUX_PEER_OFF      0
#define SHM_LINUX_PEER_ON       1
#define SHM_NO_SHARE_NODE       2
#define SHM_LINUX_NOT_READY     3

#define SHM_DCACHE_INVALIDATE_MLINES(p, m) _DCACHE_INVALIDATE_MLINES(p, m)
#define SHM_DCACHE_FLUSH_MLINES(p, m)      _DCACHE_FLUSH_MLINES(p, m)

void clk_init_share_mem(void);
int clk_init_semaphore(unsigned int sem_num);
int clk_get_semaphore(void);
int clk_release_semaphore(void);
void clk_update_sharemem_enable(P_CLK_NODE_T p_clk, uint8_t enable);
uint8_t clk_get_sharemem_peer_enable(P_CLK_NODE_T p_clk);
void clk_init_share_mem_linux(void);
uint32_t clk_check_sharemem_peer_magic(void);
#endif
