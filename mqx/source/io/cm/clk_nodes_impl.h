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

#ifndef __clk_nodes_impl_h__
#define __clk_nodes_impl_h__

#include "clk_nodes.h"
#include "core_mutex.h"
#include "bsp_clk_shared_mgmt.h"

// set BSPCFG_CLK_DETAIL_LOGS 0 to disable all not necessary printf, which will
// reduce the code size
#if BSPCFG_CLK_DETAIL_LOGS
#define clk_detail_message(...) printf(__VA_ARGS__)
#else
#define clk_detail_message(...) do {} while (0)
#endif

#define CLK_GATE_DISABLE        0
#define CLK_GATE_ENABLE         1
#define CLK_GATE_QUERY          2
#define OPERATION_SUCCESS       0xA


uint8_t clk_open(P_CLK_NODE_T p_clk);
uint8_t clk_close(P_CLK_NODE_T p_clk);

#endif
