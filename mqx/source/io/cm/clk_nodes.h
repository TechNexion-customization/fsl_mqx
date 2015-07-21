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

#ifndef __clk_nodes_h__
#define __clk_nodes_h__

#include "mqx.h"
#include "stdarg.h"
#include "clk_name.h"

typedef struct clk_node CLK_NODE_T, *P_CLK_NODE_T;

/*
 * This macro is used to enable the clock nodes share memory CM between Linux
 * and MQX. Highly experimental. Should set to 0 at this phase
 */
#define CFG_ENABLE_SHARE_CM          1
/*
 * LINUX_PEER_TEST is only for debugging purpose
 */
#define LINUX_PEER_TEST              0

#if CFG_ENABLE_SHARE_CM
#define SHARE_CLK_MGMT_BASE_ADDRESS     (BSP_SHARED_IRAM_CM_START)
#define CM_SHMEM_SEMAPHORE_NUMBER       (BSP_SHMEM_SEMAPHORE_NUMBER)
#define CLK_CORE_NUMBER                 (_psp_core_num())
#define CLK_SUCCESS                     0
#define CLK_ERR_SEMAPHORE               5
#define MAX_SHARE_NODE_CNT              (BSP_MAX_SHARE_CLOCK_NODE_CNT)

#define SHAREMEM_READY_MAGIC            0x12345678

struct share_node_struct {
    void* linux_pointer;
    void* linux_parent_pointer;
    P_CLK_NODE_T p_clk;
    P_CLK_NODE_T p_parent;
    uint8_t linux_enable;
    uint8_t mqx_enable;
};

typedef struct share_clk_mgmt_struct {
    uint32_t linux_ready_magic;
    uint32_t mqx_ready_magic;
    struct share_node_struct share_node[MAX_SHARE_NODE_CNT];
} SHARE_CLK_MGMT_STRUCT;

void clk_init_share_mem(void);
#endif

typedef enum gate_type {
    GATE_NONE,    /*Root, Inherit*/
    GATE_SINGLE_CG,
    GATE_PLL,
    GATE_PLL_USB, /*PLL USB is different from PLL for it has POWER instead of POWERDOWN bit*/
    GATE_PLL_PFD,
    GATE_OSC
} GATE_TYPE;

typedef struct single_gate_data {
    uint16_t reg_offset;
    uint16_t reg_shift:5;
    uint16_t reg_width:3;
} SINGLE_GATE_DATA, *P_ONE_GATE_DATA;

typedef struct pll_gate_data {
    uint16_t reg_offset;
    uint16_t enable_shift:5;
    uint16_t enable_width:3;
    uint16_t pwr_shift:5;
    uint16_t pwr_width:3;
    uint16_t lock_shift:5;
    uint16_t lock_width:3;
    uint16_t bypass_shift:5;
    uint16_t bypass_width:3;
} PLL_GATE_DATA, *P_PLL_GATE_DATA;

typedef struct pfd_gate_data {
    uint16_t reg_offset;
    uint16_t gate_shift:5;
    uint16_t gate_width:3;
    uint16_t stable_shift:5;
    uint16_t stable_width:3;
} PFD_GATE_DATA, *P_PFD_GATE_DATA;

typedef struct osc_gate_data {
    uint16_t reg_offset;
    uint16_t enable_shift:5;
    uint16_t enable_width:3;
    uint16_t ready_shift:5;
    uint16_t ready_width:3;
} OSC_GATE_DATA, *P_OSC_GATE_DATA;

typedef enum freq_type {
    FREQ_INHERIT,
    FREQ_FIX,
    FREQ_FIX_DIV,
    FREQ_PLL_A,
    FREQ_PLL_B,
    FREQ_PLL_PFD,
    FREQ_SINGLE_DIV,
    FREQ_DUAL_DIV,
} FREQ_TYPE;

typedef struct fix_freq_data {
    uint32_t freq_val;
} FIX_FREQ_DATA, *P_FIX_FREQ_DATA;

typedef struct fix_div_freq_data {
    uint8_t div_val;
} FIX_DIV_FREQ_DATA, *P_FIX_DIV_FREQ_DATA;

typedef struct pll_a_freq_data {
    uint16_t reg_offset;
    uint16_t shift:5;
    uint16_t width:3;
} PLL_A_FREQ_DATA, *P_PLL_A_FREQ_DATA;

typedef struct pll_b_freq_data {
    uint16_t reg_offset;
    uint16_t num_reg_offset;
    uint16_t denom_reg_offset;
    uint16_t shift:5;
    uint16_t width:3;
} PLL_B_FREQ_DATA, *P_PLL_B_FREQ_DATA;

typedef struct pfd_freq_data {
    uint16_t reg_offset;
    uint16_t shift:5;
    uint16_t width:3;
} PFD_FREQ_DATA, *P_PFD_FREQ_DATA;

typedef struct single_div_freq_data {
    uint16_t reg_offset;
    uint16_t shift:5;
    uint16_t width:3;
} SINGLE_DIV_FREQ_DATA, *P_SINGLE_DIV_FREQ_DATA;

typedef struct dual_div_freq_data {
    uint16_t reg_offset;
    uint16_t pred_shift:5;
    uint16_t pred_width:3;
    uint16_t podf_shift:5;
    uint16_t podf_width:3;
} DUAL_DIV_FREQ_DATA, *P_DUAL_DIV_FREQ_DATA;

typedef enum parent_type {
    PARENT_NONE,
    PARENT_FIXED,
    PARENT_ONE_SEL,
    PARENT_TWO_SEL,
} PARENT_TYPE;

// fixed parent
typedef struct fix_parent_data {
    CLOCK_NAME parent;
} FIX_PARENT_DATA, *P_FIX_PARENT_DATA;

// parent selected by single selector
typedef struct single_sel_parent_data {
    CLOCK_NAME* alt_parent;   // pointer to alternate parent array
    uint16_t reg_offset;
    uint16_t shift:5;
    uint16_t width:3;
    uint8_t alt_parent_nr;
} SINGLE_SEL_PARENT_DATA, *P_SINGLE_SEL_PARENT_DATA;

struct clk_node {
    const uint16_t name:8;
    const uint16_t gate_type:3;
    const uint16_t freq_type:3;
    const uint16_t parent_type:2;
    uint16_t active:1;

    uint16_t is_always_open:1;    // for some node, even enable_cnt and descendant_cnt
                                  // are both 0, the clock node can not be closed.
                                  // Obviously, this property is upward inherit. this
                                  // value will only be set at clk node definition, no
                                  // user can change it
    uint16_t freq_valid:1;        // indicate the "freq" field contain a valid frequency value
    char* name_str;
    P_CLK_NODE_T parent;
    P_CLK_NODE_T child_gated;
    P_CLK_NODE_T child_ungated;
    P_CLK_NODE_T sibling;
    const void* gate_data;
    const void* freq_data;
    const void* parent_data;
#if CFG_ENABLE_SHARE_CM
    struct share_node_struct* share_node_pointer;
#endif
    uint32_t freq;      // freq of the current clk node
    uint8_t enable_cnt;
    uint8_t descendant_cnt;
};

void clk_inc_descendant(P_CLK_NODE_T p_clk);
void clk_dec_descendant(P_CLK_NODE_T p_clk);

void clk_add_child(P_CLK_NODE_T parent, P_CLK_NODE_T child);
void clk_remove_child(P_CLK_NODE_T parent, P_CLK_NODE_T child);

int clk_get_nodes_nr(void);
int clk_get_individual_nodes_nr(void);

uint8_t is_clk_opened(P_CLK_NODE_T p_clk);
P_CLK_NODE_T clk_find_node(CLOCK_NAME clk_name);
P_CLK_NODE_T clk_get_entry(int i);
P_CLK_NODE_T clk_get_individual_entry(int i);

uint8_t clk_open(P_CLK_NODE_T p_clk);
uint8_t clk_close(P_CLK_NODE_T p_clk);
uint32_t clk_query_peer_status(void);
P_CLK_NODE_T clk_set_parent(P_CLK_NODE_T p_clk, P_CLK_NODE_T p_parent);
void clk_update_child_freq(P_CLK_NODE_T p_clk, uint8_t old_val, uint8_t new_val);
uint8_t is_physically_open(P_CLK_NODE_T p_clk);
P_CLK_NODE_T get_parent(P_CLK_NODE_T p_clk);

void clk_init_node_freq(P_CLK_NODE_T p_clk);
void clk_initialize_pre_enable_node(void);

#endif
