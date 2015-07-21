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
*   Implementation of subset of clock manager API to provide clock frequencies
*   on platforms without full-featured clock manager
*
*       _bsp_get_clock_configuration();
*       _bsp_get_clock();
*
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>
#include <bsp_prv.h>
#include <clk_nodes_impl.h>

/* Clock Manager Errors  */
#define ERR_OK          CM_ERR_OK               /* OK */
#define ERR_SPEED       CM_ERR_SPEED            /* This device does not work in the active speed mode. */
#define ERR_RANGE       CM_ERR_RANGE            /* Parameter out of range. */
#define ERR_VALUE       CM_ERR_VALUE            /* Parameter of incorrect value. */
#define ERR_FAILED      CM_ERR_FAILED           /* Requested functionality or process failed. */
#define ERR_PARAM_MODE  CM_ERR_PARAM_MODE       /* Invalid mode. */

#define CCM_CCSR_SYS_CLK_SEL_PLL1_PFD   0x4     /* select PLL1_PFD as SYS clock root*/

static uint8_t ClockConfigurationID = CPU_CLOCK_CONFIG_0; /* Active clock configuration */

/*
** ===================================================================
**     Method      :  Cpu_GetClockConfiguration
**
**     Description :
**         Returns the active clock configuration identifier.
**     Parameters  : None
**     Returns     :
**         ---             - Active clock configuration identifier
** ===================================================================
*/
uint8_t Cpu_GetClockConfiguration(void)
{
  return ClockConfigurationID;         /* Return the actual clock configuration identifier */
}

/*
** ===================================================================
**     Method      :  Cpu_SetClockConfiguration (component MK60N512MD100)
**
**     Description :
**         Calling of this method will cause the clock configuration
**         change and reconfiguration of all components according to
**         the requested clock configuration setting.
**     Parameters  :
**         NAME            - DESCRIPTION
**         ModeID          - Clock configuration identifier
**     Returns     :
**         ---             - ERR_OK - OK.
**                           ERR_RANGE - Mode parameter out of range
** ===================================================================
*/
uint16_t Cpu_SetClockConfiguration(uint8_t ModeID)
{
  return ERR_OK;
}

BSP_CLOCK_CONFIGURATION _bsp_get_clock_configuration
(
    void
)
{
    return (BSP_CLOCK_CONFIGURATION)Cpu_GetClockConfiguration();
}

uint16_t _bsp_set_clock_configuration(const BSP_CLOCK_CONFIGURATION clock_configuration)
{
    return ERR_OK;
}

static void determine_entry_active(P_CLK_NODE_T entry)
{
    if (!entry->active)
        entry->active = is_physically_open(entry);
}

/*
** ===================================================================
**     Method      :  clock_tree_init
**
**     Description :
**         Initialize the whole clock tree
**     Parameters  : None
**     Returns     : None
** ===================================================================
*/
static void clock_tree_init(void)
{
    int i, ele_cnt;

#if CFG_ENABLE_SHARE_CM
    // semaphore will be used at very early stage
    clk_init_semaphore(CM_SHMEM_SEMAPHORE_NUMBER);
#endif

    ele_cnt = clk_get_nodes_nr();

    for (i=0; i!=ele_cnt; i++) {
        P_CLK_NODE_T entry = clk_get_entry(i);
        entry->parent = get_parent(entry);
        clk_add_child(entry->parent, entry);

        determine_entry_active(entry);

        if (entry->active == 1) {
            clk_inc_descendant(entry->parent);
        }
    }

    // second round, calc freq
    for (i=0; i!=ele_cnt; i++) {
        P_CLK_NODE_T entry = clk_get_entry(i);
        clk_init_node_freq(entry);
    }


#if CFG_ENABLE_SHARE_CM
    // share memory initialization
    clk_init_share_mem();
#endif

    /*
     * These section of code are temperorily commented out.
     * to enable this logic, enable_cnt for every clock node
     * must be properly initialized, so that no clock node
     * in use is closed by accident
     */
    clk_initialize_pre_enable_node();
    // last round, close node whose "enable_cnt" and "descendant_cnt" are both 0
    // but the active is 1
    for (i=0; i!=ele_cnt; i++) {
        P_CLK_NODE_T entry = clk_get_entry(i);
        if ((entry->active == 1) && (entry->descendant_cnt == 0) && (entry->enable_cnt == 0) && (!entry->is_always_open)) {
            clk_close(entry);
        }
    }

    /*
     * This is only for debugging purpose
     */
#if CFG_ENABLE_SHARE_CM && LINUX_PEER_TEST
    clk_init_share_mem_linux();
#endif

    /*
     * For individual nodes, M4 node. only calculate frequency, don't
     * count it into the clock tree
     */
    ele_cnt = clk_get_individual_nodes_nr();
    for (i=0; i!=ele_cnt; i++) {
        P_CLK_NODE_T entry = clk_get_individual_entry(i);
        entry->parent = get_parent(entry);
        clk_init_node_freq(entry);
        entry->parent = NULL;
    }

}

/*
** ===================================================================
**     Method      :  _bsp_clock_manager_init
**
**     Description :
**         Placeholder for clock configuration setting
**     Parameters  : clock configuration
**     Returns     :
**         ---            zero
** ===================================================================
*/
void _bsp_clock_manager_init(void)
{
    /* initialize semaphore */
    // _lwsem_create(&pll4sem, 1);
    clock_tree_init();
}
