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
*   This file implement the clock tree management APIs for vybrid
*
*
*END************************************************************************/
#include "mqx.h"
#include "bsp.h"
#include "clk_nodes_impl.h"
#include "fio.h"
#include "clk_api.h"

#define ENABLE_CNT_WARING_THREAD 50
#define FREQ_1MHZ 1000000
#define FREQ_1KHZ 1000

#if BSPCFG_CLK_DETAIL_LOGS
static void dump_clock_childs(P_CLK_NODE_T p_clk);
static void dump_clock_path(P_CLK_NODE_T pclk);
#endif

/*!
 * \brief This function get the clock node pointer by its name
 *
 * The name of a clock node is defined as an enumeration type - CLOCK_NAME,
 * which can be found in "bsp_clk_name.h". The function searches through the
 * system clock node array "clk_table" defined in "bsp_clk_nodes.c" to find
 * a matching entry. The entry pointer is returned. It can be then used as
 * a handle to manipulate the clock node
 *
 * \param[in] clk_name clock node enumeration name
 *
 * \return pointer to corresponding clock node entry
 * \return NULL if no corresponding node is found
 *
 */
void* clock_get(CLOCK_NAME clk_name)
{
    P_CLK_NODE_T p_clk = clk_find_node(clk_name);
    if (p_clk == NULL)
        printf("No node number <%d> is found\n", clk_name);
    return (void*)p_clk;
}

#if BSPCFG_CLK_DETAIL_LOGS
/*!
 * \brief This function get the clock node name by its handle
 *
 * The clock node names are defined in "clock_names" in "bsp_clk_nodes.c".
 * This function retrieves it using the clock node handler obtained eariler.
 * It is used for infromation dump purpose only
 *
 * \param[in] pclk handle to the clock node which is obtained in "clock_get"
 *
 * \return clock node name string for a valid node
 * \return string "BAD_NODE" if not a valid node
 *
 */
char* clock_get_name(void* pclk)
{
    P_CLK_NODE_T p_clk = (P_CLK_NODE_T)pclk;
    return p_clk->name_str;
}
#endif

/*!
 * \brief This function enable/ungate the clock node by its handle
 *
 * This function is used to enable/ungate a clock node. The user don't need
 * to care about the clock tree hierarchy. He can just call "clock_enable"
 * for the end node he is interested in. the clock tree managment driver
 * will make sure all parent node be enabled/ungated too
 *
 * \param[in] pclk handle to the clock node which is obtained in "clock_get"
 *
 * \return NA
 *
 */
void clock_enable(void* pclk)
{
    P_CLK_NODE_T p_clk = (P_CLK_NODE_T)pclk;
    if (p_clk == NULL) {
        printf("%s : NULL clk node, do nothing\n", __FUNCTION__);
        return;
    }

    _int_disable();

    p_clk->enable_cnt++;

    if (p_clk->enable_cnt > ENABLE_CNT_WARING_THREAD) {
        clk_detail_message("warning : enable cnt of %s is too much, pls check\n", p_clk->name_str);
    }

    if (!p_clk->active) {
        clk_open(p_clk);
    }

    _int_enable();
}

/*!
 * \brief This function disable/gate the clock node by its handle
 *
 * This function is used to disable/gate a clock node. The user don't need
 * to care about the clock tree hierarchy. He can just call "clock_disable"
 * for the end node he is interested in. the clock tree managment driver
 * will make sure all parent node which has no other enabled/ungated child
 * node be disabled/gated too. This will save a lot of effort for maintaining
 * system level power efficiency
 *
 * \param[in] pclk handle to the clock node which is obtained in "clock_get"
 *
 * \return NA
 *
 */
void clock_disable(void* pclk)
{
    P_CLK_NODE_T p_clk = (P_CLK_NODE_T)pclk;
    if (p_clk == NULL) {
        printf("%s : NULL clk node, do nothing\n", __FUNCTION__);
        return;
    }

    if (p_clk->enable_cnt == 0) {
        clk_detail_message("error : can not call disable to clk node %s, not be enabled yet\n", p_clk->name_str);
        return;
    }

    _int_disable();

    p_clk->enable_cnt--;

    if ((p_clk->enable_cnt == 0) && (p_clk->descendant_cnt == 0) && (!p_clk->is_always_open)) {
        clk_close(p_clk);
    }

    _int_enable();
}

/*!
 * \brief This function gets the peer core status
 *
 * The function checks the peer cores's magic number. If it has been there,
 * the function will return 1, meaning peer core has finish its initialization;
 * if it is not there, the function will return 0, meaning A9 has not finish
 * its initialization
 *
 * \param none
 *
 * \return 1 peer is ready; 0 peer is not ready
 *
 */
uint32_t clock_query_peer_status(void)
{
    return clk_query_peer_status();
}

/*!
 * \brief This function gets a specified clock node's parent node handler
 *
 * The function simply returns the specified node's parent node handler. If
 * the specifed node is a top level one with no parent, NULL will be returned
 *
 * \param[in] pclk handle to the clock node which is obtained in "clock_get"
 *
 * \return parent node's handler
 *
 */
void* clock_get_parent(void* pclk)
{
    P_CLK_NODE_T p_clk = (P_CLK_NODE_T)pclk;
    P_CLK_NODE_T p_parent = p_clk->parent;
    return p_parent;
}

/*!
 * \brief This function set a specified node's alternative parent
 *
 * For clock node which have alternative parents, the function will configure
 * for a new parent to it, the old parent handle is returned for maitain
 * purpose. If the new parent specified by "pclkparent" is not valid, or there
 * is no alternative parent for the node, the function does nothing 
 * and return a NULL pointer.
 *
 * \param[in] pclk handle to the clock node which is obtained in "clock_get"
 * \param[in] pclkparent handle to the clock node alternative parent
 *
 * \return old parent handle if a new parent node is set successfully
 * \return NULL if a new parent node is not successfully set
 *
 */
void* clock_set_parent(void* pclk, void* pclkparent)
{
    P_CLK_NODE_T p_clk = (P_CLK_NODE_T)pclk;
    P_CLK_NODE_T p_clk_parent = (P_CLK_NODE_T)pclkparent;
    P_CLK_NODE_T p_old_parent = NULL;

    if (p_clk == NULL) {
        printf("%s : NULL clk node, do nothing\n", __FUNCTION__);
        return NULL;
    }

    if (p_clk_parent == NULL) {
        printf("%s : NULL parent clk node, do nothing\n", __FUNCTION__);
        return NULL;
    }

    p_old_parent = clk_set_parent(p_clk, p_clk_parent);
    return (void*)p_old_parent;
}

/*!
 * \brief This function get the specified node's frequency value
 *
 * The function returns the specified node's frequency value as a 32 bit
 * integer.
 *
 * \param[in] pclk handle to the clock node which is obtained in "clock_get"
 *
 * \return specified node's frequency value
 *
 */
uint32_t clock_get_freq(void* pclk)
{
    P_CLK_NODE_T p_clk = (P_CLK_NODE_T)pclk;

    if (!p_clk->freq_valid) {
        clk_detail_message("%s : WARNING, %s's freq is not valid\n", __FUNCTION__, p_clk->name_str);
        return 0;
    }

    return p_clk->freq;
}

/*!
 * \brief This function set the specified node's frequency
 *
 * Some clock nodes have one or more frequency configuration registers,
 * such as divider and multipliers. The first parameter for this function
 * is always the clock node handler. For each node, a call to "clock_set_freq"
 * can be first called with only a second parameter 0. This will dump
 * information about the available frequency config fields for this node and
 * the valid value range for each of them. Then he can call "clock_set_freq"
 * again with the second parameter set to number of available bitfields,
 * followed by value to written to each of these fields.
 * If a new frequency is set successfully, BSP_CLKAPI_SETFREQ_OK will be
 * returned. If it is not the case for some reason (wrong paramter, same
 * divider value, etc), BSP_CLKAPI_SETFREQ_NO_CHANGE will be returned
 *
 * Code Example:
 *     qspiclk = clock_get(CLK_QSPI0);
 *     clock_set_freq(qspiclk, 0); // the dump information will indicate
 *                                 // that 3 bitfields are avaliable for
 *                                 // QSPI nodes, and tell the user valid
 *                                 // value range for each of them.
 *
 *     clock_set_freq(qspiclk, 3, 0, 1, 2); // a correct call
 *     clock_set_freq(qspiclk, 2, 0, 1);    // wrong parameter number
 *                                          // BSP_CLKAPI_SETFREQ_NO_CHANGE
 *                                          // will be returned 
 *
 * \param[in] pclk handle to the clock node which is obtained in "clock_get"
 * \param[in] varable length parameters, see function descriptions
 *
 * \return BSP_CLKAPI_SETFREQ_OK if a new frequency is set succussfully
 * \return BSP_CLKAPI_SETFREQ_NO_CHANGE if the frequency set fails
 *
 */
uint8_t clock_set_freq(void* pclk, ...)
{
#if 1
    /*
     * This feature is temperorily disabled, we will add it back depending on
     * real use cases
     */
    return BSP_CLKAPI_SETFREQ_NO_CHANGE;
#else
    P_CLK_NODE_T p_clk = (P_CLK_NODE_T)pclk;
    va_list args;
    uint8_t old_val, new_val;
    uint8_t result;
    va_start(args, pclk);
    if (p_clk->set_freq && p_clk->set_freq(p_clk, args, &old_val, &new_val)) {
        // set freq success, update child freq
        clk_update_child_freq(p_clk, old_val, new_val);
        result = BSP_CLKAPI_SETFREQ_OK;
    }
    else
        result = BSP_CLKAPI_SETFREQ_NO_CHANGE;
    va_end(args);
    return result;
#endif
}

#if BSPCFG_CLK_DETAIL_LOGS
/*!
 * \brief This function dumps the specifed clock node's information
 *
 * The function dumps information of a specified clock node, including
 *  - clock name
 *  - clok enable/disable (gate/ungate) status
 *  - frequency
 *  - parent node
 *  - alternative parent nodes
 *  - number of times the clock node is software enabled 
 *      "clock_enable" - "clock_disable" times
 *  - number of active (enabled/ungated) child
 *  - all childs nodes and their enable/disable status
 *  - node's path, which is its parent and grand-parent, up to the top
 *    level one
 *
 * \param[in] pclk handle to the clock node which is obtained in "clock_get"
 *
 * \return 
 *
 */
void clock_dump(void* pclk)
{
    P_CLK_NODE_T p_clk=(P_CLK_NODE_T)pclk;
    uint8_t active;
    if (p_clk == NULL) {
        printf("%s : NULL clk node, do nothing\n", __FUNCTION__);
        return;
    }

    printf("-------------------------------\n");
    /*
     * printf may cause some clock nodes being enable/disable, which will affect
     * some of the dumping contents. So before calling printf, we use some
     * variable to store some content
     */
    active = p_clk->active;
    printf("CLK <%s> : <%s> [%s]", p_clk->name_str, active?"OPEN":"CLOSE", p_clk->is_always_open?"ALWAYS-ON":"NORMAL");
    if (p_clk->freq > FREQ_1MHZ)
        printf(" %dMHz\n", (p_clk->freq + FREQ_1MHZ / 2) / FREQ_1MHZ);
    else
        printf(" %dKHz\n", (p_clk->freq + FREQ_1KHZ / 2) / FREQ_1KHZ);
    printf("    - PARENT              : <%s>\n", p_clk->parent == NULL ? "NULL" : p_clk->parent->name_str);
    printf("    - EN_CNT/ACTIVE_CHILD : %d/%d\n", p_clk->enable_cnt, p_clk->descendant_cnt);
    printf("    - CHILDS :\n");
    dump_clock_childs(p_clk);
    printf("    - PATH :\n");
    dump_clock_path(p_clk);
#if 1
    /*
     * This feature is temperorily disabled, we will add it back depending on
     * real use cases
     */
#else
    if (p_clk->dump_possible_parent) {
        printf("    - ALTERNATIVE PARENT :\n");
        printf("        ");
        p_clk->dump_possible_parent();
    }
#endif
    printf("-------------------------------\n");
}

void clock_dump_simple(void* pclk)
{
    P_CLK_NODE_T p_clk=(P_CLK_NODE_T)pclk;
    uint8_t active;
    active = p_clk->active;
    printf("<%15s> : <%5s>", p_clk->name_str, active?"OPEN":"CLOSE");
    printf(" : PARENT <%15s>", p_clk->parent == NULL ? "NULL" : p_clk->parent->name_str);
    printf(" : EN/SON [%d/%d]\n", p_clk->enable_cnt, p_clk->descendant_cnt);
}

int clock_get_nodes_cnt()
{
    return clk_get_nodes_nr();
}

/*!
 * \brief This function call "clock_dump" for all clock nodes in system
 *
 * the system clock nodes are defined in "clk_table" defined in
 * "bsp_clk_nodes.c". This fuctino iterate it and call "clock_dump" for
 * each of them. Thus dumps all the system level clock tree information
 *
 * \param[in] pclk handle to the clock node which is obtained in "clock_get"
 *
 * \return 
 *
 */
void clock_dump_all()
{
    int i;
    int ele_cnt = clk_get_nodes_nr();

    printf("------------------------------------------------------------\n");
    printf("Total %d nodes\n", ele_cnt);
    for (i=0; i!=ele_cnt; i++) {
        printf("No.%d\n", i);
        clock_dump(clk_get_entry(i));
    }
    printf("------------------------------------------------------------\n");
}

void clock_dump_all_simple()
{
    int i;
    int ele_cnt = clk_get_nodes_nr();

    printf("------------------------------------------------------------\n");
    for (i=0; i!=ele_cnt; i++) {
        printf("[%2d] : ", i);
        clock_dump_simple(clk_get_entry(i));
    }
    printf("------------------------------------------------------------\n");
}

/*
 * Local tool function
 */
static void dump_clock_childs(P_CLK_NODE_T p_clk)
{
    uint8_t idx = 0;
    P_CLK_NODE_T child = p_clk->child_gated;

    printf("\tGated :");
    if (child == NULL)
        printf("\t\tNA\n");
    else {
        printf("\t\t");
        do {
            printf("(%s)%-18s", child->active?"O":"C", child->name_str);
            child = child->sibling;
            if (++idx == 8) {
                idx = 0;
                printf("\n\t\t");
            }
        } while (child != NULL);
        printf("\n");
    }

    idx = 0;
    child = p_clk->child_ungated;
    printf("\tUngated :");
    if (child == NULL)
        printf("\tNA\n");
    else {
        printf("\t");
        do {
            printf("(%s)%-18s", child->active?"O":"C", child->name_str);
            child = child->sibling;
            if (++idx == 8) {
                idx = 0;
                printf("\n\t\t");
            }
        } while (child != NULL);
        printf("\n");
    }
}

static void dump_clock_path(P_CLK_NODE_T pclk)
{
    P_CLK_NODE_T node;
    if (pclk->parent == NULL)
        printf("\tNA\n");
    else {
        printf("\t");
        for (node = pclk->parent; node != NULL; node = node->parent)
            printf("(%s)%-18s", node->active?"O":"C", node->name_str);
        printf("\n");
    }
}
#else
void clock_dump(void* pclk) {}
void clock_dump_all() {}
#endif
