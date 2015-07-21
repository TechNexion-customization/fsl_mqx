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
*   This file contains the source to demo the clock api usage
*
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>
#include <fio.h>


#if ! BSPCFG_ENABLE_IO_SUBSYSTEM
#error This application requires BSPCFG_ENABLE_IO_SUBSYSTEM defined non-zero in user_config.h. Please recompile BSP with this option.
#endif


#ifndef BSP_DEFAULT_IO_CHANNEL_DEFINED
#error This application requires BSP_DEFAULT_IO_CHANNEL to be not NULL. Please set corresponding BSPCFG_ENABLE_TTYx to non-zero in user_config.h and recompile BSP with this option.
#endif

#if BSPCFG_CM_LINUX_PEER_WALKAROUND
/*
 * !!!IMPORTANT!!!
 *     MQX and Linux share clock management is not ready at this moment, the
 *     temperory walkaround is to set every node's initial enable_cnt as 1.
 *     When this walkaround is used, we need to manually call clk_disable
 *     for some nodes
 */
#error  !!!IMPORTANT!!! MQX and Linux share clock management is not ready at this moment. The temperory walkaround is to make all node not physically closed. This will make this demo meaningless. To run this demo, make sure the macro BSPCFG_CM_LINUX_PEER_WALKAROUND is set to 0, and A9 side only run U-Boot, Linux OS is not booting. You can modify this macro in user_config.h then rebuild this demo to run. Make sure Linux OS is not booting.
#endif

#if ! BSPCFG_CLK_DETAIL_LOGS
#error This application requires BSPCFG_CLK_DETAIL_LOGS defined non-zero in user_config.h. Please recompile BSP with this option.
#endif


/* Task IDs */
#define CLK_API_DEMO_TASK 5

extern void clkapi_demo_task(uint32_t);


const TASK_TEMPLATE_STRUCT  MQX_template_list[] =
{
    /* Task Index,   Function,   Stack,  Priority, Name,     Attributes,          Param, Time Slice */
    { CLK_API_DEMO_TASK,   clkapi_demo_task, 1500,   8,        "hello",  MQX_AUTO_START_TASK, 0,     0 },
    { 0 }
};

/*TASK*-----------------------------------------------------
*
* Task Name    : clkapi_demo_task
* Comments     :
*    This task demos the basic usage of clock management APIs.
*    For clock_set_freq API, a call to the specified node with
*    only one second paramter 0 will dump the frequency node
*    configuration informatino including
*      - how many config bitfields are there
*      - vaild value range for each of the bitfield
*    A following call with correct number of paramters will
*    successfully set the new frequency. Otherwise the frequency
*    will not change. The dump information will prompt user
*    about this
*
*END*-----------------------------------------------------*/
void clkapi_demo_task
    (
        uint32_t initial_data
    )
{
    void *p_clk1, *p_clk2, *p_clk3;

    /*
     * Clock Enable / Disable Experiment
     *  - UART CLOCK is not a good one for dumping, since printf is a asynchronize
     *    invoker to UART driver. After printf ends, the serial driver may not end
     *    giving out input
     */

    /* 
     * ECSPI Start
     */
    p_clk1 = clock_get(CLK_ECSPI1);
    p_clk2 = clock_get(CLK_ECSPI2);
    p_clk3 = clock_get(CLK_PLL3_60);

    printf("**************************************************************\n");
    printf("* Demo 1 : Open and Close ECSPI Clock Nodes                  *\n");
    printf("*     ECSPI1 and ECSPI2 clock nodes will be enabled then     *\n");
    printf("*   disabled. After each enable/disable operation, each      *\n");
    printf("*   associated clock nodes (ECSPI1/ECSPI2 and their parent   *\n");
    printf("*   PLL3_60) will be dumped. Pay attention on the EN_CNT     *\n");
    printf("*   and ACTIVE_CHILD value. See how a child node's           *\n");
    printf("*   open/close affect its parent's open/close                *\n");
    printf("**************************************************************\n");
    printf("\n");

    printf("[ECSPI] 1. Dump the original clock node status\n");
    // The initial state
    clock_dump(p_clk1);
    clock_dump(p_clk2);
    clock_dump(p_clk3);

    // Enable the first child will also enable the parent
    printf("[ECSPI] 2. Now enable ECSPI1 clock\n");
    clock_enable(p_clk1);
    clock_dump(p_clk1);
    clock_dump(p_clk2);
    clock_dump(p_clk3);

    // Enable the second child
    printf("[ECSPI] 3. Now enable ECSPI2 clock\n");
    clock_enable(p_clk2);
    clock_dump(p_clk1);
    clock_dump(p_clk2);
    clock_dump(p_clk3);

    // Close the first child
    printf("[ECSPI] 4. Now disable ECSPI1 clock\n");
    clock_disable(p_clk1);
    clock_dump(p_clk1);
    clock_dump(p_clk2);
    clock_dump(p_clk3);

    printf("[ECSPI] 5. Now disable ECSPI2 clock\n");
    // Close the second child
    clock_disable(p_clk2);
    clock_dump(p_clk1);
    clock_dump(p_clk2);
    clock_dump(p_clk3);
    /*
     * ECSPI End
     */

    /*
     * SSI start
     */
    printf("\n\n\n");
    printf("**************************************************************\n");
    printf("* Demo 2 : Open and close SSI Clock Nodes                    *\n");
    printf("*     SSI1~3 clock nodes will be enabled and then disabled.  *\n");
    printf("*   This time no clock node dump is performed after each     *\n");
    printf("*   operation. Examine how SSI1~3 open/close affect their    *\n");
    printf("*   parent node PLL3_PFD2                                    *\n");
    printf("**************************************************************\n");
    printf("\n");
    p_clk1 = clock_get(CLK_SSI1);
    p_clk2 = clock_get(CLK_SSI2);
    p_clk3 = clock_get(CLK_SSI3);
    printf("[SSI] 1. Enable SSI1\n");
    clock_enable(p_clk1);
    printf("[SSI] 2. Disable SSI1\n");
    clock_disable(p_clk1);

    printf("[SSI] 3. Enable SSI1\n");
    clock_enable(p_clk1);
    printf("[SSI] 4. Enable SSI2\n");
    clock_enable(p_clk2);
    printf("[SSI] 5. Disable SSI2\n");
    clock_disable(p_clk2);
    printf("[SSI] 6. Enable SSI3\n");
    clock_enable(p_clk3);
    printf("[SSI] 7. Disable SSI1\n");
    clock_disable(p_clk1);
    printf("[SSI] 8. Disable SSI3\n");
    clock_disable(p_clk3);
    /*
     * SSI end
     */

    /*
     * Dump all clock nodes
     */
    printf("\n\n\n");
    printf("**************************************************************\n");
    printf("* Demo 3 : Dump all clock nodes                              *\n");
    printf("*     Finally, we dump all clock nodes managed by M4 core    *\n");
    printf("*   the infomation include each node's open/close status,    *\n");
    printf("*   parent node, child nodes, and the complete clock path    *\n");
    printf("**************************************************************\n");
    printf("\n");
    printf("-------- Dump all clock nodes --------\n");
    clock_dump_all();
}

/* EOF */
