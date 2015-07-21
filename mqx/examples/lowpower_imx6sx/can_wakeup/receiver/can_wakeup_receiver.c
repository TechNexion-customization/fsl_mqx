/*HEADER**********************************************************************
*
* Copyright 2008-2014 Freescale Semiconductor, Inc.
* Copyright 1989-2008 ARC International
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
*   This file contains the source for the FlexCAN wake up receiver program.
*
*END************************************************************************/


#include <mqx.h>
#include <bsp.h>
#include <fsl_flexcan_driver.h>

#include "../can_wakeup.h"

#if ! BSPCFG_ENABLE_IO_SUBSYSTEM
#error This application requires BSPCFG_ENABLE_IO_SUBSYSTEM defined non-zero in user_config.h. Please recompile BSP with this option.
#endif


#ifndef BSP_DEFAULT_IO_CHANNEL_DEFINED
#error This application requires BSP_DEFAULT_IO_CHANNEL to be not NULL. Please set corresponding BSPCFG_ENABLE_TTYx to non-zero in user_config.h and recompile BSP with this option.
#endif

static flexcan_config_t flexcan_data = {
/* The number of Message Buffers needed        */ 16,
/* The maximum number of Message Buffers       */ 16,
/* The number of total RXIMR registers         */ 64,
/* The number of RX FIFO ID filters needed     */ kFlexCanRxFifoIDFilters_8,
/* RX fifo needed?                             */ FALSE,
/* RX message buffer needed?                   */ TRUE
};

static uint32_t RX_identifier;
static uint32_t RX_mailbox_num;
static flexcan_mb_t rx_mb;
static flexcan_mb_t rx_fifo;

static void Main_Task(uint32_t parameter);

/* Task template list */
TASK_TEMPLATE_STRUCT MQX_template_list[] = {
    { 5, Main_Task, 1000L, 8L, "Main task", MQX_AUTO_START_TASK},
    { 0L, 0L, 0L, 0L, 0L, 0L }
};

static void recv_message(uint8_t instance)
{
    bool is_rx_mb_data;
    bool is_rx_fifo_data;
    uint32_t result, temp;

    result = flexcan_start_receive(instance, &flexcan_data, RX_mailbox_num, RX_identifier, 1,
                                  &is_rx_mb_data, &is_rx_fifo_data, &rx_mb, &rx_fifo);
    if (result)
        printf("FLEXCAN receive error. Error code: 0x%lx\n", result);
    else
    {
        if (is_rx_mb_data)
        {
            temp = ((rx_mb.cs) >> 16) & 0xF;
            printf("DLC=%d, mb_idx=%d\n", temp, RX_mailbox_num);
            printf("RX MB data: 0x");
            for (result = 0; result < temp; result++)
                printf ("%02x ", rx_mb.data[result]);

            printf("\nID: 0x%x\n", rx_mb.msg_id);
        }
    }
}
/*TASK*-----------------------------------------------------------
*
* Task Name : Main_Task
* Comments :
*
*
*END*-----------------------------------------------------------*/
static void Main_Task(uint32_t parameter)
{
    uint32_t result;
    uint8_t  instance;
    flexcan_mb_code_status_rx_t rx_cs1;

    rx_cs1.code = kFlexCanRX_Ranswer;
    rx_cs1.msg_id_type = kFlexCanMbId_Std;
    rx_cs1.data_length = 1;
    rx_cs1.substitute_remote = 0;
    rx_cs1.remote_transmission = 0;
    rx_cs1.local_priority_enable = 0;
    rx_cs1.local_priority_val = 0;

    instance = CAN_DEVICE;

    printf("\n*********FLEXCAN WAKE UP PROGRAM.*********");
    printf("\n   Message format: Standard (11 bit id)");
    printf("\n   Message buffer %d used for Rx.", CAN_RX_MAILBOX);
    printf("\n   Interrupt Mode: Enabled");
    printf("\n   Operation Mode: RX --> Normal");
    printf("\n***************************************\n\n");

    /* Select mailbox number */
    RX_mailbox_num = CAN_RX_MAILBOX;
    RX_identifier = CAN_IDENTIFIER;

    result = flexcan_init(instance, &flexcan_data, TRUE);
    if (result)
        printf("\nFLEXCAN initilization. result: 0x%lx", result);

    result = flexcan_set_bitrate(instance, 1000000);
    if (result)
        printf("\nFLEXCAN set bitrate. result: 0x%lx", result);

    result = flexcan_set_mask_type(instance, kFlexCanRxMask_Global);
    if (result)
        printf("\nFLEXCAN set mask type. result: 0x%lx", result);

    result = flexcan_set_rx_mb_global_mask(instance, kFlexCanMbId_Std, RX_identifier);
    if (result)
        printf("\nFLEXCAN set rx MB global mask. result: 0x%lx", result);

    result = flexcan_rx_mb_config(instance, &flexcan_data, RX_mailbox_num, &rx_cs1, RX_identifier);
    if (result)
        printf("\nFLEXCAN set rx MB config. result: 0x%lx", result);

    // start loop
    while(1)
    {
        /* Notify Linux peer we need CAN_INT as wake up source */
        result = _io_mcore_lpm_register_peer_wakeup(INT_FLEXCAN1 + CAN_DEVICE - 1, WAKEUP_ENABLE);
        if (WAKEUP_REGISTER_SUCCESS == result)
        {
            printf("Enter flexcan stop mode\n");

            /* Let flexcan enter stop mode */
            flexcan_enter_stop_mode(instance);

            /* disable systick interrupt */
            SysTick_CSR_REG(SysTick_BASE_PTR) &= ~SysTick_CSR_TICKINT_MASK;

            /* Make MQX to enter low power mode, and when MQX idle task is scheduled, Linux knows
             * MQX allow it to drive system to low bus mode or suspend mode.
             */
            _io_mcore_lpm_set_status(STATUS_LOWPOWER_RUN);
        }

        /* If it's blocked, then idle task will allow Linux to switch system power mode, and
         * if Linux happens to suspend the system, further CAN message could wake up Linux to
         * resume the system, including M4 core. And appropriate MQX ISR will get handled and
         * recv_message could get the message.
         */
        printf("Begin to receive CAN messgae\n");
        recv_message(instance);

        if (WAKEUP_REGISTER_SUCCESS == result)
        {
            /* Make MQX to leave low power mode */
            _io_mcore_lpm_set_status(STATUS_NORMAL_RUN);

            /*enable systick interrupt*/
            SysTick_CSR_REG(SysTick_BASE_PTR) |= SysTick_CSR_TICKINT_MASK;

            printf("Leave flexcan stop mode\n");
            /* Let flexcan exit stop mode. In fact in this case CAN driver will exit stop mode
             * automatically when triggered message is received. Calling it here explictly is
             * just to let user know he's able to exit CAN stop mode if the system is woken up
             * by some other wakeup source.
             */
            flexcan_exit_stop_mode(instance);

            /* Notify Linux peer we remove CAN_INT from wake up source */
            _io_mcore_lpm_register_peer_wakeup(INT_FLEXCAN1 + CAN_DEVICE - 1, WAKEUP_DISABLE);
        }

        /* Further functions could be handled as per application requires */
    }
}

/* EOF */
