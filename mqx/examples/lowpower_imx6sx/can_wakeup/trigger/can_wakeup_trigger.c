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
*   This file contains the source for the can wakeup trigger example program.
*
*END************************************************************************/


#include <mqx.h>
#include <bsp.h>
#include <fsl_flexcan_driver.h>

#include "can_wakeup.h"

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

static uint32_t TX_identifier;
static uint32_t TX_mailbox_num;

static void Main_Task(uint32_t parameter);

/* Task template list */
TASK_TEMPLATE_STRUCT MQX_template_list[] = {
    { 5, Main_Task, 1000L, 8L, "Main task", MQX_AUTO_START_TASK},
    { 0L, 0L, 0L, 0L, 0L, 0L }
};

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
    uint8_t  data = 0xF;
    flexcan_mb_code_status_tx_t tx_cs1;

    tx_cs1.code = kFlexCanTX_Data;
    tx_cs1.msg_id_type = kFlexCanMbId_Std;
    tx_cs1.data_length = 1;
    tx_cs1.substitute_remote = 0;
    tx_cs1.remote_transmission = 0;
    tx_cs1.local_priority_enable = 0;
    tx_cs1.local_priority_val = 0;

    instance = CAN_DEVICE;

    printf("\n****FLEXCAN WAKEUP TRIGGER PROGRAM.****");
    printf("\n   Message format: Standard (11 bit id)");
    printf("\n   Message buffer %d used for Tx.", CAN_TX_MAILBOX);
    printf("\n   Interrupt Mode: Enabled");
    printf("\n   Operation Mode: TX --> Normal");
    printf("\n***************************************\n");

    /* Select mailbox number */
    TX_mailbox_num = CAN_TX_MAILBOX;
    TX_identifier = CAN_IDENTIFIER;

    result = flexcan_init(instance, &flexcan_data, TRUE);
    if (result)
        printf("\nFLEXCAN initilization. result: 0x%lx", result);

    result = flexcan_set_bitrate(instance, 1000000);
    if (result)
        printf("\nFLEXCAN set bitrate. result: 0x%lx", result);

    printf("\nFlexCAN send config");
    result = flexcan_tx_mb_config(instance, &flexcan_data, TX_mailbox_num, &tx_cs1, TX_identifier);
    if (result)
        printf("\nTransmit MB config error. Error Code: 0x%lx", result);
        
    // start loop
    while(1)
    {
        data++;

        result = flexcan_send(instance, &flexcan_data, TX_mailbox_num, &tx_cs1, TX_identifier, 1, &data);
        if (result)
            printf("\nTransmit error. Error Code: 0x%lx", result);
        else
        {
            printf("\nData transmit: 0x%02x", data);
        }

        _time_delay(5000);
    }
}

/* EOF */
