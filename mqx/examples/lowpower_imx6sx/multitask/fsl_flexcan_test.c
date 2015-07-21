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
*   This file contains send/receive functions between two instance using
*   FlexCAN driver.
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>
#include <fsl_flexcan_driver.h>
#include "test.h"

/* Global variables */
uint32_t TX_identifier;
uint32_t TX_mailbox_num;
uint32_t RX_mailbox_num;
flexcan_config_t flexcan1_data;
flexcan_id_table_t id_table;
flexcan_mb_t rx_fifo;
flexcan_mb_t rx_mb;
uint8_t instance[2];

/*FUNCTION****************************************************************
*
* Function Name    : init_flexcan
* Returned Value   : none
* Comments         :
*
*
*END*********************************************************************/
void init_flexcan()
{
    uint32_t result;
    uint32_t rx_fifo_id[8];
    uint32_t i;
    uint32_t bitrate_get = 0;

    instance[0] = 1;
    instance[1] = 2;
    flexcan1_data.num_mb = 16;
    flexcan1_data.max_num_mb = 16;
    flexcan1_data.num_rximr = 64;
    flexcan1_data.num_id_filters = kFlexCanRxFifoIDFilters_8;
    flexcan1_data.is_rx_fifo_needed = TRUE;
    flexcan1_data.is_rx_mb_needed = TRUE;

    id_table.is_extended_mb = 0;
    id_table.is_remote_mb = 0;
    rx_fifo_id[0] = 0x666;
    rx_fifo_id[1] = 0x667;
    rx_fifo_id[2] = 0x676;
    rx_fifo_id[3] = 0x66E;
    rx_fifo_id[4] = 0x66F;
    for (i = 5; i < 8; i++)
        rx_fifo_id[i] = 0x6E6;
    id_table.id_filter = rx_fifo_id;

    /* Select mailbox number */
    RX_mailbox_num = 9;
    TX_mailbox_num = 13;
    TX_identifier = 0x321;

    for (i=0; i<sizeof(instance)/sizeof(uint8_t); i++)
    {
        result = flexcan_init(instance[i], &flexcan1_data, TRUE);
        if (result) {
            PRINT("\n %d\t FlexCAN initialization. result: 0x%lx", _task_get_index_from_id(_task_get_id()), result);
        }

        result = flexcan_set_bitrate(instance[i], 1000000);
        if (result)
        {
            PRINT("\n %d\t Set bitrate error. ", _task_get_index_from_id(_task_get_id()));
            return;
        }

        result = flexcan_get_bitrate(instance[i], &bitrate_get);
        if (result)
        {
            PRINT("\n %d\t FlexCAN get bitrate failed. result: 0x%lx", _task_get_index_from_id(_task_get_id()), result);
        }

        result = flexcan_set_mask_type(instance[i], kFlexCanRxMask_Global);
        if (result)
        {
            PRINT("\n %d\t FlexCAN set mask type failed. result: 0x%lx", _task_get_index_from_id(_task_get_id()), result);
        }
    }
}
/*FUNCTION****************************************************************
*
* Function Name    : flexcan_tx_task
* Params           : parameter from _task_create()
* Returned Value   : none
* Comments         :
*
*
*END*********************************************************************/
void flexcan_tx_task(uint32_t parameter)
{
    uint8_t  data;
    uint32_t result;
    uint32_t  c = 0;

    data = 0x00;
    //Standard ID
    flexcan_mb_code_status_tx_t tx_cs1;
    tx_cs1.code = kFlexCanTX_Data;
    tx_cs1.msg_id_type = kFlexCanMbId_Std;
    tx_cs1.data_length = 1;
    tx_cs1.substitute_remote = 0;
    tx_cs1.remote_transmission = 0;
    tx_cs1.local_priority_enable = 0;
    tx_cs1.local_priority_val = 0;

    result = flexcan_tx_mb_config(instance[parameter], &flexcan1_data, TX_mailbox_num, &tx_cs1, TX_identifier);
    if (result)
        PRINT("\n %d\t Transmit MB config error. Error Code: 0x%lx", _task_get_index_from_id(_task_get_id()), result);
    else
    {
        while(true)
        {
            _time_delay(1000);

            data++;

            result = flexcan_send(instance[parameter], &flexcan1_data, TX_mailbox_num, &tx_cs1, TX_identifier, 1, &data);
            if (c%500 == 0) {
                PRINT("\n %d\t FlexCAN instance %d tx task is running... \n", _task_get_index_from_id(_task_get_id()), parameter + 1);
            }
            if (result) {
                PRINT("\n %d\t FlexCAN instance %d transmit error. Error Code: 0x%lx\n", _task_get_index_from_id(_task_get_id()), parameter + 1, result);
            }
        }
    }
}
/*FUNCTION****************************************************************
*
* Function Name    : flexcan_rx_task
* Params           : parameter from _task_create()
* Returned Value   : none
* Comments         :
*
*
*END*********************************************************************/
void flexcan_rx_task(uint32_t parameter)
{
    uint32_t result;
    uint32_t  c = 0;

    bool is_rx_mb_data = FALSE;
    bool is_rx_fifo_data = FALSE;
    flexcan_mb_code_status_rx_t rx_cs1;
    rx_cs1.code = kFlexCanRX_Ranswer;
    rx_cs1.msg_id_type = kFlexCanMbId_Std;
    rx_cs1.data_length = 1;
    rx_cs1.substitute_remote = 0;
    rx_cs1.remote_transmission = 0;
    rx_cs1.local_priority_enable = 0;
    rx_cs1.local_priority_val = 0;

    if (flexcan1_data.is_rx_fifo_needed)
    {
        // Configure RX FIFO fields
        result = flexcan_rx_fifo_config(instance[parameter], &flexcan1_data, kFlexCanRxFifoIdElementFormat_A, &id_table);
        if (result)
        {
            PRINT("\n %d\t Configure RX FIFO fields error. Error Code: 0x%lx", _task_get_index_from_id(_task_get_id()), result);
            return;
        }
    }

    if (flexcan1_data.is_rx_mb_needed)
    {
        // Configure RX MB fields
        result = flexcan_rx_mb_config(instance[parameter], &flexcan1_data, RX_mailbox_num, &rx_cs1, TX_identifier);
        if (result)
        {
            PRINT("\n %d\t Configure RX MB fields error. Error Code: 0x%lx", _task_get_index_from_id(_task_get_id()), result);
            return;
        }
    }

    // Start receiving data
    while(true)
    {
        is_rx_mb_data = FALSE;
        is_rx_fifo_data = FALSE;
        result = (flexcan_start_receive(instance[parameter], &flexcan1_data, RX_mailbox_num, TX_identifier, 1,
                                  &is_rx_mb_data, &is_rx_fifo_data, &rx_mb, &rx_fifo));
        if (c%500 == 0) {
            PRINT("\n %d\t FlexCAN instance %d rx task is running... \n", _task_get_index_from_id(_task_get_id()), parameter + 1);
        }

        if (result) {
            PRINT("\n %d\t FlexCAN instance %d receive error. Error code: 0x%lx\n", _task_get_index_from_id(_task_get_id()), parameter + 1, result);
        }
    }
}

/* EOF */

