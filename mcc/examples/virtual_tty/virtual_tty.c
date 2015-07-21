
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
*   This file contains the source for one of the MCC virtual TTY examples.
*
*
*END************************************************************************/
#include <mqx.h>
#include <bsp.h>
#include "mcc_config.h"
#include "mcc_common.h"
#include "mcc_api.h"
#include "mcc_mqx.h"
#include <string.h>

#if ! BSPCFG_ENABLE_IO_SUBSYSTEM
#error This application requires BSPCFG_ENABLE_IO_SUBSYSTEM defined non-zero in user_config.h. Please recompile BSP with this option.
#endif

#ifndef BSP_DEFAULT_IO_CHANNEL_DEFINED
#error This application requires BSP_DEFAULT_IO_CHANNEL to be not NULL. Please set corresponding BSPCFG_ENABLE_TTYx to non-zero in user_config.h and recompile BSP with this option.
#endif

#define MAIN_TTN               (10)
#define RESPONDER_TTN          (11)

#define MCC_MQX_NODE_M4        (0)

#define MCC_MQX_SENDER_PORT    (1)
#define MCC_MQX_RESPONDER_PORT (2)

typedef struct the_message {
	char DATA[MCC_ATTR_BUFFER_SIZE_IN_BYTES - 24];
} THE_MESSAGE, *THE_MESSAGE_PTR;

extern void main_task(uint32_t);

TASK_TEMPLATE_STRUCT  MQX_template_list[] =
{
   /* Task Index,    Function,       Stack,  Priority,  Name,        Attributes,          Param,            Time Slice */
    { RESPONDER_TTN, main_task,      2000,   9,         "Responder", MQX_AUTO_START_TASK, MCC_MQX_NODE_M4,  0 },
    { 0 }
};

MCC_ENDPOINT    mqx_endpoint_m4 = {1,MCC_MQX_NODE_M4,MCC_MQX_RESPONDER_PORT};


void main_task(uint32_t node_num)
{
    THE_MESSAGE     msg;
    MCC_MEM_SIZE    num_of_received_bytes;
    MCC_INFO_STRUCT mcc_info;
    int             ret_value;
    MCC_ENDPOINT    mqx_endpoint_sender = {0,0,0};

    
    char control_char;

    /*
     * Wait For A9 Side Become Ready
     */
    printf("\n\n\n\n***** MCC Virtual TTY EXAMPLE *****\n");
    printf("Please wait :\n");
    printf("    1) A9 peer is ready\n");
    printf("Then press \"S\" to start the demo\n");
    printf("********************************\n");

    while (TRUE) {
        printf("\nPress \"S\" to start the demo : ");
        control_char = getchar();
        if ((control_char == 's') || (control_char == 'S')) {
            break;
        }
    }

    ret_value = mcc_initialize(node_num);
    
    if(MCC_SUCCESS != ret_value) {
        printf("\n\n\nError, attempting to initialize the MCC library failed! Application is stopped now. Error code = %i\n", ret_value);
        mcc_destroy(node_num);
        _task_block();
    }
    ret_value = mcc_get_info(node_num, &mcc_info);

    ret_value = mcc_create_endpoint(&mqx_endpoint_m4, MCC_MQX_RESPONDER_PORT);

    printf("\n\n\nResponder task started, MCC version is %s\n", mcc_info.version_string);

    while (TRUE) {
        ret_value = mcc_recv(&mqx_endpoint_sender, &mqx_endpoint_m4, &msg, sizeof(THE_MESSAGE), &num_of_received_bytes, 0xffffffff);
        
        if(MCC_SUCCESS != ret_value) {
            printf("Responder task receive error: %i\n", ret_value);
        } else {
            printf("Responder task received a msg from [%i,%i,%i] endpoint\n", mqx_endpoint_sender.core, mqx_endpoint_sender.node, mqx_endpoint_sender.port);
            printf("Message: Size=%x, DATA = \"%s\"\n", num_of_received_bytes, msg.DATA);
            ret_value = mcc_send(&mqx_endpoint_m4, &mqx_endpoint_sender, &msg, sizeof(THE_MESSAGE), 0xffffffff);
            if(MCC_SUCCESS != ret_value) {
                printf("\nError, sending the message using the send function failed");
            }
        }
    }
}
