/*HEADER**********************************************************************
*
* Copyright 2008-2014 Freescale Semiconductor, Inc.
* Copyright 2011 Embedded Access Inc.
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
*   This file contains read functions for lwadc.
*
*
*END************************************************************************/

#include <mqx.h>
#include <bsp.h>
#include "test.h"

#ifndef BSP_DEFAULT_LWADC_MODULE
#error This application requires BSP_DEFAULT_LWADC_MODULE to be not defined in the BSP. Please recompile BSP with this option.
#endif

typedef struct adc_demo_struct {
    const char *    name;
    uint32_t         input;
} ADC_DEMO_STRUCT;

const LWADC_INIT_STRUCT BSP_DEFAULT_LWADC_MODULE;

const ADC_DEMO_STRUCT adc_inputs[] = {
#ifdef BSP_ADC_INPUT_0
   {"Generic input #0", BSP_ADC_INPUT_0 },
#endif

#ifdef BSP_ADC_INPUT_1
   {"Generic input #1", BSP_ADC_INPUT_1 },
#endif

#ifdef BSP_ADC_INPUT_2
   {"Generic input #2", BSP_ADC_INPUT_2 },
#endif

#ifdef BSP_ADC_INPUT_3
   {"Generic input #3", BSP_ADC_INPUT_3 },
#endif

#ifdef BSP_ADC_INPUT_4
   {"Generic input #4", BSP_ADC_INPUT_4 },
#endif

#ifdef BSP_ADC_INPUT_5
   {"Generic input #5", BSP_ADC_INPUT_5 },
#endif

#ifdef BSP_ADC_INPUT_6
   {"Generic input #6", BSP_ADC_INPUT_6 },
#endif

#ifdef BSP_ADC_INPUT_7
   {"Generic input #7", BSP_ADC_INPUT_7 },
#endif
};

/*FUNCTION****************************************************************
*
* Function Name    : adc_task
* Params           : parameter from _task_create()
* Returned Value   : none
* Comments         :
*
*
*END*********************************************************************/
void adc_task(uint32_t parameter)
{
    LWADC_STRUCT_PTR    lwadc_inputs;
    LWADC_VALUE         i, scaled, raw;
    uint32_t            c = 0;

    _lwadc_init(&BSP_DEFAULT_LWADC_MODULE);

    lwadc_inputs = (LWADC_STRUCT_PTR) _mem_alloc_zero(ELEMENTS_OF(adc_inputs)*sizeof(LWADC_STRUCT));

    if (lwadc_inputs == NULL) {
        PRINT(" %d\t Error, Insufficient memory to run full test\n.", _task_get_index_from_id(_task_get_id()));
        _task_block();
    }

    for (i=0;i<ELEMENTS_OF(adc_inputs);i++) {
        if ( !_lwadc_init_input(&lwadc_inputs[i],adc_inputs[i].input) ) {
            /* Failed to initialize this input. We will end up failing the reads below as well. */
            PRINT(" %d\t Failed to initialize ADC input %s\n", _task_get_index_from_id(_task_get_id()),adc_inputs[i].name);
        }
    }

    while (1) {
        for (i=0;i<ELEMENTS_OF(adc_inputs);i++) {
            /* This waits until a new conversion is read on the channel */
            if (_lwadc_wait_next(&lwadc_inputs[i])) {
                if (_lwadc_read(&lwadc_inputs[i], &scaled) &&
                    _lwadc_read_raw(&lwadc_inputs[i], &raw)) {
                    c++;
                    if (c%1000000 == 0) {
                        PRINT("\n\n\n %d\t ADC task is running... \n", _task_get_index_from_id(_task_get_id()));
                    }
                }
            }
        }
    }
}

/* EOF */

