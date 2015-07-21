
/*HEADER**********************************************************************
*
* Copyright 2013-2014 Freescale Semiconductor, Inc.
* 
* SPDX-License-Identifier: BSD-3-Clause
* The BSD-3-Clause License for this file can be found in the COPYING.BSD file 
* included with this distribution or at 
* http://opensource.org/licenses/BSD-3-Clause
* 
*****************************************************************************
*
* Comments:
*
*   This file contains i.MX6SX-specific functions of the MCC library
*
*
*END************************************************************************/

#include "mcc_imx6sx.h"
#include "mcc_mqx.h"

/* This field contains CPU-to-CPU interrupt vector numbers for all device cores */
static const unsigned int mcc_cpu_to_cpu_vectors[] = {INT_MU_A9 + 32, INT_MU_M4}; /*Eric_Temperory*/

/*!
 * \brief This function gets the CPU-to-CPU vector number for the particular core.
 *
 * Platform-specific inter-CPU vector numbers for each core are defined in the mcc_cpu_to_cpu_vectors[] field.
 *
 * \param[in] core Core number.
 *
 * \return vector number for the particular core
 * \return MCC_VECTOR_NUMBER_INVALID (vector number for the particular core number not found)
 */
unsigned int mcc_get_cpu_to_cpu_vector(unsigned int core)
{
    if (core < (sizeof(mcc_cpu_to_cpu_vectors)/sizeof(mcc_cpu_to_cpu_vectors[0]))) {
        return  mcc_cpu_to_cpu_vectors[core];
    }
    return MCC_VECTOR_NUMBER_INVALID;
}

/*!
 * \brief This function enable the CPU-to-CPU interrupt on the MU module.
 *
 * MU General Purpose Interrupt need to be enabled to recieve interrupt from the other core
 *
 * \param[in] core Core number.
 *
 */
void mcc_enable_cpu_to_cpu_interrupt(unsigned int core)
{
    /* M4 enable GIP3 from A9*/
    if (core == 1) {
        // Set BCR.GIE3
        MU_BCR_REG(MUB_BASE_PTR) |= MU_BCR_GIEn(3);
    }
}

/*!
 * \brief This function clears the CPU-to-CPU interrupt flag for the particular core.
 *
 * Implementation is platform-specific.
 *
 * \param[in] core Core number.
 */
void mcc_clear_cpu_to_cpu_interrupt(unsigned int core)
{
    mu_clear_gp(MU_GP_M4_INDEX);
}

/*!
 * \brief This function triggers the CPU-to-CPU interrupt.
 *
 * Platform-specific software triggering the inter-CPU interrupts.
 */
void mcc_triger_cpu_to_cpu_interrupt(void)
{
    mu_set_gp(MU_GP_M4_INDEX);
}
