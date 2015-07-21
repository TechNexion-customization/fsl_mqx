/*HEADER**********************************************************************
*
* Copyright 2008-2014 Freescale Semiconductor, Inc.
* Copyright 2004-2008 Embedded Access Inc.
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
*   This file contains the functions for the polled serial I/O
*   low level functions for the SCI device.
*
*
*END************************************************************************/

#include "mqx.h"
#include "bsp.h"
#include "io_prv.h"
#include "charq.h"
#include "fio_prv.h"
#include "serplprv.h"

/* Polled driver functions */
extern char     _imx_uart_polled_getc(IMX_UART_INFO_STRUCT_PTR);
extern void     _imx_uart_polled_putc(IMX_UART_INFO_STRUCT_PTR, char);
extern bool     _imx_uart_polled_status(IMX_UART_INFO_STRUCT_PTR);
extern uint32_t _imx_uart_polled_ioctl(IMX_UART_INFO_STRUCT_PTR, uint32_t, uint32_t *);
extern uint32_t _imx_uart_polled_init(IMX_UART_INIT_STRUCT_PTR, void **, char *);
extern uint32_t _imx_uart_polled_deinit(IMX_UART_INIT_STRUCT_PTR, IMX_UART_INFO_STRUCT_PTR);
static uint32_t _imx_uart_clock_switch(IMX_UART_INFO_STRUCT_PTR, bool);

#define MAX_UBIR_VALUE   65536   /* 16bit register value */

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _imx_uart_get_closest_divisor
* Returned Value   : Closest divisor value
* Comments         :
*    Calculates and get the closest divisor value.
*    Algorithms: the max divisor of (a, b) is equals to max divisor
*    of (b, a%b).
*END*----------------------------------------------------------------------*/
static uint32_t _imx_uart_get_closest_divisor
    (
        uint32_t numerator,
        uint32_t denominator
    )
{
    uint32_t r = 1;

    while(denominator != 0) {
       r = denominator;
       denominator = numerator % denominator;
       numerator   = r;
    }

    return r;
}

static uint32_t _imx_uart_change_baudrate
    (
        /* [IN] SCI channel registers */
        UART_MemMapPtr sci_ptr,

        /* [IN] SCI input clock frequency */
        uint32_t        clock_frequency,

        /* [IN] Requested baud rate */
        uint32_t        baud_rate
    )
{
    uint32_t divisor;
    uint32_t numerator, denominator;
    uint32_t rf_div;
    uint32_t divider = 1;
    uint32_t m, n, max;

    numerator = clock_frequency;
    denominator = 16 * baud_rate;
    /* get the approximately maxmum divisor */
    divisor = _imx_uart_get_closest_divisor(numerator, denominator);
    numerator = numerator / divisor;
    denominator = denominator / divisor;

    /* numerator ranges from 1 ~ 7 * 64k */
    /* denominator ranges from 1 ~ 64k */
    if (numerator > (MAX_UBIR_VALUE * 7) || denominator > MAX_UBIR_VALUE) {
       m = (numerator - 1) / (MAX_UBIR_VALUE * 7) + 1;
       n = (denominator - 1) / MAX_UBIR_VALUE + 1;
       max = m > n ? m : n;
       numerator /= max;
       denominator /= max;
       if (numerator == 0)  numerator = 1;
       if (denominator == 0) denominator = 1;
    }

    divider = (numerator - 1) / MAX_UBIR_VALUE + 1;

    switch (divider) {
        case 1:
           rf_div = 0x05;
           break;
        case 2:
           rf_div = 0x04;
           break;
        case 3:
           rf_div = 0x03;
           break;
        case 4:
           rf_div = 0x02;
           break;
        case 5:
           rf_div = 0x01;
           break;
        case 6:
           rf_div = 0x00;
           break;
        case 7:
           rf_div = 0x06;
           break;
        default:
	          rf_div = 0x05;
    }

    sci_ptr->UFCR &= (~UART_UFCR_RFDIV_MASK);
    sci_ptr->UFCR |= UART_UFCR_RFDIV(rf_div);

    sci_ptr->UBIR = UART_UBIR_INC(denominator - 1);
    sci_ptr->UBMR = UART_UBMR_MOD(numerator / divider - 1);

    return MQX_OK;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _imx_uart_polled_peripheral_enable
* Returned Value   : None
* Comments         :
*    Enables the SCI peripheral.
*
*END*----------------------------------------------------------------------*/

static void _imx_uart_polled_peripheral_enable
    (
        /* [IN] SCI channel registers */
        UART_MemMapPtr sci_ptr
    )
{
    /* Transmitter and receiver enable */
    sci_ptr->UCR2 |= UART_UCR2_TXEN_MASK | UART_UCR2_RXEN_MASK;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _imx_uart_polled_peripheral_disable
* Returned Value   : None
* Comments         :
*    Disables the SCI peripheral.
*
*END*----------------------------------------------------------------------*/

static void _imx_uart_polled_peripheral_disable
    (
        /* [IN] SCI channel registers */
        UART_MemMapPtr sci_ptr
    )
{
    /* Transmitter and receiver disable */
    sci_ptr->UCR2 &= ~(UART_UCR2_TXEN_MASK | UART_UCR2_RXEN_MASK);
}

#if MQX_ENABLE_LOW_POWER
extern LPM_NOTIFICATION_RESULT _io_serial_polled_clock_configuration_callback (LPM_NOTIFICATION_STRUCT_PTR, void *);
extern LPM_NOTIFICATION_RESULT _io_serial_polled_operation_mode_callback (LPM_NOTIFICATION_STRUCT_PTR, void *);

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _io_serial_polled_clock_configuration_callback
* Returned Value   : Notification error code
* Comments         :
*    Low power clock configuration callback for polled serial.
*
*END*----------------------------------------------------------------------*/

LPM_NOTIFICATION_RESULT _io_serial_polled_clock_configuration_callback
    (
        /* [IN] Low power notification */
        LPM_NOTIFICATION_STRUCT_PTR   notification,

        /* [IN/OUT] Device specific data */
        void                         *device_specific_data
    )
{
    return LPM_NOTIFICATION_RESULT_OK;
}

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _io_serial_polled_operation_mode_callback
* Returned Value   : Notification error code
* Comments         :
*    Low power operation mode callback for polled serial.
*
*END*----------------------------------------------------------------------*/

LPM_NOTIFICATION_RESULT _io_serial_polled_operation_mode_callback
    (
        /* [IN] Low power notification */
        LPM_NOTIFICATION_STRUCT_PTR       notification,

        /* [IN/OUT] Device specific data */
        void                             *device_specific_data
    )
{
    return LPM_NOTIFICATION_RESULT_OK;
}
#endif

/*FUNCTION*-------------------------------------------------------------------
*
* Function Name    : _imx_uart_polled_install
* Returned Value   : uint32_t a task error code or MQX_OK
* Comments         :
*    Install a polled sci device.
*
*END*----------------------------------------------------------------------*/

uint32_t _imx_uart_polled_install
   (
      /* [IN] A string that identifies the device for fopen */
      char           *identifier,

      /* [IN] The I/O init data pointer */
      IMX_UART_INIT_STRUCT_CPTR            init_data_ptr,

      /* [IN] The I/O queue size to use */
      uint32_t            queue_size
   )
{ /* Body */

#if PE_LDD_VERSION
    if (PE_PeripheralUsed((uint32_t)_bsp_get_serial_base_address(init_data_ptr->DEVICE)))
    {
        return IO_ERROR;
    }
#endif

    return _io_serial_polled_install(identifier,
      (uint32_t (_CODE_PTR_)(void *, void **, char *))_imx_uart_polled_init,
      (uint32_t (_CODE_PTR_)(void *, void *))_imx_uart_polled_deinit,
      (char    (_CODE_PTR_)(void *))_imx_uart_polled_getc,
      (void    (_CODE_PTR_)(void *, char))_imx_uart_polled_putc,
      (bool (_CODE_PTR_)(void *))_imx_uart_polled_status,
      (uint32_t (_CODE_PTR_)(void *, uint32_t, void *))_imx_uart_polled_ioctl,
      (void *)init_data_ptr, queue_size);

} /* Endbody */

/*FUNCTION****************************************************************
*
* Function Name    : _imx_uart_polled_init
* Returned Value   : MQX_OK or a MQX error code.
* Comments         :
*    This function initializes the SCI
*
*END*********************************************************************/

uint32_t _imx_uart_polled_init
   (
      /* [IN] the initialization information for the device being opened */
      IMX_UART_INIT_STRUCT_PTR               io_init_ptr,

      /* [OUT] the address to store device specific information */
      void                          **io_info_ptr_ptr,

      /* [IN] the rest of the name of the device opened */
      char                           *open_name_ptr
   )
{ /* Body */
   UART_MemMapPtr                        sci_ptr;
   IMX_UART_INFO_STRUCT_PTR              sci_info_ptr;
   uint32_t                              channel, clock;
   uint8_t                               flags;

   /* Get peripheral address */
   channel = io_init_ptr->DEVICE;

   sci_ptr = _bsp_get_serial_base_address(channel);
   if (sci_ptr == NULL)
   {
      return MQX_INVALID_IO_CHANNEL;
   }

   sci_info_ptr = _mem_alloc_system_zero((uint32_t)sizeof(IMX_UART_INFO_STRUCT));

#if MQX_CHECK_MEMORY_ALLOCATION_ERRORS
   if ( sci_info_ptr == NULL )
   {
      return MQX_OUT_OF_MEMORY;
   }
#endif

   sci_info_ptr->SCI_PTR = sci_ptr;
   sci_info_ptr->SERIAL_CLK = clock_get(CLK_UART);
   sci_info_ptr->SERIAL_IPG_CLK = clock_get(CLK_UART_IPG);
   /* Save initialization values */
   sci_info_ptr->INIT = *io_init_ptr;
   *io_info_ptr_ptr = sci_info_ptr;

   _imx_uart_clock_switch(sci_info_ptr, TRUE);

   clock = clock_get_freq(sci_info_ptr->SERIAL_CLK);
   flags = IO_PERIPHERAL_PIN_MUX_ENABLE | IO_PERIPHERAL_CLOCK_ENABLE | IO_PERIPHERAL_MODULE_ENABLE;

   /* Enable HW */
   _bsp_serial_io_init (channel, flags);

   sci_ptr->UCR1 = 0;
   sci_ptr->UCR2 = 0;

   while(!(sci_ptr->UCR2 & UART_UCR2_SRST_MASK));

   sci_ptr->UCR3 = UART_UCR3_DSR_MASK | UART_UCR3_DCD_MASK | UART_UCR3_RI_MASK | UART_UCR3_RXDMUXSEL_MASK;

   sci_ptr->UCR4 &= (~UART_UCR4_CTSTL_MASK);
   sci_ptr->UCR4 |= UART_UCR4_CTSTL(32);

   sci_ptr->UESC = UART_UESC_ESC_CHAR_MASK;
   sci_ptr->UTIM = 0;

   sci_ptr->UTS = UART_UTS_DBGEN_MASK;

   /* Setup baudrate */
   _imx_uart_change_baudrate (sci_ptr, clock, sci_info_ptr->INIT.BAUD_RATE);

   sci_ptr->UCR2 = UART_UCR2_WS_MASK | UART_UCR2_IRTS_MASK | UART_UCR2_SRST_MASK;
   sci_ptr->UCR1 |= UART_UCR1_UARTEN_MASK;

   sci_ptr->UFCR &= ~(UART_UFCR_RXTL_MASK | UART_UFCR_TXTL_MASK);
   sci_ptr->UFCR |= UART_UFCR_RXTL(1) | UART_UFCR_TXTL(32);

   /* Module enable/disable */
   if (flags & IO_PERIPHERAL_MODULE_ENABLE)
   {
       _imx_uart_polled_peripheral_enable (sci_ptr);
   }
   else
   {
       _imx_uart_polled_peripheral_disable (sci_ptr);
   }

   _imx_uart_clock_switch(sci_info_ptr, FALSE);

   return MQX_OK;
} /* Endbody */


/*FUNCTION****************************************************************
*
* Function Name    : _imx_uart_polled_deinit
* Returned Value   : MQX_OK or a mqx error code.
* Comments         :
*    This function de-initializes the SCI.
*
*END*********************************************************************/

uint32_t _imx_uart_polled_deinit
   (
      /* [IN] the initialization information for the device being opened */
      IMX_UART_INIT_STRUCT_PTR io_init_ptr,

      /* [IN] the address of the device specific information */
      IMX_UART_INFO_STRUCT_PTR io_info_ptr
   )
{ /* Body */

   _imx_uart_clock_switch(io_info_ptr, TRUE);

   /* Disable interrupts, transmitter, receiver */
   io_info_ptr->SCI_PTR->UCR1 &= ~UART_UCR1_UARTEN_MASK;
   io_info_ptr->SCI_PTR->UCR2 &= ~(UART_UCR2_TXEN_MASK | UART_UCR2_RXEN_MASK);

   /* Disable module clocks if required */
   _bsp_serial_io_init (io_init_ptr->DEVICE, IO_PERIPHERAL_CLOCK_DISABLE);

   _mem_free(io_info_ptr);
   io_info_ptr = NULL;

   _imx_uart_clock_switch(io_info_ptr, FALSE);

   return(MQX_OK);
} /* Endbody */


/*FUNCTION****************************************************************
*
* Function Name    : _imx_uart_polled_getc
* Returned Value   : char
* Comments         :
*   Return a character when it is available.  This function polls the
* device for input.
*
*************************************************************************/

char _imx_uart_polled_getc
   (
      /* [IN] the address of the device specific information */
      IMX_UART_INFO_STRUCT_PTR io_info_ptr
   )
{ /* Body */
    char c = 0;

    UART_MemMapPtr sci_ptr = io_info_ptr->SCI_PTR;

   _imx_uart_clock_switch(io_info_ptr, TRUE);

    while (sci_ptr->UTS & UART_UTS_RXEMPTY_MASK) {
        /*Wait while buffer is empty*/
    }

    io_info_ptr->RX_CHARS++;
    c = sci_ptr->URXD & UART_URXD_RX_DATA_MASK;

   _imx_uart_clock_switch(io_info_ptr, FALSE);

    return c;
} /* Endbody */


/*FUNCTION****************************************************************
*
* Function Name    : _imx_uart_polled_putc
* Returned Value   : void
* Comments         :
*   Writes the provided character.
*
*END*********************************************************************/

void _imx_uart_polled_putc
   (
      /* [IN] the address of the device specific information */
      IMX_UART_INFO_STRUCT_PTR io_info_ptr,

      /* [IN] the character to write */
      char                               c
   )
{
    _imx_uart_clock_switch(io_info_ptr, TRUE);

    UART_MemMapPtr sci_ptr = io_info_ptr->SCI_PTR;

    while (sci_ptr->UTS & UART_UTS_TXFULL_MASK){
        /* wait while buffer is full */
    }

    sci_ptr->UTXD = c;
    io_info_ptr->TX_CHARS++;

    _imx_uart_clock_switch(io_info_ptr, FALSE);
}

/*FUNCTION****************************************************************
*
* Function Name    : _imx_uart_polled_status
* Returned Value   : bool
* Comments         :
*            This function returns TRUE if a character is available
*            on the on I/O device, otherwise it returns FALSE.
*
*END*********************************************************************/

bool _imx_uart_polled_status
   (
      /* [IN] the address of the device specific information */
      IMX_UART_INFO_STRUCT_PTR io_info_ptr
   )
{ /* Body */
   UART_MemMapPtr sci_ptr = io_info_ptr->SCI_PTR;
   _imx_uart_clock_switch(io_info_ptr, TRUE);
   if((sci_ptr->UTS & UART_UTS_RXEMPTY_MASK) == 0 ) {
       _imx_uart_clock_switch(io_info_ptr, FALSE);
       return TRUE;
   }
   else {
       _imx_uart_clock_switch(io_info_ptr, FALSE);
       return FALSE;
   }
} /* Endbody */


/*FUNCTION****************************************************************
*
* Function Name    : _imx_uart_polled_ioctl
* Returned Value   : uint32_t MQX_OK or a mqx error code.
* Comments         :
*    This function performs miscellaneous services for
*    the I/O device.
*
*END*********************************************************************/

uint32_t _imx_uart_polled_ioctl
   (
      /* [IN] the address of the device specific information */
      IMX_UART_INFO_STRUCT_PTR io_info_ptr,

      /* [IN] The command to perform */
      uint32_t                    cmd,

      /* [IN] Parameters for the command */
      uint32_t                *param_ptr
   )
{ /* Body */
   UART_MemMapPtr sci_ptr = io_info_ptr->SCI_PTR;
   uint32_t       tmp;

   _imx_uart_clock_switch(io_info_ptr, TRUE);

   switch (cmd) {
      case IO_IOCTL_SERIAL_GET_DATA_BITS:
         tmp = sci_ptr->UCR2 & UART_UCR2_WS_MASK;
         *param_ptr = tmp + 7;
         /* return 7 or 8 */
         break;

      case IO_IOCTL_SERIAL_SET_DATA_BITS:
         /* set the 8 bit mode M = 0*/
         tmp = sci_ptr->UCR2 & ~UART_UCR2_WS_MASK;
         if(*param_ptr == 8)
             tmp |= UART_UCR2_WS_MASK;
         /* Write back C1 value */
         sci_ptr->UCR2 = (uint8_t) tmp;
         break;

      case IO_IOCTL_SERIAL_GET_BAUD:
         *param_ptr = io_info_ptr->INIT.BAUD_RATE;
         break;

      case IO_IOCTL_SERIAL_SET_BAUD:
          tmp = clock_get_freq(io_info_ptr->SERIAL_CLK);
          tmp = _imx_uart_change_baudrate (sci_ptr, tmp, *param_ptr);
          if (MQX_OK != tmp)
          {
              _imx_uart_clock_switch(io_info_ptr, FALSE);
              return tmp;
          }
          io_info_ptr->INIT.BAUD_RATE = *param_ptr;
         break;

      case IO_IOCTL_SERIAL_GET_STATS:
         *param_ptr++ = io_info_ptr->INTERRUPTS;
         *param_ptr++ = io_info_ptr->RX_CHARS;
         *param_ptr++ = io_info_ptr->TX_CHARS;
         *param_ptr++ = io_info_ptr->RX_BREAKS;
         *param_ptr++ = io_info_ptr->RX_PARITY_ERRORS;
         *param_ptr++ = io_info_ptr->RX_FRAMING_ERRORS;
         *param_ptr++ = io_info_ptr->RX_OVERRUNS;
         *param_ptr++ = io_info_ptr->RX_DROPPED_INPUT;
         break;

      case IO_IOCTL_SERIAL_CLEAR_STATS:
         io_info_ptr->INTERRUPTS = 0;
         io_info_ptr->RX_CHARS = 0;
         io_info_ptr->TX_CHARS = 0;
         io_info_ptr->RX_BREAKS = 0;
         io_info_ptr->RX_PARITY_ERRORS = 0;
         io_info_ptr->RX_FRAMING_ERRORS = 0;
         io_info_ptr->RX_OVERRUNS = 0;
         io_info_ptr->RX_DROPPED_INPUT = 0;
         break;

      case IO_IOCTL_SERIAL_CAN_TRANSMIT:
         *param_ptr = sci_ptr->USR1 & UART_USR1_TRDY_MASK? 1 : 0;
         break;

      case IO_IOCTL_SERIAL_CAN_RECEIVE:
         *param_ptr = sci_ptr->USR1 & UART_USR1_RRDY_MASK? 1 : 0;
         break;

      case IO_IOCTL_SERIAL_GET_PARITY:
         tmp = IO_SERIAL_PARITY_NONE;
         if (sci_ptr->UCR2 & UART_UCR2_PREN_MASK) {
            if (sci_ptr->UCR2 & UART_UCR2_PROE_MASK) {
               tmp = IO_SERIAL_PARITY_ODD;
            } else {
               tmp = IO_SERIAL_PARITY_EVEN;
            }
         }
         *param_ptr = tmp;
         break;

      case IO_IOCTL_SERIAL_SET_PARITY:
         tmp = sci_ptr->UCR2 & ~(UART_UCR2_PREN_MASK | UART_UCR2_PROE_MASK);
         switch (*param_ptr) {
            case IO_SERIAL_PARITY_NONE:
               break;
            case IO_SERIAL_PARITY_ODD:
               tmp |= UART_UCR2_PREN_MASK | UART_UCR2_PROE_MASK;
               break;
            case IO_SERIAL_PARITY_EVEN:
               tmp |= UART_UCR2_PREN_MASK;
               break;
            default:
               _imx_uart_clock_switch(io_info_ptr, FALSE);
               return MQX_INVALID_PARAMETER;
         }
         sci_ptr->UCR2 = (uint8_t) tmp;
         break;

      case IO_IOCTL_SERIAL_DISABLE_RX:
         if( *(bool *)param_ptr == TRUE )
         {
            /* disable receiver */
            sci_ptr->UCR2 &= ~UART_UCR2_RXEN_MASK;
         }
         else
         {
            /* enable receiver */
            sci_ptr->UCR2 |= UART_UCR2_RXEN_MASK;
         }
         break;

      case IO_IOCTL_SERIAL_WAIT_FOR_TC:
         /* wait for transmission end signal */
         while(!(sci_ptr->USR2 & UART_USR2_TXDC_MASK))
             { };
         break;

      case IO_IOCTL_FLUSH_OUTPUT:
         while (!(sci_ptr->USR2 & UART_USR2_TXFE_MASK))
             { };
         while (!(sci_ptr->USR2 & UART_USR2_TXDC_MASK))
             { };
         break;

      case IO_IOCTL_SERIAL_GET_STOP_BITS:
          *param_ptr = IO_SERIAL_STOP_BITS_1;
          break;

      case IO_IOCTL_SERIAL_SET_FLAGS:
          break;

      default:
         _imx_uart_clock_switch(io_info_ptr, FALSE);
         return IO_ERROR_INVALID_IOCTL_CMD;
   } /* Endswitch */

    _imx_uart_clock_switch(io_info_ptr, FALSE);

   return (MQX_OK);

} /* Endbody */

static uint32_t _imx_uart_clock_switch
   (
      /* [IN] the address of the device specific information */
      IMX_UART_INFO_STRUCT_PTR io_info_ptr,
      bool    enable
   )
{
   if (enable) {
      clock_enable(io_info_ptr->SERIAL_CLK);
      clock_enable(io_info_ptr->SERIAL_IPG_CLK);
   } else {
      clock_disable(io_info_ptr->SERIAL_CLK);
      clock_disable(io_info_ptr->SERIAL_IPG_CLK);
   }

   return MQX_OK;
}
/* EOF */
