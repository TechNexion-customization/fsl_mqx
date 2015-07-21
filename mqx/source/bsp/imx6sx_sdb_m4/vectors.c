/*HEADER**********************************************************************
*
* Copyright 2011 Freescale Semiconductor, Inc.
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
*    This file contains the exception vector table and flash configuration.
*
*
*END************************************************************************/

#include "mqx_inc.h"
#include "bsp.h"

#define BOOT_START      __boot
#if MQX_ROM_VECTORS
#define DEFAULT_VECTOR  _int_kernel_isr
#else
#define DEFAULT_VECTOR  __boot_exception
#endif

extern unsigned long __BOOT_STACK_ADDRESS[];

extern void __boot(void);
extern void _svc_handler(void);
extern void _pend_svc(void);

#if ! MQX_ROM_VECTORS
/* ram vector */
    static void __boot_exception(void) {
        while(1);
    }

    #if   defined(__ICCARM__)
        #pragma language = extended
        #pragma location = ".vectors_ram"
        #pragma segment  = ".vectors_ram"
        vector_entry ram_vector[256] @ ".vectors_ram" =
    #elif defined (__CC_ARM) || defined(__GNUC__)
        __attribute__((section(".vectors_ram"))) vector_entry ram_vector[256] __attribute__((used)) =
    #else /* CodeWarrior compiler assumed */
        #pragma  define_section vectors_ram ".vectors_ram" far_abs RW
        /*
         * Array for exception vectors in ram + space (6 words)
        // array for exception vectors in ram + space (6 words) for CW fun (when CW debugger handle exceptions, using(rewrite) VBR+0x408 address
         * VBR+0x408 address
         */
        __declspec(vectors_ram) vector_entry ram_vector[256 + 6] =
    #endif /* CodeWarrior compiler assumed */
    {
        (vector_entry)__BOOT_STACK_ADDRESS,
        BOOT_START,         /* 0x01  0x00000004   -   ivINT_Initial_Program_Counter     */
        _int_kernel_isr,    /* 0x02  0x00000008   -   ivINT_NMI                         */
        _int_kernel_isr,    /* 0x03  0x0000000C   -   ivINT_Hard_Fault                  */
        _int_kernel_isr,    /* 0x04  0x00000010   -   ivINT_Mem_Manage_Fault            */
        _int_kernel_isr,    /* 0x05  0x00000014   -   ivINT_Bus_Fault                   */
        _int_kernel_isr,    /* 0x06  0x00000018   -   ivINT_Usage_Fault                 */
        0,                  /* 0x07  0x0000001C   -   ivINT_Reserved7                   */
        0,                  /* 0x08  0x00000020   -   ivINT_Reserved8                   */
        0,                  /* 0x09  0x00000024   -   ivINT_Reserved9                   */
        0,                  /* 0x0A  0x00000028   -   ivINT_Reserved10                  */
        _svc_handler,       /* 0x0B  0x0000002C   -   ivINT_SVCall                      */
        _int_kernel_isr,    /* 0x0C  0x00000030   -   ivINT_DebugMonitor                */
        0,                  /* 0x0D  0x00000034   -   ivINT_Reserved13                  */
        _pend_svc,          /* 0x0E  0x00000038   -   ivINT_PendableSrvReq              */
        _int_kernel_isr     /* 0x0F  0x0000003C   -   ivINT_SysTick                     */
    };
#endif

/*
** exception vector table - this table is really used
*/
#ifdef __ICCARM__
    #pragma language = extended
    #pragma segment  = "CSTACK"

    #pragma location = ".intvec"
    #pragma segment  = ".intvec"
    const vector_entry __vector_table[] =
#elif defined (__CC_ARM) || defined(__GNUC__)
    __attribute__((section(".vectors_rom"))) const vector_entry __vector_table[256] __attribute__((used)) =
#else /* CodeWarrior compiler assumed */
    #pragma  define_section vectors_rom ".vectors_rom" ".vectors_rom" ".vectors_rom" far_abs R
    __declspec(vectors_rom) vector_entry __vector_table[] =
#endif /* CodeWarrior compiler assumed */
{
    (vector_entry)__BOOT_STACK_ADDRESS,
    BOOT_START,             /* 0x01  0x00000004   -   ivINT_Initial_Program_Counter     */
    DEFAULT_VECTOR,         /* 0x02  0x00000008   -   ivINT_NMI                         */
    DEFAULT_VECTOR,         /* 0x03  0x0000000C   -   ivINT_Hard_Fault                  */
    DEFAULT_VECTOR,         /* 0x04  0x00000010   -   ivINT_Mem_Manage_Fault            */
    DEFAULT_VECTOR,         /* 0x05  0x00000014   -   ivINT_Bus_Fault                   */
    DEFAULT_VECTOR,         /* 0x06  0x00000018   -   ivINT_Usage_Fault                 */
    0,                      /* 0x07  0x0000001C   -   ivINT_Reserved7                   */
    0,                      /* 0x08  0x00000020   -   ivINT_Reserved8                   */
    0,                      /* 0x09  0x00000024   -   ivINT_Reserved9                   */
    0,                      /* 0x0A  0x00000028   -   ivINT_Reserved10                  */
    _svc_handler,           /* 0x0B  0x0000002C   -   ivINT_SVCall                      */
    DEFAULT_VECTOR,         /* 0x0C  0x00000030   -   ivINT_DebugMonitor                */
    0,                      /* 0x0D  0x00000034   -   ivINT_Reserved13                  */
    _pend_svc,              /* 0x0E  0x00000038   -   ivINT_PendableSrvReq              */
    DEFAULT_VECTOR,         /* 0x0F  0x0000003C   -   ivINT_SysTick                     */
    /* Cortex external interrupt vectors                                                */
    DEFAULT_VECTOR,         /* 0x10  0x00000040   -   ivINT_Reserved16                  */
    DEFAULT_VECTOR,         /* 0x11  0x00000044   -   ivINT_Reserved17                  */
    DEFAULT_VECTOR,         /* 0x12  0x00000048   -   ivINT_SDMA                        */
    DEFAULT_VECTOR,         /* 0x13  0x0000004C   -   ivINT_Reserved19                  */
    DEFAULT_VECTOR,         /* 0x14  0x00000050   -   ivINT_Reserved20                  */
    DEFAULT_VECTOR,         /* 0x15  0x00000054   -   ivINT_Reserved21                  */
    DEFAULT_VECTOR,         /* 0x16  0x00000058   -   ivINT_Reserved22                  */
    DEFAULT_VECTOR,         /* 0x17  0x0000005C   -   ivINT_Reserved23                  */
    DEFAULT_VECTOR,         /* 0x18  0x00000060   -   ivINT_Reserved24                  */
    DEFAULT_VECTOR,         /* 0x19  0x00000064   -   ivINT_Reserved25                  */
    DEFAULT_VECTOR,         /* 0x1A  0x00000068   -   ivINT_Reserved26                  */
    DEFAULT_VECTOR,         /* 0x1B  0x0000006C   -   ivINT_Reserved27                  */
    DEFAULT_VECTOR,         /* 0x1C  0x00000070   -   ivINT_SEMA4_CP1                   */
    DEFAULT_VECTOR,         /* 0x1D  0x00000074   -   ivINT_Reserved29                  */
    DEFAULT_VECTOR,         /* 0x1E  0x00000078   -   ivINT_Reserved30                  */
    DEFAULT_VECTOR,         /* 0x1F  0x0000007C   -   ivINT_Reserved31                  */
    DEFAULT_VECTOR,         /* 0x20  0x00000080   -   ivINT_Reserved32                  */
    DEFAULT_VECTOR,         /* 0x21  0x00000084   -   ivINT_UART6                       */
    DEFAULT_VECTOR,         /* 0x22  0x00000088   -   ivINT_ECSPI5                      */
    DEFAULT_VECTOR,         /* 0x23  0x0000008C   -   ivINT_SNVS_CONSOLIDATED           */
    DEFAULT_VECTOR,         /* 0x24  0x00000090   -   ivINT_SNVS_SECURITY               */
    DEFAULT_VECTOR,         /* 0x25  0x00000094   -   ivINT_Reserved37                  */
    DEFAULT_VECTOR,         /* 0x26  0x00000098   -   ivINT_Reserved38                  */
    DEFAULT_VECTOR,         /* 0x27  0x0000009C   -   ivINT_Reserved39                  */
    DEFAULT_VECTOR,         /* 0x28  0x000000A0   -   ivINT_Reserved40                  */
    DEFAULT_VECTOR,         /* 0x29  0x000000A4   -   ivINT_Reserved41                  */
    DEFAULT_VECTOR,         /* 0x2A  0x000000A8   -   ivINT_UART1                       */
    DEFAULT_VECTOR,         /* 0x2B  0x000000AC   -   ivINT_UART2                       */
    DEFAULT_VECTOR,         /* 0x2C  0x000000B0   -   ivINT_UART3                       */
    DEFAULT_VECTOR,         /* 0x2D  0x000000B4   -   ivINT_UART4                       */
    DEFAULT_VECTOR,         /* 0x2E  0x000000B8   -   ivINT_UART5                       */
    DEFAULT_VECTOR,         /* 0x2F  0x000000BC   -   ivINT_ECSPI1                      */
    DEFAULT_VECTOR,         /* 0x30  0x000000C0   -   ivINT_ECSPI2                      */
    DEFAULT_VECTOR,         /* 0x31  0x000000C4   -   ivINT_ECSPI3                      */
    DEFAULT_VECTOR,         /* 0x32  0x000000C8   -   ivINT_ECSPI4                      */
    DEFAULT_VECTOR,         /* 0x33  0x000000CC   -   ivINT_I2C4                        */
    DEFAULT_VECTOR,         /* 0x34  0x000000D0   -   ivINT_I2C1                        */
    DEFAULT_VECTOR,         /* 0x35  0x000000D4   -   ivINT_I2C2                        */
    DEFAULT_VECTOR,         /* 0x36  0x000000D8   -   ivINT_I2C3                        */
    DEFAULT_VECTOR,         /* 0x37  0x000000DC   -   ivINT_RDC                         */
    DEFAULT_VECTOR,         /* 0x38  0x000000E0   -   ivINT_Reserved56                  */
    DEFAULT_VECTOR,         /* 0x39  0x000000E4   -   ivINT_Reserved57                  */
    DEFAULT_VECTOR,         /* 0x3A  0x000000E8   -   ivINT_Reserved58                  */
    DEFAULT_VECTOR,         /* 0x3B  0x000000EC   -   ivINT_Reserved59                  */
    DEFAULT_VECTOR,         /* 0x3C  0x000000F0   -   ivINT_Reserved60                  */
    DEFAULT_VECTOR,         /* 0x3D  0x000000F4   -   ivINT_Reserved61                  */
    DEFAULT_VECTOR,         /* 0x3E  0x000000F8   -   ivINT_Reserved62                  */
    DEFAULT_VECTOR,         /* 0x3F  0x000000FC   -   ivINT_Reserved63                  */
    DEFAULT_VECTOR,         /* 0x40  0x00000100   -   ivINT_Reserved64                  */
    DEFAULT_VECTOR,         /* 0x41  0x00000104   -   ivINT_Reserved65                  */
    DEFAULT_VECTOR,         /* 0x42  0x00000108   -   ivINT_Reserved66                  */
    DEFAULT_VECTOR,         /* 0x43  0x0000010C   -   ivINT_Reserved67                  */
    DEFAULT_VECTOR,         /* 0x44  0x00000110   -   ivINT_Reserved68                  */
    DEFAULT_VECTOR,         /* 0x45  0x00000114   -   ivINT_Reserved69                  */
    DEFAULT_VECTOR,         /* 0x46  0x00000118   -   ivINT_Reserved70                  */
    DEFAULT_VECTOR,         /* 0x47  0x0000011C   -   ivINT_Reserved71                  */
    DEFAULT_VECTOR,         /* 0x48  0x00000120   -   ivINT_EPIT1                       */
    DEFAULT_VECTOR,         /* 0x49  0x00000124   -   ivINT_EPIT2                       */
    DEFAULT_VECTOR,         /* 0x4A  0x00000128   -   ivINT_GPIO1_INT7                  */
    DEFAULT_VECTOR,         /* 0x4B  0x0000012C   -   ivINT_GPIO1_INT6                  */
    DEFAULT_VECTOR,         /* 0x4C  0x00000130   -   ivINT_GPIO1_INT5                  */
    DEFAULT_VECTOR,         /* 0x4D  0x00000134   -   ivINT_GPIO1_INT4                  */
    DEFAULT_VECTOR,         /* 0x4E  0x00000138   -   ivINT_GPIO1_INT3                  */
    DEFAULT_VECTOR,         /* 0x4F  0x0000013C   -   ivINT_GPIO1_INT2                  */
    DEFAULT_VECTOR,         /* 0x50  0x00000140   -   ivINT_GPIO1_INT1                  */
    DEFAULT_VECTOR,         /* 0x51  0x00000144   -   ivINT_GPIO1_INT0                  */
    DEFAULT_VECTOR,         /* 0x52  0x00000148   -   ivINT_GPIO1_INT15_0               */
    DEFAULT_VECTOR,         /* 0x53  0x0000014C   -   ivINT_GPIO1_INT31_16              */
    DEFAULT_VECTOR,         /* 0x54  0x00000150   -   ivINT_GPIO2_INT15_0               */
    DEFAULT_VECTOR,         /* 0x55  0x00000154   -   ivINT_GPIO2_INT31_16              */
    DEFAULT_VECTOR,         /* 0x56  0x00000158   -   ivINT_GPIO3_INT15_0               */
    DEFAULT_VECTOR,         /* 0x57  0x0000015C   -   ivINT_GPIO3_INT31_16              */
    DEFAULT_VECTOR,         /* 0x58  0x00000160   -   ivINT_GPIO4_INT15_0               */
    DEFAULT_VECTOR,         /* 0x59  0x00000164   -   ivINT_GPIO4_INT31_16              */
    DEFAULT_VECTOR,         /* 0x5A  0x00000168   -   ivINT_GPIO5_INT15_0               */
    DEFAULT_VECTOR,         /* 0x5B  0x0000016C   -   ivINT_GPIO5_INT31_16              */
    DEFAULT_VECTOR,         /* 0x5C  0x00000170   -   ivINT_GPIO6_INT15_0               */
    DEFAULT_VECTOR,         /* 0x5D  0x00000174   -   ivINT_GPIO6_INT31_16              */
    DEFAULT_VECTOR,         /* 0x5E  0x00000178   -   ivINT_GPIO7_INT15_0               */
    DEFAULT_VECTOR,         /* 0x5F  0x0000017C   -   ivINT_GPIO7_INT31_16              */
    DEFAULT_VECTOR,         /* 0x60  0x00000180   -   ivINT_Reserved96                  */
    DEFAULT_VECTOR,         /* 0x61  0x00000184   -   ivINT_Reserved97                  */
    DEFAULT_VECTOR,         /* 0x62  0x00000188   -   ivINT_Reserved98                  */
    DEFAULT_VECTOR,         /* 0x63  0x0000018C   -   ivINT_Reserved99                  */
    DEFAULT_VECTOR,         /* 0x64  0x00000190   -   ivINT_Reserved100                 */
    DEFAULT_VECTOR,         /* 0x65  0x00000194   -   ivINT_Reserved101                 */
    DEFAULT_VECTOR,         /* 0x66  0x00000198   -   ivINT_Reserved102                 */
    DEFAULT_VECTOR,         /* 0x67  0x0000019C   -   ivINT_CCM_INT1                    */
    DEFAULT_VECTOR,         /* 0x68  0x000001A0   -   ivINT_CCM_INT2                    */
    DEFAULT_VECTOR,         /* 0x69  0x000001A4   -   ivINT_Reserved105                 */
    DEFAULT_VECTOR,         /* 0x6A  0x000001A8   -   ivINT_MU_INT_A9                   */
    DEFAULT_VECTOR,         /* 0x6B  0x000001AC   -   ivINT_Reserved107                 */
    DEFAULT_VECTOR,         /* 0x6C  0x000001B0   -   ivINT_Reserved108                 */
    DEFAULT_VECTOR,         /* 0x6D  0x000001B4   -   ivINT_Reserved109                 */
    DEFAULT_VECTOR,         /* 0x6E  0x000001B8   -   ivINT_Reserved110                 */
    DEFAULT_VECTOR,         /* 0x6F  0x000001BC   -   ivINT_Reserved111                 */
    DEFAULT_VECTOR,         /* 0x70  0x000001C0   -   ivINT_Reserved112                 */
    DEFAULT_VECTOR,         /* 0x71  0x000001C4   -   ivINT_Reserved113                 */
    DEFAULT_VECTOR,         /* 0x72  0x000001C8   -   ivINT_Reserved114                 */
    DEFAULT_VECTOR,         /* 0x73  0x000001CC   -   ivINT_MU_INT_M4                   */
    DEFAULT_VECTOR,         /* 0x74  0x000001D0   -   ivINT_ADC1                        */
    DEFAULT_VECTOR,         /* 0x75  0x000001D4   -   ivINT_ADC2                        */
    DEFAULT_VECTOR,         /* 0x76  0x000001D8   -   ivINT_Reserved118                 */
    DEFAULT_VECTOR,         /* 0x77  0x000001DC   -   ivINT_Reserved119                 */
    DEFAULT_VECTOR,         /* 0x78  0x000001E0   -   ivINT_Reserved120                 */
    DEFAULT_VECTOR,         /* 0x79  0x000001E4   -   ivINT_Reserved121                 */
    DEFAULT_VECTOR,         /* 0x7A  0x000001E8   -   ivINT_Reserved122                 */
    DEFAULT_VECTOR,         /* 0x7B  0x000001EC   -   ivINT_QSPI1                       */
    DEFAULT_VECTOR,         /* 0x7C  0x000001F0   -   ivINT_Reserved124                 */
    DEFAULT_VECTOR,         /* 0x7D  0x000001F4   -   ivINT_QSPI2                       */
    DEFAULT_VECTOR,         /* 0x7E  0x000001F8   -   ivINT_FLEXCAN1                    */
    DEFAULT_VECTOR,         /* 0x7F  0x000001FC   -   ivINT_FLEXCAN2                    */
    DEFAULT_VECTOR,         /* 0x80  0x00000200   -   ivINT_Reserved128                 */
    DEFAULT_VECTOR,         /* 0x81  0x00000204   -   ivINT_Reserved129                 */
    DEFAULT_VECTOR,         /* 0x82  0x00000208   -   ivINT_CANFD1                      */
    DEFAULT_VECTOR,         /* 0x83  0x0000020C   -   ivINT_CANFD2                      */
    DEFAULT_VECTOR,         /* 0x84  0x00000210   -   ivINT_SEMA4_CP0                   */
    DEFAULT_VECTOR,         /* 0x85  0x00000214   -   ivINT_Reserved133                 */
    DEFAULT_VECTOR,         /* 0x86  0x00000218   -   ivINT_Reserved134                 */
    DEFAULT_VECTOR,         /* 0x87  0x0000021C   -   ivINT_Reserved135                 */
    DEFAULT_VECTOR,         /* 0x88  0x00000220   -   ivINT_Reserved136                 */
    DEFAULT_VECTOR,         /* 0x89  0x00000224   -   ivINT_Reserved137                 */
    DEFAULT_VECTOR,         /* 0x8A  0x00000228   -   ivINT_Reserved138                 */
    DEFAULT_VECTOR,         /* 0x8B  0x0000022C   -   ivINT_Reserved139                 */
    DEFAULT_VECTOR,         /* 0x8C  0x00000230   -   ivINT_Reserved140                 */
    DEFAULT_VECTOR,         /* 0x8D  0x00000234   -   ivINT_Reserved141                 */
    DEFAULT_VECTOR,         /* 0x8E  0x00000238   -   ivINT_Reserved142                 */
    DEFAULT_VECTOR,         /* 0x8F  0x0000023C   -   ivINT_Reserved143                 */
    DEFAULT_VECTOR,         /* 0x90  0x00000240   -   ivINT_Reserved144                 */
    DEFAULT_VECTOR,         /* 0x91  0x00000244   -   ivINT_Reserved145                 */
    DEFAULT_VECTOR,         /* 0x92  0x00000248   -   ivINT_Reserved146                 */
    DEFAULT_VECTOR,         /* 0x93  0x0000024C   -   ivINT_Reserved147                 */
    DEFAULT_VECTOR,         /* 0x94  0x00000250   -   ivINT_Reserved148                 */
    DEFAULT_VECTOR,         /* 0x95  0x00000254   -   ivINT_Reserved149                 */
    DEFAULT_VECTOR,         /* 0x96  0x00000258   -   ivINT_Reserved150                 */
    DEFAULT_VECTOR,         /* 0x97  0x0000025C   -   ivINT_Reserved151                 */
    DEFAULT_VECTOR,         /* 0x98  0x00000260   -   ivINT_Reserved152                 */
    DEFAULT_VECTOR,         /* 0x99  0x00000264   -   ivINT_Reserved153                 */
    DEFAULT_VECTOR,         /* 0x9A  0x00000268   -   ivINT_Reserved154                 */
    DEFAULT_VECTOR,         /* 0x9B  0x0000026C   -   ivINT_Reserved155                 */
    DEFAULT_VECTOR,         /* 0x9C  0x00000270   -   ivINT_Reserved156                 */
    DEFAULT_VECTOR,         /* 0x9D  0x00000274   -   ivINT_Reserved157                 */
    DEFAULT_VECTOR,         /* 0x9E  0x00000278   -   ivINT_Reserved158                 */
    DEFAULT_VECTOR,         /* 0x9F  0x0000027C   -   ivINT_Reserved159                 */
    DEFAULT_VECTOR,         /* 0xA0  0x00000280   -   ivINT_Reserved160                 */
    DEFAULT_VECTOR,         /* 0xA1  0x00000284   -   ivINT_Reserved161                 */
    DEFAULT_VECTOR,         /* 0xA2  0x00000288   -   ivINT_Reserved162                 */
    DEFAULT_VECTOR,         /* 0xA3  0x0000028C   -   ivINT_Reserved163                 */
    DEFAULT_VECTOR,         /* 0xA4  0x00000290   -   ivINT_Reserved164                 */
    DEFAULT_VECTOR,         /* 0xA5  0x00000294   -   ivINT_Reserved165                 */
    DEFAULT_VECTOR,         /* 0xA6  0x00000298   -   ivINT_Reserved166                 */
    DEFAULT_VECTOR,         /* 0xA7  0x0000029C   -   ivINT_Reserved167                 */
    DEFAULT_VECTOR,         /* 0xA8  0x000002A0   -   ivINT_Reserved168                 */
    DEFAULT_VECTOR,         /* 0xA9  0x000002A4   -   ivINT_Reserved169                 */
    DEFAULT_VECTOR,         /* 0xAA  0x000002A8   -   ivINT_Reserved170                 */
    DEFAULT_VECTOR,         /* 0xAB  0x000002AC   -   ivINT_Reserved171                 */
    DEFAULT_VECTOR,         /* 0xAC  0x000002B0   -   ivINT_Reserved172                 */
    DEFAULT_VECTOR,         /* 0xAD  0x000002B4   -   ivINT_Reserved173                 */
    DEFAULT_VECTOR,         /* 0xAE  0x000002B8   -   ivINT_Reserved174                 */
    DEFAULT_VECTOR,         /* 0xAF  0x000002BC   -   ivINT_Reserved175                 */
    DEFAULT_VECTOR,         /* 0xB0  0x000002C0   -   ivINT_Reserved176                 */
    DEFAULT_VECTOR,         /* 0xB1  0x000002C4   -   ivINT_Reserved177                 */
    DEFAULT_VECTOR,         /* 0xB2  0x000002C8   -   ivINT_Reserved178                 */
    DEFAULT_VECTOR,         /* 0xB3  0x000002CC   -   ivINT_Reserved179                 */
    DEFAULT_VECTOR,         /* 0xB4  0x000002D0   -   ivINT_Reserved180                 */
    DEFAULT_VECTOR,         /* 0xB5  0x000002D4   -   ivINT_Reserved181                 */
    DEFAULT_VECTOR,         /* 0xB6  0x000002D8   -   ivINT_Reserved182                 */
    DEFAULT_VECTOR,         /* 0xB7  0x000002DC   -   ivINT_Reserved183                 */
    DEFAULT_VECTOR,         /* 0xB8  0x000002E0   -   ivINT_Reserved184                 */
    DEFAULT_VECTOR,         /* 0xB9  0x000002E4   -   ivINT_Reserved185                 */
    DEFAULT_VECTOR,         /* 0xBA  0x000002E8   -   ivINT_Reserved186                 */
    DEFAULT_VECTOR,         /* 0xBB  0x000002EC   -   ivINT_Reserved187                 */
    DEFAULT_VECTOR,         /* 0xBC  0x000002F0   -   ivINT_Reserved188                 */
    DEFAULT_VECTOR,         /* 0xBD  0x000002F4   -   ivINT_Reserved189                 */
    DEFAULT_VECTOR,         /* 0xBE  0x000002F8   -   ivINT_Reserved190                 */
    DEFAULT_VECTOR,         /* 0xBF  0x000002FC   -   ivINT_Reserved191                 */
    DEFAULT_VECTOR,         /* 0xC0  0x00000300   -   ivINT_Reserved192                 */
    DEFAULT_VECTOR,         /* 0xC1  0x00000304   -   ivINT_Reserved193                 */
    DEFAULT_VECTOR,         /* 0xC2  0x00000308   -   ivINT_Reserved194                 */
    DEFAULT_VECTOR,         /* 0xC3  0x0000030C   -   ivINT_Reserved195                 */
    DEFAULT_VECTOR,         /* 0xC4  0x00000310   -   ivINT_Reserved196                 */
    DEFAULT_VECTOR,         /* 0xC5  0x00000314   -   ivINT_Reserved197                 */
    DEFAULT_VECTOR,         /* 0xC6  0x00000318   -   ivINT_Reserved198                 */
    DEFAULT_VECTOR,         /* 0xC7  0x0000031C   -   ivINT_Reserved199                 */
    DEFAULT_VECTOR,         /* 0xC8  0x00000320   -   ivINT_Reserved200                 */
    DEFAULT_VECTOR,         /* 0xC9  0x00000324   -   ivINT_Reserved201                 */
    DEFAULT_VECTOR,         /* 0xCA  0x00000328   -   ivINT_Reserved202                 */
    DEFAULT_VECTOR,         /* 0xCB  0x0000032C   -   ivINT_Reserved203                 */
    DEFAULT_VECTOR,         /* 0xCC  0x00000330   -   ivINT_Reserved204                 */
    DEFAULT_VECTOR,         /* 0xCD  0x00000334   -   ivINT_Reserved205                 */
    DEFAULT_VECTOR,         /* 0xCE  0x00000338   -   ivINT_Reserved206                 */
    DEFAULT_VECTOR,         /* 0xCF  0x0000033C   -   ivINT_Reserved207                 */
    DEFAULT_VECTOR,         /* 0xD0  0x00000340   -   ivINT_Reserved208                 */
    DEFAULT_VECTOR,         /* 0xD1  0x00000344   -   ivINT_Reserved209                 */
    DEFAULT_VECTOR,         /* 0xD2  0x00000348   -   ivINT_Reserved210                 */
    DEFAULT_VECTOR,         /* 0xD3  0x0000034C   -   ivINT_Reserved211                 */
    DEFAULT_VECTOR,         /* 0xD4  0x00000350   -   ivINT_Reserved212                 */
    DEFAULT_VECTOR,         /* 0xD5  0x00000354   -   ivINT_Reserved213                 */
    DEFAULT_VECTOR,         /* 0xD6  0x00000358   -   ivINT_Reserved214                 */
    DEFAULT_VECTOR,         /* 0xD7  0x0000035C   -   ivINT_Reserved215                 */
    DEFAULT_VECTOR,         /* 0xD8  0x00000360   -   ivINT_Reserved216                 */
    DEFAULT_VECTOR,         /* 0xD9  0x00000364   -   ivINT_Reserved217                 */
    DEFAULT_VECTOR,         /* 0xDA  0x00000368   -   ivINT_Reserved218                 */
    DEFAULT_VECTOR,         /* 0xDB  0x0000036C   -   ivINT_Reserved219                 */
    DEFAULT_VECTOR,         /* 0xDC  0x00000370   -   ivINT_Reserved220                 */
    DEFAULT_VECTOR,         /* 0xDD  0x00000374   -   ivINT_Reserved221                 */
    DEFAULT_VECTOR,         /* 0xDE  0x00000378   -   ivINT_Reserved222                 */
    DEFAULT_VECTOR,         /* 0xDF  0x0000037C   -   ivINT_Reserved223                 */
    DEFAULT_VECTOR,         /* 0xE0  0x00000380   -   ivINT_Reserved224                 */
    DEFAULT_VECTOR,         /* 0xE1  0x00000384   -   ivINT_Reserved225                 */
    DEFAULT_VECTOR,         /* 0xE2  0x00000388   -   ivINT_Reserved226                 */
    DEFAULT_VECTOR,         /* 0xE3  0x0000038C   -   ivINT_Reserved227                 */
    DEFAULT_VECTOR,         /* 0xE4  0x00000390   -   ivINT_Reserved228                 */
    DEFAULT_VECTOR,         /* 0xE5  0x00000394   -   ivINT_Reserved229                 */
    DEFAULT_VECTOR,         /* 0xE6  0x00000398   -   ivINT_Reserved230                 */
    DEFAULT_VECTOR,         /* 0xE7  0x0000039C   -   ivINT_Reserved231                 */
    DEFAULT_VECTOR,         /* 0xE8  0x000003A0   -   ivINT_Reserved232                 */
    DEFAULT_VECTOR,         /* 0xE9  0x000003A4   -   ivINT_Reserved233                 */
    DEFAULT_VECTOR,         /* 0xEA  0x000003A8   -   ivINT_Reserved234                 */
    DEFAULT_VECTOR,         /* 0xEB  0x000003AC   -   ivINT_Reserved235                 */
    DEFAULT_VECTOR,         /* 0xEC  0x000003B0   -   ivINT_Reserved236                 */
    DEFAULT_VECTOR,         /* 0xED  0x000003B4   -   ivINT_Reserved237                 */
    DEFAULT_VECTOR,         /* 0xEE  0x000003B8   -   ivINT_Reserved238                 */
    DEFAULT_VECTOR,         /* 0xEF  0x000003BC   -   ivINT_Reserved239                 */
    DEFAULT_VECTOR,         /* 0xF0  0x000003C0   -   ivINT_Reserved240                 */
    DEFAULT_VECTOR,         /* 0xF1  0x000003C4   -   ivINT_Reserved241                 */
    DEFAULT_VECTOR,         /* 0xF2  0x000003C8   -   ivINT_Reserved242                 */
    DEFAULT_VECTOR,         /* 0xF3  0x000003CC   -   ivINT_Reserved243                 */
    DEFAULT_VECTOR,         /* 0xF4  0x000003D0   -   ivINT_Reserved244                 */
    DEFAULT_VECTOR,         /* 0xF5  0x000003D4   -   ivINT_Reserved245                 */
    DEFAULT_VECTOR,         /* 0xF6  0x000003D8   -   ivINT_Reserved246                 */
    DEFAULT_VECTOR,         /* 0xF7  0x000003DC   -   ivINT_Reserved247                 */
    DEFAULT_VECTOR,         /* 0xF8  0x000003E0   -   ivINT_Reserved248                 */
    DEFAULT_VECTOR,         /* 0xF9  0x000003E4   -   ivINT_Reserved249                 */
    DEFAULT_VECTOR,         /* 0xFA  0x000003E8   -   ivINT_Reserved250                 */
    DEFAULT_VECTOR,         /* 0xFB  0x000003EC   -   ivINT_Reserved251                 */
    DEFAULT_VECTOR,         /* 0xFC  0x000003F0   -   ivINT_Reserved252                 */
    DEFAULT_VECTOR,         /* 0xFD  0x000003F4   -   ivINT_Reserved253                 */
    DEFAULT_VECTOR,         /* 0xFE  0x000003F8   -   ivINT_Reserved254                 */
    DEFAULT_VECTOR,         /* 0xFF  0x000003FC   -   ivINT_Reserved255                 */
};
