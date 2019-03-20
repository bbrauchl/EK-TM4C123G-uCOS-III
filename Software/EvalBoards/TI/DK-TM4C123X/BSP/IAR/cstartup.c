/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*               This file is provided as an example on how to use Micrium products.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only. This file can be modified as
*               required to meet the end-product requirements.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can find our product's user manual, API reference, release notes and
*               more information at https://doc.micrium.com.
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*
*                                          EXCEPTION VECTORS
*
*                                      Texas Instruments TM4C129x
*                                                on the
*
*                                             DK-TM4C129X
*                                           Development Kit
*
* Filename      : cstartup.c
* Version       : V1.00
* Programmer(s) : FF
*********************************************************************************************************
*/

#include  "..\..\..\..\..\uc-lib\lib_def.h"
#include  "..\bsp\bsp_int.h"
#include  <os.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/

typedef  union {
    CPU_FNCT_VOID   Fnct;
    void           *Ptr;
} APP_INTVECT_ELEM;

/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

#pragma language=extended
#pragma segment="CSTACK"

static  void  App_Reset_ISR       (void);

static  void  App_NMI_ISR         (void);

static  void  App_Fault_ISR       (void);

static  void  App_BusFault_ISR    (void);

static  void  App_UsageFault_ISR  (void);

static  void  App_MemFault_ISR    (void);

static  void  App_Spurious_ISR    (void);

extern  void  __iar_program_start (void);

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                  EXCEPTION / INTERRUPT VECTOR TABLE
*
* Note(s) : (1) The Cortex-M3 may have up to 256 external interrupts, which are the final entries in the
*               vector table.  The TM4C129xxx has 113 external interrupt vectors.
*
*           (2) Interrupts vector 2-13 are implemented in this file as infinite loop for debuging
*               purposes only. The application might implement a recover procedure if it is needed.
*
*           (3) OS_CPU_PendSVHandler() and OS_CPU_SysTickHandler() are implemented in the generic OS
*               port.
*********************************************************************************************************
*/

__root  const  APP_INTVECT_ELEM  __vector_table[] @ ".intvec" = {
    { .Ptr = (void *)__sfe( "CSTACK" )},                        /*   0, SP start value.                                 */
    App_Reset_ISR,                                              /*   1, PC start value.                                 */
    App_NMI_ISR,                                                /*   2, NMI.                                            */
    App_Fault_ISR,                                              /*   3, Hard Fault.                                     */
    App_MemFault_ISR,                                           /*   4, Memory Management.                              */
    App_BusFault_ISR,                                           /*   5, Bus Fault.                                      */
    App_UsageFault_ISR,                                         /*   6, Usage Fault.                                    */
    App_Spurious_ISR,                                           /*   7, Reserved.                                       */
    App_Spurious_ISR,                                           /*   8, Reserved.                                       */
    App_Spurious_ISR,                                           /*   9, Reserved.                                       */
    App_Spurious_ISR,                                           /*  10, Reserved.                                       */
    App_Spurious_ISR,                                           /*  11, SVCall.                                         */
    App_Spurious_ISR,                                           /*  12, Debug Monitor.                                  */
    App_Spurious_ISR,                                           /*  13, Reserved.                                       */
    OS_CPU_PendSVHandler,                                       /*  14, PendSV Handler.                                 */
    OS_CPU_SysTickHandler,                                      /*  15, uC/OS-III Tick ISR Handler.                     */

    BSP_IntHandlerGPIOA,                                        /*  16, INTISR[  0]  GPIO Port A.                       */
    BSP_IntHandlerGPIOB,                                        /*  17, INTISR[  1]  GPIO Port B.                       */
    BSP_IntHandlerGPIOC,                                        /*  18, INTISR[  2]  GPIO Port C.                       */
    BSP_IntHandlerGPIOD,                                        /*  19, INTISR[  3]  GPIO Port D.                       */
    BSP_IntHandlerGPIOE,                                        /*  20, INTISR[  4]  GPIO Port E.                       */
    BSP_IntHandlerUART0,                                        /*  21, INTISR[  5]  UART0.                             */
    BSP_IntHandlerUART1,                                        /*  22, INTISR[  6]  UART1.                             */
    BSP_IntHandlerSSI0,                                         /*  23, INTISR[  7]  SSI0.                              */
    BSP_IntHandlerI2C0,                                         /*  24, INTISR[  8]  I2C0.                              */
    BSP_IntHandlerPWM_FAULT,                                    /*  25, INTISR[  9]  PWM Fault.                         */
    BSP_IntHandlerPWM_GEN0,                                     /*  26, INTISR[ 10]  PWM Generator 0.                   */
    BSP_IntHandlerPWM_GEN1,                                     /*  27, INTISR[ 11]  PWM Generator 1.                   */
    BSP_IntHandlerPWM_GEN2,                                     /*  28, INTISR[ 12]  PWM Generator 2.                   */
    BSP_IntHandlerQEI0,                                         /*  29, INTISR[ 13]  QEI0.                              */
    BSP_IntHandlerADC0_0,                                       /*  30, INTISR[ 14]  ADC0 Sequence 0.                   */
    BSP_IntHandlerADC0_1,                                       /*  31, INTISR[ 15]  ADC0 Sequence 1.                   */
    BSP_IntHandlerADC0_2,                                       /*  32, INTISR[ 16]  ADC0 Sequence 2.                   */
    BSP_IntHandlerADC0_3,                                       /*  33, INTISR[ 17]  ADC0 Sequence 3.                   */
    BSP_IntHandlerWDTO_WDT1,                                    /*  34, INTISR[ 18]  Watchdog Timers 0 and 1.           */
    BSP_IntHandlerTMR0A,                                        /*  35, INTISR[ 19]  16/32-Bit Timer 0A.                */
    BSP_IntHandlerTMR0B,                                        /*  36, INTISR[ 20]  16/32-Bit Timer 0B.                */
    BSP_IntHandlerTMR1A,                                        /*  37, INTISR[ 21]  16/32-Bit Timer 1A.                */
    BSP_IntHandlerTMR1B,                                        /*  38, INTISR[ 22]  16/32-Bit Timer 1B.                */
    BSP_IntHandlerTMR2A,                                        /*  39, INTISR[ 23]  16/32-Bit Timer 2A.                */
    BSP_IntHandlerTMR2B,                                        /*  40, INTISR[ 24]  16/32-Bit Timer 2B.                */
    BSP_IntHandlerACOMP0,                                       /*  41, INTISR[ 25]  Analog Comparator 0.               */
    BSP_IntHandlerACOMP1,                                       /*  42, INTISR[ 26]  Analog Comparator 1.               */
    BSP_IntHandlerACOMP2,                                       /*  43, INTISR[ 27]  Analog Comparator 2.               */
    BSP_IntHandlerSYS_CTRL,                                     /*  44, INTISR[ 28]  System Control.                    */
    BSP_IntHandlerFLASH,                                        /*  45, INTISR[ 29]  Flash Memory Control.              */
    BSP_IntHandlerGPIOF,                                        /*  46, INTISR[ 30]  GPIO Port F.                       */
    BSP_IntHandlerGPIOG,                                        /*  47, INTISR[ 31]  GPIO Port G.                       */
    BSP_IntHandlerGPIOH,                                        /*  48, INTISR[ 32]  GPIO Port H.                       */
    BSP_IntHandlerUART2,                                        /*  49, INTISR[ 33]  UART2.                             */
    BSP_IntHandlerSSI1,                                         /*  50, INTISR[ 34]  SSI1.                              */
    BSP_IntHandlerTMR3A,                                        /*  51, INTISR[ 35]  16/32-Bit Timer 3A.                */
    BSP_IntHandlerTMR3B,                                        /*  52, INTISR[ 36]  16/32-Bit Timer 3B.                */
    BSP_IntHandlerI2C1,                                         /*  53, INTISR[ 37]  I2C1.                              */
    BSP_IntHandlerCAN0,                                         /*  54, INTISR[ 38]  CAN0.                              */
    BSP_IntHandlerCAN1,                                         /*  55, INTISR[ 39]  CAN1.                              */
    BSP_IntHandlerETHER_MAC,                                    /*  56, INTISR[ 40]  Ethernet MAC.                      */
    BSP_IntHandlerHIB,                                          /*  57, INTISR[ 41]  HIB(Power Island).                 */
    BSP_IntHandlerUSB_MAC,                                      /*  58, INTISR[ 42]  USB MAC.                           */
    BSP_IntHandlerPWM_GEN3,                                     /*  59, INTISR[ 43]  PWM Generator 3.                   */
    BSP_IntHandlerUDMA0_SOFT,                                   /*  60, INTISR[ 44]  uDMA 0 Software.                   */
    BSP_IntHandlerUDAM0_ERR,                                    /*  61, INTISR[ 45]  uDMA 0 Error.                      */
    BSP_IntHandlerADC1_0,                                       /*  62, INTISR[ 46]  ADC1 Sequence 0.                   */
    BSP_IntHandlerADC1_1,                                       /*  63, INTISR[ 47]  ADC1 Sequence 1.                   */
    BSP_IntHandlerADC1_2,                                       /*  64, INTISR[ 48]  ADC1 Sequence 2.                   */
    BSP_IntHandlerADC1_3,                                       /*  65, INTISR[ 49]  ADC1 Sequence 3.                   */
    BSP_IntHandlerEPI0,                                         /*  66, INTISR[ 50]  EPI0.                              */
    BSP_IntHandlerGPIOJ,                                        /*  67, INTISR[ 51]  GPIO Port J.                       */
    BSP_IntHandlerGPIOK,                                        /*  68, INTISR[ 52]  GPIO Port K.                       */
    BSP_IntHandlerGPIOL,                                        /*  69, INTISR[ 53]  GPIO Port L.                       */
    BSP_IntHandlerSSI2,                                         /*  70, INTISR[ 54]  SSI2.                              */
    BSP_IntHandlerSSI3,                                         /*  71, INTISR[ 55]  SSI3.                              */
    BSP_IntHandlerUART3,                                        /*  72, INTISR[ 56]  UART3.                             */
    BSP_IntHandlerUART4,                                        /*  73, INTISR[ 57]  UART4.                             */
    BSP_IntHandlerUART5,                                        /*  74, INTISR[ 58]  UART5.                             */
    BSP_IntHandlerUART6,                                        /*  75, INTISR[ 59]  UART6.                             */
    BSP_IntHandlerUART7,                                        /*  76, INTISR[ 60]  UART7.                             */
    BSP_IntHandlerI2C2,                                         /*  77, INTISR[ 61]  I2C 2.                             */
    BSP_IntHandlerI2C3,                                         /*  78, INTISR[ 62]  I2C 3.                             */
    BSP_IntHandlerTMR4A,                                        /*  79, INTISR[ 63]  Timer 4A.                          */
    BSP_IntHandlerTMR4B,                                        /*  80, INTISR[ 64]  Timer 4B.                          */
    BSP_IntHandlerTMR5A,                                        /*  81, INTISR[ 65]  Timer 5A.                          */
    BSP_IntHandlerTMR5B,                                        /*  82, INTISR[ 66]  Timer 5B.                          */
    BSP_IntHandlerFP,                                           /*  83, INTISR[ 67]  FP Exception(imprecise).           */

    BSP_IntHandlerRSVD68,                                       /*  84, INTISR[ 68]  Reserved.                          */
    BSP_IntHandlerRSVD69,                                       /*  85, INTISR[ 69]  Reserved.                          */

    BSP_IntHandlerI2C4,                                         /*  86, INTISR[ 70]  I2C 4.                             */
    BSP_IntHandlerI2C5,                                         /*  87, INTISR[ 71]  I2C 5.                             */
    BSP_IntHandlerGPIOM,                                        /*  88, INTISR[ 72]  GPIO Port M.                       */
    BSP_IntHandlerGPION,                                        /*  89, INTISR[ 73]  GPIO Port N.                       */

    BSP_IntHandlerRSVD74,                                       /*  90, INTISR[ 74]  Reserved.                          */

    BSP_IntHandlerTAMPER,                                       /*  91, INTISR[ 75]  Tamper.                            */
    BSP_IntHandlerGPIOP0,                                       /*  92, INTISR[ 76]  GPIO Port P(Summary or P0).        */
    BSP_IntHandlerGPIOP1,                                       /*  93, INTISR[ 77]  GPIO Port P1.                      */
    BSP_IntHandlerGPIOP2,                                       /*  94, INTISR[ 78]  GPIO Port P2.                      */
    BSP_IntHandlerGPIOP3,                                       /*  95, INTISR[ 79]  GPIO Port P3.                      */
    BSP_IntHandlerGPIOP4,                                       /*  96, INTISR[ 80]  GPIO Port P4.                      */
    BSP_IntHandlerGPIOP5,                                       /*  97, INTISR[ 81]  GPIO Port P5.                      */
    BSP_IntHandlerGPIOP6,                                       /*  98, INTISR[ 82]  GPIO Port P6.                      */
    BSP_IntHandlerGPIOP7,                                       /*  99, INTISR[ 83]  GPIO Port P7.                      */
    BSP_IntHandlerGPIOQ0,                                       /* 100, INTISR[ 84]  GPIO Port Q(Summary or Q0).        */
    BSP_IntHandlerGPIOQ1,                                       /* 101, INTISR[ 85]  GPIO Port Q1.                      */
    BSP_IntHandlerGPIOQ2,                                       /* 102, INTISR[ 86]  GPIO Port Q2.                      */
    BSP_IntHandlerGPIOQ3,                                       /* 103, INTISR[ 87]  GPIO Port Q3.                      */
    BSP_IntHandlerGPIOQ4,                                       /* 104, INTISR[ 88]  GPIO Port Q4.                      */
    BSP_IntHandlerGPIOQ5,                                       /* 105, INTISR[ 89]  GPIO Port Q5.                      */
    BSP_IntHandlerGPIOQ6,                                       /* 106, INTISR[ 90]  GPIO Port Q6.                      */
    BSP_IntHandlerGPIOQ7,                                       /* 107, INTISR[ 91]  GPIO Port Q7.                      */
    BSP_IntHandlerGPIOR,                                        /* 108, INTISR[ 92]  GPIO Port R.                       */
    BSP_IntHandlerGPIOS,                                        /* 109, INTISR[ 93]  GPIO Port S.                       */
    BSP_IntHandlerSHA_MD5,                                      /* 110, INTISR[ 94]  SHA/MD5.                           */
    BSP_IntHandlerAES,                                          /* 111, INTISR[ 95]  AES.                               */
    BSP_IntHandlerDES,                                          /* 112, INTISR[ 96]  DES.                               */
    BSP_IntHandlerLCD,                                          /* 113, INTISR[ 97]  LCD.                               */
    BSP_IntHandlerTMR6A,                                        /* 114, INTISR[ 98]  16/32-Bit Timer 6A.                */
    BSP_IntHandlerTMR6B,                                        /* 115, INTISR[ 99]  16/32-Bit Timer 6B.                */
    BSP_IntHandlerTMR7A,                                        /* 116, INTISR[100]  16/32-Bit Timer 7A.                */
    BSP_IntHandlerTMR7B,                                        /* 117, INTISR[101]  16/32-Bit Timer 7B.                */
    BSP_IntHandlerI2C6,                                         /* 118, INTISR[102]  I2C 6.                             */
    BSP_IntHandlerI2C7,                                         /* 119, INTISR[103]  I2C 7.                             */

    BSP_IntHandlerRSVD104,                                      /* 120, INTISR[104]  Reserved.                          */

    BSP_IntHandler1WIRE,                                        /* 121, INTISR[105]  1-Wire.                            */

    BSP_IntHandlerRSVD106,                                      /* 122, INTISR[106]  Reserved.                          */
    BSP_IntHandlerRSVD107,                                      /* 123, INTISR[107]  Reserved.                          */
    BSP_IntHandlerRSVD108,                                      /* 124, INTISR[108]  Reserved.                          */

    BSP_IntHandlerI2C8,                                         /* 125, INTISR[109]  I2C 8.                             */
    BSP_IntHandlerI2C9,                                         /* 126, INTISR[110]  I2C 9.                             */
    BSP_IntHandlerGPIOT                                         /* 127, INTISR[111]  GPIO T.                            */
};

/*
*********************************************************************************************************
*                                            App_Reset_ISR()
*
* Description : Handle Reset.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_Reset_ISR (void)
{
#if __ARMVFP__                                                  /* Enable access to Floating-point coprocessor.         */
    CPU_REG_NVIC_CPACR = CPU_REG_NVIC_CPACR_CP10_FULL_ACCESS | CPU_REG_NVIC_CPACR_CP11_FULL_ACCESS;

    DEF_BIT_CLR(CPU_REG_SCB_FPCCR, DEF_BIT_31);                 /* Disable automatic FP register content                */
    DEF_BIT_CLR(CPU_REG_SCB_FPCCR, DEF_BIT_30);                 /* Disable Lazy context switch                          */
#endif

    __iar_program_start();
}

/*
*********************************************************************************************************
*                                            App_NMI_ISR()
*
* Description : Handle Non-Maskable Interrupt (NMI).
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : (1) Since the NMI is not being used, this serves merely as a catch for a spurious
*                   exception.
*********************************************************************************************************
*/

static  void  App_NMI_ISR (void)
{
    while (DEF_TRUE) {
        ;
    }
}


/*
*********************************************************************************************************
*                                             App_Fault_ISR()
*
* Description : Handle hard fault.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_Fault_ISR (void)
{
    while (DEF_TRUE) {
        ;
    }
}


/*
*********************************************************************************************************
*                                           App_BusFault_ISR()
*
* Description : Handle bus fault.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_BusFault_ISR (void)
{
    while (DEF_TRUE) {
        ;
    }
}


/*
*********************************************************************************************************
*                                          App_UsageFault_ISR()
*
* Description : Handle usage fault.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_UsageFault_ISR (void)
{
    while (DEF_TRUE) {
        ;
    }
}


/*
*********************************************************************************************************
*                                           App_MemFault_ISR()
*
* Description : Handle memory fault.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_MemFault_ISR (void)
{
    while (DEF_TRUE) {
        ;
    }
}


/*
*********************************************************************************************************
*                                           App_Spurious_ISR()
*
* Description : Handle spurious interrupt.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  App_Spurious_ISR (void)
{
    while (DEF_TRUE) {
        ;
    }
}
