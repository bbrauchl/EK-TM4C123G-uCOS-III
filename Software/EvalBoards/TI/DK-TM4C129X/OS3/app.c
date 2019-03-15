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
*                                           APPLICATION CODE
*
*                                      Texas Instruments TM4C129x
*                                                on the
*
*                                             DK-TM4C129X
*                                           Development Kit
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : FF
*********************************************************************************************************
* Note(s)       : None.
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include  "app_cfg.h"
#include  <cpu_core.h>
#include  <os.h>

#include  "..\bsp\bsp.h"
#include  "..\bsp\bsp_led.h"
#include  "..\bsp\bsp_sys.h"


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/


/*$PAGE*/
/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

static  CPU_STK  AppStartTaskStk[APP_CFG_TASK_START_STK_SIZE];
static  OS_TCB   AppStartTaskTCB;

/*
*********************************************************************************************************
*                                            LOCAL MACRO'S
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart (void  *p_arg);

static  void  AppTaskCreate (void);


/*$PAGE*/
/*
*********************************************************************************************************
*                                               main()
*
* Description : Entry point for C code.
*
* Arguments   : none.
*
* Returns     : none.
*
* Note(s)     : (1) It is assumed that your code will call main() once you have performed all necessary
*                   initialization.
*********************************************************************************************************
*/

int  main (void)
{
    OS_ERR  err;


    CPU_IntDis();                                               /* Disable all interrupts.                              */

    OSInit(&err);                                               /* Init uC/OS-III.                                      */

    OSTaskCreate((OS_TCB     *)&AppStartTaskTCB,                /* Create the start task                                */
                 (CPU_CHAR   *)"Startup Task",
                 (OS_TASK_PTR ) AppTaskStart,
                 (void       *) 0,
                 (OS_PRIO     ) APP_CFG_TASK_START_PRIO,
                 (CPU_STK    *)&AppStartTaskStk[0],
                 (CPU_STK_SIZE) APP_CFG_TASK_START_STK_SIZE / 10u,
                 (CPU_STK_SIZE) APP_CFG_TASK_START_STK_SIZE,
                 (OS_MSG_QTY  ) 0u,
                 (OS_TICK     ) 0u,
                 (void       *) 0,
                 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR     *)&err);

    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */

    while (1) {
        ;
    }
}


/*$PAGE*/
/*
*********************************************************************************************************
*                                           App_TaskStart()
*
* Description : Startup task example code.
*
* Arguments   : p_arg       Argument passed by 'OSTaskCreate()'.
*
* Returns     : none.
*
* Created by  : main().
*
* Notes       : (1) The ticker MUST be initialized AFTER multitasking has started.
*********************************************************************************************************
*/

static  void  AppTaskStart (void *p_arg)
{
    CPU_INT32U  cpu_clk_freq;
    CPU_INT32U  cnts;
    OS_ERR    err_os;


   (void)&p_arg;

    BSP_Init();                                                 /* Initialize BSP functions                             */

    cpu_clk_freq = BSP_SysClkFreqGet();                         /* Determine SysTick reference freq.                    */
    cnts         = cpu_clk_freq                                 /* Determine nbr SysTick increments                     */
                 / (CPU_INT32U)OSCfg_TickRate_Hz;

    OS_CPU_SysTickInit(cnts);
    CPU_Init();                                                 /* Initialize the uC/CPU services                       */

#if (OS_CFG_STAT_TASK_EN > 0u)
    OSStatTaskCPUUsageInit(&err_os);                            /* Compute CPU capacity with no task running            */
#endif

    Mem_Init();

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif

    AppTaskCreate();                                            /* Creates all the necessary application tasks.         */

    while (DEF_ON) {
        BSP_LED_Toggle(1);
        OSTimeDlyHMSM(0, 0, 0, 500,
                      OS_OPT_TIME_HMSM_STRICT,
                     &err_os);
    }
}


/*
*********************************************************************************************************
*                                         AppTaskCreate()
*
* Description :  Create the application tasks.
*
* Argument(s) :  none.
*
* Return(s)   :  none.
*
* Caller(s)   :  AppTaskStart()
*
* Note(s)     :  none.
*********************************************************************************************************
*/

static  void  AppTaskCreate (void)
{

}

