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

#include  <stdlib.h>
#include  <stdint.h>
#include  <stdbool.h>

#include  <driverlib/sysctl.h>
#include  <driverlib/gpio.h>
#include  <inc/hw_types.h>
#include  <inc/hw_gpio.h>
#include  <inc/hw_memmap.h>


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

static  CPU_STK  BlinkRedTaskStk[APP_CFG_TASK_START_STK_SIZE];
static  OS_TCB   BlinkRedTaskTCB;

static  CPU_STK  BlinkBlueTaskStk[APP_CFG_TASK_START_STK_SIZE];
static  OS_TCB   BlinkBlueTaskTCB;

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

static  void  BlinkRedTask (void  *p_arg);
static  void  BlinkBlueTask (void  *p_arg);

static  void  AppTaskCreate (void);

void InitGPIOPortF(void);


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
	
		SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);                                                 /* Initialize clock */
	
		InitGPIOPortF(); /* Initailize port F */
	
    Mem_Init();
	
		OS_CPU_SysTickInit(SysCtlClockGet() / (CPU_INT32U)OSCfg_TickRate_Hz);
    CPU_Init();                                                 /* Initialize the uC/CPU services                       */

		AppTaskCreate(); /* Initalize tasks to run at the start */

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

static  void  BlinkRedTask (void *p_arg) {
    OS_ERR    err_os;

   (void)&p_arg;

    while (DEF_ON) {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1));
        OSTimeDlyHMSM(0, 0, 0, 700, OS_OPT_TIME_HMSM_STRICT, &err_os);
    }
}

static  void  BlinkBlueTask (void *p_arg) {
    OS_ERR    err_os;

   (void)&p_arg;

    while (DEF_ON) {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2));
        OSTimeDlyHMSM(0, 0, 0, 400, OS_OPT_TIME_HMSM_STRICT, &err_os);
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

static  void  AppTaskCreate (void) {
	OS_ERR  err;
	OSTaskCreate((OS_TCB     *)&BlinkRedTaskTCB,                /* Create the start task                                */
							 (CPU_CHAR   *)"Blink Red Task",
							 (OS_TASK_PTR ) BlinkRedTask,
							 (void       *) 0,
							 (OS_PRIO     ) APP_CFG_TASK_START_PRIO,
							 (CPU_STK    *)&BlinkRedTaskStk[0],
							 (CPU_STK_SIZE) APP_CFG_TASK_START_STK_SIZE / 10u,
							 (CPU_STK_SIZE) APP_CFG_TASK_START_STK_SIZE,
							 (OS_MSG_QTY  ) 0u,
							 (OS_TICK     ) 0u,
							 (void       *) 0,
							 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
							 (OS_ERR     *)&err);

	OSTaskCreate((OS_TCB     *)&BlinkBlueTaskTCB,                /* Create the start task                                */
							 (CPU_CHAR   *)"Blink Blue Task",
							 (OS_TASK_PTR ) BlinkBlueTask,
							 (void       *) 0,
							 (OS_PRIO     ) APP_CFG_TASK_START_PRIO,
							 (CPU_STK    *)&BlinkBlueTaskStk[0],
							 (CPU_STK_SIZE) APP_CFG_TASK_START_STK_SIZE / 10u,
							 (CPU_STK_SIZE) APP_CFG_TASK_START_STK_SIZE,
							 (OS_MSG_QTY  ) 0u,
							 (OS_TICK     ) 0u,
							 (void       *) 0,
							 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
							 (OS_ERR     *)&err);
}


void InitGPIOPortF(void) {
	// Configure the GPIO Port F Pin 2 for output as indicators
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){}
		
	// Unlock PF1 so it may be used as an input rather than the NMI
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
			
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1); //Set the LEDs at outputs
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0); // set up the buttons as inputs
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor for PF4
}
