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
#include  <stdio.h>
#include  <string.h>

#include  <driverlib/sysctl.h>
#include  <driverlib/gpio.h>
#include  <driverlib/ssi.h>
#include  <driverlib/interrupt.h>
#include  <driverlib/uart.h>
#include  <driverlib/pin_map.h>
#include  <utils/uartstdio.h>
#include  <inc/hw_types.h>
#include  <inc/hw_gpio.h>
#include  <inc/hw_memmap.h>
#include  <inc/hw_nvic.h>
#include  <inc/hw_uart.h>
#include  <inc/tm4c123gh6pm.h>

//#include  "lis3dh-i2c.h"
#include  "lis3dh-spi.h"

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

#define LIS3DH_DATA_READY 0x80

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

static  CPU_STK  AccelTaskStk[APP_CFG_TASK_START_STK_SIZE];
static  OS_TCB   AccelTaskTCB;

static  OS_FLAG_GRP f_LIS3DHEvents;

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
static  void  AccelTask (void  *p_arg);

static  void  AppTaskCreate (void);

void InitConsole(void);

void InitGPIOPortF(void);
void InitGPIOPortB(void);

void GPIOPortB_ISR(void);


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
                   SYSCTL_XTAL_16MHZ);                                                 /* Initialize clock to 80 MHz*/
	
		InitSSI0();
	
		InitGPIOPortF(); /* Initailize port F */
		InitGPIOPortB();
	
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

    while (DEF_ON) {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2));
        OSTimeDlyHMSM(0, 0, 0, 400, OS_OPT_TIME_HMSM_STRICT, &err_os);
    }
}

static  void  AccelTask (void *p_arg) {
    OS_ERR    err_os;
		int16_t x, y, z;
		t_LIS3DH_settings settings = {
			/*.adcEnabled = */true,
				//Temperature settings
			/*.tempEnabled = */true,
				//Accelerometer settings
			/*.accelSampleRate = */50, //Hz. Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
			/*.accelRange = */2, //Max G force readable. Can be: 2, 4, 8, 16
			/*.xAccelEnabled = */true,
			/*.yAccelEnabled = */true,
			/*.zAccelEnabled = */true,
				//FIFO control settings
			/*.fifoEnabled = */false,
			/*.fifoThreshold = */20, //Can be 0 to 32
			/*.fifoMode = */0, //FIFO mode.
		};

   (void)&p_arg;
		
		InitConsole();
		OSTaskSemSet(NULL, 0, &err_os);
		OSFlagCreate(&f_LIS3DHEvents, "LIS3DH Events", 0x00, &err_os);
		LIS3DH_applySettings(settings);

    while (DEF_ON) {
			OSFlagPend(&f_LIS3DHEvents, LIS3DH_DATA_READY, 0, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_BLOCKING, NULL, &err_os);
			LIS3DH_read(&x, &y, &z);
			UARTprintf("%d,\t%d,\t%d\n", x, y, z);
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

	OSTaskCreate((OS_TCB     *)&AccelTaskTCB,                /* Create the start task                                */
							 (CPU_CHAR   *)"Accel. Task",
							 (OS_TASK_PTR ) AccelTask,
							 (void       *) 0,
							 (OS_PRIO     ) APP_CFG_TASK_START_PRIO,
							 (CPU_STK    *)&AccelTaskStk[0],
							 (CPU_STK_SIZE) APP_CFG_TASK_START_STK_SIZE / 10u,
							 (CPU_STK_SIZE) APP_CFG_TASK_START_STK_SIZE,
							 (OS_MSG_QTY  ) 0u,
							 (OS_TICK     ) 0u,
							 (void       *) 0,
							 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP), //save floating point registers
							 (OS_ERR     *)&err);
}

void InitConsole(void)
{
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	
		SysCtlDelay(3);//added to prevent hard fault
	
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
   
		SysCtlDelay(3);//added to prevent hard fault

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

		// Select the alternate (UART) function for these pins.   
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O. 9600 BAUD
    UARTStdioConfig(0, 9600, 16000000);
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

void InitGPIOPortB(void) {
	// Configure the GPIO Port F Pin 2 for output as indicators
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)){}
		
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0); // set up the buttons as inputs
	GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);  // Enable weak pullup resistor for PF4
	GPIOIntRegister(GPIO_PORTB_BASE, GPIOPortB_ISR);
		
	IntPrioritySet(INT_GPIOF, 0x20); // allow OS interrupts to have higher priority
		
	GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_RISING_EDGE); // set the interrupt type to rising edge with the light that is used in the systick
	GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_0); //clear any interrupts if any exist
	GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0); //enable interrupts on the port.
}

void GPIOPortB_ISR(void) {
	OS_ERR err;
	
  CPU_SR_ALLOC();

  CPU_CRITICAL_ENTER();                                       /* Tell the OS that we are starting an ISR            */
  OSIntEnter();
  CPU_CRITICAL_EXIT();
	
	GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_0);
	
	OSFlagPost(&f_LIS3DHEvents, LIS3DH_DATA_READY, OS_OPT_POST_FLAG_SET, &err);
	OSIntExit();
}
