
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "driverlib/debug.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/ssi.h"
#include "inc/hw_ssi.h"
#include "utils/uartstdio.h"
#include "inc/tm4c123gh6pm.h"

#include  <app_cfg.h>
#include  <cpu_core.h>
#include  <os.h>

#include "lis3dh-spi.h"

void InitSSI0(void) {
		OS_ERR err;
		uint32_t tmp[8];
    //enable SSI module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0)){}
     
    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){}
 
    //reset module (also gives harware time to start up)
    SysCtlPeripheralReset(SYSCTL_PERIPH_SSI0);
 
    // Configure the pin muxing for ssi0 functions on port A2 - A4
    GPIOPinConfigure(GPIO_PA2_SSI0CLK); //configure pins to the ssi hardware
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
     
    // Select the I2C function for these pins.
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);
 
    // Enable and initialize the SSI master modual
    SSIConfigSetExpClk(SSI0_BASE, //SSI base 0
												SysCtlClockGet(),	//Clock rate of the SSI module
												SSI_FRF_MOTO_MODE_3, //Motorola spi format, clock phase second edge, clock idle high
												SSI_MODE_MASTER, //master mode
												10000000, //10 MHz transfer
												8); // 8 bits per frame
												
		
		
		//enable the SSI0 hardware
		SSIEnable(SSI0_BASE);
		//empty the recieve FIFO
		while(SSIDataGetNonBlocking(SSI0_BASE, tmp)){} // clear the fifo before we start
		
		//register and enable transmit end interrupt
		SSIIntRegister(SSI0_BASE, SSI0_OSHandler);
		
		//enable TXRIS after transmission interrupt mode
		//This means where will be an interrupt every time the transmission finishes
			
		/*******************************************************************************
			ERRATA NOTE: this interrupt does not latch and the flag will not be visible
			in the SSI interrupt resgisters, so it cannot be checked for in the ISR
			******************************************************************************/
		HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_EOT;
			
		SSIIntEnable(SSI0_BASE, SSI_TXFF | SSI_RXFF);
			
		//create task for spooling							 
		OSTaskCreate((OS_TCB     *)&SSI0SpoolTaskTCB,                /* Create the start task                                */
							 (CPU_CHAR   *)"SSI0 Spool Task",
							 (OS_TASK_PTR ) SSI0SpoolTask,
							 (void       *) 0,
							 (OS_PRIO     ) APP_CFG_TASK_START_PRIO - 1,
							 (CPU_STK    *)&SSI0SpoolTaskStk[0],
							 (CPU_STK_SIZE) APP_CFG_TASK_START_STK_SIZE / 10u,
							 (CPU_STK_SIZE) APP_CFG_TASK_START_STK_SIZE,
							 (OS_MSG_QTY  ) 0u,
							 (OS_TICK     ) 0u,
							 (void       *) 0,
							 (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
							 (OS_ERR     *)&err);
							 
		//initalize a lock for the SSI hardware
		OSMutexCreate(&m_SSI0Lock, "SSI0 Lock", &err);
		
		//initalize event flags for the SSI
		OSFlagCreate(&f_SSI0Events, "SSI0 Events", 0x00, &err);
							 
		//initalize message queue for SPI messages
		OSQCreate(&q_SSI0Messages, "SSI0 receive", 10, &err);
		OSQFlush(&q_SSI0Messages, &err);
}

void writeRegister(uint32_t rAddress, uint32_t data) { //wrapper for the arduino function
  OS_ERR    err_os;
	struct ssi_message_t wMsg;
	uint32_t wData[2];
	
	wData[0] = rAddress | LIS3DH_DATA_WRITE;
	wData[1] = data;
	
	wMsg.data = wData;
	wMsg.length = 2;
	wMsg.rThread = NULL;
	OSQPost(&q_SSI0Messages, (void *)&wMsg, sizeof(struct ssi_message_t),
							OS_OPT_POST_FIFO, &err_os);
	
	//call for processing
	//OSTaskSemPost(&SSI0SpoolTaskTCB, OS_OPT_POST_NONE, &err_os);
}

uint32_t readRegister(uint32_t rAddress) { //wrapper for the arduino function
  OS_ERR    err_os;
	struct ssi_message_t rMsg;
	uint32_t rData[2];
	
	rData[0] = rAddress | LIS3DH_DATA_READ;
	
	rMsg.data = rData;
	rMsg.length = 2;
	rMsg.rThread = OSTCBCurPtr;
	OSTaskSemSet(&SSI0SpoolTaskTCB, 0, &err_os);
	OSQPost(&q_SSI0Messages, (void *)&rMsg, sizeof(struct ssi_message_t),
							OS_OPT_POST_FIFO, &err_os);
	
	OSTaskSemPend(0, OS_OPT_PEND_BLOCKING, NULL, &err_os);
	//return last thing recieved
	return rMsg.data[rMsg.length-1];
}

//Code ripped from the arduino library. writeRegister is used by the code the above wrapper makes the code below function as intended
void LIS3DH_applySettings(t_LIS3DH_settings settings) {	
	uint8_t dataToWrite = 0; //Temporary variable
	
	//total part reset
	writeRegister(LIS3DH_CTRL_REG5, 0x80);
	
	//Build TEMP_CFG_REG
	dataToWrite = 0; //Start Fresh!
	dataToWrite = ((settings.tempEnabled & 0x01) << 6) | ((settings.adcEnabled & 0x01) << 7);
	//Now, write the patched together data
	
	writeRegister(LIS3DH_TEMP_CFG_REG, dataToWrite);
	//Build CTRL_REG1
	dataToWrite = 0; //Start Fresh!
	// Convert ODR
	switch(settings.accelSampleRate)
	{
		case 1:
		dataToWrite |= (0x01 << 4);
		break;
		case 10:
		dataToWrite |= (0x02 << 4);
		break;
		case 25:
		dataToWrite |= (0x03 << 4);
		break;
		case 50:
		dataToWrite |= (0x04 << 4);
		break;
		case 100:
		dataToWrite |= (0x05 << 4);
		break;
		case 200:
		dataToWrite |= (0x06 << 4);
		break;
		default:
		case 400:
		dataToWrite |= (0x07 << 4);
		break;
		case 1600:
		dataToWrite |= (0x08 << 4);
		break;
		case 5000:
		dataToWrite |= (0x09 << 4);
		break;
	}
	dataToWrite |= (settings.zAccelEnabled & 0x01) << 2;
	dataToWrite |= (settings.yAccelEnabled & 0x01) << 1;
	dataToWrite |= (settings.xAccelEnabled & 0x01);
	//Now, write the patched together data
	writeRegister(LIS3DH_CTRL_REG1, dataToWrite);
	//Build CTRL_REG4
	dataToWrite = 0; //Start Fresh!
	// Convert scaling (in terms of G)
	switch(settings.accelRange)
	{
		case 2:
		dataToWrite |= (0x00 << 4);
		break;
		case 4:
		dataToWrite |= (0x01 << 4);
		break;
		case 8:
		dataToWrite |= (0x02 << 4);
		break;
		default:
		case 16:
		dataToWrite |= (0x03 << 4);
		break;
	}
	dataToWrite |= 0x80; //set block update -- this means that values wont update until both the MSB and LSB are read preventing weird syncing issues
	dataToWrite |= 0x08; //set high resolution
	//Now, write the patched together data
	writeRegister(LIS3DH_CTRL_REG4, dataToWrite);
	
	//enable data ready interrupts on int1
	writeRegister(LIS3DH_CTRL_REG3, 0x50);
	
	//Interrupt 1 is latched **Is this needed??**
	writeRegister(LIS3DH_CTRL_REG5, 0x08);
}

//function to read all of the axis at once.
void LIS3DH_read(int16_t *x, int16_t *y, int16_t *z) {
  //specify that we are writing (a register address) to the
  //slave device
	
	
  OS_ERR    err_os;
	struct ssi_message_t rMsg;
	uint32_t rData[7];
	
	rData[0] = LIS3DH_OUT_X_L | LIS3DH_DATA_READ | LIS3DH_INCREMENT_ADDRESS_MASK;
	
	rMsg.data = rData;
	rMsg.length = 7;
	rMsg.rThread = OSTCBCurPtr;
	OSTaskSemSet(&SSI0SpoolTaskTCB, 0, &err_os);
	OSQPost(&q_SSI0Messages, (void *)&rMsg, sizeof(struct ssi_message_t),
							OS_OPT_POST_FIFO, &err_os);
	
	OSTaskSemPost(&SSI0SpoolTaskTCB, OS_OPT_POST_NONE, &err_os);
	OSTaskSemPend(0, OS_OPT_PEND_BLOCKING, NULL, &err_os);
	
	*x = (int16_t)(rData[1] | rData[2] << 8);
	*y = (int16_t)(rData[3] | rData[4] << 8);	
	*z = (int16_t)(rData[5] | rData[6] << 8);
}

void SSI0EmptyFIFO(uint32_t *ptr) {
	
}

static  void  SSI0SpoolTask (void *p_arg) {
  OS_ERR    err;
	static struct ssi_message_t *dataStruct = NULL;
	static uint32_t *sendingDataptr = NULL, *receivingDataptr = NULL;
	static OS_MSG_SIZE msg_size;
	static OS_FLAGS flgs;
	CPU_SR_ALLOC();
		
	(void)&p_arg;
	
	//clear all event flags
	OSFlagPost(&f_SSI0Events, 0xFF, OS_OPT_POST_FLAG_CLR, &err);
	
	while(DEF_ON) {
		//release lock on SSI for other tasks
		OSMutexPost(&m_SSI0Lock, OS_OPT_POST_NONE, &err);
		
		//wait for data to start the SPI transper
		dataStruct = OSQPend(&q_SSI0Messages, 0, OS_OPT_PEND_BLOCKING, &msg_size, NULL, &err);
		sendingDataptr = dataStruct->data;
		receivingDataptr = dataStruct->data;
		
		//get the lock for the SSI0 hardware
		OSMutexPend(&m_SSI0Lock, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
		
		CPU_CRITICAL_ENTER();
		SSIIntDisable(SSI0_BASE, SSI_RXFF);
		
		while(DEF_ON) {
			//if we are able to read from hardware
			if(SSIDataGetNonBlocking(SSI0_BASE, receivingDataptr)) {
				receivingDataptr++;
			}
			
			//check if there is still data to send
			if(sendingDataptr - dataStruct->data < dataStruct->length) {
				if(SSIDataPutNonBlocking(SSI0_BASE, *sendingDataptr)) {
					sendingDataptr++;
					//always continue if there is data left
					continue;
				}
			}
			
			//Either all data has sent or the TX FIFO is full
			//Empty the receive FIFO
			while(SSIDataGetNonBlocking(SSI0_BASE, receivingDataptr)) {
				receivingDataptr++;
			}
			
			//enable the interrupt that will wake us.
			//this must be disabled during processing or else an interrupt lock can occur because the data
			//never gets read from the hardware in the ISR. once the task is done reading it can be re-enabled
			//due to the errata in the SSI hardware, this should only be re-enabled when there is still data to send.
			//Otherwise we should wait for and EOT event. (This Errata is stupid)
			if(sendingDataptr - dataStruct->data < dataStruct->length)
				SSIIntEnable(SSI0_BASE, SSI_RXFF);
			
			//We have done all the work we can. sleep until there is an event to allow other tasks to run.
			//valid events: RX buffer half or more full (RXFF) and TX end of transmission (TXFF) **SEE ERATTA MASK**
			CPU_CRITICAL_EXIT(); //enter critical section for pend operation, here we are purposly allowing the scheduler to run
			flgs = OSFlagPend(&f_SSI0Events, SSI_TXFF | SSI_RXFF, 1, OS_OPT_PEND_FLAG_SET_ANY | OS_OPT_PEND_FLAG_CONSUME | OS_OPT_PEND_BLOCKING, NULL, &err);
			if(err) {
				//if we got a timeout, we will assume that the transmission finished because of errata
				break;
			}
			CPU_CRITICAL_ENTER(); //re-enter critical section to continue to protect
			
			//check the flags that caused us to wake up
			if(flgs & SSI_TXFF) {
				// Break out of trasmit loop on end of transmission event
				break;
			}
		}
		
		//exit spooling critical section
		CPU_CRITICAL_EXIT();
		
		//Empty the receive FIFO
		while(SSIDataGetNonBlocking(SSI0_BASE, receivingDataptr)) {
			receivingDataptr++;
		}
		
		//sanity check
		if(receivingDataptr - dataStruct->data == dataStruct->length) {
			// dont try to wake threads that passed no TCB
			if(dataStruct->rThread) {
				//signal the thread associated to the message
				OSTaskSemPost(dataStruct->rThread, OS_OPT_POST_NONE, &err);
				//no need to invalidate struct, it will get overwritten on new loop.
			}
		}
		else {
			while(DEF_ON){__asm("NOP \n");}
		}
  }
}

//interrupt handler for spi driver, triggers after the spi transaction has completed.
//this is important because it allows the spi to run while the rest of the OS cotinues. Once done it can wake the task using SPI.
void SSI0_OSHandler(void) {
	OS_ERR err;
	
  CPU_SR_ALLOC();

  CPU_CRITICAL_ENTER();                                       /* Tell the OS that we are starting an ISR            */
  OSIntEnter();
  CPU_CRITICAL_EXIT();
	
	//the TXFF interrupt acts very odd, it will not latch so check for absence of any set interrupt flags.
	if (!SSIIntStatus(SSI0_BASE, true)) {
		OSFlagPost(&f_SSI0Events, SSI_TXFF, OS_OPT_POST_FLAG_SET, &err);
	}
	else {
		//turn off interrupt so it is possible to return without processing
		SSIIntDisable(SSI0_BASE, SSI_RXFF);
		OSFlagPost(&f_SSI0Events, SSI_RXFF, OS_OPT_POST_FLAG_SET, &err);
	}
	SSIIntClear(SSI0_BASE, SSI_TXFF | SSI_RXFF);
	
	OSIntExit();
}
