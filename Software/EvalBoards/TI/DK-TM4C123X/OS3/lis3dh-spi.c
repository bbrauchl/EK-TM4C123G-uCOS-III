
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

#include  "app_cfg.h"
#include  <cpu_core.h>
#include  <os.h>

#include "lis3dh-spi.h"
	
#define LIS3DH_DATA_WRITE  0x00
#define LIS3DH_DATA_READ   0x80
#define LIS3DH_DATA_RW_INC 0x40

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
												2000000, //2 MHz transfer
												8); // 8 bits per frame
												
		
		
		//enable the SSI0 hardware
		SSIEnable(SSI0_BASE);
		//empty the recieve FIFO
		while(SSIDataGetNonBlocking(SSI0_BASE, tmp)){} // clear the fifo before we start
		
		//register and enable transmit end interrupt
		SSIIntRegister(SSI0_BASE, SSI0_OSHandler);
		
		//enable TXRIS after transmission interrupt mode
		//This means where will be an interrupt every time the transmission finishes
		//HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_EOT;
			
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
}

void writeRegister(uint32_t rAddress, uint32_t data) { //wrapper for the arduino function
  OS_ERR    err_os;
	struct ssi_message_t wMsg;
	uint32_t wData[3];
	
	wData[0] = LIS3DH_slave_address | LIS3DH_DATA_WRITE;
	wData[1] = rAddress;
	wData[2] = data;
	
	wMsg.data = wData;
	wMsg.length = 3;
	wMsg.rThread = OSTCBCurPtr;
	OSQPost(&q_SSI0receivedMessage, (void *)&wMsg, sizeof(struct ssi_message_t),
							OS_OPT_POST_FIFO, &err_os);
	
	//call for processing
	OSTaskSemPost(&SSI0SpoolTaskTCB, OS_OPT_POST_NONE, &err_os);
}

uint32_t readRegister(uint32_t rAddress) { //wrapper for the arduino function
  OS_ERR    err_os;
	struct ssi_message_t rMsg;
	uint32_t rData[3];
	
	rData[0] = LIS3DH_slave_address | LIS3DH_DATA_READ;
	rData[1] = rAddress;
	
	rMsg.data = rData;
	rMsg.length = 3;
	rMsg.rThread = OSTCBCurPtr;
	OSTaskSemSet(&SSI0SpoolTaskTCB, 0, &err_os);
	OSQPost(&q_SSI0receivedMessage, (void *)&rMsg, sizeof(struct ssi_message_t),
							OS_OPT_POST_FIFO, &err_os);
	
	OSTaskSemPost(&SSI0SpoolTaskTCB, OS_OPT_POST_NONE, &err_os);
	OSTaskSemPend(0, OS_OPT_PEND_BLOCKING, NULL, &err_os);
	//return last thing recieved
	return rMsg.data[rMsg.length-1];
}

//Code ripped from the arduino library. writeRegister is used by the code the above wrapper makes the code below function as intended
void LIS3DH_applySettings(t_LIS3DH_settings settings) {	
	uint8_t dataToWrite = 0; //Temporary variable
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
	writeRegister(LIS3DH_CTRL_REG3, 0x10);
}

//function to read all of the axies at once.
void LIS3DH_read(int16_t *x, int16_t *y, int16_t *z) {
  //specify that we are writing (a register address) to the
  //slave device
	
	
  OS_ERR    err_os;
	struct ssi_message_t rMsg;
	uint32_t rData[8];
	
	rData[0] = LIS3DH_slave_address | LIS3DH_DATA_READ;
	rData[1] = LIS3DH_INCREMENT_ADDRESS_MASK | LIS3DH_OUT_X_L;
	
	rMsg.data = rData;
	rMsg.length = 8;
	rMsg.rThread = OSTCBCurPtr;
	OSTaskSemSet(&SSI0SpoolTaskTCB, 0, &err_os);
	OSQPost(&q_SSI0receivedMessage, (void *)&rMsg, sizeof(struct ssi_message_t),
							OS_OPT_POST_FIFO, &err_os);
	
	OSTaskSemPost(&SSI0SpoolTaskTCB, OS_OPT_POST_NONE, &err_os);
	OSTaskSemPend(0, OS_OPT_PEND_BLOCKING, NULL, &err_os);
	
	*x = (int16_t)(rData[2] | rData[3] << 8);
	*y = (int16_t)(rData[4] | rData[5] << 8);	
	*z = (int16_t)(rData[6] | rData[7] << 8);
}

static  void  SSI0SpoolTask (void *p_arg) {
  OS_ERR    err;
	static struct ssi_message_t *sendingStruct = NULL, *receivingStruct = NULL;
	static uint32_t *sendingDataptr = NULL, *receivingDataptr = NULL;
	static OS_Q q_SSI0Internal;
	static OS_MSG_SIZE msg_size;
		
	(void)&p_arg;
	//create internal queue for spooling management	
	//make the Queue reasonably long (10 items)
	OSTaskSemSet(&SSI0SpoolTaskTCB, 0, &err);
	OSQCreate(&q_SSI0receivedMessage, "SSI0 receive", 10, &err);
	OSQFlush(&q_SSI0receivedMessage, &err);
	OSQCreate(&q_SSI0Internal, "SSI0 Internal", 10, &err);
	OSQFlush(&q_SSI0Internal, &err);
		
    
	while(DEF_ON) {
		//release lock before going to sleep
		OSMutexPost(&m_SSI0Lock, OS_OPT_POST_NONE, &err);
		//suspend self when work is finished. On interrupt and write this will be woken up
		OSTaskSemPend(0, OS_OPT_PEND_BLOCKING, NULL, &err);
		
		//aquire lock
		OSMutexPend(&m_SSI0Lock, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
		SSIIntDisable(SSI0_BASE, SSI_TXFF | SSI_RXFF);
		//receiving data loop
		while(DEF_ON) {
			//check if the current queue is empty. If so, load next item
			if(receivingStruct == NULL) {
				//attempt to load another message from the queue
				receivingStruct = (struct ssi_message_t*)OSQPend(&q_SSI0Internal,
															0, OS_OPT_PEND_NON_BLOCKING, &msg_size, NULL, &err);
				
				//if there is an error, task has nowhere to put data, ending receiver processing
				if(err) {
					break;
				}
				//set the pointer
				receivingDataptr = receivingStruct->data;
			}
			
			if(SSIDataGetNonBlocking(SSI0_BASE, receivingDataptr)) {
				receivingDataptr++;
			}
			else {
				break;
			}
			
			//check if it is the last element of the set
			if (receivingDataptr - receivingStruct->data == receivingStruct->length) {
				//when the data is done, it needs to be sent to the calling task
				OSTaskQPost(receivingStruct->rThread, (void *)receivingStruct, 
											sizeof(struct ssi_message_t), OS_OPT_POST_FIFO, &err);
				
				// If there was an error, try just waking the task. The task can either continue execution
				// and check for a message for when data is complete or will be waked when the data is ready.
				if(err) {
					OSTaskSemPost(receivingStruct->rThread, OS_OPT_POST_NONE, &err);
				}
				
				//de-reference the data structure for next time
				receivingStruct = NULL;
			}
		}
		
		//sending data loop
		while(DEF_ON) {
			//check if the current queue is empty. If so, load next item
			if(sendingStruct == NULL) {
				//attempt to load another message from the queue
				sendingStruct = (struct ssi_message_t*)OSQPend(&q_SSI0receivedMessage,
															0, OS_OPT_PEND_NON_BLOCKING, &msg_size, NULL, &err);
				//if an item was sccessfully loaded, forwared it to the receiver logic
				if(!err) {
				  //set the pointer
				  sendingDataptr = sendingStruct->data;
					//pass the structure to the internal queue
					OSQPost(&q_SSI0Internal, sendingStruct, msg_size, OS_OPT_POST_FIFO, &err);
				}
				else {
					break;
				}
			}
			
			//try to put normal data in
			if(SSIDataPutNonBlocking(SSI0_BASE, *sendingDataptr)) {
				sendingDataptr++;
			}
			else {
				break;
			}
			
			//check if it is the last element of the set
			if (sendingDataptr - sendingStruct->data == sendingStruct->length - 1) {
				if(SSIAdvDataPutFrameEndNonBlocking(SSI0_BASE, *sendingDataptr)) {
					//we have finished sending the Queue
					sendingStruct = NULL;
				}
				//if we failed to put the last item in the queue is full so break
				else
					break;
			}
		}
	SSIIntEnable(SSI0_BASE, SSI_TXFF | SSI_RXFF);
  }
}

//interrupt handler for spi driver, triggers after the spi transaction has completed.
//this is important because it allows the spi to run while the rest of the OS cotinues. Once done it can wake the task using SPI.
void SSI0_OSHandler(void) {
	OS_ERR err;
	
	OSIntEnter();
	
	SSIIntClear(SSI0_BASE, SSI_TXFF | SSI_RXFF); //clear the intterupt flag
 	SSIIntDisable(SSI0_BASE, SSI_TXFF | SSI_RXFF);

	OSTaskSemPost(&SSI0SpoolTaskTCB, OS_OPT_POST_NO_SCHED, &err);
	OSIntExit();
}
