
#include <stdbool.h>
#include <stdint.h>
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
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "inc/tm4c123gh6pm.h"
#include "lis3dh-i2c.h"
	

void InitI2C1(void) {
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
     
    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
 
    //reset module (also gives harware time to start up)
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
 
    // Configure the pin muxing for I2C1 functions on port A6 and A7.
    GPIOPinConfigure(GPIO_PA6_I2C1SCL); //configure pins to the i2c hardware
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
     
    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
 
    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);
     
    //clear I2C FIFOs
    HWREG(I2C1_BASE + I2C_O_FIFOCTL) = 80008000;
}

void writeRegister(uint8_t rAddress, uint8_t data) { //wrapper for the arduino function
	//we are writing a register address followed by the data to write there
	
	I2CMasterSlaveAddrSet(I2C1_BASE, LIS3DH_slave_address, false); //writing to the slave address
	//specify the register
	I2CMasterDataPut(I2C1_BASE, rAddress);
	//send the register address data
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	
	// Wait until MCU is done transferring.
  while(I2CMasterBusy(I2C1_BASE));
	
	//write the value
	I2CMasterDataPut(I2C1_BASE, data);
	//finish the transfer
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	
	//wait for transaction to finish
	while(I2CMasterBusy(I2C1_BASE));
}

uint8_t readRegister(uint8_t rAddress) {
  //specify that we are writing (a register address) to the
  //slave device
	
  I2CMasterSlaveAddrSet(I2C1_BASE, LIS3DH_slave_address, false);
  //specify register to be read
  I2CMasterDataPut(I2C1_BASE, rAddress);
  //send control byte and register address byte to slave device
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
   
  //wait for MCU to finish transaction
  while(I2CMasterBusy(I2C1_BASE));
   
  //specify that we are going to read from slave device
  I2CMasterSlaveAddrSet(I2C1_BASE, LIS3DH_slave_address, true);
   
  //send control byte and read from the register we
  //specified
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
   
  //wait for MCU to finish transaction
  while(I2CMasterBusy(I2C1_BASE));
   
  //return data pulled from the specified register
 return I2CMasterDataGet(I2C1_BASE);
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
}

//function to read all of the axies at once.
void LIS3DH_read(int16_t *x, int16_t *y, int16_t *z) {
  //specify that we are writing (a register address) to the
  //slave device
	
  I2CMasterSlaveAddrSet(I2C1_BASE, LIS3DH_slave_address, false);
  //specify register to be read
  I2CMasterDataPut(I2C1_BASE, (LIS3DH_INCREMENT_ADDRESS_MASK | LIS3DH_OUT_X_L));
  //send control byte and register address byte to slave device
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
   
  //wait for MCU to finish transaction
  while(I2CMasterBusy(I2C1_BASE));
   
  //specify that we are going to read from slave device
  I2CMasterSlaveAddrSet(I2C1_BASE, LIS3DH_slave_address, true);
   
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
  while(I2CMasterBusy(I2C1_BASE));
	*x = I2CMasterDataGet(I2C1_BASE);
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C1_BASE));
	*x |= I2CMasterDataGet(I2C1_BASE)<<8;
	
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C1_BASE));
	*y = I2CMasterDataGet(I2C1_BASE);
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C1_BASE));
	*y |= I2CMasterDataGet(I2C1_BASE)<<8;
	
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
  while(I2CMasterBusy(I2C1_BASE));
	*z = I2CMasterDataGet(I2C1_BASE);
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
  while(I2CMasterBusy(I2C1_BASE));
	*z |= I2CMasterDataGet(I2C1_BASE)<<8;
}
