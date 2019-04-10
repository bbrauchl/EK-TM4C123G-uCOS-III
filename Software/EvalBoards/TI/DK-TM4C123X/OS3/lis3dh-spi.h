
#ifndef __LIS3DH_H__
#define __LIS3DH_H__

#include  "app_cfg.h"
#include  <cpu_core.h>
#include  <os.h>

#include <stdint.h>
#include <stdbool.h>

#define LIS3DH_STATUS_REG_AUX 0x07
#define LIS3DH_OUT_ADC1_L 0x08
#define LIS3DH_OUT_ADC1_H 0x09
#define LIS3DH_OUT_ADC2_L 0x0A
#define LIS3DH_OUT_ADC2_H 0x0B
#define LIS3DH_OUT_ADC3_L 0x0C
#define LIS3DH_OUT_ADC3_H 0x0D
#define LIS3DH_INT_COUNTER_REG 0x0E
#define LIS3DH_WHO_AM_I 0x0F
#define LIS3DH_TEMP_CFG_REG 0x1F
#define LIS3DH_CTRL_REG1 0x20
#define LIS3DH_CTRL_REG2 0x21
#define LIS3DH_CTRL_REG3 0x22
#define LIS3DH_CTRL_REG4 0x23
#define LIS3DH_CTRL_REG5 0x24
#define LIS3DH_CTRL_REG6 0x25
#define LIS3DH_REFERENCE 0x26
#define LIS3DH_STATUS_REG2 0x27
#define LIS3DH_OUT_X_L 0x28
#define LIS3DH_OUT_X_H 0x29
#define LIS3DH_OUT_Y_L 0x2A
#define LIS3DH_OUT_Y_H 0x2B
#define LIS3DH_OUT_Z_L 0x2C
#define LIS3DH_OUT_Z_H 0x2D
#define LIS3DH_FIFO_CTRL_REG 0x2E
#define LIS3DH_FIFO_SRC_REG 0x2F
#define LIS3DH_INT1_CFG 0x30
#define LIS3DH_INT1_SRC 0x31
#define LIS3DH_INT1_THS 0x32
#define LIS3DH_INT1_DURATION 0x33
#define LIS3DH_CLICK_CFG 0x38
#define LIS3DH_CLICK_SRC 0x39
#define LIS3DH_CLICK_THS 0x3A
#define LIS3DH_TIME_LIMIT 0x3B
#define LIS3DH_TIME_LATENCY 0x3C
#define LIS3DH_TIME_WINDOW 0x3D

#define LIS3DH_INCREMENT_ADDRESS_MASK 0x40

#define LIS3DH_DATA_WRITE  0x00
#define LIS3DH_DATA_READ   0x80

static  CPU_STK  SSI0SpoolTaskStk[APP_CFG_TASK_START_STK_SIZE];
static  OS_TCB   SSI0SpoolTaskTCB;

static  OS_Q		 q_SSI0receivedMessage;
static  OS_MUTEX m_SSI0Lock;

struct ssi_message_t {
	//include TCB block of thread that should be returned to
	OS_TCB	 *rThread;
	uint32_t *data;
	uint8_t  length;
};

typedef struct _t_LIS3DH_settings {
	bool adcEnabled;
//Temperature settings
	bool tempEnabled;
//Accelerometer settings
	uint16_t accelSampleRate; //Hz. Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
	uint8_t accelRange; //Max G force readable. Can be: 2, 4, 8, 16
	bool xAccelEnabled;
	bool yAccelEnabled;
	bool zAccelEnabled;
//FIFO control settings
	bool fifoEnabled;
	uint8_t fifoThreshold; //Can be 0 to 32
	uint8_t fifoMode; //FIFO mode.
} t_LIS3DH_settings;

extern void InitSSI0(void);
extern void writeRegister(uint32_t rAddress, uint32_t data);
extern uint32_t readRegister(uint32_t rAddress);
extern void LIS3DH_applySettings(t_LIS3DH_settings settings);
extern void LIS3DH_read(int16_t *x, int16_t *y, int16_t *z);
extern void SSI0SpoolTask (void *p_arg);
extern void SSI0_OSHandler(void);

#endif // __LIS3DH_H__
