#ifndef _SENSOR_H_
#define _SENSOR_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "spi.h"
#include "usart.h"

#define H3LIS331_SCALE_FACTOR_400g			1 // in real app should be 81.92
#define H3LIS331_I2C_ADDR 					0x32 						// 8-bit address. p17 & p18 H3LIS331DL datasheet

#define H3LIS331_WHO_AM_I_REG				0x0F
#define H3LIS331_ACCEL_XOUT_L_REG        	0x28
#define H3LIS331_ACCEL_XOUT_H_REG         	0x29
#define H3LIS331_ACCEL_YOUT_L_REG         	0x2A
#define H3LIS331_ACCEL_YOUT_H_REG     	    0x2B
#define H3LIS331_ACCEL_ZOUT_L_REG         	0x2C
#define H3LIS331_ACCEL_ZOUT_H_REG         	0x2D

#define H3LIS331_CTRL_REG_1      			0x20
#define H3LIS331_CTRL_REG_2     		 	0x21
#define H3LIS331_CTRL_REG_3     			0x22
#define H3LIS331_CTRL_REG_4      			0x23
#define H3LIS331_CTRL_REG_5      			0x24
#define H3LIS331_STATUS_REG      			0x27


#define H3LIS331_POWER_MODE_POWER_DOWN		0x00
#define H3LIS331_POWER_MODE_NORMAL			0x20
#define H3LIS331_POWER_MODE_LP_0_5Hz		0x40
#define H3LIS331_POWER_MODE_LP_1Hz			0x60
#define H3LIS331_POWER_MODE_LP_2Hz			0x80
#define H3LIS331_POWER_MODE_LP_5Hz			0xA0
#define H3LIS331_POWER_MODE_LP_10Hz			0xC0

#define H3LIS331_ODR_50Hz					0x00
#define H3LIS331_ODR_100Hz					0x08
#define H3LIS331_ODR_400Hz					0x10
#define H3LIS331_ODR_1000Hz					0x18

#define H3LIS331_ENABLE_Z_AXIS				0x04
#define H3LIS331_ENABLE_Y_AXIS				0x02
#define H3LIS331_ENABLE_X_AXIS				0x01

#define H3LIS331_BDU						0x80
#define H3LIS331_SCALE_100g					0x00
#define H3LIS331_SCALE_200g					0x10
#define H3LIS331_SCALE_400g					0x30				


#define H3LIS331_INT1_CFG        			0x30
#define H3LIS331_INT1_SRC        			0x31
#define H3LIS331_INT1_THS        			0x32
#define H3LIS331_INT1_DURATION   			0x33
#define H3LIS331_INT2_CFG        			0x34
#define H3LIS331_INT2_SRC        			0x35
#define H3LIS331_INT2_THS        			0x36
#define H3LIS331_INT2_DURATION   			0x37

#define H3LIS331_POWER_DOWN             	0x00
#define H3LIS331_NORMAL_50HZ            	0x27
#define H3LIS331_NORMAL_100HZ           	0x2F
#define H3LIS331_NORMAL_400HZ           	0x37
#define H3LIS331_NORMAL_1000HZ          	0x3F
#define H3LIS331_LOW_POWER_0_5HZ        	0x47
#define H3LIS331_LOW_POWER_1HZ          	0x67
#define H3LIS331_LOW_POWER_2HZ          	0x87
#define H3LIS331_LOW_POWER_5HZ         	 	0xA7
#define H3LIS331_LOW_POWER_10HZ         	0xC7

#define LSM330_I2C_ADDR_A 					0x30 						// 8-bit address. 
#define LSM330_I2C_ADDR_G 					0xD4 						// 8-bit address.

#define LSM330_CTRL_REG1_A      			0x20
#define LSM330_CTRL_REG2_A     		 		0x21
#define LSM330_CTRL_REG3_A     				0x22
#define LSM330_CTRL_REG4_A      			0x23
#define LSM330_CTRL_REG5_A      			0x24
#define LSM330_XOUT_L_REG_A        			0x28
#define LSM330_XOUT_H_REG_A         		0x29
#define LSM330_YOUT_L_REG_A       			0x2A
#define LSM330_YOUT_H_REG_A   	  			0x2B
#define LSM330_ZOUT_L_REG_A       			0x2C
#define LSM330_ZOUT_H_REG_A       			0x2D

#define LSM330_WHO_AM_I_REG_G				0x0F
#define LSM330_CTRL_REG1_G      			0x20
#define LSM330_CTRL_REG2_G     			 	0x21
#define LSM330_CTRL_REG3_G     				0x22
#define LSM330_CTRL_REG4_G      			0x23
#define LSM330_CTRL_REG5_G      			0x24
#define LSM330_XOUT_L_REG_G        			0x28
#define LSM330_XOUT_H_REG_G        			0x29
#define LSM330_YOUT_L_REG_G        			0x2A
#define LSM330_YOUT_H_REG_G   	  			0x2B
#define LSM330_ZOUT_L_REG_G       			0x2C
#define LSM330_ZOUT_H_REG_G       			0x2D

#define LSM330_A_ODR_POWER_DOWN				0x00
#define LSM330_A_ODR_NORMAL_LP_1Hz			0x10
#define LSM330_A_ODR_NORMAL_LP_10Hz			0x20
#define LSM330_A_ODR_NORMAL_LP_25Hz			0x30
#define LSM330_A_ODR_NORMAL_LP_50Hz			0x40
#define LSM330_A_ODR_NORMAL_LP_100Hz		0x50
#define LSM330_A_ODR_NORMAL_LP_200Hz		0x60
#define LSM330_A_ODR_NORMAL_LP_400Hz		0x70
#define LSM330_A_ODR_LP_1620Hz				0x80
#define LSM330_A_ODR_NORMAL_LP_1344Hz		0x90
#define LSM330_A_LOW_POWER_MODE				0x08
#define LSM330_A_ENABLE_Z_AXIS				0x04
#define LSM330_A_ENABLE_Y_AXIS				0x02
#define LSM330_A_ENABLE_X_AXIS				0x01
#define LSM330_A_SCALE_2G					0x00
#define LSM330_A_SCALE_4G					0x10
#define LSM330_A_SCALE_8G					0x20
#define LSM330_A_SCALE_16G					0x30
#define LSM330_A_HIGH_RES					0x08


#define LSM330_G_ODR_95						0x00
#define LSM330_G_ODR_190					0x40
#define LSM330_G_ODR_380					0x80
#define LSM330_G_ODR_760					0xC0
#define LSM330_G_ODR_BW_0					0x00
#define LSM330_G_ODR_BW_1					0x10
#define LSM330_G_ODR_BW_2					0x20
#define LSM330_G_ODR_BW_3					0x30

#define LSM330_G_NORMAL_MODE				0x08
#define LSM330_G_POWER_DOWN_MODE			0x00
#define LSM330_G_ENABLE_Z_AXIS				0x04
#define LSM330_G_ENABLE_Y_AXIS				0x02
#define LSM330_G_ENABLE_X_AXIS				0x01

#define LSM330_G_BDU						0x80
#define LSM330_G_SCALE_250					0x00
#define LSM330_G_SCALE_500					0x10
#define LSM330_G_SCALE_1000					0x20
#define LSM330_G_SCALE_2000					0x30

#define ADXL_WRITE_REQUETS					0x0A
#define ADXL_THRESH_ACT_L					0x20
#define ADXL_THRESH_ACT_H					0x21
#define ADXL_THRESH_INACT_L					0x23
#define ADXL_THRESH_INACT_H					0x24
#define ADXL_TIME_INACT_L					0x25
#define ADXL_ACT_INACT_CTL					0x27
#define ADXL_INTMAP1						0x2A
#define ADXL_INTMAP2						0x2B
#define ADXL_POWER_CTL						0x2D

#define ADXL_SET_AWAKE						0x40
#define ADXL_SET_WAKEUP_START				0x0A
#define ADXL_SET_MOTION_DETECT_MODE			0x3F


#define BR25S_WREN							0x06
#define BR25S_WRDI							0x04
#define BR25S_READ							0x03
#define BR25S_WRITE							0x02
#define BR25S_RDSR							0x05
#define BR25S_WRSR							0x01
#define BR25S_SET_MODE						0x00

#define BR25S_MIN_ADDR						0x0000
#define BR25S_MAX_ADDR						0x1FFF-32
#define BR25S_ADDR_BATTERY					0x0000
#define BR25S_ADDR_CALIB					0x0004
#define BR25S_ADDR_SETTINGS					0x0010
#define BR25S_ADDR_DATA						0x0020
#define BR25S_MAX_WRITE_BYTE				32
#define BR25S_CIRCULAR_BUFFER				4080-32
#define BR25S_PRESPACE						200

#define SENSOR_COL_SIZE						20
#define SENSOR_ROW_SIZE						20

#define EN_SPI_H3LIS331		g_spi_cs_pin = (H3LIS331_SPI_CS);
#define EN_SPI_A_LSM330		g_spi_cs_pin = (LSM330_SPI_CS_A);
#define EN_SPI_G_LSM330		g_spi_cs_pin = (LSM330_SPI_CS_G);
#define EN_SPI_ADXL362		g_spi_cs_pin = (ADXL362_SPI_CS);
#define EN_SPI_BR25S		g_spi_cs_pin = (BR25S_SPI_CS);

#define SENSOR_READY2READ					0x00
#define SENSOR_NOT_READY2READ				0x01

uint8_t initH3LIS331(void);
uint8_t initLSM330(void);
uint8_t initADXL(void);
uint8_t initBR25S(void);
void initSENSOR(void);
uint8_t prepareDataSENSOR(uint8_t battery);
uint8_t prepareDataStickSENSOR(uint16_t* out_data);
void dispDataH3LIS331(void);
void dispDataLSM330(void);
void stopTIMER2(void);
void initTIMER2(void);
uint16_t getBatteryLevel();
void setBatteryLevel(uint16_t battery_level);
void getSettings(uint8_t* ptr);
void setSettings(uint8_t* ptr);
void getDatas(uint8_t* ptr, uint8_t nb_data, uint16_t addr);
void setDatas(uint8_t* ptr, uint8_t nb_data, uint16_t addr);


extern volatile uint8_t g_sensor_ridx;
extern volatile uint8_t g_sensor_widx;
extern volatile uint16_t g_sensor_rcnt;
extern volatile uint8_t g_sensor_read;
extern volatile uint8_t g_sensor_read_flag;
// Five rows of 6 data each
extern volatile uint8_t g_cooked_data[6];
extern volatile uint8_t g_sensor_index;
extern volatile uint8_t settings_flag;
extern volatile uint8_t g_data_send[30];
extern volatile uint8_t g_index_data; 
extern volatile uint16_t g_real_index;
extern volatile uint8_t g_settings[18];
extern volatile uint8_t g_valid;
#endif
