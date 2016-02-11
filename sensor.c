#include "sensor.h"

volatile uint8_t g_sensor_data[SENSOR_COL_SIZE][SENSOR_ROW_SIZE];
volatile uint8_t g_sensor_ridx;
volatile uint8_t g_sensor_widx;
static uint8_t g_sensor_tx_buff[10];
volatile uint16_t g_sensor_rcnt = 0x0000;
volatile uint8_t g_sensor_read = (SENSOR_NOT_READY2READ);
volatile uint8_t g_sensor_read_flag = 0;

void initSENSOR(void)
{
	uint32_t utmp32;
	uint8_t k, n;
	
	for(k=0;k<(SENSOR_COL_SIZE);k++)
	{
		for(n=0;n<(SENSOR_ROW_SIZE);n++)
		{
			g_sensor_data[k][n] = 0x00;
		}
	}
	for(k=0;k<10;k++)
	{
		g_sensor_tx_buff[k] = 0x00;
	}
	
	g_sensor_ridx = (SENSOR_COL_SIZE) - 1;								// last read memory location in the sensor data buffer
	g_sensor_widx = 0x00;												// next memory location in sensor data buffer in which we will write 
	
	
	initSPI0(SPI_DATARATE_4Mbps);										// init SPI interface at 4MHz
	
	return;
		
	if(!initH3LIS331())													// init H3LIS331 sensor
	{
       printUSART0("H3LIS331 init   [ERROR]\n",0);
       while(1);
    }
	printUSART0("H3LIS331 init   [DONE]\n",0);
	
	if(!initLSM330())													// init LSM330 sensor
	{
       printUSART0("LSM330 init     [ERROR]\n",0);
       while(1);
    }
	printUSART0("LSM330 init     [DONE]\n",0);
	
	NVIC_SetPriority(TIMER2_IRQn, 3);
	
	//initTIMER2();														// init TIMER2 with 1ms interrupt
}

uint8_t initH3LIS331(void)
{/// init H3LIS331 sensor using SPI interface
	uint8_t tx_data[4], rx_data[4];
	uint8_t r_val;
	
	EN_SPI_H3LIS331;													// enable SPI communication for H3LIS331 sensor
	tx_data[0] = (H3LIS331_WHO_AM_I_REG)|(SPI_READ_DATA)|(SPI_SINGLE_TRANS);
	tx_data[1] = 0x00;
	r_val = rxtxSPI0(2, tx_data, rx_data);
	if(r_val == 0x00)													
		return 0x00;													// SPI failed to communicate with sensor
	
	if(rx_data[1] != (H3LIS331_I2C_ADDR))								// check if we have correct sensor -> H3LIS331
		return 0x00;
	
	//wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	// setup accelerometer parameters
	//wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	tx_data[0] = (H3LIS331_CTRL_REG_1)|(SPI_WRITE_DATA)|(SPI_SINGLE_TRANS);
	// normal work mode, 1kHz sample rate and enable all three axeses 
	tx_data[1] = (H3LIS331_POWER_MODE_NORMAL)|(H3LIS331_ODR_1000Hz);		
	tx_data[1] |= (H3LIS331_ENABLE_Z_AXIS)|(H3LIS331_ENABLE_Y_AXIS)|(H3LIS331_ENABLE_X_AXIS);
	r_val = rxtxSPI0(2, tx_data, rx_data);
	if(r_val == 0x00)													
		return 0x00;													// SPI failed to communicate with sensor
	
	// setup sensor parameters, write config data to Control Register 4
	tx_data[0] = (H3LIS331_CTRL_REG_4)|(SPI_WRITE_DATA)|(SPI_SINGLE_TRANS);
	tx_data[1] = (H3LIS331_BDU)|(H3LIS331_SCALE_400g);					// 400g range + disable update during the read
	r_val = rxtxSPI0(2, tx_data, rx_data);
	if(r_val == 0x00)													// SPI failed to communicate with sensor
		return 0x00;
	
	return 0x01;
}

uint8_t initLSM330(void)
{/// init LSM330 sensor using SPI interface
	uint8_t tx_data[2], rx_data[2];
	uint8_t r_val;
	
	EN_SPI_G_LSM330;													// enable SPI communication for LSM330 A sensor
	tx_data[0] = (LSM330_WHO_AM_I_REG_G)|(SPI_READ_DATA)|(SPI_SINGLE_TRANS);
	tx_data[1] = 0x00;
	r_val = rxtxSPI0(2, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;													// SPI failed to communicate with sensor
	
	if(rx_data[1] != (LSM330_I2C_ADDR_G))
		return 0x00;													// failed to detect LSM330 sensor 
	
	//wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	// setup gyroscope parameters
	//wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	tx_data[0] = (LSM330_CTRL_REG1_G)|(SPI_WRITE_DATA)|(SPI_SINGLE_TRANS);
	// normal work mode, 760Hz sample rate and enable all three axeses 
	tx_data[1] = (LSM330_G_ODR_760)|(LSM330_G_ODR_BW_3)|(LSM330_G_NORMAL_MODE)|(LSM330_G_ENABLE_Z_AXIS);
	tx_data[1] |= (LSM330_G_ENABLE_Y_AXIS)|(LSM330_G_ENABLE_X_AXIS);	
	r_val = rxtxSPI0(2, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;													// SPI failed to communicate with sensor
		
	tx_data[0] = (LSM330_CTRL_REG4_G)|(SPI_WRITE_DATA)|(SPI_SINGLE_TRANS);
	tx_data[1] = (LSM330_G_BDU)|(LSM330_G_SCALE_2000);					// 2000dps range + disable data update during read
	r_val = rxtxSPI0(2, tx_data, rx_data);
	if(r_val == 0x00)	
		return 0x00;													// SPI failed to communicate with sensor
	
	//wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	// setup accelerometer parameters
	//wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	EN_SPI_A_LSM330;													// enable SPI communication for LSM330 G sensor
	
	tx_data[0] = (LSM330_CTRL_REG1_A)|(SPI_WRITE_DATA)|(SPI_SINGLE_TRANS);
	// normal mode, 1.620kHz sample rate and enable all three axeses 
	tx_data[1] = (LSM330_A_ODR_NORMAL_LP_1344Hz)|(LSM330_A_ENABLE_Z_AXIS);
	tx_data[1] |= (LSM330_A_ENABLE_Y_AXIS)|(LSM330_A_ENABLE_X_AXIS);	
	r_val = rxtxSPI0(2, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;													// SPI failed to communicate with sensor
		
	tx_data[0] = (LSM330_CTRL_REG4_A)|(SPI_WRITE_DATA)|(SPI_SINGLE_TRANS);
	tx_data[1] = (LSM330_A_SCALE_16G)|(LSM330_A_HIGH_RES);				// 16g range + high resolution
	r_val = rxtxSPI0(2, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;													// SPI failed to communicate with sensor
				
	return 0x01;
}

void getDataSENSOR(void)
{
	uint8_t tx_data[10];
	uint8_t data[10];
	
	
	if(g_sensor_widx == g_sensor_ridx)									// sensor data buffer is full
		return;						
	
	g_sensor_data[g_sensor_widx][0] = (g_sensor_rcnt>>8);
	g_sensor_data[g_sensor_widx][1] = g_sensor_rcnt;
	g_sensor_rcnt++;

	// for H3LIS331 accelerometer we are collecting x & y data
	EN_SPI_H3LIS331;													// enable SPI communication for L3LIS331 sensor
	g_sensor_tx_buff[0] = (H3LIS331_ACCEL_XOUT_L_REG)|(SPI_READ_DATA)|(SPI_MULTI_TRANS);	
	rxtxSPI0(5, g_sensor_tx_buff, data);
	g_sensor_data[g_sensor_widx][2] = data[1];
	g_sensor_data[g_sensor_widx][3] = data[2];
	g_sensor_data[g_sensor_widx][4] = data[3];
	g_sensor_data[g_sensor_widx][5] = data[4];
		
	// for LSM330 accelerometer we are collecting x, y & z data
	EN_SPI_A_LSM330;													// enable SPI communication for LSM330 A sensor
	g_sensor_tx_buff[0] = (LSM330_XOUT_L_REG_A)|(SPI_READ_DATA)|(SPI_MULTI_TRANS);
	rxtxSPI0(7, g_sensor_tx_buff, data);
	g_sensor_data[g_sensor_widx][6] = data[1];
	g_sensor_data[g_sensor_widx][7] = data[2];
	g_sensor_data[g_sensor_widx][8] = data[3];
	g_sensor_data[g_sensor_widx][9] = data[4];
	g_sensor_data[g_sensor_widx][10] = data[5];
	g_sensor_data[g_sensor_widx][11] = data[6];
	
	// for LSM330 gyroscope we are collecting z data
	EN_SPI_G_LSM330;													// enable SPI communication for LSM330 G sensor
	g_sensor_tx_buff[0] = (LSM330_ZOUT_L_REG_G)|(SPI_READ_DATA)|(SPI_MULTI_TRANS);
	rxtxSPI0(3, g_sensor_tx_buff, data);
	g_sensor_data[g_sensor_widx][12] = data[1];
	g_sensor_data[g_sensor_widx][13] = data[2];
	
	g_sensor_widx++;													// step to next location
	if(g_sensor_widx == (SENSOR_COL_SIZE))								// check if we have reached end of the circular buffer
		g_sensor_widx = 0x00;
}
	
void initTIMER2(void)
{/// setup and start 16bit TIMER2 to measure elapsed time in ms and also trigger interrupt every ms for sensor data collection
	NRF_TIMER2->MODE = 0x0000;				  							// set timer mode
	NRF_TIMER2->TASKS_CLEAR = 1;               							// clear the task first to be usable for later
	NRF_TIMER2->PRESCALER = 0x0004;										// 16MHz clock & 16 prescaler means 1MHz timer clock
	NRF_TIMER2->BITMODE = 0x00;											// Set counter to 16 bit resolution
	NRF_TIMER2->CC[0] = 1250;											// tick on each 800Hz!
	
	// enable interrupt on Timer 2
	//NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Enabled
        //<< TIMER_INTENSET_COMPARE0_Pos;
	NRF_TIMER2->INTENSET = (1<<16);
	NVIC_EnableIRQ(TIMER2_IRQn);
		
	NRF_TIMER2->TASKS_START = 1;               							// start TIMER2
}

void stopTIMER2(void)
{
	NRF_TIMER2->TASKS_STOP = 1;               							// stop TIMER2
}

void TIMER2_IRQHandler(void)
{/// timer 2 capture compare interrupt event execution 	
	
	g_sensor_read_flag++;
	NRF_TIMER2->EVENTS_COMPARE[0] = 0;									// clear the interrupt flag	
	NRF_TIMER2->TASKS_CLEAR = 1;										// restart TIMER2
}

void dispDataH3LIS331(void)
{
	uint8_t data[7];
	int8_t * pdata;
	uint32_t utmp32;
	
	EN_SPI_H3LIS331;
	// read H3LIS331 accelerometer data x, y & z 
	g_sensor_tx_buff[0] = (H3LIS331_ACCEL_XOUT_L_REG)|(SPI_READ_DATA)|(SPI_MULTI_TRANS);
	
	rxtxSPI0(7, g_sensor_tx_buff, data);
	pdata = data;
	
	utmp32 = (((int16_t)pdata[2]<<8)|((int16_t)pdata[1]));
	printUSART0("H3LIS331 Accel x: [%d]\n",&utmp32);
	
	utmp32 = (((int16_t)pdata[4]<<8)|((int16_t)pdata[3]));
	printUSART0("H3LIS331 Accel y: [%d]\n",&utmp32);
	
	utmp32 = (((int16_t)pdata[6]<<8)|((int16_t)pdata[5]));
	printUSART0("H3LIS331 Accel z: [%d]\n",&utmp32);	
	
}

void dispDataLSM330(void)
{
	uint8_t a_data[7];
	uint8_t g_data[7];
	int8_t * pdata;
	uint32_t utmp32;
	
	EN_SPI_A_LSM330;
	// read LSM330 accelerometer data x, y & z 
	g_sensor_tx_buff[0] = (LSM330_XOUT_L_REG_A)|(SPI_READ_DATA)|(SPI_MULTI_TRANS);
	
	rxtxSPI0(7, g_sensor_tx_buff, a_data);
	pdata = a_data;
	
	utmp32 = (((int16_t)pdata[2]<<8)|((int16_t)pdata[1]));
	printUSART0("LSM330 Accel x: [%d]\n",&utmp32);
	
	utmp32 = (((int16_t)pdata[4]<<8)|((int16_t)pdata[3]));
	printUSART0("LSM330 Accel y: [%d]\n",&utmp32);
	
	utmp32 = (((int16_t)pdata[6]<<8)|((int16_t)pdata[5]));
	printUSART0("LSM330 Accel z: [%d]\n",&utmp32);	
	
	
	EN_SPI_G_LSM330;
	// read LSM330 gyroscope data x, y & z 
	g_sensor_tx_buff[0] = (LSM330_XOUT_L_REG_G)|(SPI_READ_DATA)|(SPI_MULTI_TRANS);
	
	rxtxSPI0(7, g_sensor_tx_buff, g_data);
	pdata = g_data;
	
	utmp32 = (((int16_t)pdata[2]<<8)|((int16_t)pdata[1]));
	printUSART0("LSM330 Gyro x: [%d]\n",&utmp32);
	
	utmp32 = (((int16_t)pdata[4]<<8)|((int16_t)pdata[3]));
	printUSART0("LSM330 Gyro y: [%d]\n",&utmp32);
	
	utmp32 = (((int16_t)pdata[6]<<8)|((int16_t)pdata[5]));
	printUSART0("LSM330 Gyro z: [%d]\n",&utmp32);	
	

}

