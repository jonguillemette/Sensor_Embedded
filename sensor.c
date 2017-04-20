#include "sensor.h"
#include "ble_phy.h"

volatile uint8_t g_sensor_ridx;
volatile uint8_t g_sensor_widx;
static uint8_t g_sensor_tx_buff[10];
volatile uint16_t g_sensor_rcnt = 0x0000;
volatile uint8_t g_sensor_read = (SENSOR_NOT_READY2READ);
volatile uint8_t g_sensor_read_flag = 0;
volatile uint8_t g_sensor_shot_data[5][6];
volatile uint8_t g_sensor_index = 0;


void initSENSOR(void)
{
	uint32_t utmp32;
	uint8_t k, n;
	
	
	for(k=0;k<10;k++)
	{
		g_sensor_tx_buff[k] = 0x00;
	}
	
	g_sensor_ridx = (SENSOR_COL_SIZE) - 1;								// last read memory location in the sensor data buffer
	g_sensor_widx = 0x00;												// next memory location in sensor data buffer in which we will write 
	
	
	initSPI0(SPI_DATARATE_4Mbps);										// init SPI interface at 4MHz
	
	if(!initADXL())													// init LSM330 sensor
	{
       printUSART0("ADXL init     [ERROR]\n",0);
       //while(1);
    }
	printUSART0("ADXL init     [DONE]\n",0);

	if(!initBR25S())													// init LSM330 sensor
	{
       printUSART0("BR25S init     [ERROR]\n",0);
       //while(1);
    }
	printUSART0("BR25S init     [DONE]\n",0);

	
	
	if(!initLSM330())													// init LSM330 sensor
	{
       printUSART0("LSM330 init     [ERROR]\n",0);
       //while(1);
    }
	printUSART0("LSM330 init     [DONE]\n",0);
	

	initLIS3MDL();
	

	NVIC_SetPriority(TIMER2_IRQn, 3);
	
	//initTIMER2();														// init TIMER2 with 1ms interrupt
}

uint8_t initLIS3MDL(void)
{/// init Magneto sensor using SPI interface
	uint8_t tx_data[6], rx_data[6];
	uint8_t r_val;
	
	EN_SPI_LIS3MDL;	
	tx_data[0] = 0x60;
	tx_data[0] = 0x62; //CTRL_REG1 
	tx_data[0] = 0x00; //CTRL_REG2
	tx_data[0] = 0x00; //CTRL_REG3
	tx_data[0] = 0x0C; //CTRL_REG4
	tx_data[0] = 0x00; //CTRL_REG5 
	r_val = rxtxSPI0(6, tx_data, rx_data);
	if(r_val == 0x00)													
		return 0x00;													// SPI failed to communicate with sensor
	
	
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
	tx_data[1] = LSM330_G_POWER_DOWN_MODE;
	r_val = rxtxSPI0(2, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;													// SPI failed to communicate with sensor
		
	tx_data[0] = (LSM330_CTRL_REG4_G)|(SPI_WRITE_DATA)|(SPI_SINGLE_TRANS);
	tx_data[1] = (LSM330_G_BDU)|(LSM330_G_SCALE_250);					// 2000dps range + disable data update during read
	r_val = rxtxSPI0(2, tx_data, rx_data);
	if(r_val == 0x00)	
		return 0x00;													// SPI failed to communicate with sensor
	
	//wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	// setup accelerometer parameters
	//wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	EN_SPI_A_LSM330;													// enable SPI communication for LSM330 G sensor
	
	tx_data[0] = (LSM330_CTRL_REG1_A)|(SPI_WRITE_DATA)|(SPI_SINGLE_TRANS);
	tx_data[1] = LSM330_A_ODR_POWER_DOWN;
	r_val = rxtxSPI0(2, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;													// SPI failed to communicate with sensor
		
	tx_data[0] = (LSM330_CTRL_REG4_A)|(SPI_WRITE_DATA)|(SPI_SINGLE_TRANS);
	tx_data[1] = (LSM330_A_SCALE_2G)|(LSM330_A_HIGH_RES);				// 16g range + high resolution
	r_val = rxtxSPI0(2, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;													// SPI failed to communicate with sensor
				
	return 0x01;
}

uint8_t initPowerLSM330(void)
{/// init LSM330 sensor using SPI interface
	uint8_t tx_data[2], rx_data[2];
	uint8_t r_val;

	EN_SPI_G_LSM330;
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
	tx_data[1] = (LSM330_A_SCALE_8G)|(LSM330_A_HIGH_RES);				// 16g range + high resolution
	r_val = rxtxSPI0(2, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;													// SPI failed to communicate with sensor
				
	return 0x01;
}


uint8_t initADXL(void)
{/// init ADXL362 sensor using SPI interface
	uint8_t tx_data[3], rx_data[3];
	uint8_t r_val;
	
	EN_SPI_ADXL362;	

	//wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	// setup interrupt mode
	//wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	tx_data[0] = ADXL_WRITE_REQUETS;
	tx_data[1] = ADXL_THRESH_ACT_L;
	tx_data[2] = 0xFA; //250 mg
	r_val = rxtxSPI0(3, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;

	tx_data[0] = ADXL_WRITE_REQUETS;
	tx_data[1] = ADXL_THRESH_ACT_H;
	tx_data[2] = 0x02; //250 mg + 512 mg detection
	r_val = rxtxSPI0(3, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;

	tx_data[0] = ADXL_WRITE_REQUETS;
	tx_data[1] = ADXL_THRESH_INACT_L;
	tx_data[2] = 0x96; //150 mg
	r_val = rxtxSPI0(3, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;

	tx_data[0] = ADXL_WRITE_REQUETS;
	tx_data[1] = ADXL_THRESH_INACT_H;
	tx_data[2] = 0x00; //150 mg + 0g
	r_val = rxtxSPI0(3, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;

	tx_data[0] = ADXL_WRITE_REQUETS;
	tx_data[1] = ADXL_TIME_INACT_L;
	tx_data[2] = 0x1E; //Active for around 5 seconds (30 samples)
	r_val = rxtxSPI0(3, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;

	tx_data[0] = ADXL_WRITE_REQUETS;
	tx_data[1] = ADXL_ACT_INACT_CTL;
	tx_data[2] = ADXL_SET_MOTION_DETECT_MODE;
	r_val = rxtxSPI0(3, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;

	tx_data[0] = ADXL_WRITE_REQUETS;
	tx_data[1] = ADXL_INTMAP2;
	tx_data[2] = ADXL_SET_AWAKE;
	r_val = rxtxSPI0(3, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;

	tx_data[0] = ADXL_WRITE_REQUETS;
	tx_data[1] = ADXL_POWER_CTL;
	tx_data[2] = ADXL_SET_WAKEUP_START;
	r_val = rxtxSPI0(3, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;
	
	return 0x01;
}

uint8_t initBR25S(void)
{/// init BR25S sensor using SPI interface
	uint8_t tx_data[2], rx_data[2];
	uint8_t r_val;
	
	EN_SPI_BR25S;	

	//wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	// setup interrupt mode
	//wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	tx_data[0] = BR25S_WRSR;
	tx_data[1] = BR25S_SET_MODE;
	r_val = rxtxSPI0(2, tx_data, rx_data);
	if(r_val == 0x00)
		return 0x00;

	return 0x01;
}

uint16_t getBatteryLevel() {
	uint8_t tx_data[6], rx_data[6];
	uint16_t final_value;
	uint8_t validate;
	
	EN_SPI_BR25S;

	tx_data[0] = BR25S_READ;
	tx_data[1] = 0x00;
	tx_data[2] = 0x00;

	rxtxSPI0(6, tx_data, rx_data);
	final_value = 0;
	final_value += rx_data[3];
	final_value += rx_data[4] << 8;
	validate = rx_data[5];

	if (validate != 0x4F) {
		final_value = 65534;
	}
	
	return final_value;
}

void setBatteryLevel(uint16_t battery_level) {
	uint8_t tx_data[6], rx_data[6];


	EN_SPI_BR25S;

	tx_data[0] = BR25S_WREN;
	rxtxSPI0(1, tx_data, rx_data);

	tx_data[0] = BR25S_WRITE;
	tx_data[1] = (uint8_t) 0x00;
	tx_data[2] = (uint8_t) 0x00;
	tx_data[3] = (uint8_t) (battery_level & 0xFF);
	tx_data[4] = (uint8_t) ((battery_level & 0xFF00) >> 8);
	tx_data[5] = 0x4F;	

	rxtxSPI0(6, tx_data, rx_data);
}

// 18 bytes of settings, ptr is already prepared
void getSettings(uint8_t* ptr) {
	uint8_t tx_data[21], rx_data[21];
	uint8_t i;
	
	EN_SPI_BR25S;

	tx_data[0] = BR25S_READ;
	tx_data[1] = BR25S_ADDR_SETTINGS >> 8;
	tx_data[2] = BR25S_ADDR_SETTINGS & 0xFF;

	rxtxSPI0(21, tx_data, rx_data);
	
	for(i=0; i<18; i++) {
		ptr[i] = rx_data[3+i]; 
	}
}

// 18 bytes of settings, ptr is already prepared
void setSettings(uint8_t* ptr) {
	uint8_t tx_data[21], rx_data[21];
	uint8_t i;
	
	EN_SPI_BR25S;

	tx_data[0] = BR25S_WREN;
	rxtxSPI0(1, tx_data, rx_data);

	tx_data[0] = BR25S_WRITE;
	tx_data[1] = BR25S_ADDR_SETTINGS >> 8;
	tx_data[2] = BR25S_ADDR_SETTINGS & 0xFF;
	for(i=0; i<18; i++) {
		tx_data[3+i] = ptr[i];
	}

	rxtxSPI0(21, tx_data, rx_data);
}

void getDatas(uint8_t* ptr, uint8_t nb_data, uint16_t addr) {
	uint8_t tx_data[BR25S_MAX_WRITE_BYTE + 3], rx_data[BR25S_MAX_WRITE_BYTE + 3];
	uint8_t i;
	
	addr += BR25S_ADDR_DATA;

	EN_SPI_BR25S;

	tx_data[0] = BR25S_READ;
	tx_data[1] = addr >> 8;
	tx_data[2] = addr & 0xFF;

	rxtxSPI0(nb_data+3, tx_data, rx_data);
	
	for(i=0; i<nb_data; i++) {
		ptr[i] = rx_data[3+i]; 
	}
}

void setDatas(uint8_t* ptr, uint8_t nb_data, uint16_t addr) {
	uint8_t tx_data[BR25S_MAX_WRITE_BYTE + 3], rx_data[BR25S_MAX_WRITE_BYTE + 3];
	uint8_t i;
	
	addr += BR25S_ADDR_DATA;

	EN_SPI_BR25S;

	tx_data[0] = BR25S_WREN;
	rxtxSPI0(1, tx_data, rx_data);

	tx_data[0] = BR25S_WRITE;
	tx_data[1] = addr >> 8;
	tx_data[2] = addr & 0xFF;
	for(i=0; i<nb_data; i++) {
		tx_data[3+i] = ptr[i];
	}

	rxtxSPI0(nb_data+3, tx_data, rx_data);
}


double toDouble(uint8_t low, uint8_t high) {
	uint16_t conversion_form;

	conversion_form = (uint16_t)low | ((uint16_t)(high) << 8);
	short shortVal = (short) conversion_form;
	if (shortVal<0) {
		return (double) (shortVal*-1);
	}
	return (double) shortVal;
}

double toDoubleError(uint8_t low, uint8_t high, uint8_t sign, uint16_t value) {
	uint16_t conversion_form;
	double double_form;

	conversion_form = (uint16_t)low | ((uint16_t)(high) << 8);
	short shortVal = (short) conversion_form;
	
	if (sign > 0) {
		double_form = (double) (shortVal) - (double)value;
	} else {
		double_form = (double) (shortVal) + (double)value;
	}

	if (double_form < 0) {
		double_form *= -1;
	}

	return double_form;
}

uint16_t toUint16Error(uint8_t low, uint8_t high, uint8_t sign, uint16_t value) {
	uint16_t conversion_form;

	conversion_form = (uint16_t)low | ((uint16_t)(high) << 8);
	short shortVal = (short) conversion_form;

	if (sign > 0) {
		shortVal = shortVal - value;
	} else {
		shortVal = shortVal + value;
	}

	if (shortVal<0) {
		return (uint16_t) (shortVal*-1);
	}
	return (uint16_t) shortVal;
}

uint16_t toUint16(uint8_t low, uint8_t high) {
	uint16_t conversion_form;

	conversion_form = (uint16_t)low | ((uint16_t)(high) << 8);
	short shortVal = (short) conversion_form;
	if (shortVal<0) {
		return (uint16_t) (shortVal*-1);
	}
	return (uint16_t) shortVal;
}

uint16_t getSign(uint8_t low, uint8_t high) {
	uint16_t conversion_form;

	conversion_form = (uint16_t)low | ((uint16_t)(high) << 8);
	short shortVal = (short) conversion_form;

	if (shortVal<0) {
		return 0;
	}
	return 1;
}

uint16_t getSignError(uint8_t low, uint8_t high, uint8_t sign, uint16_t value) {
	uint16_t conversion_form;

	conversion_form = (uint16_t)low | ((uint16_t)(high) << 8);
	short shortVal = (short) conversion_form;

	if (sign > 0) {
		shortVal = shortVal - value;
	} else {
		shortVal = shortVal + value;
	}

	if (shortVal<0) {
		return 0;
	}
	return 1;
}

double pythagore3(double a, double b, double c) {
	return sqrt((a*a) + (b*b) + (c*c));
}

double pythagore2(double a, double b) {
	return sqrt((a*a) + (b*b));
}


// Return value: 0 below threshold
// value : 1 exceed threshold
uint8_t prepareDataSENSOR(uint8_t battery)
{
	uint8_t tx_data[10];
	uint8_t data[10];
	uint16_t conversion_form;
	double convert;
	uint16_t thresh_high = (uint16_t) (g_settings[2])<<8;
	thresh_high |= g_settings[1];
	uint16_t thresh_low = (uint16_t) (g_settings[4])<<8;
	thresh_low |= g_settings[3];
	uint8_t value_ret = 0;

	uint8_t val1, val2;

	uint16_t high_x_g, high_y_g, low_x_g, low_y_g;
	uint8_t high_x_sign, high_y_sign, low_x_sign, low_y_sign;

	high_x_g = g_calib_axis[0] << 4;
	high_y_g = g_calib_axis[1] << 4;
	low_x_g = g_calib_axis[2] << 4;
	low_y_g = g_calib_axis[3] << 4;

	high_x_sign = g_calib_axis[4];
	high_y_sign = g_calib_axis[5];
	low_x_sign = g_calib_axis[6];
	low_y_sign = g_calib_axis[7];

	
	// for LSM330 accelerometer we are collecting x, y & z data
	EN_SPI_A_LSM330;													// enable SPI communication for LSM330 A sensor
	g_sensor_tx_buff[0] = (LSM330_XOUT_L_REG_A)|(SPI_READ_DATA)|(SPI_MULTI_TRANS);
	rxtxSPI0(7, g_sensor_tx_buff, data);

	/*conversion_form = pythagore2(toDoubleError(data[1], data[2], low_x_sign, low_x_g)/16, 
		toDoubleError(data[3], data[4], low_y_sign, low_y_g)/16);
*/

	g_cooked_data[0] = data[3];
	g_cooked_data[1] = data[4];

	g_cooked_data[2] = data[5];
	g_cooked_data[3] = data[6];

	if (thresh_high == 0 && (conversion_form >= thresh_low)) {
		value_ret = 1;
	}
	

	// for LSM330 gyroscope we are collecting z data
	EN_SPI_G_LSM330;													// enable SPI communication for LSM330 G sensor
	g_sensor_tx_buff[0] = (LSM330_XOUT_L_REG_G)|(SPI_READ_DATA)|(SPI_MULTI_TRANS);
	rxtxSPI0(3, g_sensor_tx_buff, data);
	
	conversion_form = (uint16_t) toDouble(data[1], data[2]);

	g_cooked_data[4] = conversion_form & 0xFF;
	g_cooked_data[5] = conversion_form>>8 & 0xFF;

	return value_ret;
}


// Return the pythagorian magnetic field in 2 bytes.
// Using two's complement, the return is "g_magneto_data"
void getMagneto(void)
{
	uint8_t tx_data[7];
	uint8_t data[7];
	uint16_t conversion_form;

	EN_SPI_LIS3MDL;													// enable SPI communication for LSM330 A sensor
	g_sensor_tx_buff[0] = 0xE8;
	rxtxSPI0(7, g_sensor_tx_buff, data);

	conversion_form = pythagore3(toDouble(data[1], data[2])/16, 
		toDouble(data[3], data[4])/16,
		toDouble(data[5], data[6])/16);

	g_magneto_data[0] = conversion_form & 0xFF;
	g_magneto_data[1] = conversion_form>>8 & 0xFF;

}

// Return the Player ID if one is detected.
// The value is filled inside "g_player_id"
// Set all zeros if not found.
// Return value indicate if player was found or not
uint8_t getPlayerID(void) 
{

}


// Calbrate the axis.
// ** Don't care about High G
void calibrationAxis() 
{
	//Step 1, get 10 data of high G and low G
	uint8_t tx_data[10];
	uint8_t data[10];

	short high_x_g[10];
	short high_y_g[10];
	short low_x_g[10];
	short low_y_g[10];
	uint16_t conversion_form;
	unsigned char i;
	double casting_low_x, casting_low_y, casting_high_x, casting_high_y;

	uint16_t saveBig[8];
	uint8_t save[12];
	

	for (i=0; i<10; i++) {
		/*
		//Read HIGH G
		EN_SPI_H3LIS331;													// enable SPI communication for H3LIS331 sensor
		g_sensor_tx_buff[0] = (H3LIS331_ACCEL_XOUT_L_REG)|(SPI_READ_DATA)|(SPI_MULTI_TRANS);	
		rxtxSPI0(5, g_sensor_tx_buff, data);

		conversion_form = (uint16_t)data[1] | ((uint16_t)(data[2]) << 8);
		high_x_g[i] = (short) conversion_form / 16;

		conversion_form = (uint16_t)data[3] | ((uint16_t)(data[4]) << 8);
		high_y_g[i] = (short) conversion_form / 16;
		*/

		//Read LOW_G 
		EN_SPI_A_LSM330;													// enable SPI communication for LSM330 A sensor
		g_sensor_tx_buff[0] = (LSM330_XOUT_L_REG_A)|(SPI_READ_DATA)|(SPI_MULTI_TRANS);
		rxtxSPI0(7, g_sensor_tx_buff, data);

		conversion_form = (uint16_t)data[1] | ((uint16_t)(data[2]) << 8);
		low_x_g[i] = (short) conversion_form / 16;

		conversion_form = (uint16_t)data[3] | ((uint16_t)(data[4]) << 8);
		low_y_g[i] = (short) conversion_form / 16;
	}

	casting_low_x = 0;
	casting_low_y = 0;
	casting_high_x = 0;
	casting_high_y = 0;
	
	for (i=0; i<10; i++) {
		casting_high_x += (double) high_x_g[i];
		casting_high_y += (double) high_y_g[i];
		casting_low_x += (double) low_x_g[i];
		casting_low_y += (double) low_y_g[i];
	}
	casting_high_x /= 10;
	casting_high_y /= 10;
	casting_low_x /= 10;
	casting_low_y /= 10;

	high_x_g[0] = (short) casting_high_x;
	high_y_g[0] = (short) casting_high_y;
	low_x_g[0] = (short) casting_low_x;
	low_y_g[0] = (short) casting_low_y;

	high_x_g[1] = casting_high_x < 0 ? 0 : 1;
	high_y_g[1] = casting_high_y < 0 ? 0 : 1;
	low_x_g[1] = casting_low_x < 0 ? 0 : 1;
	low_y_g[1] = casting_low_y < 0 ? 0 : 1;
	

	g_calib_axis[0] = high_x_g[0] < 0 ? (uint16_t) (high_x_g[0]*-1) : (uint16_t) high_x_g[0];
	g_calib_axis[1] = high_y_g[0] < 0 ? (uint16_t) (high_y_g[0]*-1) : (uint16_t) high_y_g[0];
	g_calib_axis[2] = low_x_g[0] < 0 ? (uint16_t) (low_x_g[0]*-1) : (uint16_t) low_x_g[0];
	g_calib_axis[3] = low_y_g[0] < 0 ? (uint16_t) (low_y_g[0]*-1) : (uint16_t) low_y_g[0];
	
	g_calib_axis[4] = (uint16_t) high_x_g[1];
	g_calib_axis[5] = (uint16_t) high_y_g[1];
	g_calib_axis[6] = (uint16_t) low_x_g[1];
	g_calib_axis[7] = (uint16_t) low_y_g[1];

	save[0] = g_calib_axis[0] & 0xFF;
	save[1] = g_calib_axis[0]>>8 & 0xFF;

	save[2] = g_calib_axis[1] & 0xFF;
	save[3] = g_calib_axis[1]>>8 & 0xFF;

	save[4] = g_calib_axis[2] & 0xFF;
	save[5] = g_calib_axis[2]>>8 & 0xFF;

	save[6] = g_calib_axis[3] & 0xFF;
	save[7] = g_calib_axis[3]>>8 & 0xFF;

	save[8] = g_calib_axis[4] & 0x0001;
	save[9] = g_calib_axis[5] & 0x0001;
	save[10] = g_calib_axis[6] & 0x0001;
	save[11] = g_calib_axis[7] & 0x0001;

	setDatas(save, 12, BR25S_ADDR_CALIB);

}

void loadCalibration() {
	uint8_t save[12];

	getDatas(save, 12, BR25S_ADDR_CALIB);

	g_calib_axis[0] = (uint16_t) save[0];
	g_calib_axis[0] += (uint16_t) (save[1])<<8;
	
	g_calib_axis[1] = (uint16_t) save[2];
	g_calib_axis[1] += (uint16_t) (save[3])<<8;

	g_calib_axis[2] = (uint16_t) save[4];
	g_calib_axis[2] += (uint16_t) (save[5])<<8;

	g_calib_axis[3] = (uint16_t) save[6];
	g_calib_axis[3] += (uint16_t) (save[7])<<8;

	g_calib_axis[4] = save[8];
	g_calib_axis[5] = save[9];
	g_calib_axis[6] = save[10];
	g_calib_axis[7] = save[11];
}
	
void initTIMER2(void)
{/// setup and start 16bit TIMER2 to measure elapsed time in ms and also trigger interrupt every ms for sensor data collection
	NRF_TIMER2->MODE = 0x0000;				  							// set timer mode
	NRF_TIMER2->TASKS_CLEAR = 1;               							// clear the task first to be usable for later
	NRF_TIMER2->PRESCALER = 0x0004;										// 16MHz clock & 16 prescaler means 1MHz timer clock
	NRF_TIMER2->BITMODE = 0x00;		// Set counter to 16 bit resolution
	if (ble_mode == BLE_LAUNCH_MODE)			 {
		NRF_TIMER2->CC[0] = 1000; // 1000Hz
	} else {
		NRF_TIMER2->CC[0] = 1250; // 800Hz
	}
	
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
	if (g_valid)
		g_sensor_read_flag++;
	NRF_TIMER2->EVENTS_COMPARE[0] = 0;									// clear the interrupt flag	
	NRF_TIMER2->TASKS_CLEAR = 1;										// restart TIMER2
}
