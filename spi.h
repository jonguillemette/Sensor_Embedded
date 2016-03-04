#ifndef _SPI_H__
#define _SPI_H__

#include "nrf51.h"
#include "nrf_gpio.h"


#define SPI_WRITE_DATA			0x00
#define SPI_READ_DATA			0x80
#define SPI_SINGLE_TRANS		0x00
#define SPI_MULTI_TRANS			0x40

extern volatile uint8_t g_spi_cs_pin;


#define LSM330_SPI_CS_A			4
#define LSM330_SPI_CS_G			5
#define SPI0_SCK				8
#define SPI0_MOSI				9
#define H3LIS331_SPI_CS			10
#define SPI0_MISO				11
#define ADXL362_SPI_CS			15
#define BR25S_SPI_CS			28


#define SPI_DATARATE_125kbps	0x02000000
#define SPI_DATARATE_250kbps 	0x04000000 
#define SPI_DATARATE_500kbps	0x08000000 
#define SPI_DATARATE_1Mbps		0x10000000 
#define SPI_DATARATE_2Mbps	 	0x20000000 
#define SPI_DATARATE_4Mbps 		0x40000000 
#define SPI_DATARATE_8Mbps 		0x80000000 		// this is not advisable

#define SPI_ENABLE 				0x00000001

void 	initSPI0(uint32_t freq);
//uint8_t rxtxSPI0(uint16_t tx_size, const uint8_t *tx_data, uint8_t *rx_data);
//uint8_t rxtxSPI0(uint16_t transfer_size, const uint8_t *tx_data, uint8_t *rx_data);
#endif 
