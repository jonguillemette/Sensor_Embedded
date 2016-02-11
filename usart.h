#ifndef __USART_H
#define __USART_H

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "misc.h"


#define USAR0_BAUDRATE_9600					0x00275000	
#define USAR0_BAUDRATE_115200				0x01D7E000	
#define USAR0_BAUDRATE_115200				0x01D7E000	
#define USAR0_BAUDRATE_230400				0x03AFB000	
#define USAR0_BAUDRATE_460800				0x075F7000	
#define USAR0_BAUDRATE_921600				0x0EBEDFA4			

void initUSART0(uint8_t tx, uint8_t rx, uint32_t baudrate);
void putcharUSART0(uint8_t data);
void printUSART0(char * str, uint32_t * num);
 
#endif 
