#ifndef __DELAY_H
#define __DELAY_H

#include "nrf51.h"

#define SYSTIM_TIMEOUT							1
#define SYSTIM_KEEP_ALIVE						0

void 		delay_ms(uint32_t ms);
void 		delay_us(uint16_t us);

void 		initSYSTIM(void);
uint32_t 	getSYSTIM(void);
void 		stopSYSTIM(void);
uint8_t 	testSYSTIM(uint32_t t_beg, uint32_t t_period);

extern volatile uint32_t g_systim; 

#endif 
