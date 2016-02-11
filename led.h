#ifndef _LED_H_
#define _LED_H_

#include <stdbool.h>
#include <stdint.h>
#include <nrf51.h>

#define LED_RED		2
#define LED_GREEN	7
#define LED_BLUE	5


void initLED(void);
void clrAllLED(void);
void setLED(uint8_t led);
void clrLED(uint8_t led);
void actLED(uint8_t data);
#endif 
