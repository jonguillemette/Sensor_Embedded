#include "led.h"

void initLED(void)
{
	NRF_GPIO->PIN_CNF[LED_RED] = 0x00000001;							// define pin as output 
	NRF_GPIO->PIN_CNF[LED_GREEN] = 0x00000001;							// define pin as output 
	NRF_GPIO->PIN_CNF[LED_BLUE] = 0x00000001;							// define pin as output
	clrAllLED(); 
}

void clrAllLED(void)
{
	NRF_GPIO->OUTSET = (1<<LED_RED);
	NRF_GPIO->OUTSET = (1<<LED_BLUE);
	NRF_GPIO->OUTSET = (1<<LED_GREEN);	
}

void setLED(uint8_t led)
{
	NRF_GPIO->OUTCLR = (1<<led);
}

void clrLED(uint8_t led)
{
	NRF_GPIO->OUTSET = (1<<led);
}

void actLED(uint8_t data)
{
	if(data == 0x01)
	{
		clrAllLED();
		setLED(LED_RED);
	}
	else if(data == 0x02)
	{
		clrAllLED();
		setLED(LED_GREEN);
	}
	else if(data == 0x03)
	{
		clrAllLED();
		setLED(LED_BLUE);
	}
	else 
	{
		clrAllLED();
	}
}
