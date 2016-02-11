#include "delay.h"
volatile uint32_t g_systim = 0;

void delay_ms(uint32_t ms)
{/// delay based on 16 bit TIMER1, max delay 2^32 ms
	uint32_t k;
	for(k=0;k<ms;k++)
	{
		delay_us(1000);
	}
}

void delay_us(uint16_t us)
{/// delay based on 16 bit TIMER1, max delay 2^16 us
	NRF_TIMER1->TASKS_CLEAR = 1;
	
	NRF_TIMER1->MODE = 0x0000;											// select counter mode
	NRF_TIMER1->PRESCALER = 0x0004; 									// 16 MHz clock, prescaler 16 -> 1us time step
	NRF_TIMER1->BITMODE = 0x0000;										// 16 bit timer mode
	
	NRF_TIMER1->CC[0] = us;
	NRF_TIMER1->SHORTS = 0x0001;
	NRF_TIMER1->TASKS_START = 1;
	

	while(NRF_TIMER1->EVENTS_COMPARE[0] == 0);
	NRF_TIMER1->EVENTS_COMPARE[0] = 0;
	NRF_TIMER1->TASKS_STOP = 1;
}


void initSYSTIM(void)
{/// setup and start 16bit TIMER0 to measure elapsed time in ms!
	NRF_TIMER0->MODE = 0x0000;				  							// set timer mode
	NRF_TIMER0->TASKS_CLEAR = 1;               							// clear the task first to be usable for later
	NRF_TIMER0->PRESCALER = 0x0004;										// 16MHz clock & 16 prescaler means 1MHz timer clock
	NRF_TIMER0->BITMODE = 0x00;											// Set counter to 16 bit resolution
	NRF_TIMER0->CC[0] = 1000;											// tick on each ms!
	
	// enable interrupt on Timer 2
	//NRF_TIMER0->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
	NRF_TIMER0->INTENSET = (1<<16);
	NVIC_EnableIRQ(TIMER0_IRQn);
		
	NRF_TIMER0->TASKS_START = 1;               							// start TIMER2
}

void TIMER0_IRQHandler(void)
{/// timer 0 capture compare interrupt event execution 	
	NRF_TIMER0->EVENTS_COMPARE[0] = 0;									// clear the interrupt flag
	g_systim++;
	NRF_TIMER0->TASKS_CLEAR = 1;										// restart TIMER0
}

uint32_t getSYSTIM(void)
{
	return g_systim;
}

void stopSYSTIM(void)
{/// stop TIMER0 and return elapsed time!
	NRF_TIMER0->TASKS_START = 0;               							// stop TIMER0
}

uint8_t testSYSTIM(uint32_t t_beg, uint32_t t_period)
{	
	if(g_systim > t_beg)
	{
		if((g_systim - t_beg) >= t_period)
			return (SYSTIM_TIMEOUT);
		else
			return (SYSTIM_KEEP_ALIVE);
	}
	else
	{
		if(g_systim > (t_beg + t_period))
			return (SYSTIM_TIMEOUT);
		else
			return (SYSTIM_KEEP_ALIVE);
	}
}

