#include "usart.h"


void initUSART0(uint8_t tx, uint8_t rx, uint32_t baudrate)
{
	//wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
	// USART1 initialization on tx -> TX & rx -> RX
	//------------------------------------------------------------------
	NRF_GPIO->PIN_CNF[tx] = 0x00000001;									// set as output
	NRF_GPIO->PIN_CNF[rx] = 0x00000000;									// set as input, sense off, drive S0S1, pull-down
																		// connect input buffer  
	NRF_UART0->PSELTXD = tx;
	NRF_UART0->PSELRXD = rx;

	NRF_UART0->BAUDRATE         = baudrate;
	NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
	NRF_UART0->TASKS_STARTTX    = 1;
	NRF_UART0->TASKS_STARTRX    = 1;
	NRF_UART0->EVENTS_RXDRDY    = 0;
}

void putcharUSART0(uint8_t data)
{
	NRF_UART0->TXD = data;
	while (NRF_UART0->EVENTS_TXDRDY != 1);

	NRF_UART0->EVENTS_TXDRDY = 0;
}



void printUSART0(char * str, uint32_t  * num)
{/// print text and one signed integer or float number
  // the 'num' number is not modified!
	uint8_t r_str[MAX_PRINT_STRING_SIZE];
	uint16_t k = 0;

	getASCIIString(str, num, r_str);
	
	while(r_str[k] != '\0')
	{
		putcharUSART0(r_str[k]);
		if(r_str[k] == '\n')
			putcharUSART0('\r');
		k++;
		
		if(k == MAX_PRINT_STRING_SIZE)
			break;
	}
}
