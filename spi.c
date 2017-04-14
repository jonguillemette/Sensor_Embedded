#include "spi.h"

volatile uint8_t g_spi_cs_pin;

void initSPI0(uint32_t freq)
{/// init SPI0 module
	NRF_TWI0->ENABLE = 0x00;											// disable TWI0 since it shares some registers with SPI0 
 
	NRF_GPIO->PIN_CNF[SPI0_SCK] = 0x00000001;							// set as output
	NRF_GPIO->PIN_CNF[SPI0_MOSI] = 0x00000001;							// set as output
	NRF_GPIO->PIN_CNF[SPI0_MISO] = 0x00000000;							// set as input, sense off, drive S0S1, pull-down
	
	NRF_GPIO->PIN_CNF[H3LIS331_SPI_CS] = 0x00000301;					// set as output
	NRF_GPIO->OUTSET = (1<<H3LIS331_SPI_CS);							// set CS to high state
	
	NRF_GPIO->PIN_CNF[LSM330_SPI_CS_A] = 0x00000001;					// set as output
	NRF_GPIO->OUTSET = (1<<LSM330_SPI_CS_A);							// set CS to high state
	
	NRF_GPIO->PIN_CNF[LSM330_SPI_CS_G] = 0x00000001;					// set as output
	NRF_GPIO->OUTSET = (1<<LSM330_SPI_CS_G);

    NRF_GPIO->PIN_CNF[ADXL362_SPI_CS] = 0x00000001;                    // set as output
    NRF_GPIO->OUTSET = (1<<ADXL362_SPI_CS);							// set CS to high state
    
    NRF_GPIO->PIN_CNF[BR25S_SPI_CS] = 0x00000001;                    // set as output
    NRF_GPIO->OUTSET = (1<<BR25S_SPI_CS);                         // set CS to high state


	NRF_SPI0->PSELSCK  = SPI0_SCK;										
	NRF_SPI0->PSELMOSI = SPI0_MOSI;
	NRF_SPI0->PSELMISO = SPI0_MISO;
	
    NRF_SPI0->FREQUENCY = freq;											// set SPI clock rate
	NRF_SPI0->CONFIG  = 0x00;											// Tx MSB first, CPHA_Leading, CPOL_ActiveHigh
    NRF_SPI0->EVENTS_READY = 0x00;										// clear the status flags
    NRF_SPI0->ENABLE = (SPI_ENABLE);									// enable SPI0
}

//uint8_t rxtxSPI0(uint16_t tx_size, const uint8_t *tx_data, uint8_t *rx_data)
uint8_t rxtxSPI0(uint16_t transfer_size, const uint8_t *tx_data, uint8_t *rx_data)
{
    uint32_t counter = 0;
    uint16_t number_of_txd_bytes = 0;
    //uint32_t SEL_SS_PINOUT;
    uint32_t TIMEOUT_COUNTER = 50000;

	//SEL_SS_PINOUT = SPI0_NSS;

    /* enable slave (slave select active low) */
    //nrf_gpio_pin_clear(SEL_SS_PINOUT);
	NRF_GPIO->OUTCLR = (1<<g_spi_cs_pin);
    while(number_of_txd_bytes < transfer_size)
    {
		NRF_SPI0->TXD = (uint32_t)(tx_data[number_of_txd_bytes]);

        /* Wait for the transaction complete or timeout (about 10ms - 20 ms) */
        counter = 0;
        while ((NRF_SPI0->EVENTS_READY == 0) && (counter < TIMEOUT_COUNTER))
        {
            counter++;
        }

        if (counter == TIMEOUT_COUNTER)
        {
            /* timed out, disable slave (slave select active low) and return with error */
            //nrf_gpio_pin_set(SEL_SS_PINOUT);
            NRF_GPIO->OUTSET = (1<<g_spi_cs_pin);
            //return false;
            return 0;
        }
        else
        {   /* clear the event to be ready to receive next messages */
            NRF_SPI0->EVENTS_READY = 0U;
        }

		//NRF_SPI0->EVENTS_READY = 0;
        rx_data[number_of_txd_bytes] = (uint8_t)NRF_SPI0->RXD;
        number_of_txd_bytes++;
    }

    /* disable slave (slave select active low) */
    //nrf_gpio_pin_set(SEL_SS_PINOUT);
    NRF_GPIO->OUTCLR = (1<<g_spi_cs_pin);
	NRF_GPIO->OUTSET = (1<<g_spi_cs_pin);
    return 1;
}
