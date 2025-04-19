#include <atmel_start.h>

#include "driver_init.h"
#include "utils.h"


#define BUFF_SIZE	4096
#define READ_ONLY_SIZE	5
uint8_t spiRxBuff[BUFF_SIZE];
uint8_t spiTxBuff[BUFF_SIZE];

uint8_t allZeros[READ_ONLY_SIZE];

int bytesInBuff = 0;
bool readSPI;
uint32_t g_timeMs = 0;

uint8_t UARTInBuff[BUFF_SIZE];
uint16_t UARTInSize = 0;


int SPI_0_transfer(uint8_t* buf, uint32_t len)
{
	struct io_descriptor *io;
	spi_m_sync_get_io_descriptor(&SPI_0, &io);

	spi_m_sync_enable(&SPI_0);
	return io_write(io, buf, len);
}

void UART_0_write(uint8_t* buf, uint32_t len)
{
	if (len == 0) return; 
	
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&UART_0, &io);
	usart_sync_enable(&UART_0);

	io_write(io, buf, len);
}

int main(void)
{
	int readAmt = 0;
	int lastUARTInSize = 0;
	uint32_t lastSizeChangeMs = 0;
	
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	TIMER_0_example();
	
	struct spi_xfer spi_xfer_data = {
		.txbuf = spiTxBuff,
		.rxbuf = spiRxBuff,
		.size = BUFF_SIZE,
	};
	
	
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&UART_0, &io);
	usart_sync_enable(&UART_0);
	
	spi_m_sync_get_io_descriptor(&SPI_0, &io);
	spi_m_sync_enable(&SPI_0);
	
	memset(allZeros, 0x00, READ_ONLY_SIZE);
	
	// enable the interupt
	SERCOM1->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;

	/* Replace with your application code */
	while (1) 
	{
		memset(spiRxBuff, 0x00, READ_ONLY_SIZE);
		if (UARTInSize > 0)
		{
			if (UARTInSize > lastUARTInSize)
			{
				lastSizeChangeMs = g_timeMs;
				lastUARTInSize = UARTInSize;
			}
			else if (g_timeMs > (lastSizeChangeMs+2))
			{
				//SERCOM1->USART.INTENCLR.reg = SERCOM_USART_INTENSET_RXC;
				CRITICAL_SECTION_ENTER();
				memcpy(spiTxBuff, UARTInBuff, UARTInSize);
				spi_xfer_data.size = UARTInSize;
				UARTInSize = 0;
				lastUARTInSize = 0;
				CRITICAL_SECTION_LEAVE();
				//SERCOM1->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
			
				//readAmt = SPI_0_transfer(transferBuff, readAmt);
				gpio_set_pin_level(SPI_CS, false);
				readAmt = spi_m_sync_transfer(&SPI_0, &spi_xfer_data);
				gpio_set_pin_level(SPI_CS, true);
			}
		}
		//if (readSPI)
		else{
			spi_xfer_data.size = READ_ONLY_SIZE;
			memset(spiTxBuff, 0xff, READ_ONLY_SIZE);
			readAmt = 0;
			gpio_set_pin_level(SPI_CS, false);
			while (readAmt+READ_ONLY_SIZE < BUFF_SIZE)
			{
				spi_xfer_data.rxbuf = &spiRxBuff[readAmt];
				readAmt += spi_m_sync_transfer(&SPI_0, &spi_xfer_data);
				if (memcmp(&spiRxBuff[readAmt-READ_ONLY_SIZE], allZeros, READ_ONLY_SIZE) == 0)
					break;
			}
			gpio_set_pin_level(SPI_CS, true);
			
			readSPI = false;
		}
				
		if (memcmp(spiRxBuff, allZeros, READ_ONLY_SIZE) != 0)
			UART_0_write(spiRxBuff, readAmt);
		
		SERCOM1->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
	}
}
