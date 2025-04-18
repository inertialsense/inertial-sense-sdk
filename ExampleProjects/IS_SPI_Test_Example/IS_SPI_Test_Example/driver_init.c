/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>

#include <hpl_rtc_base.h>

struct spi_m_sync_descriptor SPI_0;
struct timer_descriptor      TIMER_0;

struct usart_sync_descriptor UART_0;

void SPI_0_PORT_init(void)
{

	gpio_set_pin_level(SPI_MOSI,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(SPI_MOSI, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SPI_MOSI, PINMUX_PA04D_SERCOM0_PAD0);

	gpio_set_pin_level(SPI_CLK,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(SPI_CLK, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SPI_CLK, PINMUX_PA05D_SERCOM0_PAD1);

	// Set pin direction to input
	gpio_set_pin_direction(SPI_MISO, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(SPI_MISO,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SPI_MISO, PINMUX_PA06D_SERCOM0_PAD2);
}

void SPI_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
	_gclk_enable_channel(SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC);
}

void SPI_0_init(void)
{
	SPI_0_CLOCK_init();
	spi_m_sync_init(&SPI_0, SERCOM0);
	SPI_0_PORT_init();
}

void UART_0_PORT_init(void)
{
	gpio_set_pin_function(PA16, PINMUX_PA16C_SERCOM1_PAD0);
	gpio_set_pin_function(PA17, PINMUX_PA17C_SERCOM1_PAD1);
}

void UART_0_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM1);
	_gclk_enable_channel(SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC);
}

extern uint8_t UARTInBuff[];
extern uint16_t UARTInSize;
extern uint16_t UARTInPtr;

void SERCOM1_Handler()
{
	// CHECK ERROR
	if(SERCOM1->USART.INTFLAG.reg & SERCOM_USART_INTENSET_ERROR)
	{
		SERCOM1->USART.INTFLAG.reg |= SERCOM_USART_INTENSET_ERROR;
	}
	// CHECK RXC or Receive complete
	else if(SERCOM1->USART.INTFLAG.reg & SERCOM_USART_INTENSET_RXC)
	{
		UARTInBuff[UARTInSize] = SERCOM1->USART.DATA.reg;
		UARTInSize++;
	}
	// CHECK RXS or Receive start
	else if(SERCOM1->USART.INTFLAG.reg & SERCOM_USART_INTENSET_RXS)
	{
		UARTInBuff[UARTInSize] = SERCOM1->USART.DATA.reg;
		UARTInSize++;
	}
	
	SERCOM1->USART.INTFLAG.reg |= SERCOM_USART_INTENSET_RXS;
	SERCOM1->USART.INTFLAG.reg |= SERCOM_USART_INTENSET_RXC;
	SERCOM1->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
	SERCOM1->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXS;
	SERCOM1->USART.INTENSET.reg = SERCOM_USART_INTENSET_ERROR;
}

void UART_0_init(void)
{
	UART_0_CLOCK_init();
	usart_sync_init(&UART_0, SERCOM1, (void *)NULL);
	UART_0_PORT_init();
	
	
	NVIC_DisableIRQ((IRQn_Type)SERCOM1_IRQn);
	NVIC_ClearPendingIRQ((IRQn_Type)SERCOM1_IRQn);
	NVIC_EnableIRQ((IRQn_Type)SERCOM1_IRQn);
}

/**
 * \brief Timer initialization function
 *
 * Enables Timer peripheral, clocks and initializes Timer driver
 */
static void TIMER_0_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBA, RTC);
	_gclk_enable_channel(RTC_GCLK_ID, CONF_GCLK_RTC_SRC);
	timer_init(&TIMER_0, RTC, _rtc_get_timer());
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA03

	gpio_set_pin_level(SPI_CS,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(SPI_CS, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SPI_CS, GPIO_PIN_FUNCTION_OFF);

	SPI_0_init();

	UART_0_init();

	TIMER_0_init();
}
