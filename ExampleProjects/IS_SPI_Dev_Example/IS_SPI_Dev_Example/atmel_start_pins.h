/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMD21 has 8 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7

#define SPI_MOSI GPIO(GPIO_PORTA, 16)
#define SPI_CLK GPIO(GPIO_PORTA, 17)
#define SPI_MISO GPIO(GPIO_PORTA, 19)
#define SPI_CS GPIO(GPIO_PORTA, 18)
#define MODE_SELECT GPIO(GPIO_PORTA, 4)
//#define UART_0_TX GPIO(GPIO_PORTA, 16)
//#define UART_0_RX GPIO(GPIO_PORTA, 17)
#define USB_DM GPIO(GPIO_PORTA, 24)
#define USB_DP GPIO(GPIO_PORTA, 25)
#define DATA_READY GPIO(GPIO_PORTB, 9)

#endif // ATMEL_START_PINS_H_INCLUDED
