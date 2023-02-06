#ifndef _D_USART_H
#define _D_USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ISBoards.h"
#include "d_dma.h"
#include "conf_comm.h"

#include <stdlib.h>

#define USART_EXTERNAL_DEFAULT_BAUDRATE		921600U
#define USART_INTERNAL_DEFAULT_BAUDRATE		115200U

typedef struct
{
	uint32_t 			   baudRate;
	uint8_t 			   parity;
	uint8_t 			   stopBits;
} serial_options_t;

enum
{
	SER_PARITY_NONE 	= 0,    // No parity
	SER_PARITY_ODD 		= 1,    // Odd parity
	SER_PARITY_EVEN  	= 2,    // Even parity
	SER_PARITY_DEFAULT 	= SER_PARITY_NONE,  // No parity
};

enum
{
	SER_STOP_1_0 	= 0,    // 1 stop bit
	SER_STOP_0_5 	= 1,    // 0.5 stop bits
	SER_STOP_2_0  	= 2,    // 2 stop bits
	SER_STOP_1_5 	= 3,    // 1.5 stop bits
    SER_STOP_DEFAULT = SER_STOP_1_0,    // 1 stop bit
};

typedef struct
{
    serial_options_t    coding;
	IRQn_Type 		    interrupt;
} usart_cfg_t;

typedef struct
{
	USART_TypeDef 	*instance;
	usart_cfg_t 	cfg;
	dma_ch_t 		*rxdma;
	dma_ch_t 		*txdma;
	uint8_t 		rxbuf[USART_BUF_LEN];
	uint8_t 		txbuf[USART_BUF_LEN];
	bool			inited;		// True if peripheral is init'ed
} usart_t;

extern usart_t g_usart[NUM_USART];

/*
 * Populates a usart_t struct with the USART defaults for the application.
 * 
 * Fill in the init->instance field with the hardware instance of the USART/UART 
 * before running this function. 
 */
void usart_get_defaults(usart_t *init);

/*
 * Sets up the DMA and USART peripherals for TX and RX
 */
int8_t usart_init(usart_t *init);
int8_t usart_deinit(usart_t *init);

/**
 * Sets the parity mode for the USART peripheral (see enum above)
 */
void usart_parity_set(usart_t *init, uint8_t parity);

/*
 * General purpose TX/RX functions. Return actual bytes transmitted/received
 */
int usart_receive(usart_t *usart, uint8_t *buf, int length);
int usart_transmit(usart_t *usart, const uint8_t *buf, int length);

/*
 * These functions clear the RX/TX buffers. usart_rx_clear clears the specified
 * amount of bytes, -1 for all. usart_tx_clear clears full buffer. Both return
 * number of bytes cleared.
 */
int usart_rx_clear(usart_t *init, int length);
int usart_tx_clear(usart_t *init);

int usart_rx_available(usart_t *init);
int usart_tx_free(usart_t *init);

int usart_find_character(usart_t *init, uint8_t ch);

uint8_t usart_tx_done(usart_t *init);

void usart_set_options(usart_t *init, serial_options_t *options);
void usart_get_options(usart_t *init, serial_options_t *options);

#ifdef __cplusplus
}
#endif
#endif
