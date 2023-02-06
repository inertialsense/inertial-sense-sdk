#ifndef _D_SERIAL_H
#define _D_SERIAL_H

#include "ISBoards.h"
#include "conf_comm.h"
#include "data_sets.h"
#include "d_usart.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	uint32_t                timeMsLast;
	uint32_t                txByteCount;
	uint32_t                rxByteCount;
	uint32_t                txBytesPerS;
	uint32_t                rxBytesPerS;
	uint32_t                status;
} port_monitor_helper_t;

extern port_monitor_helper_t g_portMonitorHelper[NUM_COM_PORTS];

int8_t serInit(int serialNum, int baud, void* options, uint32_t* overrunStatus);
int8_t serDeinit(int serialNum);

/*
 * Write data on USART/USB. Returns number of bytes written, or -1 for error
 */
int serWrite(int serialNum, const unsigned char *buf, int size);

/*
 * Read data on USART/USB.  Returns number of bytes read, or -1 for error
 */
int serRead(int serialNum, unsigned char *buf, int size);

/*
 * Removes data from USART Rx buffer.  Returns number of bytes removed, or -1 for error. Length
 * of -1 removes all data from USART buffer
 */
int serRxClear(int serialNum, int len);

/*
 * Clear the entire Tx buffer, returns how much was cleared, , or -1 for error
 */
int serTxClear(int serialNum);

/*
 * Clear all data in the buffer before the matching character.  Returns the number 
 * of characters cleared or -1 if the character was not found or there was an error
 */
int serFindCharacter(int serialNum, uint8_t ch);

/*
 * Number of bytes not used in the Tx buffer, or -1 for error
 */
int serTxFree(int serialNum);

/*
 * Sets the baud rate of USART or the terminal settings of USB
 */
int8_t serSetBaudRate(int serialNum, int baudRate);

/**
 * Sets or gets the serial options (parity, stop bits, baudrate, etc.)
 */
int8_t serSetOptions(int serialNum, serial_options_t* options);
int8_t serGetOptions(int serialNum, serial_options_t* options);

#ifdef __cplusplus
}
#endif

#endif
