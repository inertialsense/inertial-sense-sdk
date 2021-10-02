/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <asf.h>
#include <string.h>
#include <stdio.h>

#include "d_i2c.h"

//Configuration for TWI instance
#define BOARD_ID_TWI	ID_TWIHS0
#define BOARD_TWI		TWIHS0
#define TWI_CLK			400000

int i2c_get_defaults(i2c_t *init)
{
	
	init->cfg.master = true;

	init->cfg.speed = 400000;


	return STATUS_OK;
}

int i2cInit( void )
{
	//Configure pins
	ioport_set_pin_peripheral_mode(I2C_0_SDA_PIN, I2C_0_SDA_FLAGS);
	ioport_set_pin_peripheral_mode(I2C_0_SCL_PIN, I2C_0_SCL_FLAGS);
	
	/* Enable the peripheral clock for TWI */
	pmc_enable_periph_clk(BOARD_ID_TWI);
	
	
	
	
	
	

	/* Configure the options of TWI driver */
	twihs_options_t opt;
	opt.master_clk = sysclk_get_peripheral_hz();
	opt.speed      = TWI_CLK;

	if (twihs_master_init(BOARD_TWI, &opt) != TWIHS_SUCCESS)
		return -1;
		
	return 0;		
}

int i2cRead(uint8_t chip_addr, uint32_t address, uint8_t address_len, void *data, uint32_t len)
{
	twihs_packet_t packet_rx;
	
	//Configure the data packet
	packet_rx.chip        = chip_addr;
	packet_rx.addr_length = address_len;
	if(address_len == 3)
	{
		packet_rx.addr[0]     = address >> 16;
		packet_rx.addr[1]     = address >> 8;
		packet_rx.addr[2]     = address;
	}
	else if(address_len == 2)
	{
		packet_rx.addr[0]     = address >> 8;
		packet_rx.addr[1]     = address;
	}
	else if(address_len == 1)
	{
		packet_rx.addr[0]     = address;
	}
	else
	{
		return 0;
	}
		
	packet_rx.buffer      = data;
	packet_rx.length      = len;	
	
	//Read Data
	if (twihs_master_read(BOARD_TWI, &packet_rx) != TWIHS_SUCCESS)
		return 0;
		
	return len;
}

int i2cWrite(uint8_t chip_addr, uint32_t address, uint8_t address_len, void *data, uint32_t len)
{
	twihs_packet_t packet_tx;

	//Configure the data packet
	packet_tx.chip        = chip_addr;
	packet_tx.addr_length = address_len;
	if(address_len == 3)
	{
		packet_tx.addr[0]     = address >> 16;
		packet_tx.addr[1]     = address >> 8;
		packet_tx.addr[2]     = address;
	}
	else if(address_len == 2)
	{
		packet_tx.addr[0]     = address >> 8;
		packet_tx.addr[1]     = address;
	}
	else if(address_len == 1)
	{
		packet_tx.addr[0]     = address;
	}
	else
	{
		return 0;
	}
	packet_tx.buffer      = data;
	packet_tx.length      = len;	

	//Write data
	if (twihs_master_write(BOARD_TWI, &packet_tx) != TWIHS_SUCCESS)
		return 0;
		
	return len;
}
