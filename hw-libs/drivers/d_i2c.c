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

i2c_t i2c0 = {0};

//Configuration for TWI instance
#define TWI_CLK			400000
#define I2C_BUF_SIZE_RX	256
#define I2C_BUF_SIZE_TX	256	

#ifndef INT_PRIORITY_DMA
#define INT_PRIORITY_DMA  3
#endif

COMPILER_ALIGNED(32) static volatile uint8_t _rxbuf[I2C_BUF_SIZE_RX];
COMPILER_ALIGNED(32) static volatile uint8_t _txbuf[I2C_BUF_SIZE_TX];

static const uint32_t _xdmaint =
	XDMAC_CIE_BIE | // End of Block Interrupt Enable
	XDMAC_CIE_LIE | // End of Linked List Interrupt Enable
	XDMAC_CIE_DIE | // End of Disable Interrupt Enable
	XDMAC_CIE_FIE | // End of Flush Interrupt Enable
	XDMAC_CIE_RBIE | // Read Bus Error Interrupt Enable
	XDMAC_CIE_WBIE | // Write Bus Error Interrupt Enable
	XDMAC_CIE_ROIE; // Request Overflow Error Interrupt Enable

int i2c_get_defaults(i2c_t *init)
{
	init->cfg.master_clk = sysclk_get_peripheral_hz();
	init->cfg.speed = TWI_CLK;
	init->cfg.chip = 0x40;
	init->cfg.smbus = 0;
	init->instance = TWIHS0;
	init->instance_id = ID_TWIHS0;

	// If/when we add I2C to the uINS-3, this needs to be wrapped in a macro: 
	init->rxdma.chan = DMA_CH_EVB_I2C_SENSORS_RX;
	init->txdma.chan = DMA_CH_EVB_I2C_SENSORS_TX;

	init->rxdma.xdmac.mbr_sa = (uint32_t)&(init->instance->TWIHS_RHR);
	init->rxdma.xdmac.mbr_da = (uint32_t)_rxbuf;
	init->rxdma.xdmac.mbr_cfg =
		XDMAC_CC_TYPE_PER_TRAN |
		XDMAC_CC_MBSIZE_SINGLE |
		XDMAC_CC_DSYNC_PER2MEM |
		XDMAC_CC_MEMSET_NORMAL_MODE |
		XDMAC_CC_CSIZE_CHK_1 |
		XDMAC_CC_DWIDTH_BYTE |
		XDMAC_CC_SIF_AHB_IF1 |
		XDMAC_CC_DIF_AHB_IF0 |
		XDMAC_CC_SAM_FIXED_AM |
		XDMAC_CC_DAM_INCREMENTED_AM |
		XDMAC_CC_PERID(XDMAC_PERID_TWIHS0_RX);
	init->rxdma.xdmac.mbr_ubc = I2C_BUF_SIZE_RX;
	
	init->txdma.xdmac.mbr_sa = (uint32_t)_txbuf;
	init->txdma.xdmac.mbr_da = (uint32_t)&(init->instance->TWIHS_THR);
	init->txdma.xdmac.mbr_cfg =
		XDMAC_CC_TYPE_PER_TRAN |
		XDMAC_CC_MBSIZE_SINGLE |
		XDMAC_CC_DSYNC_MEM2PER |
		XDMAC_CC_MEMSET_NORMAL_MODE |
		XDMAC_CC_CSIZE_CHK_1 |
		XDMAC_CC_DWIDTH_BYTE |
		XDMAC_CC_SIF_AHB_IF0 |
		XDMAC_CC_DIF_AHB_IF1 |
		XDMAC_CC_SAM_INCREMENTED_AM |
		XDMAC_CC_DAM_FIXED_AM |
		XDMAC_CC_PERID(XDMAC_PERID_TWIHS0_TX);
	init->txdma.xdmac.mbr_ubc = 0;

	return STATUS_OK;
}


static void i2c_dma_init_rx(i2c_t *init)
{
	xdmac_configure_transfer(XDMAC, init->txdma.chan, &init->rxdma.xdmac);

	xdmac_channel_set_descriptor_control(XDMAC, init->rxdma.chan, 0);

	xdmac_enable_interrupt(XDMAC, init->txdma.chan);
	xdmac_channel_enable_interrupt(XDMAC, init->txdma.chan, _xdmaint);
	NVIC_DisableIRQ(XDMAC_IRQn);
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority(XDMAC_IRQn, INT_PRIORITY_DMA);
	NVIC_EnableIRQ(XDMAC_IRQn);

	xdmac_channel_enable(XDMAC, init->txdma.chan);
}


static void i2c_dma_init_tx(i2c_t *init)
{
	xdmac_configure_transfer(XDMAC, init->txdma.chan, &init->txdma.xdmac);
	
	xdmac_channel_set_descriptor_control(XDMAC, init->txdma.chan, 0);

	xdmac_enable_interrupt(XDMAC, init->txdma.chan);
	xdmac_channel_enable_interrupt(XDMAC, init->txdma.chan, _xdmaint);
	NVIC_DisableIRQ(XDMAC_IRQn);
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority(XDMAC_IRQn, INT_PRIORITY_DMA);
	NVIC_EnableIRQ(XDMAC_IRQn);
}

static void i2c_dma_config_tx(i2c_t *init, uint32_t len)
{
	init->txdma.xdmac.mbr_ubc = len;
	xdmac_configure_transfer(XDMAC, init->txdma.chan, &init->txdma.xdmac);
}

static void i2c_dma_start_tx(i2c_t *init)
{
	xdmac_channel_enable(XDMAC, init->txdma.chan);
}

int i2c_init(i2c_t *init)
{
	//Configure pins
	ioport_set_pin_peripheral_mode(I2C_0_SDA_PIN, I2C_0_SDA_FLAGS);
	ioport_set_pin_peripheral_mode(I2C_0_SCL_PIN, I2C_0_SCL_FLAGS);
	
	sysclk_enable_peripheral_clock(init->instance_id);
	
	if(twihs_master_init(init->instance, &init->cfg) != TWIHS_SUCCESS)
		return -1;
	
	i2c_dma_init_rx(init);
	i2c_dma_init_tx(init);
	
	return 0;
}

int i2c_write(i2c_t *init, uint8_t addr, uint8_t *buf, uint8_t len)
{
	/* Check argument */
	if (len == 0) {
		return -1;
	}
	
	i2c_dma_config_tx(init, len-1);			// Datasheet says we have to transmit last byte manually
	init->last_byte[0] = buf[len-1];		// Last byte gets stored so we can transmit it manually

	/* Set write mode, slave address and 3 internal address byte lengths */
	init->instance->TWIHS_MMR = TWIHS_MMR_DADR(addr);

	/* Set internal address for remote chip (none, just add the internal address to the buffer if there is one) */
	init->instance->TWIHS_IADR = 0;
	
	i2c_dma_start_tx(init);

	return 0;
}


void TWIHS0_Handler(void)
{
	uint32_t status = TWIHS0->TWIHS_SR;
	
	if(status & TWIHS_SR_TXRDY)
	{
		TWIHS0->TWIHS_CR |= TWIHS_CR_STOP;
		TWIHS0->TWIHS_THR = i2c0.last_byte[0];
	}
}




//int i2cRead(uint8_t chip_addr, uint32_t address, uint8_t address_len, void *data, uint32_t len)
//{
	//twihs_packet_t packet_rx;
	//
	////Configure the data packet
	//packet_rx.chip        = chip_addr;
	//packet_rx.addr_length = address_len;
	//if(address_len == 3)
	//{
		//packet_rx.addr[0]     = address >> 16;
		//packet_rx.addr[1]     = address >> 8;
		//packet_rx.addr[2]     = address;
	//}
	//else if(address_len == 2)
	//{
		//packet_rx.addr[0]     = address >> 8;
		//packet_rx.addr[1]     = address;
	//}
	//else if(address_len == 1)
	//{
		//packet_rx.addr[0]     = address;
	//}
	//else
	//{
		//return 0;
	//}
		//
	//packet_rx.buffer      = data;
	//packet_rx.length      = len;	
	//
	////Read Data
	//if (twihs_master_read(BOARD_TWI, &packet_rx) != TWIHS_SUCCESS)
		//return 0;
		//
	//return len;
//}
