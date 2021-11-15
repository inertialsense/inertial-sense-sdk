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

#ifndef INT_PRIORITY_DMA
#define INT_PRIORITY_DMA	3
#endif

#define INT_PRIORITY_I2C	INT_PRIORITY_DMA

static const uint32_t _xdmaint = XDMAC_CIE_BIE; // End of Block Interrupt Enable

static const uint32_t _i2cint = 
	TWIHS_IER_RXRDY |
	TWIHS_IER_TXRDY;

int i2c_master_get_defaults(i2c_t *init)
{
	init->cfg.master_clk = sysclk_get_peripheral_hz();
	init->cfg.speed = TWI_CLK;
	init->cfg.chip = 0x40;
	init->cfg.smbus = 0;
	init->instance = TWIHS0;
	init->instance_id = ID_TWIHS0;
	
	init->tx_status = I2C_TXSTATUS_IDLE;
	init->rx_status = I2C_RXSTATUS_IDLE;

#ifdef __INERTIAL_SENSE_EVB_2__
	init->rx_dma.chan = DMA_CH_EVB_I2C_SENSORS_RX;
	init->tx_dma.chan = DMA_CH_EVB_I2C_SENSORS_TX;
#else
	init->rx_dma.chan = DMA_CH_I2C_SENSORS_RX;
	init->tx_dma.chan = DMA_CH_I2C_SENSORS_TX;
#endif

	init->rx_dma.xdmac.mbr_sa = (uint32_t)&(init->instance->TWIHS_RHR);
	init->rx_dma.xdmac.mbr_cfg =
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
	init->rx_dma.xdmac.mbr_ubc = 0;
	
	init->tx_dma.xdmac.mbr_da = (uint32_t)&(init->instance->TWIHS_THR);
	init->tx_dma.xdmac.mbr_sa = (uint32_t)init->tx_buf;
	init->tx_dma.xdmac.mbr_cfg =
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
	init->tx_dma.xdmac.mbr_ubc = 0;

	return STATUS_OK;
}


static void i2c_master_dma_init_rx(i2c_t *init)
{
	xdmac_configure_transfer(XDMAC, init->rx_dma.chan, &init->rx_dma.xdmac);

	xdmac_channel_set_descriptor_control(XDMAC, init->rx_dma.chan, 0);

	xdmac_enable_interrupt(XDMAC, init->rx_dma.chan);
	xdmac_channel_enable_interrupt(XDMAC, init->rx_dma.chan, _xdmaint);
	NVIC_DisableIRQ(XDMAC_IRQn);
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority(XDMAC_IRQn, INT_PRIORITY_DMA);
	NVIC_EnableIRQ(XDMAC_IRQn);
}

static void i2c_master_dma_config_rx(i2c_t *init, uint8_t *buf, uint32_t len)
{
	init->rx_dma.xdmac.mbr_da = (uint32_t)buf;
	init->rx_dma.xdmac.mbr_ubc = len;
	xdmac_configure_transfer(XDMAC, init->rx_dma.chan, &init->rx_dma.xdmac);
}

static void i2c_master_dma_start_rx(i2c_t *init)
{
	xdmac_channel_enable(XDMAC, init->rx_dma.chan);
}

static void i2c_master_dma_stop_rx(i2c_t *init)
{
	xdmac_channel_disable(XDMAC, init->rx_dma.chan);
}

static void i2c_master_dma_init_tx(i2c_t *init)
{
	xdmac_configure_transfer(XDMAC, init->tx_dma.chan, &init->tx_dma.xdmac);
	
	xdmac_channel_set_descriptor_control(XDMAC, init->tx_dma.chan, 0);

	xdmac_enable_interrupt(XDMAC, init->tx_dma.chan);
	xdmac_channel_enable_interrupt(XDMAC, init->tx_dma.chan, _xdmaint);
	NVIC_DisableIRQ(XDMAC_IRQn);
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority(XDMAC_IRQn, INT_PRIORITY_DMA);
	NVIC_EnableIRQ(XDMAC_IRQn);
}

static void i2c_master_dma_config_tx(i2c_t *init, uint8_t *buf, uint32_t len)
{
	if(len > I2C_BUF_SIZE_TX)
		len = I2C_BUF_SIZE_TX;
	
	memcpy((uint8_t*)init->tx_buf, buf, len);
	init->tx_dma.xdmac.mbr_ubc = len;
	xdmac_configure_transfer(XDMAC, init->tx_dma.chan, &init->tx_dma.xdmac);
}

static void i2c_master_dma_start_tx(i2c_t *init)
{
	xdmac_channel_enable(XDMAC, init->tx_dma.chan);
}

static void i2c_master_dma_stop_tx(i2c_t *init)
{
	xdmac_channel_disable(XDMAC, init->tx_dma.chan);
}


/*
 *	Make sure to call i2c_get_defaults() first
 */
int i2c_master_init(i2c_t *init)	// TODO: Rewrite this so it can be used for any instance of I2C
{
	// Configure pins (done in common location on uINS-3)
#ifdef __INERTIAL_SENSE_EVB_2__
	ioport_set_pin_peripheral_mode(I2C_0_SDA_PIN, I2C_0_SDA_FLAGS);
	ioport_set_pin_peripheral_mode(I2C_0_SCL_PIN, I2C_0_SCL_FLAGS);
#endif

	sysclk_enable_peripheral_clock(init->instance_id);
	
	init->instance->TWIHS_CR |= TWIHS_CR_SWRST;
	
	if(twihs_master_init(init->instance, &init->cfg) != TWIHS_SUCCESS)
		return -1;
		
//	twihs_enable_interrupt(init->instance, _i2cint);
	NVIC_DisableIRQ(TWIHS0_IRQn);
	NVIC_ClearPendingIRQ(TWIHS0_IRQn);
	NVIC_SetPriority(TWIHS0_IRQn, INT_PRIORITY_I2C);
	NVIC_EnableIRQ(TWIHS0_IRQn);
	
	i2c_master_dma_init_rx(init);
	i2c_master_dma_init_tx(init);
	
	return 0;
}

int i2c_master_write(i2c_t *init, uint16_t addr, uint8_t *buf, uint8_t len)
{
	/* Check argument */
	if (len == 0) {
		return -1;
	}
	
	// Bypass DMA
	if (len == 1) {
		init->instance->TWIHS_THR = buf[0];
		init->instance->TWIHS_CR = TWIHS_CR_STOP;
		init->instance->TWIHS_MMR = TWIHS_MMR_DADR(addr);
		return 0;
	}
	
	i2c_master_dma_config_tx(init, buf, len-1);	// Datasheet says we have to transmit last byte manually
	init->tx_last_byte = buf[len-1];		// Last byte gets stored

	/* Set write mode, slave address and 3 internal address byte lengths */
	init->instance->TWIHS_MMR = TWIHS_MMR_DADR(addr);

	/* Set internal address for remote chip (none, just add the internal address to the buffer if there is one) */
	init->instance->TWIHS_IADR = 0;
	
	init->tx_status = I2C_TXSTATUS_TRANSMIT_DMA;
	
	i2c_master_dma_start_tx(init);
	
//	twihs_enable_interrupt(init->instance, TWIHS_IER_TXRDY);
	
	return 0;
}

int i2c_master_read(i2c_t *init, uint16_t addr, uint8_t *buf, uint8_t len)
{
	/* Check argument */
	if (len == 0) {
		return -1;
	}
	
	init->rx_len = len;
	init->rx_buf = buf;
	
	i2c_master_dma_config_rx(init, buf, len-2);			// Datasheet says we have to read last 2 bytes manually

	/* Set write mode, slave address and 3 internal address byte lengths */
	init->instance->TWIHS_MMR = TWIHS_MMR_DADR(addr) | TWIHS_MMR_MREAD;
	
	/* Set internal address for remote chip (none, just add the internal address to the buffer if there is one) */
	init->instance->TWIHS_IADR = 0;
	
	init->rx_status = I2C_RXSTATUS_READ_DMA;
	
	i2c_master_dma_start_rx(init);
	
	twihs_enable_interrupt(init->instance, TWIHS_IER_RXRDY);
	
	init->instance->TWIHS_CR |= TWIHS_CR_START;
	
	return 0;
}

uint8_t i2c_get_status(i2c_t *init)
{
	uint8_t status = 0;
	
	if(init->rx_status != I2C_RXSTATUS_IDLE)
		status |= I2C_STATUS_RXBUSY;
	if(init->tx_status != I2C_TXSTATUS_IDLE)
		status |= I2C_STATUS_TXBUSY;
		
	return status;
}


void TWIHS0_Handler(void)
{
	uint32_t status = TWIHS0->TWIHS_SR;
	
	if(status & TWIHS_SR_TXRDY)
	{
		twihs_disable_interrupt(TWIHS0, TWIHS_IDR_TXRDY);	// Without this we get spammed by the interrupt
		
		if(i2c0.tx_status == I2C_TXSTATUS_TRANSMIT_DMA_WAIT_TXRDY)
		{
			TWIHS0->TWIHS_CR |= TWIHS_CR_STOP;
			TWIHS0->TWIHS_THR = i2c0.tx_last_byte;

			
			i2c0.tx_status = I2C_TXSTATUS_IDLE;
		}
	}
	
	if(status & TWIHS_SR_RXRDY)
	{
		if(i2c0.rx_status == I2C_RXSTATUS_READ_DMA_WAIT_RXRDY)
		{
			TWIHS0->TWIHS_CR |= TWIHS_CR_STOP;
			
			i2c0.rx_buf[i2c0.rx_len - 2] = TWIHS0->TWIHS_RHR;
			
			i2c0.rx_status = I2C_RXSTATUS_READ_PENULTIMATE;
		}
		else if(i2c0.rx_status == I2C_RXSTATUS_READ_PENULTIMATE)
		{
			i2c0.rx_buf[i2c0.rx_len - 1] = TWIHS0->TWIHS_RHR;
			
			twihs_disable_interrupt(TWIHS0, TWIHS_IDR_RXRDY);
			
			i2c0.rx_status = I2C_RXSTATUS_IDLE;
		}
	}
}

/*
 *	Sub-handler that is called from the XDMAC_Handler() in d_dma.c
 */
void XDMAC_i2c_Handler(void)
{
	uint32_t status = XDMAC->XDMAC_CHID[i2c0.rx_dma.chan].XDMAC_CIS;
	
	if((status & XDMAC_CIS_BIS) && (i2c0.rx_status == I2C_RXSTATUS_READ_DMA))
	{
		i2c_master_dma_stop_rx(&i2c0);
		
		i2c0.rx_status = I2C_RXSTATUS_READ_DMA_WAIT_RXRDY;
		
		if(TWIHS0->TWIHS_SR & TWIHS_SR_RXRDY)
		{
			TWIHS0->TWIHS_CR |= TWIHS_CR_STOP;
			
			i2c0.rx_buf[i2c0.rx_len - 2] = TWIHS0->TWIHS_RHR;
			
			i2c0.rx_status = I2C_RXSTATUS_READ_PENULTIMATE;
		}
	}
	
	status = XDMAC->XDMAC_CHID[i2c0.tx_dma.chan].XDMAC_CIS;
	
	if((status & XDMAC_CIS_BIS) && (i2c0.tx_status == I2C_TXSTATUS_TRANSMIT_DMA))
	{
		i2c_master_dma_stop_tx(&i2c0);
		
		twihs_enable_interrupt(TWIHS0, TWIHS_IDR_TXRDY);
		i2c0.tx_status = I2C_TXSTATUS_TRANSMIT_DMA_WAIT_TXRDY;
	}
}
