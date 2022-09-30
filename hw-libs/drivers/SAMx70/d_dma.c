#include <asf.h>
#include <string.h>

#ifdef ENABLE_DMA_INTERRUPTS
#include "conf_interrupts.h"
#endif // ENABLE_DMA_INTERRUPTS

#ifdef DEBUG
//#define DMA_DEBUG
//#define DMA_SPI_DEBUG
//#define DMA_CACHE_DEBUG
#endif // DEBUG

#ifdef __INERTIAL_SENSE_EVB_2__
#include "spiTouINS.h"
#endif

#include "d_dma.h"
#include "d_serial.h"

#ifndef TESTBED
#include "d_i2c.h"
#endif

#include "xdmac.h"

static volatile uint8_t s_xfer_done[DMA_CHAN_COUNT] = {0};
	
void XDMAC_Handler(void)
{	
	XDMAC_usartDMA_Handler();
	
#ifndef TESTBED

	XDMAC_i2c_Handler();
	
#ifdef __INERTIAL_SENSE_EVB_2__
	
	#ifdef CONF_BOARD_SPI_UINS
		//Forward for spiTouINS
	XDMAC_spiTouINS_Handler();
	#endif

#else		// uINS-3

	#ifdef ENABLE_SPI_COMM_INTERRUPTS
	uint32_t status;

	// receive status
	status = xdmac_channel_get_interrupt_status(XDMAC, DMA_CH_SPI_COMM_RX);
	if (status & XDMAC_CIS_BIS)
	{
		NVIC_ClearPendingIRQ(XDMAC_IRQn);
		NVIC_DisableIRQ(XDMAC_IRQn);
		_handler();
		xdmac_channel_enable(XDMAC, DMA_CH_SPI_COMM_RX);
	}
	#endif // ENABLE_SPI_COMM_INTERRUPTS

#endif	// __INERTIAL_SENSE_EVB_2__

#endif // #ifndef TESTBED

}

#ifdef ENABLE_DMA_INTERRUPTS
void XDMAC_Handler(void)		// We can't just merge this with the above function because reading status erases the interrupt flags.
{
	uint32_t i;
	uint32_t status;

	for (i = 0; i < DMA_CHAN_COUNT; i++)
	{
		status = xdmac_channel_get_interrupt_status(XDMAC, i);
		if (status & XDMAC_CIS_BIS)
		{
			s_xfer_done[i] = 1;
		}
	}
}
#endif // ENABLE_DMA_INTERRUPTS


#ifdef DMA_CACHE_DEBUG
static inline int verify_32byte_aligned(dma_channel_config_t *cc)
{
	int retval = 0;
	if (cc->xdmac.mbr_cfg & XDMAC_CC_TYPE_PER_TRAN)
	{
		if (cc->xdmac.mbr_cfg & XDMAC_CC_DSYNC_MEM2PER)
		{
			if (!IS_32B_ALIGNED(cc->xdmac.mbr_sa))
				retval |= -1;
			if (cc->next.len && !IS_32B_ALIGNED(cc->next.sa))
				retval |= -1;
		}
		else
		{
			if (!IS_32B_ALIGNED(cc->xdmac.mbr_da))
				retval |= -1;
			if (cc->next.len && !IS_32B_ALIGNED(cc->next.da))
				retval |= -1;
		}
	}
	else
	{
		if (!IS_32B_ALIGNED(cc->xdmac.mbr_da))
			retval |= -1;
		if (!IS_32B_ALIGNED(cc->xdmac.mbr_sa))
			retval |= -1;
		if (cc->next.len && !IS_32B_ALIGNED(cc->next.da))
			retval |= -1;
		if (cc->next.len && !IS_32B_ALIGNED(cc->next.sa))
			retval |= -1;
	}
	return retval;
}
#endif // DMA_CACHE_DEBUG


inline int dma_configure(dma_channel_config_t* cc, lld_view1* lld)
{
	static xdmac_channel_config_t xdmac = {0};
	xdmac = cc->xdmac; // copy

#ifdef DMA_CACHE_DEBUG
	if (verify_32byte_aligned(cc) != 0)
	{
		halt(3);
	}
#endif // DMA_CACHE_DEBUG

	// initialize channel config
	xdmac.mbr_ubc = XDMAC_UBC_UBLEN(xdmac.mbr_ubc);
	xdmac_configure_transfer(XDMAC, cc->chan, &xdmac);

	if (cc->next.len)
	{
		// initialize linked list descriptor
		lld[0].mbr_nda = (uint32_t)(&lld[1]);
		lld[0].mbr_ubc = XDMAC_UBC_NVIEW_NDV1 |
			XDMAC_UBC_NDE_FETCH_EN |
			XDMAC_UBC_NSEN |
			XDMAC_UBC_NDEN |
			cc->xdmac.mbr_ubc;
		lld[0].mbr_sa  = cc->xdmac.mbr_sa;
		lld[0].mbr_da  = cc->xdmac.mbr_da;
		lld[1].mbr_nda = 0;
		lld[1].mbr_ubc = XDMAC_UBC_UBLEN(cc->next.len);
		lld[1].mbr_sa  = cc->next.sa;
		lld[1].mbr_da  = cc->next.da;

		xdmac_channel_set_descriptor_control(XDMAC, cc->chan,
			XDMAC_CNDC_NDVIEW_NDV1 |
			XDMAC_CNDC_NDE_DSCR_FETCH_EN |
			XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED |
			XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED);
		xdmac_channel_set_descriptor_addr(XDMAC, cc->chan, (uint32_t)lld, 0);
	}
	else
	{
		xdmac_channel_set_descriptor_control(XDMAC, cc->chan, 0);
	}

	return 0;
}


void dma_init(void)
{
	// initialize and enable DMA controller
	pmc_enable_periph_clk(ID_XDMAC);

#ifdef ENABLE_DMA_INTERRUPTS

	// enable xdma interrupt
	NVIC_DisableIRQ(XDMAC_IRQn);
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority(XDMAC_IRQn, INT_PRIORITY_DMA);
	NVIC_EnableIRQ(XDMAC_IRQn);

#endif // ENABLE_DMA_INTERRUPTS

}


inline void dma_chan_disable(uint32_t ch)
{
	if (ch >= DMA_CHAN_COUNT) { return; }

#ifndef ENABLE_DMA_INTERRUPTS
	__DSB();
	s_xfer_done[ch] = !DMA_CHAN_BUSY(ch);
#endif // ENABLE_DMA_INTERRUPTS

	DMA_INT_DISABLE(ch);
	xdmac_channel_disable(XDMAC, ch);
}


inline int dma_chan_enable(uint32_t ch)
{
	if (ch >= DMA_CHAN_COUNT) { return false; }
	while (DMA_CHAN_BUSY(ch)) {}
	s_xfer_done[ch] = 0;
	DMA_INT_ENABLE(ch);
	xdmac_channel_enable(XDMAC, ch);

	return 0;
}


int dma_transfer_is_complete(uint32_t ch)
{
	if (ch >= DMA_CHAN_COUNT) { return false; }

	#ifndef ENABLE_DMA_INTERRUPTS
	__DSB();
	s_xfer_done[ch] = !DMA_CHAN_BUSY(ch);
	#endif // ENABLE_DMA_INTERRUPTS

	return s_xfer_done[ch];
}
