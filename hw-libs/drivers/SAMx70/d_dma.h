#ifndef _D_DMA_H_
#define _D_DMA_H_
#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// includes
#include <xdmac.h>
#include "core_cm7_4p30.h"

#if defined(__INERTIAL_SENSE_EVB_2__) || 1

#define MEMCPY_DCACHE_CLEAN(dst, src, size) \
memcpy((void*)(dst), (const void*)(src), (size)); \
SCB_CLEAN_DCACHE_BY_ADDR_32BYTE_ALIGNED((dst), (size)); 

// Memcpy then clear Data Cache to memory for before DMA starts
#define MEMCPY_DCACHE_CLEAN_INVALIDATE(dst, src, size) \
memcpy((void*)(dst), (const void*)(src), (size)); \
SCB_CLEANINVALIDATE_DCACHE_BY_ADDR_32BYTE_ALIGNED((dst), (size)); 

// Ensure data cache has valid Rx data prior to memcpy
#define DCACHE_CLEAN_INVALIDATE_MEMCPY(dst, src, size) \
SCB_CLEANINVALIDATE_DCACHE_BY_ADDR_32BYTE_ALIGNED((src), (size)); \
memcpy((void*)(dst), (const void*)(src), (size));

#else

// Memcpy then clean Data Cache to memory for before DMA starts
#define MEMCPY_DCACHE_CLEAN(dst, src, size) \
memcpy((void*)(dst), (const void*)(src), (size)); \
DBGPIO_START(DBG_TX_DCACHE_CLEAN_PIN); \
SCB_CLEAN_DCACHE_BY_ADDR_32BYTE_ALIGNED((dst), (size)); \
DBGPIO_END(DBG_TX_DCACHE_CLEAN_PIN);

// Memcpy then clear Data Cache to memory for before DMA starts
#define MEMCPY_DCACHE_CLEAN_INVALIDATE(dst, src, size) \
memcpy((void*)(dst), (const void*)(src), (size)); \
DBGPIO_START(DBG_TX_DCACHE_CLEAN_PIN); \
SCB_CLEANINVALIDATE_DCACHE_BY_ADDR_32BYTE_ALIGNED((dst), (size)); \
DBGPIO_END(DBG_TX_DCACHE_CLEAN_PIN);

// Ensure data cache has valid Rx data prior to memcpy
#define DCACHE_CLEAN_INVALIDATE_MEMCPY(dst, src, size) \
DBGPIO_START( DBG_RX_DCACHE_CLEAN_PIN ); \
SCB_CLEANINVALIDATE_DCACHE_BY_ADDR_32BYTE_ALIGNED((src), (size)); \
DBGPIO_END( DBG_RX_DCACHE_CLEAN_PIN ); \
memcpy((void*)(dst), (const void*)(src), (size));

#endif


// defines
//#define ENABLE_DMA_INTERRUPTS		// TODO: This will enable a duplicate XDMAC_Handler in d_dma.c. Might just remove this?
#ifdef ENABLE_DMA_INTERRUPTS
#define DMA_INT_ENABLE(_ch_) \
	xdmac_enable_interrupt(XDMAC, _ch_); \
	xdmac_channel_enable_interrupt(XDMAC, _ch_, XDMAC_CIE_BIE);
#define DMA_INT_DISABLE(_ch_) \
	xdmac_disable_interrupt(XDMAC, _ch_); \
	xdmac_channel_disable_interrupt(XDMAC, _ch_, XDMAC_CIE_BIE);
#else
#define DMA_INT_ENABLE(_ch_)
#define DMA_INT_DISABLE(_ch_)
#endif // ENABLE_DMA_INTERRUPTS

// XDMAC channel HW Interface number for peripherals; refer to datasheet
#define XDMAC_PERID_SPI0_TX    1
#define XDMAC_PERID_SPI0_RX    2
#define XDMAC_PERID_SPI1_TX    3
#define XDMAC_PERID_SPI1_RX    4
#define XDMAC_PERID_USART0_TX  7
#define XDMAC_PERID_USART0_RX  8
#define XDMAC_PERID_USART1_TX  9
#define XDMAC_PERID_USART1_RX  10
#define XDMAC_PERID_USART2_TX  11
#define XDMAC_PERID_USART2_RX  12
#define XDMAC_PERID_TWIHS0_TX  14
#define XDMAC_PERID_TWIHS0_RX  15
#define XDMAC_PERID_TWIHS1_TX  16
#define XDMAC_PERID_TWIHS1_RX  17
#define XDMAC_PERID_TWIHS2_TX  18
#define XDMAC_PERID_TWIHS2_RX  19
#define XDMAC_PERID_UART0_TX   20
#define XDMAC_PERID_UART0_RX   21
#define XDMAC_PERID_UART1_TX   22
#define XDMAC_PERID_UART1_RX   23
#define XDMAC_PERID_UART2_TX   24
#define XDMAC_PERID_UART2_RX   25
#define XDMAC_PERID_UART3_TX   26
#define XDMAC_PERID_UART3_RX   27
#define XDMAC_PERID_UART4_TX   28
#define XDMAC_PERID_UART4_RX   29

// cache specific
#define ALIGN_32B_MASK          0xffffffe0
#define IS_32B_ALIGNED(_addr_)  !(((uint32_t)(_addr_)) & 0x0000001f)
#define ALIGN_32B(_bytes_)      ((_bytes_) + (sizeof(uint32_t) - ((_bytes_) % sizeof(uint32_t))))

// enums
enum
{
	// uINS specific
	DMA_CH_SPI_SENSORS_TX = 0,
	DMA_CH_SPI_SENSORS_RX,
	DMA_CH_SPI_COMM_TX,
	DMA_CH_SPI_COMM_RX,
	DMA_CH_USART_SERIAL0_TX,
	DMA_CH_USART_SERIAL0_RX,
	DMA_CH_USART_SERIAL1_TX,	// Also used for SPI on uINS
	DMA_CH_USART_SERIAL1_RX,	// Also used for SPI on uINS
	DMA_CH_USART_SERIAL2_TX,
	DMA_CH_USART_SERIAL2_RX,
	DMA_CH_USART_GPS1_TX,
	DMA_CH_USART_GPS1_RX,
	DMA_CH_USART_GPS2_TX,
	DMA_CH_USART_GPS2_RX,
	DMA_CH_I2C_SENSORS_TX,
	DMA_CH_I2C_SENSORS_RX,
	// add more channels before this line
	DMA_CHAN_COUNT,
	DMA_CHAN_MAX = 24
};

// enums
enum
{
	// EVB-2 specific
	DMA_CH_EVB_SPI_SENSORS_TX = 0,
	DMA_CH_EVB_SPI_SENSORS_RX,
	DMA_CH_EVB_SPI_COMM_TX,
	DMA_CH_EVB_SPI_COMM_RX,
	DMA_CH_EVB_UART_UINS0_TX,
	DMA_CH_EVB_UART_UINS0_RX,
	DMA_CH_EVB_UART_UINS1_TX,
	DMA_CH_EVB_UART_UINS1_RX,
	DMA_CH_EVB_UART_XBEE_TX,
	DMA_CH_EVB_UART_XBEE_RX,
	DMA_CH_EVB_UART_XRADIO_TX,
	DMA_CH_EVB_UART_XRADIO_RX,
	DMA_CH_EVB_UART_WINC_BLE_TX,
	DMA_CH_EVB_UART_WINC_BLE_RX,
	DMA_CH_EVB_UART_SP330_TX,
	DMA_CH_EVB_UART_SP330_RX,
	DMA_CH_EVB_UART_GPIO_TTL_TX,
	DMA_CH_EVB_UART_GPIO_TTL_RX,
	DMA_CH_EVB_SD_CARD,
	DMA_CH_EVB_SPI_INS_TX,
	DMA_CH_EVB_SPI_INS_RX,
	DMA_CH_EVB_I2C_SENSORS_TX,
	DMA_CH_EVB_I2C_SENSORS_RX,
	// add more channels before this line
	DMA_EVB_CHAN_COUNT,
	DMA_EVB_CHAN_MAX = 24
};

// enums
enum
{
	// testbed specific
	DMA_CH_TESTBED_UINS_TX = 0,
	DMA_CH_TESTBED_UINS_RX,			// Unused
	DMA_CH_TESTBED_SENSONOR_TX,
	DMA_CH_TESTBED_SENSONOR_RX,
	// add more channels before this line
	DMA_TESTBED_CHAN_COUNT,
	DMA_TESTBED_CHAN_MAX = 24
};

// structs
typedef struct
{
	uint32_t len : 24;
	uint32_t sa;
	uint32_t da;
} dma_transfer_info_t;

typedef struct
{
	uint32_t               chan;
	xdmac_channel_config_t xdmac;
	dma_transfer_info_t    next;
} dma_channel_config_t;

// prototypes
void dma_init(void);

int dma_configure(dma_channel_config_t* hd, lld_view1* lld);
void dma_chan_disable(uint32_t ch);
int dma_chan_enable(uint32_t ch);
int dma_transfer_is_complete(uint32_t ch);

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // _D_DMA_H_
