/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <string.h>
#include <chrono>
#include <ctime>

// STEP 1: Add Includes
// Change these include paths to the correct paths for your project
#include "../../src/InertialSense.h"

#define SCRATCH_SIZE    2048

typedef struct
{
    uint32_t _CR1;         /*!< USART Control register 1,                 Address offset: 0x00  */
    uint32_t _CR2;         /*!< USART Control register 2,                 Address offset: 0x04  */
    uint32_t _CR3;         /*!< USART Control register 3,                 Address offset: 0x08  */
    uint32_t BRR;         /*!< USART Baud rate register,                 Address offset: 0x0C  */
    uint32_t GTPR;        /*!< USART Guard time and prescaler register,  Address offset: 0x10  */
    uint32_t RTOR;        /*!< USART Receiver Time Out register,         Address offset: 0x14  */
    uint32_t RQR;         /*!< USART Request register,                   Address offset: 0x18  */
    uint32_t ISR;         /*!< USART Interrupt and status register,      Address offset: 0x1C  */
    uint32_t ICR;         /*!< USART Interrupt flag Clear register,      Address offset: 0x20  */
    uint32_t RDR;         /*!< USART Receive Data register,              Address offset: 0x24  */
    uint32_t TDR;         /*!< USART Transmit Data register,             Address offset: 0x28  */
    uint32_t PRESC;       /*!< USART Prescaler register,                 Address offset: 0x2C  */
    uint32_t AUTOCR;      /*!< USART Autonomous mode control register    Address offset: 0x30  */
} USART_TypeDef;

enum class IRQn_Type : int8_t
{
    /* =======================================  ARM Cortex-M33 Specific Interrupt Numbers  ======================================= */
    Reset_IRQn = -15,    /*!< -15 Reset Vector, invoked on Power up and warm reset              */
    NonMaskableInt_IRQn = -14,    /*!< -14 Non maskable Interrupt, cannot be stopped or preempted        */
    HardFault_IRQn = -13,    /*!< -13 Hard Fault, all classes of Fault                              */
    MemoryManagement_IRQn = -12,    /*!< -12 Memory Management, MPU mismatch, including Access Violation
                                                 and No Match                                                  */
    BusFault_IRQn = -11,    /*!< -11 Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory
                                                 related Fault                                                 */
    UsageFault_IRQn = -10,    /*!< -10 Usage Fault, i.e. Undef Instruction, Illegal State Transition */
    SecureFault_IRQn = -9,    /*!< -9  Secure Fault                                                  */
    SVCall_IRQn = -5,    /*!< -5  System Service Call via SVC instruction                       */
    DebugMonitor_IRQn = -4,    /*!< -4  Debug Monitor                                                 */
    PendSV_IRQn = -2,    /*!< -2  Pendable request for system service                           */
    SysTick_IRQn = -1,    /*!< -1  System Tick Timer                                             */

    /* ===========================================  STM32U575xx Specific Interrupt Numbers  ================================= */
    WWDG_IRQn = 0,      /*!< Window WatchDog interrupt                                         */
    PVD_PVM_IRQn = 1,      /*!< PVD/PVM through EXTI Line detection Interrupt                     */
    RTC_IRQn = 2,      /*!< RTC non-secure interrupt                                          */
    RTC_S_IRQn = 3,      /*!< RTC secure interrupt                                              */
    TAMP_IRQn = 4,      /*!< Tamper non-secure interrupt                                       */
    RAMCFG_IRQn = 5,      /*!< RAMCFG global interrupt                                           */
    FLASH_IRQn = 6,      /*!< FLASH non-secure global interrupt                                 */
    FLASH_S_IRQn = 7,      /*!< FLASH secure global interrupt                                     */
    GTZC_IRQn = 8,      /*!< Global TrustZone Controller interrupt                             */
    RCC_IRQn = 9,      /*!< RCC non secure global interrupt                                   */
    RCC_S_IRQn = 10,     /*!< RCC secure global interrupt                                       */
    EXTI0_IRQn = 11,     /*!< EXTI Line0 interrupt                                              */
    EXTI1_IRQn = 12,     /*!< EXTI Line1 interrupt                                              */
    EXTI2_IRQn = 13,     /*!< EXTI Line2 interrupt                                              */
    EXTI3_IRQn = 14,     /*!< EXTI Line3 interrupt                                              */
    EXTI4_IRQn = 15,     /*!< EXTI Line4 interrupt                                              */
    EXTI5_IRQn = 16,     /*!< EXTI Line5 interrupt                                              */
    EXTI6_IRQn = 17,     /*!< EXTI Line6 interrupt                                              */
    EXTI7_IRQn = 18,     /*!< EXTI Line7 interrupt                                              */
    EXTI8_IRQn = 19,     /*!< EXTI Line8 interrupt                                              */
    EXTI9_IRQn = 20,     /*!< EXTI Line9 interrupt                                              */
    EXTI10_IRQn = 21,     /*!< EXTI Line10 interrupt                                             */
    EXTI11_IRQn = 22,     /*!< EXTI Line11 interrupt                                             */
    EXTI12_IRQn = 23,     /*!< EXTI Line12 interrupt                                             */
    EXTI13_IRQn = 24,     /*!< EXTI Line13 interrupt                                             */
    EXTI14_IRQn = 25,     /*!< EXTI Line14 interrupt                                             */
    EXTI15_IRQn = 26,     /*!< EXTI Line15 interrupt                                             */
    IWDG_IRQn = 27,     /*!< IWDG global interrupt                                             */
    GPDMA1_Channel0_IRQn = 29,     /*!< GPDMA1 Channel 0 global interrupt                                 */
    GPDMA1_Channel1_IRQn = 30,     /*!< GPDMA1 Channel 1 global interrupt                                 */
    GPDMA1_Channel2_IRQn = 31,     /*!< GPDMA1 Channel 2 global interrupt                                 */
    GPDMA1_Channel3_IRQn = 32,     /*!< GPDMA1 Channel 3 global interrupt                                 */
    GPDMA1_Channel4_IRQn = 33,     /*!< GPDMA1 Channel 4 global interrupt                                 */
    GPDMA1_Channel5_IRQn = 34,     /*!< GPDMA1 Channel 5 global interrupt                                 */
    GPDMA1_Channel6_IRQn = 35,     /*!< GPDMA1 Channel 6 global interrupt                                 */
    GPDMA1_Channel7_IRQn = 36,     /*!< GPDMA1 Channel 7 global interrupt                                 */
    ADC1_IRQn = 37,     /*!< ADC1 global interrupt                                             */
    DAC1_IRQn = 38,     /*!< DAC1 global interrupt                                             */
    FDCAN1_IT0_IRQn = 39,     /*!< FDCAN1 interrupt 0                                                */
    FDCAN1_IT1_IRQn = 40,     /*!< FDCAN1 interrupt 1                                                */
    TIM1_BRK_IRQn = 41,     /*!< TIM1 Break interrupt                                              */
    TIM1_UP_IRQn = 42,     /*!< TIM1 Update interrupt                                             */
    TIM1_TRG_COM_IRQn = 43,     /*!< TIM1 Trigger and Commutation interrupt                            */
    TIM1_CC_IRQn = 44,     /*!< TIM1 Capture Compare interrupt                                    */
    TIM2_IRQn = 45,     /*!< TIM2 global interrupt                                             */
    TIM3_IRQn = 46,     /*!< TIM3 global interrupt                                             */
    TIM4_IRQn = 47,     /*!< TIM4 global interrupt                                             */
    TIM5_IRQn = 48,     /*!< TIM5 global interrupt                                             */
    TIM6_IRQn = 49,     /*!< TIM6 global interrupt                                             */
    TIM7_IRQn = 50,     /*!< TIM7 global interrupt                                             */
    TIM8_BRK_IRQn = 51,     /*!< TIM8 Break interrupt                                              */
    TIM8_UP_IRQn = 52,     /*!< TIM8 Update interrupt                                             */
    TIM8_TRG_COM_IRQn = 53,     /*!< TIM8 Trigger and Commutation interrupt                            */
    TIM8_CC_IRQn = 54,     /*!< TIM8 Capture Compare interrupt                                    */
    I2C1_EV_IRQn = 55,     /*!< I2C1 Event interrupt                                              */
    I2C1_ER_IRQn = 56,     /*!< I2C1 Error interrupt                                              */
    I2C2_EV_IRQn = 57,     /*!< I2C2 Event interrupt                                              */
    I2C2_ER_IRQn = 58,     /*!< I2C2 Error interrupt                                              */
    SPI1_IRQn = 59,     /*!< SPI1 global interrupt                                             */
    SPI2_IRQn = 60,     /*!< SPI2 global interrupt                                             */
    USART1_IRQn = 61,     /*!< USART1 global interrupt                                           */
    USART2_IRQn = 62,     /*!< USART2 global interrupt                                           */
    USART3_IRQn = 63,     /*!< USART3 global interrupt                                           */
    UART4_IRQn = 64,     /*!< UART4 global interrupt                                            */
    UART5_IRQn = 65,     /*!< UART5 global interrupt                                            */
    LPUART1_IRQn = 66,     /*!< LPUART1 global interrupt                                          */
    LPTIM1_IRQn = 67,     /*!< LPTIM1 global interrupt                                           */
    LPTIM2_IRQn = 68,     /*!< LPTIM2 global interrupt                                           */
    TIM15_IRQn = 69,     /*!< TIM15 global interrupt                                            */
    TIM16_IRQn = 70,     /*!< TIM16 global interrupt                                            */
    TIM17_IRQn = 71,     /*!< TIM17 global interrupt                                            */
    COMP_IRQn = 72,     /*!< COMP1 and COMP2 through EXTI Lines interrupts                     */
    OTG_FS_IRQn = 73,     /*!< USB OTG FS global interrupt                                       */
    CRS_IRQn = 74,     /*!< CRS global interrupt                                              */
    FMC_IRQn = 75,     /*!< FSMC global interrupt                                             */
    OCTOSPI1_IRQn = 76,     /*!< OctoSPI1 global interrupt                                         */
    PWR_S3WU_IRQn = 77,     /*!< PWR wake up from Stop3 interrupt                                  */
    SDMMC1_IRQn = 78,     /*!< SDMMC1 global interrupt                                           */
    SDMMC2_IRQn = 79,     /*!< SDMMC2 global interrupt                                           */
    GPDMA1_Channel8_IRQn = 80,     /*!< GPDMA1 Channel 8 global interrupt                                 */
    GPDMA1_Channel9_IRQn = 81,     /*!< GPDMA1 Channel 9 global interrupt                                 */
    GPDMA1_Channel10_IRQn = 82,     /*!< GPDMA1 Channel 10 global interrupt                                */
    GPDMA1_Channel11_IRQn = 83,     /*!< GPDMA1 Channel 11 global interrupt                                */
    GPDMA1_Channel12_IRQn = 84,     /*!< GPDMA1 Channel 12 global interrupt                                */
    GPDMA1_Channel13_IRQn = 85,     /*!< GPDMA1 Channel 13 global interrupt                                */
    GPDMA1_Channel14_IRQn = 86,     /*!< GPDMA1 Channel 14 global interrupt                                */
    GPDMA1_Channel15_IRQn = 87,     /*!< GPDMA1 Channel 15 global interrupt                                */
    I2C3_EV_IRQn = 88,     /*!< I2C3 event interrupt                                              */
    I2C3_ER_IRQn = 89,     /*!< I2C3 error interrupt                                              */
    SAI1_IRQn = 90,     /*!< Serial Audio Interface 1 global interrupt                         */
    SAI2_IRQn = 91,     /*!< Serial Audio Interface 2 global interrupt                         */
    TSC_IRQn = 92,     /*!< Touch Sense Controller global interrupt                           */
    RNG_IRQn = 94,     /*!< RNG global interrupt                                              */
    FPU_IRQn = 95,     /*!< FPU global interrupt                                              */
    HASH_IRQn = 96,     /*!< HASH global interrupt                                             */
    LPTIM3_IRQn = 98,     /*!< LPTIM3 global interrupt                                           */
    SPI3_IRQn = 99,     /*!< SPI3 global interrupt                                             */
    I2C4_ER_IRQn = 100,    /*!< I2C4 Error interrupt                                              */
    I2C4_EV_IRQn = 101,    /*!< I2C4 Event interrupt                                              */
    MDF1_FLT0_IRQn = 102,    /*!< MDF1 Filter 0 global interrupt                                    */
    MDF1_FLT1_IRQn = 103,    /*!< MDF1 Filter 1 global interrupt                                    */
    MDF1_FLT2_IRQn = 104,    /*!< MDF1 Filter 2 global interrupt                                    */
    MDF1_FLT3_IRQn = 105,    /*!< MDF1 Filter 3 global interrupt                                    */
    UCPD1_IRQn = 106,    /*!< UCPD1 global interrupt                                            */
    ICACHE_IRQn = 107,    /*!< Instruction cache global interrupt                                */
    LPTIM4_IRQn = 110,    /*!< LPTIM4 global interrupt                                           */
    DCACHE1_IRQn = 111,    /*!< Data cache global interrupt                                       */
    ADF1_IRQn = 112,    /*!< ADF interrupt                                                     */
    ADC4_IRQn = 113,    /*!< ADC4 (12bits) global interrupt                                    */
    LPDMA1_Channel0_IRQn = 114,    /*!< LPDMA1 SmartRun Channel 0 global interrupt                        */
    LPDMA1_Channel1_IRQn = 115,    /*!< LPDMA1 SmartRun Channel 1 global interrupt                        */
    LPDMA1_Channel2_IRQn = 116,    /*!< LPDMA1 SmartRun Channel 2 global interrupt                        */
    LPDMA1_Channel3_IRQn = 117,    /*!< LPDMA1 SmartRun Channel 3 global interrupt                        */
    DMA2D_IRQn = 118,    /*!< DMA2D global interrupt                                            */
    DCMI_PSSI_IRQn = 119,    /*!< DCMI/PSSI global interrupt                                        */
    OCTOSPI2_IRQn = 120,    /*!< OCTOSPI2 global interrupt                                         */
    MDF1_FLT4_IRQn = 121,    /*!< MDF1 Filter 4 global interrupt                                    */
    MDF1_FLT5_IRQn = 122,    /*!< MDF1 Filter 5 global interrupt                                    */
    CORDIC_IRQn = 123,    /*!< CORDIC global interrupt                                           */
    FMAC_IRQn = 124,    /*!< FMAC global interrupt                                             */
};

typedef struct
{
    serial_options_t    coding;
    IRQn_Type 		    interrupt;
} usart_cfg_t;

typedef struct
{
    uint32_t CLBAR;        /*!< DMA channel x linked-list base address register, Address offset: 0x50 + (x * 0x80) */
    uint32_t RESERVED1[2]; /*!< Reserved 1,                                      Address offset: 0x54 -- 0x58      */
    uint32_t CFCR;         /*!< DMA channel x flag clear register,               Address offset: 0x5C + (x * 0x80) */
    uint32_t CSR;          /*!< DMA channel x flag status register,              Address offset: 0x60 + (x * 0x80) */
    uint32_t CCR;          /*!< DMA channel x control register,                  Address offset: 0x64 + (x * 0x80) */
    uint32_t RESERVED2[10];/*!< Reserved 2,                                      Address offset: 0x68 -- 0x8C      */
    uint32_t CTR1;         /*!< DMA channel x transfer register 1,               Address offset: 0x90 + (x * 0x80) */
    uint32_t CTR2;         /*!< DMA channel x transfer register 2,               Address offset: 0x94 + (x * 0x80) */
    uint32_t CBR1;         /*!< DMA channel x block register 1,                  Address offset: 0x98 + (x * 0x80) */
    uint32_t CSAR;         /*!< DMA channel x source address register,           Address offset: 0x9C + (x * 0x80) */
    uint32_t CDAR;         /*!< DMA channel x destination address register,      Address offset: 0xA0 + (x * 0x80) */
    uint32_t CTR3;         /*!< DMA channel x transfer register 3,               Address offset: 0xA4 + (x * 0x80) */
    uint32_t CBR2;         /*!< DMA channel x block register 2,                  Address offset: 0xA8 + (x * 0x80) */
    uint32_t RESERVED3[8]; /*!< Reserved 3,                                      Address offset: 0xAC -- 0xC8      */
    uint32_t CLLR;         /*!< DMA channel x linked-list address register,      Address offset: 0xCC + (x * 0x80) */
} DMA_Channel_TypeDef;

typedef void(*dma_callback_function)(void*);

typedef struct
{
    uint32_t                parent;				// Pointer to parent peripheral management struct
    uint8_t 				mode;				// DMA_MODE_...
    uint32_t 	            tc_handler;			// If non-null, this function pointer is called from within transfer complete interrupt handler when all data including queued data has been transferred.
    uint8_t				    interrupt;
    uint8_t 				interrupt_priority;
    uint8_t 				priority;			// DMA priority (DMA_PRIO_...)
    uint8_t					request_num;		// DMA request ID, links channel to peripheral (DMA_IDX_... for index)
    uint32_t				periph_reg;			// target 8-bit peripheral register (DMA_IDX_... for index)
    uint32_t                buf;
    uint16_t				buf_len;			// Actual usable buffer length is one less (buf_len - 1)
} dma_config_t_GPX;

typedef struct
{
	bool					dir;							// DMA_RX or DMA_TX
	bool					circular;						// DMA_CIRC_ON or DMA_CIRC_OFF
	uint8_t 				priority;						// DMA_PRIO_LOW, DMA_PRIO_MEDIUM, DMA_PRIO_HIGH, DMA_PRIO_VERY_HIGH
	uint8_t 				interrupt;
	uint8_t 				interrupt_priority;				// 0 to 15, 15 is low
	uint8_t 				dma_channel_select;				// 0 to 7. See RM0394 11.6.7
	uint8_t 				parent_type;					// DMA_PARENT_USART, ...
	uint32_t 				parent;						    // Pointer to parent init base
	uint32_t				periph_reg;					    // Pointer to peripheral register
	uint32_t				buf;
	uint16_t				buf_len;						// This doesn't correspond to the length register, it is just however big the buffer is
	bool 					linear_buf;			 			// If true, the buffer is user-specified and we treat it like a non-circular buffer.
	uint32_t 	            tcie_handler;					// If non-null on init, transfer complete irq will be enabled, and this fn called by the IRQ
} dma_config_t_IMX;

typedef struct
{
    uint32_t 		br1;   // transfer size
    uint32_t 		sar;   // source address
    uint32_t 		llr;   // next linked list address

} dma_tx_lli_t;

typedef struct
{
    uint32_t 		cdar;
} dma_rx_circ_lli_t;

#define DMA_TX_LLI_COUNT	3
typedef union
{
    dma_tx_lli_t			tx[DMA_TX_LLI_COUNT];
    dma_rx_circ_lli_t		rx;
} dma_lli_u;

typedef struct
{
    volatile uint16_t 		active_tx_len;
    uint32_t                lli_head;		// Linked list output to dma
    uint32_t                lli_tail;		// Linked list input from dma_buffer_write() 
    volatile bool			dma_running;
} dma_tx_state_t;

typedef struct dma_channel_
{
    uint32_t                instance;
    volatile uint32_t       ptr_start;
    volatile uint32_t       ptr_end;
    dma_config_t_GPX		cfg;
    dma_lli_u				lli;				// Linked list memory.  DMA_MODE_TX_LLI uses 6, DMA_MODE_RX_CIRC uses 1.
    dma_tx_state_t			txState;
    int 					lastDmaUsed;
    uint8_t					overflow;			// Buffer overflow
} dma_ch_t_GPX;

typedef struct
{
    uint32_t MODER;       /*!< GPIO port mode register,               Address offset: 0x00      */
    uint32_t OTYPER;      /*!< GPIO port output type register,        Address offset: 0x04      */
    uint32_t OSPEEDR;     /*!< GPIO port output speed register,       Address offset: 0x08      */
    uint32_t PUPDR;       /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    uint32_t IDR;         /*!< GPIO port input data register,         Address offset: 0x10      */
    uint32_t ODR;         /*!< GPIO port output data register,        Address offset: 0x14      */
    uint32_t BSRR;        /*!< GPIO port bit set/reset  register,     Address offset: 0x18      */
    uint32_t LCKR;        /*!< GPIO port configuration lock register, Address offset: 0x1C      */
    uint32_t AFR[2];      /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
    uint32_t BRR;         /*!< GPIO Bit Reset register,               Address offset: 0x28      */
    uint32_t HSLVR;       /*!< GPIO high-speed low voltage register,  Address offset: 0x2C      */
    uint32_t SECCFGR;     /*!< GPIO secure configuration register,    Address offset: 0x30      */
} GPIO_TypeDef;

typedef struct
{
    uint32_t MODER;       /*!< GPIO port mode register,               Address offset: 0x00      */
    uint32_t OTYPER;      /*!< GPIO port output type register,        Address offset: 0x04      */
    uint32_t OSPEEDR;     /*!< GPIO port output speed register,       Address offset: 0x08      */
    uint32_t PUPDR;       /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    uint32_t IDR;         /*!< GPIO port input data register,         Address offset: 0x10      */
    uint32_t ODR;         /*!< GPIO port output data register,        Address offset: 0x14      */
    uint32_t BSRR;        /*!< GPIO port bit set/reset  register,     Address offset: 0x18      */
    uint32_t LCKR;        /*!< GPIO port configuration lock register, Address offset: 0x1C      */
    uint32_t AFR[2];      /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
    uint32_t BRR;         /*!< GPIO Bit Reset register,               Address offset: 0x28      */

} GPIO_TypeDef_IMX;

typedef struct
{
    uint32_t CCR;         /*!< DMA channel x configuration register        */
    uint32_t CNDTR;       /*!< DMA channel x number of data register       */
    uint32_t CPAR;        /*!< DMA channel x peripheral address register   */
    uint32_t CMAR;        /*!< DMA channel x memory address register       */
} DMA_Channel_TypeDef_IMX;

typedef struct
{
    uint32_t                instance;
    uint32_t                ptr_start;
    uint32_t                ptr_end;
    uint16_t 		        active_tx_len;
    bool 			        done;							// Currently only used in TX
    dma_config_t_IMX		cfg;
    int 					lastDmaUsed;					// Number of bytes in the buffer minus bytes last read.  This is used to identify buffer overflow.
    uint8_t					overflow;						// Buffer overflow
} dma_ch_t_IMX;

typedef struct
{
    uint32_t _CR1;         /*!< USART Control register 1,                 Address offset: 0x00 */
    uint32_t _CR2;         /*!< USART Control register 2,                 Address offset: 0x04 */
    uint32_t _CR3;         /*!< USART Control register 3,                 Address offset: 0x08 */
    uint32_t BRR;         /*!< USART Baud rate register,                 Address offset: 0x0C */
    uint16_t GTPR;        /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
    uint16_t  RESERVED2;       /*!< Reserved, 0x12                                                 */
    uint32_t RTOR;        /*!< USART Receiver Time Out register,         Address offset: 0x14 */
    uint16_t RQR;         /*!< USART Request register,                   Address offset: 0x18 */
    uint16_t  RESERVED3;       /*!< Reserved, 0x1A                                                 */
    uint32_t ISR;         /*!< USART Interrupt and status register,      Address offset: 0x1C */
    uint32_t ICR;         /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
    uint16_t RDR;         /*!< USART Receive Data register,              Address offset: 0x24 */
    uint16_t  RESERVED4;       /*!< Reserved, 0x26                                                 */
    uint16_t TDR;         /*!< USART Transmit Data register,             Address offset: 0x28 */
    uint16_t  RESERVED5;       /*!< Reserved, 0x2A                                                 */
} USART_TypeDef_IMX;

PUSH_PACK_1

typedef struct{
    uint32_t                inst_CCR;         /*!< DMA channel x configuration register        */
    uint32_t                inst_CNDTR;       /*!< DMA channel x number of data register       */
    uint32_t                inst_CPAR;        /*!< DMA channel x peripheral address register   */
    uint32_t                inst_CMAR;        /*!< DMA channel x memory address register       */

	uint32_t 		        ptr_start;
	uint32_t 		        ptr_end;
	uint16_t 		        active_tx_len;
	uint8_t 			    done;							// Currently only used in TX

	uint8_t					cfg_dir;						// DMA_RX or DMA_TX
	uint8_t					cfg_circular;					// DMA_CIRC_ON or DMA_CIRC_OFF
	uint8_t 				cfg_priority;					// DMA_PRIO_LOW, DMA_PRIO_MEDIUM, DMA_PRIO_HIGH, DMA_PRIO_VERY_HIGH
	uint8_t				    cfg_interrupt;
	uint8_t 				cfg_interrupt_priority;			// 0 to 15, 15 is low
	uint8_t 				cfg_dma_channel_select;			// 0 to 7. See RM0394 11.6.7
	uint8_t 				cfg_parent_type;				// DMA_PARENT_USART, ...
	uint32_t				cfg_parent;					    // Pointer to parent init base
	uint32_t				cfg_periph_reg;				    // Pointer to peripheral register
	uint32_t				cfg_buf;
	uint16_t				cfg_buf_len;					// This doesn't correspond to the length register, it is just however big the buffer is
	uint8_t 				cfg_linear_buf;			 		// If true, the buffer is user-specified and we treat it like a non-circular buffer.
	uint32_t                cfg_tcie_handler;	            // If n
	
    int 					lastDmaUsed;					// Number of bytes in the buffer minus bytes last read.  This is used to identify buffer overflow.
	uint8_t					overflow;						// Buffer overflow
} eventImxDmaTxInst_local_t;

POP_PACK

std::string s_fileName;
std::string s_deviceFolder;
uint8_t s_evScratch[SCRATCH_SIZE + DID_EVENT_HEADER_SIZE] = {0};

int printCurString(char* c)
{
    // Create a string from the buffer
    std::string result(c);
    std::cout << result;
    return result.length();
}

void event_outputEvToFile(uint8_t* data, int len)
{
    std::ofstream outfile;

    outfile.open(s_fileName, std::ios_base::app | std::ios_base::binary); // append instead of overwrite
    outfile.write((const char*)data, len);
    outfile.close();
}

void sprintfLab(const char* label)
{
    sprintf((char*)s_evScratch, "%s\r\n", label);
    event_outputEvToFile(s_evScratch, printCurString((char*)s_evScratch));
}

void sprintfU32(const char* label, const uint32_t value)
{
    sprintf((char*)s_evScratch, "%s0x%08x\r\n", label, value);
    event_outputEvToFile(s_evScratch, printCurString((char*)s_evScratch));
}

void writeIMXGpioReg(did_event_t* ev)
{
    s_fileName = s_deviceFolder + "/IMX_GPIO.txt";
    assert(ev->length == sizeof(GPIO_TypeDef_IMX));
    if (ev->length > sizeof(GPIO_TypeDef_IMX)) return;
    GPIO_TypeDef_IMX tmpIMXUartGpio = {};
    memcpy(&tmpIMXUartGpio, ev->data, ev->length);
    sprintfLab("------------Start IMX GPIO Tx0 Reg-------------");
    sprintfU32("AFR[0]:\t\t", tmpIMXUartGpio.AFR[0]);
    sprintfU32("AFR[1]:\t\t", tmpIMXUartGpio.AFR[1]);
    sprintfU32("BRR:\t\t", tmpIMXUartGpio.BRR);
    sprintfU32("BSRR:\t\t", tmpIMXUartGpio.BSRR);
    sprintfU32("IDR:\t\t", tmpIMXUartGpio.IDR);
    sprintfU32("LCKR:\t\t", tmpIMXUartGpio.LCKR);
    sprintfU32("MODER:\t\t", tmpIMXUartGpio.MODER);
    sprintfU32("ODR:\t\t", tmpIMXUartGpio.ODR);
    sprintfU32("OSPEEDR:\t", tmpIMXUartGpio.OSPEEDR);
    sprintfU32("OTYPER:\t\t", tmpIMXUartGpio.OTYPER);
    sprintfU32("PUPDR:\t\t", tmpIMXUartGpio.PUPDR);
    sprintfLab("------------End IMX GPIO Tx0 Reg-------------\r\n");
}

void writeIMXDmaTx0Chan(did_event_t* ev)
{
    s_fileName = s_deviceFolder + "/IMX_DMA0_chan.txt";
    assert(ev->length == sizeof(dma_ch_t_IMX));
    if (ev->length > sizeof(dma_ch_t_IMX)) return;
    dma_ch_t_IMX tmpIMXUartDmaChan = {};
    memcpy(&tmpIMXUartDmaChan, ev->data, ev->length);
    sprintfLab("------------Start IMX Dma Tx0 Channel-------------");
    sprintfU32("instance:\t\t\t\t", tmpIMXUartDmaChan.instance);
    sprintfU32("ptr_end:\t\t\t\t", tmpIMXUartDmaChan.ptr_end);
    sprintfU32("ptr_start:\t\t\t\t", tmpIMXUartDmaChan.ptr_start);
    sprintfU32("active_tx_len:\t\t\t", tmpIMXUartDmaChan.active_tx_len);
    sprintfU32("done:\t\t\t\t\t", tmpIMXUartDmaChan.done);
    sprintfU32("cfg.dir:\t\t\t\t", tmpIMXUartDmaChan.cfg.dir);
    sprintfU32("cfg.circular:\t\t\t", tmpIMXUartDmaChan.cfg.circular);
    sprintfU32("cfg.priority:\t\t\t", tmpIMXUartDmaChan.cfg.priority);
    sprintfU32("cfg.interrupt:\t\t\t", static_cast<int32_t>(tmpIMXUartDmaChan.cfg.interrupt));
    sprintfU32("cfg.interrupt_priority:\t", tmpIMXUartDmaChan.cfg.interrupt_priority);
    sprintfU32("cfg.dma_channel_select:\t", tmpIMXUartDmaChan.cfg.dma_channel_select);
    sprintfU32("cfg.parent_type:\t\t", tmpIMXUartDmaChan.cfg.parent_type);
    sprintfU32("cfg.parent:\t\t\t\t", tmpIMXUartDmaChan.cfg.parent);
    sprintfU32("cfg.periph_reg:\t\t\t", tmpIMXUartDmaChan.cfg.periph_reg);
    sprintfU32("cfg.buf:\t\t\t\t", tmpIMXUartDmaChan.cfg.buf);
    sprintfU32("cfg.buf_len:\t\t\t", tmpIMXUartDmaChan.cfg.buf_len);
    sprintfU32("cfg.linear_buf:\t\t\t", tmpIMXUartDmaChan.cfg.linear_buf);
    sprintfU32("cfg.tcie_handler:\t\t", tmpIMXUartDmaChan.cfg.tcie_handler);
    sprintfU32("lastDmaUsed:\t\t\t", tmpIMXUartDmaChan.lastDmaUsed);
    sprintfU32("overflow:\t\t\t\t", tmpIMXUartDmaChan.overflow);
    sprintfLab("------------End IMX Dma Tx0 Channel-------------\r\n");
}

void writeSer0Cfg(did_event_t* ev, std::string name)
{
    s_fileName = s_deviceFolder + "/" + name + (name.size() ? "_" : "") + "Ser0_cfg.txt";
    assert(ev->length == sizeof(usart_cfg_t));
    if (ev->length > sizeof(usart_cfg_t)) return;
    usart_cfg_t tmpUartCfg = {};
    memcpy(&tmpUartCfg, ev->data, ev->length);
    name = std::string("------------Start ") + name + std::string(" Ser0 Cfg-------------");
    sprintfLab(name.c_str());
    sprintfU32("coding.baud:\t\t", tmpUartCfg.coding.baudRate);
    sprintfU32("coding.parity:\t\t", tmpUartCfg.coding.parity);
    sprintfU32("coding.stopBits:\t", tmpUartCfg.coding.stopBits);
    sprintfU32("interrupt:\t\t\t", static_cast<int32_t>(tmpUartCfg.interrupt));
    sprintfLab("------------End Ser0 Cfg-------------\r\n");
}

void writeIMXSer0TxReg(did_event_t* ev)
{
    s_fileName = s_deviceFolder + "/IMX_Ser0_reg.txt";
    assert(ev->length == sizeof(USART_TypeDef_IMX));
    if (ev->length > sizeof(USART_TypeDef_IMX)) return;
    USART_TypeDef_IMX tmpIMXUartReg = {};
    memcpy(&tmpIMXUartReg, ev->data, ev->length);
    sprintfLab("------------Start IMX Ser0 Reg-------------");
    sprintfU32("BRR:\t\t", tmpIMXUartReg.BRR);
    sprintfU32("CR1:\t\t", tmpIMXUartReg._CR1);
    sprintfU32("CR2:\t\t", tmpIMXUartReg._CR2);
    sprintfU32("CR3:\t\t", tmpIMXUartReg._CR3);
    sprintfU32("GTPR:\t\t", tmpIMXUartReg.GTPR);
    sprintfU32("ICR:\t\t", tmpIMXUartReg.ICR);
    sprintfU32("ISR:\t\t", tmpIMXUartReg.ISR);
    sprintfU32("RDR:\t\t", tmpIMXUartReg.RDR);
    sprintfU32("RQR:\t\t", tmpIMXUartReg.RQR);
    sprintfU32("RTOR:\t\t", tmpIMXUartReg.RTOR);
    sprintfU32("TDR:\t\t", tmpIMXUartReg.TDR);
    sprintfU32("RESERVED2:\t", tmpIMXUartReg.RESERVED2);
    sprintfU32("RESERVED3:\t", tmpIMXUartReg.RESERVED3);
    sprintfU32("RESERVED4:\t", tmpIMXUartReg.RESERVED4);
    sprintfU32("RESERVED5:\t", tmpIMXUartReg.RESERVED5);
    sprintfLab("------------End IMX Ser0 Reg-------------\r\n");
}

void writeIMXDmaTx0Reg(did_event_t* ev)
{
    s_fileName = s_deviceFolder + "/IMX_DMA0_inst.txt";
    assert(ev->length == sizeof(eventImxDmaTxInst_local_t));
    if (ev->length > sizeof(eventImxDmaTxInst_local_t)) return;
    eventImxDmaTxInst_local_t tmpIMXUartDmaInst = {};
    memcpy(&tmpIMXUartDmaInst, ev->data, ev->length);
    sprintfLab("------------Start IMX Dma Tx0 Inst-------------");
    sprintfU32("inst.CCR:\t\t\t\t", tmpIMXUartDmaInst.inst_CCR);
    sprintfU32("inst.CNDTR:\t\t\t\t", tmpIMXUartDmaInst.inst_CNDTR);
    sprintfU32("inst.CPAR:\t\t\t\t", tmpIMXUartDmaInst.inst_CPAR);
    sprintfU32("inst.CMAR:\t\t\t\t", tmpIMXUartDmaInst.inst_CMAR);
    sprintfU32("ptr_start:\t\t\t\t", tmpIMXUartDmaInst.ptr_start);
    sprintfU32("ptr_end:\t\t\t\t", tmpIMXUartDmaInst.ptr_end);
    sprintfU32("active_tx_len:\t\t\t", tmpIMXUartDmaInst.active_tx_len);
    sprintfU32("done:\t\t\t\t\t", tmpIMXUartDmaInst.done);
    sprintfU32("cfg.dir:\t\t\t\t", tmpIMXUartDmaInst.cfg_dir);
    sprintfU32("cfg.circular:\t\t\t", tmpIMXUartDmaInst.cfg_circular);
    sprintfU32("cfg.priority:\t\t\t", tmpIMXUartDmaInst.cfg_priority);
    sprintfU32("cfg.interrupt:\t\t\t", tmpIMXUartDmaInst.cfg_interrupt);
    sprintfU32("cfg.interrupt_priority:\t", tmpIMXUartDmaInst.cfg_interrupt_priority);
    sprintfU32("cfg.dma_chan_sel:\t\t", tmpIMXUartDmaInst.cfg_dma_channel_select);
    sprintfU32("cfg.parent_type:\t\t", tmpIMXUartDmaInst.cfg_parent_type);
    sprintfU32("cfg.parent:\t\t\t\t", tmpIMXUartDmaInst.cfg_parent);
    sprintfU32("cfg.periph_reg:\t\t\t", tmpIMXUartDmaInst.cfg_periph_reg);
    sprintfU32("cfg.buf:\t\t\t\t", tmpIMXUartDmaInst.cfg_buf);
    sprintfU32("cfg.buf_len:\t\t\t", tmpIMXUartDmaInst.cfg_buf_len);
    sprintfU32("cfg.linear_buf:\t\t\t", tmpIMXUartDmaInst.cfg_linear_buf);
    sprintfU32("cfg_tcie_handler:\t\t", tmpIMXUartDmaInst.cfg_tcie_handler);
    sprintfU32("lastDmaUsed:\t\t\t", tmpIMXUartDmaInst.lastDmaUsed);
    sprintfU32("overflow:\t\t\t\t", tmpIMXUartDmaInst.overflow);
    sprintfLab("------------End IMX Dma Rx0 Inst-------------\r\n");
}

void writeGpxDmaRx0Reg(did_event_t* ev)
{
    s_fileName = s_deviceFolder + "/GPX_DMA0_reg.txt";
    assert(ev->length == sizeof(DMA_Channel_TypeDef));
    if (ev->length > sizeof(DMA_Channel_TypeDef)) return;
    DMA_Channel_TypeDef tmpUartDmaReg = {};
    memcpy(&tmpUartDmaReg, ev->data, ev->length);
    sprintfLab("------------Start Dma Rx0 Reg-------------");
    sprintfU32("CBR1:\t", tmpUartDmaReg.CBR1);
    sprintfU32("CBR2:\t", tmpUartDmaReg.CBR2);
    sprintfU32("CCR:\t", tmpUartDmaReg.CCR);
    sprintfU32("CDAR:\t", tmpUartDmaReg.CDAR);
    sprintfU32("CFCR:\t", tmpUartDmaReg.CFCR);
    sprintfU32("CLBAR:\t", tmpUartDmaReg.CLBAR);
    sprintfU32("CLLR:\t", tmpUartDmaReg.CLLR);
    sprintfU32("CSAR:\t", tmpUartDmaReg.CSAR);
    sprintfU32("CSR:\t", tmpUartDmaReg.CSR);
    sprintfU32("CTR1:\t", tmpUartDmaReg.CTR1);
    sprintfU32("CTR2:\t", tmpUartDmaReg.CTR2);
    sprintfU32("CTR3:\t", tmpUartDmaReg.CTR3);
    sprintfLab("------------End Dma Rx0 Reg-------------\r\n");
}

void writeGpxSer0RxReg(did_event_t* ev)
{
    s_fileName = s_deviceFolder + "/GPX_Ser0_reg.txt"; 
    assert(ev->length == sizeof(USART_TypeDef));
    if (ev->length > sizeof(USART_TypeDef)) return;
    USART_TypeDef tmpUartReg = {};
    memcpy(&tmpUartReg, ev->data, ev->length);
    sprintfLab("------------Start Ser0 Reg-------------");
    sprintfU32("CR1:\t", tmpUartReg._CR1);
    sprintfU32("CR2:\t", tmpUartReg._CR2);
    sprintfU32("CR3:\t", tmpUartReg._CR3);
    sprintfU32("BRR:\t", tmpUartReg.BRR);
    sprintfU32("GTPR:\t", tmpUartReg.GTPR);
    sprintfU32("RTOR:\t", tmpUartReg.RTOR);
    sprintfU32("RQR:\t", tmpUartReg.RQR);
    sprintfU32("ISR:\t", tmpUartReg.ISR);
    sprintfU32("ICR:\t", tmpUartReg.ICR);
    sprintfU32("RDR:\t", tmpUartReg.RDR);
    sprintfU32("TDR:\t", tmpUartReg.TDR);
    sprintfU32("PRESC:\t", tmpUartReg.PRESC);
    sprintfU32("AUTOCR:\t", tmpUartReg.AUTOCR);
    sprintfLab("------------End Ser0 Reg-------------\r\n");
}

void writeGpxDma0RxChan(did_event_t* ev)
{
    s_fileName = s_deviceFolder + "/GPX_DMA0_chan.txt";
    assert(ev->length == sizeof(dma_ch_t_GPX));
    if (ev->length > sizeof(dma_ch_t_GPX)) return;
    dma_ch_t_GPX tmpUartDmaChan = {};
    memcpy(&tmpUartDmaChan, ev->data, ev->length);
    sprintfLab("------------Start Dma Rx0 Channel-------------");
    sprintfU32("cfg.buf:\t\t\t\t", tmpUartDmaChan.cfg.buf);
    sprintfU32("cfg.buf_len:\t\t\t", tmpUartDmaChan.cfg.buf_len);
    sprintfU32("cfg.interrupt:\t\t\t", static_cast<int32_t>(tmpUartDmaChan.cfg.interrupt));
    sprintfU32("cfg.interrupt_priority:\t", tmpUartDmaChan.cfg.interrupt_priority);
    sprintfU32("cfg.mode:\t\t\t\t", tmpUartDmaChan.cfg.mode);
    sprintfU32("cfg.parent:\t\t\t\t", tmpUartDmaChan.cfg.parent);
    sprintfU32("cfg.periph_reg:\t\t\t", tmpUartDmaChan.cfg.periph_reg);
    sprintfU32("cfg.priority:\t\t\t", tmpUartDmaChan.cfg.priority);
    sprintfU32("cfg.request_num:\t\t", tmpUartDmaChan.cfg.request_num);
    sprintfU32("cfg.tc_handler:\t\t\t", tmpUartDmaChan.cfg.tc_handler);
    sprintfU32("instance:\t\t\t\t", tmpUartDmaChan.instance);
    sprintfU32("lastDmaUsed:\t\t\t", tmpUartDmaChan.lastDmaUsed);
    sprintfU32("lli.rx:\t\t\t\t\t", tmpUartDmaChan.lli.rx.cdar);
    sprintfU32("lli.tx[0].br1:\t\t\t", tmpUartDmaChan.lli.tx[0].br1);
    sprintfU32("lli.tx[0].sar:\t\t\t", tmpUartDmaChan.lli.tx[0].sar);
    sprintfU32("lli.tx[0].llr:\t\t\t", tmpUartDmaChan.lli.tx[0].llr);
    sprintfU32("lli.tx[1].br1:\t\t\t", tmpUartDmaChan.lli.tx[1].br1);
    sprintfU32("lli.tx[1].sar:\t\t\t", tmpUartDmaChan.lli.tx[1].sar);
    sprintfU32("lli.tx[1].llr:\t\t\t", tmpUartDmaChan.lli.tx[1].llr);
    sprintfU32("lli.tx[2].br1:\t\t\t", tmpUartDmaChan.lli.tx[2].br1);
    sprintfU32("lli.tx[2].sar:\t\t\t", tmpUartDmaChan.lli.tx[2].sar);
    sprintfU32("lli.tx[2].llr:\t\t\t", tmpUartDmaChan.lli.tx[2].llr);
    sprintfU32("overflow:\t\t\t\t", tmpUartDmaChan.overflow);
    sprintfU32("ptr_end:\t\t\t\t", tmpUartDmaChan.ptr_end);
    sprintfU32("ptr_start:\t\t\t\t", tmpUartDmaChan.ptr_start);
    sprintfU32("txState.active_tx_len:\t", tmpUartDmaChan.txState.active_tx_len);
    sprintfU32("txState.dma_running:\t", tmpUartDmaChan.txState.dma_running);
    sprintfU32("txState.lli_head:\t\t", tmpUartDmaChan.txState.lli_head);
    sprintfU32("txState.lli_tail:\t\t", tmpUartDmaChan.txState.lli_tail);
    sprintfLab("------------End Dma Rx0 Channel-------------\r\n");
}

void writeGpxGpioRxReg(did_event_t* ev)
{
    s_fileName = s_deviceFolder + "/GPX_GPIO_Rx0_reg.txt"; 
    assert(ev->length == sizeof(GPIO_TypeDef));
    if (ev->length > sizeof(GPIO_TypeDef)) return;
    GPIO_TypeDef tmpUartGpio = {};
    memcpy(&tmpUartGpio, ev->data, ev->length);
    sprintfLab("------------Start GPIO Rx0 Reg-------------");
    sprintfU32("AFR[0]:\t\t", tmpUartGpio.AFR[0]);
    sprintfU32("AFR[1]:\t\t", tmpUartGpio.AFR[1]);
    sprintfU32("BRR:\t\t", tmpUartGpio.BRR);
    sprintfU32("BSRR:\t\t", tmpUartGpio.BSRR);
    sprintfU32("HSLVR:\t\t", tmpUartGpio.HSLVR);
    sprintfU32("IDR:\t\t", tmpUartGpio.IDR);
    sprintfU32("LCKR:\t\t", tmpUartGpio.LCKR);
    sprintfU32("MODER:\t\t", tmpUartGpio.MODER);
    sprintfU32("ODR:\t\t", tmpUartGpio.ODR);
    sprintfU32("OSPEEDR:\t", tmpUartGpio.OSPEEDR);
    sprintfU32("OTYPER:\t\t", tmpUartGpio.OTYPER);
    sprintfU32("PUPDR:\t\t", tmpUartGpio.PUPDR);
    sprintfU32("SECCFGR:\t", tmpUartGpio.SECCFGR);
    sprintfLab("------------End GPIO Rx0 Reg-------------\r\n");
}

static void msgHandlerIsb(InertialSense* i, p_data_t* data, int pHandle)
{
	static uint64_t dataCount;
	printf("Data count: %" PRIu64 "          \r", ++dataCount);
}

int main(int argc, char* argv[])
{
    is_comm_instance_t c;
    c.rxBuf.start = s_evScratch;
    c.rxBuf.size = SCRATCH_SIZE + DID_EVENT_HEADER_SIZE;
    std::time_t logTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string outDir = "out/";// + logTime;

	if (argc < 1)
	{
		printf("Please pass the data log directory path (i.e. \"C:\\Users\\[username]\\Documents\\Inertial Sense\\Logs\\20180716_172323)\"\r\n");
		// In Visual Studio IDE, this can be done through "Project Properties -> Debugging -> Command Arguments: COM3 kml" 
		return -1;
	}

    cISLogger logger;
    if (!logger.LoadFromDirectory(argv[1], cISLogger::eLogType::LOGTYPE_RAW, {"ALL"}))
    {
        printf("Failed to load log files: %s\r\n", argv[0]);
        return false;
    }

    // time do handle our output dir
    // does it exist?
    struct stat info;
    if (stat(outDir.c_str(), &info) == 0)
    {
        if (info.st_mode & S_IFDIR)
            std::cout << "Found output dir: " << outDir << std::endl;
        else
        {
            std::cout << "Output dir not a folder: " << outDir << std::endl;
            return false;
        }
    }
    else
    {
        // the folder does not exist try to create it
        // creating output files
#if PLATFORM_IS_WINDOWS
        if (mkdir(outDir.c_str()) == 0)
#else
        if (mkdir(outDir.c_str(), 0777) == 0)
#endif
            std::cout << "Created output dir: " << outDir << std::endl;
        else
        {
            std::cout << "Failed to created output dir: " << outDir << std::endl;
            return false;
        }
    }

    std::cout << "Parsing log files: " << outDir << std::endl;

    p_data_buf_t* data;
    // for (int d=0; d<logger.DeviceCount(); d++)
    for (auto dl : logger.DeviceLogs())
    {

        s_deviceFolder = outDir + "SN-" + std::to_string(dl->SerialNumber());

        if (stat(s_deviceFolder.c_str(), &info) == 0)
        {
            if (!(info.st_mode & S_IFDIR))
            {
                std::cout << "Output dir not a folder skipping: " << s_deviceFolder << std::endl;
                continue;
            }
        }
        else
        {
            // the folder does not exist try to create it
            // creating output files
#if PLATFORM_IS_WINDOWS
            if (mkdir(s_deviceFolder.c_str()) == 0)
#else
            if (mkdir(s_deviceFolder.c_str(), 0777) == 0)
#endif
                std::cout << "Created output dir: " << s_deviceFolder << std::endl;
            else
            {
                std::cout << "Failed to created output dir: " << s_deviceFolder << std::endl;
                continue;
            }
        }

        int count = 0;
        bool logged = false;

        // cycle through data
        while (((data = logger.ReadData(dl)) != NULL))
        {
            p_data_t d = { data->hdr, data->buf };

            if (d.hdr.id == DID_EVENT)
            {
                did_event_t* ev = (did_event_t*)data->buf;
                memset(s_evScratch, 0, SCRATCH_SIZE + DID_EVENT_HEADER_SIZE);
                memcpy(s_evScratch, ev->data, _MIN(ev->length, sizeof(s_evScratch)));

                switch (ev->msgTypeID)
                {
                case EVENT_MSG_TYPE_ID_RAW:  s_fileName = s_deviceFolder + "/out.raw"; break;
                case EVENT_MSG_TYPE_ID_ASCII: s_fileName = s_deviceFolder + "/out.txt";  break;
                case EVENT_MSG_TYPE_ID_RTMC3_RCVR1:
                    s_fileName = s_deviceFolder + "/rcvr1.rtcm";
                    c.rxBuf.size = ev->length;
                    c.rxBuf.head = s_evScratch;
                    c.rxBuf.end = s_evScratch + ev->length;
                    c.rxBuf.tail = s_evScratch + ev->length;
                    c.rxBuf.scan = s_evScratch;

                    c.processPkt = nullptr;

                    is_comm_parse_timeout(&c, 0);
                    break;
                case EVENT_MSG_TYPE_ID_RTMC3_RCVR2: s_fileName = s_deviceFolder + "/rcvr2.rtcm";  break;
                case EVENT_MSG_TYPE_ID_RTMC3_EXT: s_fileName = s_deviceFolder + "/rcvr_ext.rtcm";  break;
                case EVENT_MSG_TYPE_ID_SONY_BIN_RCVR1: s_fileName = s_deviceFolder + "/rcvr1.sbp";  break;
                case EVENT_MSG_TYPE_ID_SONY_BIN_RCVR2: s_fileName = s_deviceFolder + "/rcvr2.sbp";  break;

                case EVENT_MSG_TYPE_ID_IMX_DMA_TX_0_INST:
                    writeIMXDmaTx0Reg(ev);
                    logged = true;
                    break;
                case EVENT_MSG_TYPE_ID_IMX_SER0_REG:
                    writeIMXSer0TxReg(ev);
                    logged = true;
                    break;
                case  EVENT_MSG_TYPE_ID_IMX_SER0_CFG:
                    writeSer0Cfg(ev, "IMX");
                    logged = true;
                    break;
                case EVENT_MSG_TYPE_ID_IMX_DMA_TX_0_CHAN:
                    writeIMXDmaTx0Chan(ev);
                    logged = true;
                    break;
                case EVENT_MSG_TYPE_ID_IMX_GPIO_TX_0_REG:
                    writeIMXGpioReg(ev);
                    logged = true;
                    break;

                case EVENT_MSG_TYPE_ID_GPX_DMA_RX_0_INST:
                    writeGpxDmaRx0Reg(ev);
                    logged = true;
                    break;

                case EVENT_MSG_TYPE_ID_GPX_SER0_REG: 
                    writeGpxSer0RxReg(ev);
                    logged = true;
                    break;

                case EVENT_MSG_TYPE_ID_GPX_SER0_CFG: 
                    writeSer0Cfg(ev, "GPX");
                    logged = true;
                    break;

                case EVENT_MSG_TYPE_ID_GPX_DMA_RX_0_CHAN:
                    writeGpxDma0RxChan(ev);
                    logged = true;
                    break;

                case EVENT_MSG_TYPE_ID_GPX_GPIO_RX_0_REG: 
                    writeGpxGpioRxReg(ev);
                    logged = true;
                    break;
                
                default:
                    s_fileName = s_deviceFolder + "/UNKNOWN_" + std::to_string(ev->msgTypeID) + ".Bin";
                    printf("Event type %d found but is not supported. Output at: %s\n", ev->msgTypeID, s_fileName.c_str());
                    break;
                }

                if (!logged)
                    event_outputEvToFile(s_evScratch, ev->length);

                logged = false;
            }

            if (++count % 5000 == 0)
                printf("Read %d msgs from SN-%d\n", count, dl->SerialNumber());

        }
    }

    std::cout << "Done parsing log files: " << argv[1] << std::endl;
    std::cout << "Output dir: " << s_deviceFolder << std::endl;
    return true;
}


