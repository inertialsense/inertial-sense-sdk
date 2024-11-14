/*
MIT LICENSE

Copyright 2014 Inertial Sense, LLC - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <string.h>

// STEP 1: Add Includes
// Change these include paths to the correct paths for your project
#include "../../src/InertialSense.h"

#define SCRATCH_SIZE    2048


typedef struct
{
    uint32_t CR1;         /*!< USART Control register 1,                 Address offset: 0x00  */
    uint32_t CR2;         /*!< USART Control register 2,                 Address offset: 0x04  */
    uint32_t CR3;         /*!< USART Control register 3,                 Address offset: 0x08  */
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

typedef enum
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
} IRQn_Type;

typedef struct
{
    serial_options_t    coding;
    IRQn_Type           interrupt;
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
    void*                   parent;             // Pointer to parent peripheral management struct
    uint8_t                 mode;               // DMA_MODE_...
    dma_callback_function   tc_handler;         // If non-null, this function pointer is called from within transfer complete interrupt handler when all data including queued data has been transferred.
    IRQn_Type               interrupt;
    uint8_t                 interrupt_priority;
    uint8_t                 priority;           // DMA priority (DMA_PRIO_...)
    uint8_t                 request_num;        // DMA request ID, links channel to peripheral (DMA_IDX_... for index)
    uint32_t                periph_reg;         // target 8-bit peripheral register (DMA_IDX_... for index)
    uint8_t*                buf;
    uint16_t                buf_len;            // Actual usable buffer length is one less (buf_len - 1)
} dma_config_t;

typedef struct
{
    uint32_t                br1;   // transfer size
    uint32_t                sar;   // source address
    uint32_t                llr;   // next linked list address

} dma_tx_lli_t;

typedef struct
{
    uint32_t         cdar;
} dma_rx_circ_lli_t;

#define DMA_TX_LLI_COUNT    3
typedef union
{
    dma_tx_lli_t            tx[DMA_TX_LLI_COUNT];
    dma_rx_circ_lli_t       rx;
} dma_lli_u;

typedef struct
{
    volatile uint16_t       active_tx_len;
    dma_tx_lli_t*           lli_head;        // Linked list output to dma
    dma_tx_lli_t*           lli_tail;        // Linked list input from dma_buffer_write() 
    volatile bool           dma_running;
} dma_tx_state_t;

typedef struct dma_channel_
{
    DMA_Channel_TypeDef* instance;
    volatile uint8_t* ptr_start;
    volatile uint8_t* ptr_end;
    dma_config_t            cfg;
    dma_lli_u               lli;                // Linked list memory.  DMA_MODE_TX_LLI uses 6, DMA_MODE_RX_CIRC uses 1.
    dma_tx_state_t          txState;
    int                     lastDmaUsed;
    uint8_t                 overflow;            // Buffer overflow
} dma_ch_t;

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
    DMA_Channel_TypeDef*    instance;
    uint8_t*                ptr_start;
    uint8_t*                ptr_end;
    uint16_t                active_tx_len;
    bool                    done;                            // Currently only used in TX
    dma_config_t            cfg;
    int                     lastDmaUsed;                    // Number of bytes in the buffer minus bytes last read.  This is used to identify buffer overflow.
    uint8_t                 overflow;                        // Buffer overflow
} dma_ch_t_IMX;

typedef struct
{
    uint32_t CR1;         /*!< USART Control register 1,                 Address offset: 0x00 */
    uint32_t CR2;         /*!< USART Control register 2,                 Address offset: 0x04 */
    uint32_t CR3;         /*!< USART Control register 3,                 Address offset: 0x08 */
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


DMA_Channel_TypeDef tmpUartDmaReg;
USART_TypeDef tmpUartReg;
usart_cfg_t tmpUartCfg;
dma_ch_t tmpUartDmaChan;
GPIO_TypeDef tmpUartGpio;

eventImxDmaTxInst_t tmpIMXUartDmaInst;
USART_TypeDef_IMX tmpIMXUartReg;
dma_ch_t_IMX tmpIMXUartDmaChan;
GPIO_TypeDef_IMX tmpIMXUartGpio;

int printCurString(char* c)
{
    // Create a string from the buffer
    std::string result(c);
    std::cout << result;
    return result.length();
}

void event_outputEvToFile(std::string fileName, uint8_t* data, int len)
{
    std::ofstream outfile;

    outfile.open(fileName, std::ios_base::app | std::ios_base::binary); // append instead of overwrite
    outfile.write((const char*)data, len);
    outfile.close();
}

void writeIMXGpioReg(uint8_t* evScratch, std::string fileName)
{
    sprintf((char*)evScratch, "------------Start IMX GPIP Tx0 Reg-------------\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "AFR[0]:\t\t0x%08x\r\n", tmpIMXUartGpio.AFR[0]);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "AFR[1]:\t\t0x%08x\r\n", tmpIMXUartGpio.AFR[1]);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "BRR:\t\t0x%08x\r\n", tmpIMXUartGpio.BRR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "BSRR:\t\t0x%08x\r\n", tmpIMXUartGpio.BSRR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "IDR:\t\t0x%08x\r\n", tmpIMXUartGpio.IDR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "LCKR:\t\t0x%08x\r\n", tmpIMXUartGpio.LCKR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "MODER:\t\t0x%08x\r\n", tmpIMXUartGpio.MODER);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "ODR:\t\t0x%08x\r\n", tmpIMXUartGpio.ODR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "OSPEEDR:\t0x%08x\r\n", tmpIMXUartGpio.OSPEEDR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "OTYPER:\t\t0x%08x\r\n", tmpIMXUartGpio.OTYPER);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "PUPDR:\t\t0x%08x\r\n", tmpIMXUartGpio.PUPDR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "\r\n------------End IMX GPIP Tx0 Reg-------------\r\n\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));
}

void writeIMXDmaTx0Chan(uint8_t* evScratch, std::string fileName)
{
    sprintf((char*)evScratch, "------------Start IMX Dma Tx0 Channel-------------\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "active_tx_len:\t\t\t0x%08x\r\n", tmpIMXUartDmaChan.active_tx_len);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.buf:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaChan.cfg.buf);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.buf_len:\t\t\t0x%08x\r\n", tmpIMXUartDmaChan.cfg.buf_len);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.interrupt:\t\t\t0x%08x\r\n", tmpIMXUartDmaChan.cfg.interrupt);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.interrupt_priority:\t0x%08x\r\n", tmpIMXUartDmaChan.cfg.interrupt_priority);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.mode:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaChan.cfg.mode);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.parent:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaChan.cfg.parent);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.periph_reg:\t\t\t0x%08x\r\n", tmpIMXUartDmaChan.cfg.periph_reg);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.priority:\t\t\t0x%08x\r\n", tmpIMXUartDmaChan.cfg.priority);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.request_num:\t\t0x%08x\r\n", tmpIMXUartDmaChan.cfg.request_num);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.tc_handler:\t\t\t0x%08x\r\n", tmpIMXUartDmaChan.cfg.tc_handler);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "done:\t\t\t\t\t0x%08x\r\n", tmpIMXUartDmaChan.done);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "instance:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaChan.instance);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "lastDmaUsed:\t\t\t0x%08x\r\n", tmpIMXUartDmaChan.lastDmaUsed);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "overflow:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaChan.overflow);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "ptr_end:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaChan.ptr_end);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "ptr_start:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaChan.ptr_start);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "\r\n------------End IMX Dma Tx0 Channel-------------\r\n\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

}

void writeIMXSer0TxCfg(uint8_t* evScratch, std::string fileName)
{
    sprintf((char*)evScratch, "------------Start IMX Ser0 Cfg-------------\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "coding.baud:\t\t0x%08x\r\n", tmpUartCfg.coding.baudRate);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "coding.parity:\t\t0x%08x\r\n", tmpUartCfg.coding.parity);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "coding.stopBits:\t0x%08x\r\n", tmpUartCfg.coding.stopBits);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "interrupt:\t\t\t0x%08x\r\n", tmpUartCfg.interrupt);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "\r\n------------End IMX Ser0 Cfg-------------\r\n\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

}

void writeIMXSer0TxReg(uint8_t* evScratch, std::string fileName)
{
    sprintf((char*)evScratch, "------------Start IMX Ser0 Reg-------------\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "BRR:\t\t0x%08x\r\n", tmpIMXUartReg.BRR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CR1:\t\t0x%08x\r\n", tmpIMXUartReg.CR1);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CR2:\t\t0x%08x\r\n", tmpIMXUartReg.CR2);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CR3:\t\t0x%08x\r\n", tmpIMXUartReg.CR3);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "GTPR:\t\t0x%08x\r\n", tmpIMXUartReg.GTPR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "ICR:\t\t0x%08x\r\n", tmpIMXUartReg.ICR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "ISR:\t\t0x%08x\r\n", tmpIMXUartReg.ISR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "RDR:\t\t0x%08x\r\n", tmpIMXUartReg.RDR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "RQR:\t\t0x%08x\r\n", tmpIMXUartReg.RQR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "RTOR:\t\t0x%08x\r\n", tmpIMXUartReg.RTOR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "TDR:\t\t0x%08x\r\n", tmpIMXUartReg.TDR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "RESERVED2:\t0x%08x\r\n", tmpIMXUartReg.RESERVED2);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "RESERVED3:\t0x%08x\r\n", tmpIMXUartReg.RESERVED3);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "RESERVED4:\t0x%08x\r\n", tmpIMXUartReg.RESERVED4);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "RESERVED5:\t0x%08x\r\n", tmpIMXUartReg.RESERVED5);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "\r\n------------End IMX Ser0 Reg-------------\r\n\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

}

void dataToImxDmaInst(uint8_t* inBuf, int len)
{
    int startIndex;
    int endIndex;

    // do inital copy
    memcpy(&tmpIMXUartDmaInst, inBuf, ((uint32_t)&tmpIMXUartDmaInst.ptr_start) - ((uint32_t)&tmpIMXUartDmaInst));

    // now fix things
    startIndex = 0x10;
    tmpIMXUartDmaInst.ptr_start = (uint8_t*)((((uint32_t)inBuf[startIndex + 3]) << 24) |
        (((uint32_t)inBuf[startIndex + 2]) << 16) |
        (((uint16_t)inBuf[startIndex + 1]) << 8) | inBuf[startIndex]);

    startIndex = 0x14;
    tmpIMXUartDmaInst.ptr_end = (uint8_t*)((((uint32_t)inBuf[startIndex+3]) << 24) | 
                                (((uint32_t)inBuf[startIndex+2]) << 16) | 
                                (((uint16_t)inBuf[startIndex+1]) << 8) | inBuf[startIndex]);

    startIndex = 0x18;
    endIndex = 0x22;
    memcpy(&tmpIMXUartDmaInst.active_tx_len, &inBuf[startIndex], endIndex - startIndex);

    startIndex = 0x22;
    tmpIMXUartDmaInst.cfg_parent = (uint8_t*)((((uint32_t)inBuf[startIndex + 3]) << 24) |
        (((uint32_t)inBuf[startIndex + 2]) << 16) |
        (((uint16_t)inBuf[startIndex + 1]) << 8) | inBuf[startIndex]);

    startIndex = 0x26;
    tmpIMXUartDmaInst.cfg_periph_reg = (uint32_t*)((((uint32_t)inBuf[startIndex + 3]) << 24) |
        (((uint32_t)inBuf[startIndex + 2]) << 16) |
        (((uint16_t)inBuf[startIndex + 1]) << 8) | inBuf[startIndex]);

    startIndex = 0x2a;
    tmpIMXUartDmaInst.cfg_buf = (uint8_t*)((((uint32_t)inBuf[startIndex + 3]) << 24) |
        (((uint32_t)inBuf[startIndex + 2]) << 16) |
        (((uint16_t)inBuf[startIndex + 1]) << 8) | inBuf[startIndex]);

    startIndex = 0x2e;
    endIndex = 0x31;
    memcpy(&tmpIMXUartDmaInst.cfg_buf_len, &inBuf[startIndex], endIndex - startIndex);

    startIndex = 0x31;
    tmpIMXUartDmaInst.cfg_tcie_handler = (uint8_t*)((((uint32_t)inBuf[startIndex + 3]) << 24) |
        (((uint32_t)inBuf[startIndex + 2]) << 16) |
        (((uint16_t)inBuf[startIndex + 1]) << 8) | inBuf[startIndex]);

    startIndex = 0x35;
    endIndex = 0x3a;
    memcpy(&tmpIMXUartDmaInst.lastDmaUsed, &inBuf[startIndex], endIndex - startIndex);

}

void writeIMXDmaTx0Reg(uint8_t* evScratch, std::string fileName)
{
    sprintf((char*)evScratch, "------------Start IMX Dma Tx0 Inst-------------\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "inst.CCR:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.inst_CCR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "inst.CNDTR:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.inst_CNDTR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "inst.CPAR:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.inst_CPAR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "inst.CMAR:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.inst_CMAR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "ptr_start:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.ptr_start);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "ptr_end:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.ptr_end);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "active_tx_len:\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.active_tx_len);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "done:\t\t\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.done);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.dir:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.cfg_dir);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.circular:\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.cfg_circular);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.priority:\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.cfg_priority);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.interrupt:\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.cfg_interrupt);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.interrupt_priority:\t0x%08x\r\n", tmpIMXUartDmaInst.cfg_interrupt_priority);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.dma_chan_sel:\t\t0x%08x\r\n", tmpIMXUartDmaInst.cfg_dma_channel_select);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.parent_type:\t\t0x%08x\r\n", tmpIMXUartDmaInst.cfg_parent_type);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.parent:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.cfg_parent);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.periph_reg:\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.cfg_periph_reg);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.buf:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.cfg_buf);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.buf_len:\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.cfg_buf_len);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.linear_buf:\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.cfg_linear_buf);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg_tcie_handler:\t\t0x%08x\r\n", tmpIMXUartDmaInst.cfg_tcie_handler);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "lastDmaUsed:\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.lastDmaUsed);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "overflow:\t\t\t\t0x%08x\r\n", tmpIMXUartDmaInst.overflow);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "\r\n------------End IMX Dma Rx0 Inst-------------\r\n\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));
}

void writeDmaRx0Reg(uint8_t* evScratch, std::string fileName)
{
    sprintf((char*)evScratch, "------------Start Dma Rx0 Reg-------------\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CBR1:\t0x%08x\r\n", tmpUartDmaReg.CBR1);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CBR2:\t0x%08x\r\n", tmpUartDmaReg.CBR2);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CCR:\t0x%08x\r\n", tmpUartDmaReg.CCR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CDAR:\t0x%08x\r\n", tmpUartDmaReg.CDAR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CFCR:\t0x%08x\r\n", tmpUartDmaReg.CFCR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CLBAR:\t0x%08x\r\n", tmpUartDmaReg.CLBAR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CLLR:\t0x%08x\r\n", tmpUartDmaReg.CLLR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CSAR:\t0x%08x\r\n", tmpUartDmaReg.CSAR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CSR:\t0x%08x\r\n", tmpUartDmaReg.CSR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CTR1:\t0x%08x\r\n", tmpUartDmaReg.CTR1);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CTR2:\t0x%08x\r\n", tmpUartDmaReg.CTR2);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CTR3:\t0x%08x\r\n", tmpUartDmaReg.CTR3);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "\r\n------------End Dma Rx0 Reg-------------\r\n\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));
}

void writeSer0RxReg(uint8_t* evScratch, std::string fileName)
{
    sprintf((char*)evScratch, "------------Start Ser0 Reg-------------\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CR1:\t0x%08x\r\n", tmpUartReg.CR1);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CR2:\t0x%08x\r\n", tmpUartReg.CR2);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "CR3:\t0x%08x\r\n", tmpUartReg.CR3);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "BRR:\t0x%08x\r\n", tmpUartReg.BRR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "GTPR:\t0x%08x\r\n", tmpUartReg.GTPR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "RTOR:\t0x%08x\r\n", tmpUartReg.RTOR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "RQR:\t0x%08x\r\n", tmpUartReg.RQR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "ISR:\t0x%08x\r\n", tmpUartReg.ISR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "ICR:\t0x%08x\r\n", tmpUartReg.ICR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "RDR:\t0x%08x\r\n", tmpUartReg.RDR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "TDR:\t0x%08x\r\n", tmpUartReg.TDR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "PRESC:\t0x%08x\r\n", tmpUartReg.PRESC);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "AUTOCR:\t0x%08x\r\n", tmpUartReg.AUTOCR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "\r\n------------End Ser0 Reg-------------\r\n\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));
}

void writeSer0RxCfg(uint8_t* evScratch, std::string fileName)
{
    sprintf((char*)evScratch, "------------Start Ser0 Cfg-------------\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "coding.baud:\t\t0x%08x\r\n", tmpUartCfg.coding.baudRate);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "coding.parity:\t\t0x%08x\r\n", tmpUartCfg.coding.parity);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "coding.stopBits:\t0x%08x\r\n", tmpUartCfg.coding.stopBits);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "interrupt:\t\t\t0x%08x\r\n", tmpUartCfg.interrupt);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "\r\n------------End Ser0 Cfg-------------\r\n\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));
}

void writeDma0RxChan(uint8_t* evScratch, std::string fileName)
{
    sprintf((char*)evScratch, "------------Start Dma Rx0 Channel-------------\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.buf:\t\t\t\t0x%08x\r\n", tmpUartDmaChan.cfg.buf);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.buf_len:\t\t\t0x%08x\r\n", tmpUartDmaChan.cfg.buf_len);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.interrupt:\t\t\t0x%08x\r\n", tmpUartDmaChan.cfg.interrupt);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.interrupt_priority:\t0x%08x\r\n", tmpUartDmaChan.cfg.interrupt_priority);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.mode:\t\t\t\t0x%08x\r\n", tmpUartDmaChan.cfg.mode);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.parent:\t\t\t\t0x%08x\r\n", tmpUartDmaChan.cfg.parent);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.periph_reg:\t\t\t0x%08x\r\n", tmpUartDmaChan.cfg.periph_reg);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.priority:\t\t\t0x%08x\r\n", tmpUartDmaChan.cfg.priority);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.request_num:\t\t0x%08x\r\n", tmpUartDmaChan.cfg.request_num);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "cfg.tc_handler:\t\t\t0x%08x\r\n", tmpUartDmaChan.cfg.tc_handler);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "instance:\t\t\t\t0x%08x\r\n", tmpUartDmaChan.instance);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "lastDmaUsed:\t\t\t0x%08x\r\n", tmpUartDmaChan.lastDmaUsed);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "lli.rx:\t\t\t\t\t0x%08x\r\n", tmpUartDmaChan.lli.rx);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "lli.tx:\t\t\t\t\t0x%08x\r\n", tmpUartDmaChan.lli.tx);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "overflow:\t\t\t\t0x%08x\r\n", tmpUartDmaChan.overflow);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "ptr_end:\t\t\t\t0x%08x\r\n", tmpUartDmaChan.ptr_end);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "ptr_start:\t\t\t\t0x%08x\r\n", tmpUartDmaChan.ptr_start);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "txState.active_tx_len:\t0x%08x\r\n", tmpUartDmaChan.txState.active_tx_len);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "txState.dma_running:\t0x%08x\r\n", tmpUartDmaChan.txState.dma_running);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "txState.lli_head:\t\t0x%08x\r\n", tmpUartDmaChan.txState.lli_head);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "txState.lli_tail:\t\t0x%08x\r\n", tmpUartDmaChan.txState.lli_tail);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "\r\n------------End Dma Rx0 Channel-------------\r\n\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));
}

void writeGpioRxReg(uint8_t* evScratch, std::string fileName)
{
    sprintf((char*)evScratch, "------------Start GPIP Rx0 Reg-------------\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "AFR[0]:\t\t0x%08x\r\n", tmpUartGpio.AFR[0]);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "AFR[1]:\t\t0x%08x\r\n", tmpUartGpio.AFR[1]);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "BRR:\t\t0x%08x\r\n", tmpUartGpio.BRR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "BSRR:\t\t0x%08x\r\n", tmpUartGpio.BSRR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "HSLVR:\t\t0x%08x\r\n", tmpUartGpio.HSLVR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "IDR:\t\t0x%08x\r\n", tmpUartGpio.IDR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "LCKR:\t\t0x%08x\r\n", tmpUartGpio.LCKR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "MODER:\t\t0x%08x\r\n", tmpUartGpio.MODER);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "ODR:\t\t0x%08x\r\n", tmpUartGpio.ODR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "OSPEEDR:\t0x%08x\r\n", tmpUartGpio.OSPEEDR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "OTYPER:\t\t0x%08x\r\n", tmpUartGpio.OTYPER);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "PUPDR:\t\t0x%08x\r\n", tmpUartGpio.PUPDR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "SECCFGR:\t0x%08x\r\n", tmpUartGpio.SECCFGR);
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));

    sprintf((char*)evScratch, "\r\n------------End GPIP Rx0 Reg-------------\r\n\r\n");
    event_outputEvToFile(fileName, evScratch, printCurString((char*)evScratch));
}

static void msgHandlerIsb(InertialSense* i, p_data_t* data, int pHandle)
{
    static uint64_t dataCount;
    printf("Data count: %" PRIu64 "          \r", ++dataCount);
}

int main(int argc, char* argv[])
{
    is_comm_instance_t c;
    uint8_t evScratch[SCRATCH_SIZE + DID_EVENT_HEADER_SIZE];
    c.rxBuf.start = evScratch;
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
            printf("Found output dir: %s\r\n", outDir);
        else
        {
            printf("Output dir not a folder: %s\r\n", outDir);
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
        if (mkdir(outDir.c_str(), 0777))
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

        std::string deviceFolder = outDir + "/SN-" + std::to_string(dl->SerialNumber());

        if (stat(deviceFolder.c_str(), &info) == 0)
        {
            if (!(info.st_mode & S_IFDIR))
            {
                std::cout << "Output dir not a folder skipping: " << deviceFolder << std::endl;
                continue;
            }
        }
        else
        {
            // the folder does not exist try to create it
            // creating output files
#if PLATFORM_IS_WINDOWS
            if (mkdir(deviceFolder.c_str()) == 0)
#else
            if (mkdir(deviceFolder.c_str(), 0777))
#endif
                std::cout << "Created output dir: " << deviceFolder << std::endl;
            else
            {
                std::cout << "Failed to created output dir: " << deviceFolder << std::endl;
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
                memset(evScratch, 0, SCRATCH_SIZE + DID_EVENT_HEADER_SIZE);
                memcpy(evScratch, ev->data, ev->length);
                memcpy(&tmpUartReg, evScratch, ev->length);
                memcpy(&tmpUartCfg, evScratch, ev->length);
                memcpy(&tmpUartDmaReg, evScratch, ev->length);
                memcpy(&tmpUartDmaChan, evScratch, ev->length);
                memcpy(&tmpUartGpio, evScratch, ev->length);

                memcpy(&tmpIMXUartReg, evScratch, ev->length);
                memcpy(&tmpIMXUartDmaChan, evScratch, ev->length);
                memcpy(&tmpIMXUartGpio, evScratch, ev->length);

                std::string fileName;

                switch (ev->msgTypeID)
                {
                case EVENT_MSG_TYPE_ID_RAW:  fileName = deviceFolder + "/out.raw"; break;
                case EVENT_MSG_TYPE_ID_ASCII: fileName = deviceFolder + "/out.txt";  break;
                case EVENT_MSG_TYPE_ID_RTMC3_RCVR1:
                    fileName = deviceFolder + "/rcvr1.rtcm";
                    c.rxBuf.size = ev->length;
                    c.rxBuf.head = evScratch;
                    c.rxBuf.end = evScratch + ev->length;
                    c.rxBuf.tail = evScratch + ev->length;
                    c.rxBuf.scan = evScratch;

                    c.processPkt = nullptr;

                    is_comm_parse_timeout(&c, 0);
                    break;
                case EVENT_MSG_TYPE_ID_RTMC3_RCVR2: fileName = deviceFolder + "/rcvr2.rtcm";  break;
                case EVENT_MSG_TYPE_ID_RTMC3_EXT: fileName = deviceFolder + "/rcvr_ext.rtcm";  break;
                case EVENT_MSG_TYPE_ID_SONY_BIN_RCVR1: fileName = deviceFolder + "/rcvr1.sbp";  break;
                case EVENT_MSG_TYPE_ID_SONY_BIN_RCVR2: fileName = deviceFolder + "/rcvr2.sbp";  break;

                case EVENT_MSG_TYPE_ID_IMX_DMA_TX_0_INST:
                    fileName = deviceFolder + "/IMX_DMA0_inst.txt";
                    dataToImxDmaInst(ev->data, ev->length);
                    writeIMXDmaTx0Reg(evScratch, fileName);
                    logged = true;
                    break;
                case EVENT_MSG_TYPE_ID_IMX_SER0_REG:
                    fileName = deviceFolder + "/IMX_SER0_reg.txt";
                    writeIMXSer0TxReg(evScratch, fileName);
                    logged = true;
                    break;
                case  EVENT_MSG_TYPE_ID_IMX_SER0_CFG:
                    fileName = deviceFolder + "/IMX_SER0_chan.txt";
                    writeIMXSer0TxCfg(evScratch, fileName);
                    logged = true;
                    break;
                case EVENT_MSG_TYPE_ID_IMX_DMA_TX_0_CHAN:
                    fileName = deviceFolder + "/IMX_DMA0_chan.txt";
                    writeIMXDmaTx0Chan(evScratch, fileName);
                    logged = true;
                    break;
                case EVENT_MSG_TYPE_ID_IMX_GPIO_TX_0_REG:
                    fileName = deviceFolder + "/IMX_GPIO.txt";
                    writeIMXGpioReg(evScratch, fileName);
                    logged = true;
                    break;

                case EVENT_MSG_TYPE_ID_DMA_RX_0_INST:
                    fileName = deviceFolder + "/DMA0_reg.txt";
                    writeDmaRx0Reg(evScratch, fileName);
                    logged = true;
                    break;

                case EVENT_MSG_TYPE_ID_SER0_REG: 
                    fileName = deviceFolder + "/Ser0_reg.txt"; 
                    writeSer0RxReg(evScratch, fileName);
                    logged = true;
                    break;

                case EVENT_MSG_TYPE_ID_SER0_CFG: 
                    fileName = deviceFolder + "/Ser0_cfg.txt";
                    writeSer0RxCfg(evScratch, fileName);
                    logged = true;
                    break;

                case EVENT_MSG_TYPE_ID_DMA_RX_0_CHAN:
                    fileName = deviceFolder + "/DMA0_chan.txt";
                    writeDma0RxChan(evScratch, fileName);
                    logged = true;
                    break;


                case EVENT_MSG_TYPE_ID_GPIO_RX_0_REG: 
                    fileName = deviceFolder + "/GPIO_Rx0_reg.txt"; 
                    writeGpioRxReg(evScratch, fileName);
                    logged = true;
                    break;
                
                default:
                    fileName = deviceFolder + "/UNKNOWN_" + std::to_string(ev->msgTypeID) + ".Bin";
                    printf("Event type %d found but is not supported. Output at: %s\n", ev->msgTypeID, fileName.c_str());
                    break;
                }

                if (!logged)
                    event_outputEvToFile(fileName, evScratch, ev->length);

                logged = false;
            }

            if (++count % 5000 == 0)
                printf("Read %d msgs from SN-%d\n", count, dl->SerialNumber());

        }
    }

    std::cout << "Done parsing log files: " << argv[1] << std::endl;
    return true;
}


