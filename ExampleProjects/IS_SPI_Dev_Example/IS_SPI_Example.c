/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file or main.c
 * to avoid loosing it when reconfiguring.
 */
#include "atmel_start.h"
#include "IS_SPI_Example.h"

#include "ISComm.h"

#if CONF_USBD_HS_SP
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_HS_DESCES_LS_FS};
static uint8_t single_desc_bytes_hs[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_HS_DESCES_HS};
//#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ_HS
#else
static uint8_t single_desc_bytes[] = {
    /* Device descriptors and Configuration descriptors list. */
    CDCD_ACM_DESCES_LS_FS};
//#define CDCD_ECHO_BUF_SIZ CONF_USB_CDCD_ACM_DATA_BULKIN_MAXPKSZ
#endif

static struct usbd_descriptors single_desc[]
    = {{single_desc_bytes, single_desc_bytes + sizeof(single_desc_bytes)}
#if CONF_USBD_HS_SP
       ,
       {single_desc_bytes_hs, single_desc_bytes_hs + sizeof(single_desc_bytes_hs)}
#endif
};

// Inertial Sense is_comm_instance_t
is_comm_instance_t comm;

/** Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];

#define CDCD_ECHO_BUF_SIZ	256
#define BUFF_SIZE	2048

uint8_t USBIntBuff[CDCD_ECHO_BUF_SIZ];
uint8_t USBInBuff[BUFF_SIZE];
uint8_t USBOutBuff[BUFF_SIZE];
uint32_t USBOutSize = 0;
int USBInCnt = 0;
bool USBReady = false;

#define NO_DR_READ_SIZE	100
#define DR_READ_SIZE	10
uint8_t spiRxBuff[BUFF_SIZE];
uint8_t spiTxBuff[BUFF_SIZE];
uint8_t spiInBuff[BUFF_SIZE*2];
uint8_t spiOutBuff[BUFF_SIZE*2];
int spiOutBuffIdx = 0;
int spiInBuffIdx = 0;

bool readSPI;

uint32_t g_timeMs = 0;

is_comm_callbacks_t s_callbacks = {};
	
int loadUSBOutBuff(unsigned int port, const unsigned char* buf, int len );

int sdfa = 0;

int parse_isb(unsigned int port, p_data_t* data)
{
	is_comm_write(loadUSBOutBuff, 0, &comm, PKT_TYPE_DATA, data->hdr.id, data->hdr.size, data->hdr.offset, data->ptr);
}

/*********************************************************************************************************************/
/***************************************************** USB BEGIN *****************************************************/
/*********************************************************************************************************************/

/**
 * \brief Callback invoked when bulk OUT data received
 */
static bool usb_device_cb_bulk_in(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	cdcdf_acm_read((uint8_t *)USBIntBuff, sizeof(USBInBuff));
	memcpy(&USBInBuff[USBInCnt],USBIntBuff,count);
	USBInCnt += count;
	USBReady = true;

	/* No error. */
	return false;
}

/**
 * \brief Callback invoked when bulk IN data received
 */
static bool usb_device_cb_bulk_out(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	/* Echo data. */
	//cdcdf_acm_read((uint8_t *)USBInBuff, sizeof(USBInBuff));

	/* No error. */
	return false;
}

/**
 * \brief Callback invoked when Line State Change
 */
static bool usb_device_cb_state_c(usb_cdc_control_signal_t state)
{
	if (state.rs232.DTR) {
		/* Callbacks must be registered after endpoint allocation */
		cdcdf_acm_register_callback(CDCDF_ACM_CB_READ, (FUNC_PTR)usb_device_cb_bulk_in);
		cdcdf_acm_register_callback(CDCDF_ACM_CB_WRITE, (FUNC_PTR)usb_device_cb_bulk_out);
		/* Start Rx */
		cdcdf_acm_read((uint8_t *)USBInBuff, sizeof(USBInBuff));
	}

	/* No error. */
	return false;
}

/**
 * \brief CDC ACM Init
 */
void cdc_device_acm_init(void)
{
	/* usb stack init */
	usbdc_init(ctrl_buffer);

	/* usbdc_register_funcion inside */
	cdcdf_acm_init();

	usbdc_start(single_desc);
	usbdc_attach();
}

void usb_init(void)
{
	cdc_device_acm_init();
}

void checkUSB()
{
	CRITICAL_SECTION_ENTER();
	//cdcdf_acm_read((uint8_t *)USBInBuff, CDCD_ECHO_BUF_SIZ);
	
	if (USBReady)//USBInBuff[0] != 0xff && USBInBuff[0] != 98)
	{
		loadSPIOutBuffer(USBInBuff, USBInCnt);
		
		USBReady = false;
		USBInCnt = 0;
		
		memset(USBInBuff, 0xff, CDCD_ECHO_BUF_SIZ);
	}
	CRITICAL_SECTION_LEAVE();
}


int loadUSBOutBuff(unsigned int port, const unsigned char* buf, int len)
{
	memcpy(&USBOutBuff[USBOutSize], buf, len);
	USBOutSize += len;
}

/*******************************************************************************************************************/
/***************************************************** USB END *****************************************************/
/*******************************************************************************************************************/



/*********************************************************************************************************************/
/***************************************************** SPI BEGIN *****************************************************/
/*********************************************************************************************************************/

struct spi_xfer spi_xfer_data = {
	.txbuf = spiTxBuff,
	.rxbuf = spiRxBuff,
	.size = BUFF_SIZE,
};

uint16_t spiPortWriteBuffSize = 0;

int SPI_0_transfer(uint8_t* buf, uint32_t len)
{
	struct io_descriptor *io;
	spi_m_sync_get_io_descriptor(&SPI_0, &io);

	spi_m_sync_enable(&SPI_0);
	return io_write(io, buf, len);
}

int spiPortWriteCom(unsigned int port, const unsigned char* buf, int len )
{
	if ((len+spiPortWriteBuffSize) < BUFF_SIZE)
	{
		memcpy(&spiTxBuff[spiPortWriteBuffSize], buf, len);
		spiPortWriteBuffSize += len;
	}
}

void SPIReadNoDR()
{
	int readAmt = 0;
	
	// dont send old messages
	spiTxBuff[0] = 0xff;
	spiTxBuff[1] = 0xff;
	spiTxBuff[2] = 0xff;
	
	spi_xfer_data.size = NO_DR_READ_SIZE;
	
	gpio_set_pin_level(SPI_CS, false);
	
	spi_xfer_data.rxbuf = &spiRxBuff[readAmt];
	readAmt += spi_m_sync_transfer(&SPI_0, &spi_xfer_data);
	
	gpio_set_pin_level(SPI_CS, true);
	
	loadSPIInBuffer(spiRxBuff,readAmt);
}

void SPIReadDR()
{
	int readAmt = 0;
	
	// dont send old messages
	spiTxBuff[0] = 0xff;
	spiTxBuff[1] = 0xff;
	spiTxBuff[2] = 0xff;
	
	spi_xfer_data.size = DR_READ_SIZE;

	// while data ready line is active read
	gpio_set_pin_level(SPI_CS, false);
	while ((gpio_get_pin_level(DATA_READY)) && ((readAmt+DR_READ_SIZE) < BUFF_SIZE))
	{
		spi_xfer_data.rxbuf = &spiRxBuff[readAmt];
		readAmt += spi_m_sync_transfer(&SPI_0, &spi_xfer_data);
	}

	// make sure there is still room in buffer
	if ((readAmt+DR_READ_SIZE) < BUFF_SIZE)
	{
		// read one more since data ready will have data read go low one byte early 
		spi_xfer_data.rxbuf = &spiRxBuff[readAmt];
		readAmt += spi_m_sync_transfer(&SPI_0, &spi_xfer_data);
	}

	gpio_set_pin_level(SPI_CS, true);
	
	loadSPIInBuffer(spiRxBuff, readAmt);
}

void readEveryXms_requestDIDs()
{
	if (readSPI)
	{
		spi_xfer_data.size = 150;
		memset(spiTxBuff, 0xff, 150);
		is_comm_get_data(spiPortWriteCom, 0, &comm, DID_INS_1, sizeof(ins_1_t), 0, 0);
		is_comm_get_data(spiPortWriteCom, 0, &comm, DID_PIMU, sizeof(pimu_t), 0, 0);
		spi_xfer_data.rxbuf = spiRxBuff;
		spi_xfer_data.txbuf = spiTxBuff;

		gpio_set_pin_level(SPI_CS, false);
		spi_m_sync_transfer(&SPI_0, &spi_xfer_data);
		gpio_set_pin_level(SPI_CS, true);

		loadSPIInBuffer(spiRxBuff, spi_xfer_data.size);

		spiPortWriteBuffSize = 0;
		readSPI = false;
	}
}

void readEveryXms()
{
	if (readSPI)
	{
		SPIReadNoDR();		
		readSPI = false;
	}
}

void loadSPIOutBuffer(uint8_t* buff, uint32_t size)
{
	for (int i = 0; (i < size) && (spiOutBuffIdx < sizeof(spiOutBuff)); i++,spiOutBuffIdx++)
	{
		spiOutBuff[spiOutBuffIdx] = buff[i];
	}
}

void unloadSpiOutBuff()
{
	if (spiOutBuffIdx > 0)
	{
		memcpy(spiTxBuff, spiOutBuff, spiOutBuffIdx);
		spi_xfer_data.txbuf = spiTxBuff;
		spi_xfer_data.size = spiOutBuffIdx;
		gpio_set_pin_level(SPI_CS, false);
		uint32_t readAmt = spi_m_sync_transfer(&SPI_0, &spi_xfer_data);
		gpio_set_pin_level(SPI_CS, true);
		
		loadSPIInBuffer(spiRxBuff, readAmt);
	}
	
	spiOutBuffIdx = 0;
}

void loadSPIInBuffer(uint8_t* buff, uint32_t size)
{
	for (int i = 0; (i < size) && (spiInBuffIdx < sizeof(spiInBuff)); i++,spiInBuffIdx++)
	{
		spiInBuff[spiInBuffIdx] = buff[i];
	}
}

void unloadSpiInBuff()
{
	if (spiInBuffIdx > 0)
	{
		if (spiInBuff[0] == '$')
		{
			sdfa++;
		}

		#ifdef DIRECT_PASSTHROUGH
			memcpy(USBOutBuff, spiInBuff, spiInBuffIdx);
			USBOutSize = spiInBuffIdx;
		#else		
			is_comm_buffer_parse_messages(spiInBuff, spiInBuffIdx, &comm, &s_callbacks);
		#endif
		
		cdcdf_acm_write((uint8_t *)USBOutBuff, USBOutSize);
	}
	
	spiInBuffIdx=0;
	USBOutSize = 0;
}

/*******************************************************************************************************************/
/***************************************************** SPI END *****************************************************/
/*******************************************************************************************************************/

static struct timer_task msTimer_task1, msTimer_task2;
/**
 * Example of using TIMER_0.
 */
static void msTimer_task1_cb(const struct timer_task *const timer_task)
{
	readSPI = true;
}

static void msTimer_task2_cb(const struct timer_task *const timer_task)
{
	g_timeMs++;
}

void msTimerInit(void)
{
	msTimer_task1.interval = 10;
	msTimer_task1.cb       = msTimer_task1_cb;
	msTimer_task1.mode     = TIMER_TASK_REPEAT;
	msTimer_task2.interval = 1;
	msTimer_task2.cb       = msTimer_task2_cb;
	msTimer_task2.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&TIMER_0, &msTimer_task1);
	timer_add_task(&TIMER_0, &msTimer_task2);
	timer_start(&TIMER_0);
}


/**
* Example of how to use SPI and ISCOMM BUFF
*/
void IS_SPI_Example(void)
{
	// Init comm instance
	uint8_t buffer[2048];
	is_comm_init(&comm, buffer, sizeof(buffer));
	
	msTimerInit();
	
	s_callbacks.isbData = parse_isb;
	s_callbacks.nmea = loadUSBOutBuff;
	
	struct io_descriptor *io;
	
	spi_m_sync_get_io_descriptor(&SPI_0, &io);
	spi_m_sync_enable(&SPI_0);
	
	memset(spiTxBuff, 0xff, BUFF_SIZE);
	
	cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);

	memset(USBOutBuff, 0x00, sizeof(USBOutSize));

	while (1) 
	{
		spi_xfer_data.txbuf = spiTxBuff; 
		
		checkUSB();	
		
		// read more if needed
		if (!(gpio_get_pin_level(MODE_SELECT)))
		{
			SPIReadNoDR();
			//readEveryXms_requestDIDs();
			//readEvery();
		}
		else 
		{
			if (gpio_get_pin_level(DATA_READY))
			{
				// data ready implimentation
				SPIReadDR();
			}
		}
		
		unloadSpiOutBuff();
		unloadSpiInBuff();
	}
}
