/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file or main.c
 * to avoid loosing it when reconfiguring.
 */
#include "atmel_start.h"
#include "usb_start.h"

#include "../../../src/ISComm.h"

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


is_comm_instance_t comm;

/** Ctrl endpoint buffer */
static uint8_t ctrl_buffer[64];

#define READ_ONLY_SIZE	5
uint8_t spiRxBuff[BUFF_SIZE];
uint8_t spiTxBuff[BUFF_SIZE];

uint8_t allZeros[READ_ONLY_SIZE];

int bytesInBuff = 0;
bool readSPI;
uint32_t g_timeMs = 0;

#define CDCD_ECHO_BUF_SIZ	256

uint8_t USBIntBuff[CDCD_ECHO_BUF_SIZ];
uint8_t USBInBuff[BUFF_SIZE];
uint8_t USBOutBuff[BUFF_SIZE];
uint32_t USBReadySetMs = 0;
int USBInCnt = 0;
bool USBReady = false;
bool USBIsInit = false;

uint8_t spiInBuff[BUFF_SIZE*2];
int spiInBuffIdx = 0;

uint8_t spiOutBuff[BUFF_SIZE*2];
int spiOutBuffIdx = 0;

typedef struct parserStats
{
	uint32_t isbPktCount[DID_COUNT];
	uint32_t nmeaPktCount[NMEA_MSG_ID_COUNT];
	uint32_t errorCnt;
}parserStats_t;

parserStats_t stats;
uint32_t tmpPktType = 0;
uint32_t ioobErrCnt = 0;

int parse_isb(unsigned int port, p_data_t* data)
{
	uDatasets* d = (uDatasets*)(data->ptr);
	
	
	cdcdf_acm_write(data->ptr,data->hdr.size);

}

// Handle NMEA messages
int parse_nmea(unsigned int port, const unsigned char* msg, int msgSize)
{
	return cdcdf_acm_write(msg,msgSize);;
}

is_comm_callbacks_t s_callbacks = {};

void parseISPacket()
{
	for (int i = 0; i < spiInBuffIdx; i++)
	{
		switch (is_comm_parse_byte(&comm, spiInBuff[i]))
		{
			case _PTYPE_INERTIAL_SENSE_DATA:
				if (comm.rxPkt.hdr.id < DID_COUNT )
					stats.isbPktCount[comm.rxPkt.hdr.id]++;
				else
					ioobErrCnt++;
				break;

			case _PTYPE_NMEA:
				tmpPktType = getNmeaMsgId(comm.rxPkt.data.ptr, comm.rxPkt.dataHdr.size);
				
				if (tmpPktType < NMEA_MSG_ID_COUNT)
					stats.nmeaPktCount[tmpPktType]++;
				else
					ioobErrCnt++;
				break;
			case _PTYPE_PARSE_ERROR:
				stats.errorCnt++;
				break;
			default:
				break;
		}
	}
}

int SPI_0_transfer(uint8_t* buf, uint32_t len)
{
	struct io_descriptor *io;
	spi_m_sync_get_io_descriptor(&SPI_0, &io);

	spi_m_sync_enable(&SPI_0);
	return io_write(io, buf, len);
}

uint32_t lastSizeChangeMs = 0;
uint32_t lastReadTimeMs = 0;

struct spi_xfer spi_xfer_data = {
	.txbuf = spiTxBuff,
	.rxbuf = spiRxBuff,
	.size = BUFF_SIZE,
};

void SPIReadNoDR()
{
	int readAmt = 0;
	
	spi_xfer_data.size = READ_ONLY_SIZE;
	memset(spiTxBuff, 0xff, READ_ONLY_SIZE);
	readAmt = 0;
	gpio_set_pin_level(SPI_CS, false);
	while (readAmt+READ_ONLY_SIZE < BUFF_SIZE)
	{
		spi_xfer_data.rxbuf = &spiRxBuff[readAmt];
		readAmt += spi_m_sync_transfer(&SPI_0, &spi_xfer_data);
		if (memcmp(&spiRxBuff[readAmt-READ_ONLY_SIZE], allZeros, READ_ONLY_SIZE) == 0)
		break;
	}
	gpio_set_pin_level(SPI_CS, true);
	
	loadSPIInBuffer(spiRxBuff,readAmt);
}

void SPIReadDR()
{
	int readAmt = 0;
	
	spi_xfer_data.size = READ_ONLY_SIZE;
	memset(spiTxBuff, 0xff, READ_ONLY_SIZE);
	readAmt = 0;

	gpio_set_pin_level(SPI_CS, false);
	while ((gpio_get_pin_level(DATA_READY)) && ((readAmt+READ_ONLY_SIZE) < BUFF_SIZE))
	{
		spi_xfer_data.rxbuf = &spiRxBuff[readAmt];
		readAmt += spi_m_sync_transfer(&SPI_0, &spi_xfer_data);
	}

	// make sure there is still room in buffer
	if ((readAmt+READ_ONLY_SIZE) < BUFF_SIZE)
	{
		// read one more since data ready will have data read go low one byte early 
		spi_xfer_data.rxbuf = &spiRxBuff[readAmt];
		readAmt += spi_m_sync_transfer(&SPI_0, &spi_xfer_data);
	}

	gpio_set_pin_level(SPI_CS, true);
	
	loadSPIInBuffer(spiRxBuff, readAmt);
}

uint16_t portWriteBuffSize = 0;

int portWriteCom(unsigned int port, const unsigned char* buf, int len )
{
	if ((len+portWriteBuffSize) < BUFF_SIZE)
	{
		memcpy(&spiTxBuff[portWriteBuffSize], buf, len);
		portWriteBuffSize += len;
	}
}

void readEvery50ms_ZIV()
{
	if (readSPI)
	{
		spi_xfer_data.size = 150;
		memset(spiTxBuff, 0xff, 150);
		is_comm_get_data(portWriteCom, 0, &comm, DID_INS_1, sizeof(ins_1_t), 0, 0);
		is_comm_get_data(portWriteCom, 0, &comm, DID_PIMU, sizeof(pimu_t), 0, 0);
		spi_xfer_data.rxbuf = spiRxBuff;
		spi_xfer_data.txbuf = spiTxBuff;

		gpio_set_pin_level(SPI_CS, false);
		spi_m_sync_transfer(&SPI_0, &spi_xfer_data);
		gpio_set_pin_level(SPI_CS, true);

		loadSPIInBuffer(spiRxBuff, spi_xfer_data.size);

		portWriteBuffSize = 0;
		readSPI = false;
	}
}

int sdfa = 0;

void readEvery()
{
	if (readSPI)
	{
		int readAmt = 0;
		
		spi_xfer_data.size = READ_ONLY_SIZE;
		memset(spiTxBuff, 0xff, READ_ONLY_SIZE);
		readAmt = 0;
		gpio_set_pin_level(SPI_CS, false);
		while (readAmt+READ_ONLY_SIZE < BUFF_SIZE)
		{
			spi_xfer_data.rxbuf = &spiRxBuff[readAmt];
			readAmt += spi_m_sync_transfer(&SPI_0, &spi_xfer_data);
			if (memcmp(&spiRxBuff[readAmt-READ_ONLY_SIZE], allZeros, READ_ONLY_SIZE) == 0)
			break;
		}
		gpio_set_pin_level(SPI_CS, true);
		
		if (spiRxBuff[0] != 0)
		{
			sdfa++;
		}
		
		loadSPIInBuffer(spiRxBuff,readAmt);
		
		readSPI = false;
	}
}

/**
 * \brief Callback invoked when bulk OUT data received
 */
static bool usb_device_cb_bulk_in(const uint8_t ep, const enum usb_xfer_code rc, const uint32_t count)
{
	//cdcdf_acm_write((uint8_t *)USBOutBuff, count);
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

void loadSPIInBuffer(uint8_t* buff, uint32_t size)
{
	for (int i = 0; (i < size) && (spiInBuffIdx < sizeof(spiInBuff)); i++,spiInBuffIdx++)
	{
		spiInBuff[spiInBuffIdx] = buff[i];
	}
}

void loadSPIOutBuffer(uint8_t* buff, uint32_t size)
{
	for (int i = 0; (i < size) && (spiOutBuffIdx < sizeof(spiOutBuff)); i++,spiOutBuffIdx++)
	{
		spiOutBuff[spiOutBuffIdx] = buff[i];
	}
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

void unloadSpiInBuff()
{
	if (spiInBuffIdx > 0)
	{
		//memcpy(USBOutBuff, spiInBuff, spiInBuffIdx);
		//cdcdf_acm_write((uint8_t *)USBOutBuff, spiInBuffIdx);
	
		// SDK?
		//parseISPacket();
		is_comm_buffer_parse_messages(spiInBuff, spiInBuffIdx, &comm, &s_callbacks);
	}
	
	spiInBuffIdx=0;
}

/**
 * Example of using CDC ACM Function.
 * \note
 * In this example, we will use a PC as a USB host:
 * - Connect the DEBUG USB on XPLAINED board to PC for program download.
 * - Connect the TARGET USB on XPLAINED board to PC for running program.
 * The application will behave as a virtual COM.
 * - Open a HyperTerminal or other COM tools in PC side.
 * - Send out a character or string and it will echo the content received.
 */
void cdcd_acm_example(void)
{
	// Init comm instance
	uint8_t buffer[2048];
	is_comm_init(&comm, buffer, sizeof(buffer));
	
	TIMER_0_example();
	
	s_callbacks.isbData = parse_isb;
	s_callbacks.nmea = parse_nmea;
	
	struct io_descriptor *io;
	
	spi_m_sync_get_io_descriptor(&SPI_0, &io);
	spi_m_sync_enable(&SPI_0);
	
	memset(allZeros, 0x00, READ_ONLY_SIZE);
	memset(spiTxBuff, 0xff, BUFF_SIZE);
	
	cdcdf_acm_register_callback(CDCDF_ACM_CB_STATE_C, (FUNC_PTR)usb_device_cb_state_c);

	while (1) 
	{
		spi_xfer_data.txbuf = spiTxBuff; 
		
		checkUSB();	
		
		// read more if needed
		if (gpio_get_pin_level(MODE_SELECT))
		{
			//SPIReadNoDR();
			//readEvery50ms_ZIV();
			readEvery();
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

void usb_init(void)
{
	cdc_device_acm_init();
}
