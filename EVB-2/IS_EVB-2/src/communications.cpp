/*
MIT LICENSE

Copyright 2014-2019 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT, IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <asf.h>
#include "CAN_comm.h"
#include "CAN.h"
#include "d_flash.h"
#include "bootloaderApp.h"
#include "../../../src/convert_ins.h"
#include "../../../src/filters.h"
#include "../../../src/ISEarth.h"
#include "../../../src/ISLogger.h"
#include "../../../src/ISUtilities.h"
#include "../drivers/d_quadEnc.h"
#include "../src/protocol_nmea.h"
#include "ISLogFileFatFs.h"
#include "xbee.h"
#include "wifi.h"
#include "sd_card_logger.h"
#include "user_interface.h"
#include "wheel_encoder.h"
#include "communications.h"

typedef struct
{
	// Comm instances
	is_comm_instance_t comm;

	// Comm instance data buffer
	uint8_t comm_buffer[PKT_BUF_SIZE];

} comm_rx_port_t;


#define COM_RX_PORT_COUNT	(EVB2_PORT_COUNT-1)		// exclude CAN port
comm_rx_port_t              g_comRxPort[COM_RX_PORT_COUNT] = {};
static uint8_t				s_rxDecodeBuffer[PKT_BUF_SIZE] = {};

is_comm_instance_t 			g_commTx = {};

StreamBufferHandle_t        g_xStreamBufferUINS;
StreamBufferHandle_t        g_xStreamBufferWiFiRx;
StreamBufferHandle_t        g_xStreamBufferWiFiTx;
static uint8_t s_xStreamBufferUINS_buf[ STREAM_BUFFER_SIZE ] __attribute__((aligned(4)));
static uint8_t s_xStreamBufferWiFiRx_buf[ STREAM_BUFFER_SIZE ] __attribute__((aligned(4)));
static uint8_t s_xStreamBufferWiFiTx_buf[ STREAM_BUFFER_SIZE ] __attribute__((aligned(4)));
static StaticStreamBuffer_t s_xStreamBufferUINS_struct;
static StaticStreamBuffer_t s_xStreamBufferWiFiRx_struct;
static StaticStreamBuffer_t s_xStreamBufferWiFiTx_struct;

static pfnHandleUinsData s_pfnHandleUinsData = NULLPTR;
static pfnHandleHostData s_pfnHandleHostData = NULLPTR;
static pfnHandleBroadcst s_pfnHandleBroadcst = NULLPTR;
static pfnHandleDid2Ermc s_pfnHandleDid2Ermc = NULLPTR;


int comWrite(int serialNum, const unsigned char *buf, int size, uint32_t ledPin )
{
    int len;
    if (serialNum == EVB2_PORT_UINS1 && g_flashCfg->cbOptions&EVB2_CB_OPTIONS_SPI_ENABLE)
    {
        len = spiTouINS_serWrite(buf, size);
    }
    else
    {
        len = serWrite(serialNum, buf, size);
    }
    
    if(len)
    {
        LED_ON(ledPin);
    }
    return len;
}

int comRead(int serialNum, unsigned char *buf, int bufSize, uint32_t ledPin)
{
    int len = 0;
	
    if (serialNum == EVB2_PORT_UINS1 && g_flashCfg->cbOptions&EVB2_CB_OPTIONS_SPI_ENABLE)
    {
        len = spiTouINS_serRead(buf, bufSize);
    }
    else
    {
#if 0	// Wait for end of data.  Check Rx buffer has data and hasn't change over 1ms. 			
		static int usedLast[EVB2_PORT_COUNT] = {0};
		int used = serRxUsed(serialNum);
		if(used!=0 && used==usedLast[serialNum])
		{	// Data in Rx buffer and amount hasn't changed.
			len = serRead(serialNum, buf, bufSize, 0);
		}
		usedLast[serialNum] = used;		
#else
		// Normal read	
        len = serRead(serialNum, buf, bufSize);
#endif
    }
    
    if(len)
    {
        LED_ON(ledPin);
    }
    return len;
}

void callback_cdc_set_config(uint8_t port, usb_cdc_line_coding_t * cfg)
{
    UNUSED(port);
#if 0
    uint32_t stopbits, parity, databits;
//     uint32_t imr;

    switch (cfg->bCharFormat)
    {
    case CDC_STOP_BITS_2:
        stopbits = US_MR_NBSTOP_2_BIT;
        break;
    case CDC_STOP_BITS_1_5:
        stopbits = US_MR_NBSTOP_1_5_BIT;
        break;
    case CDC_STOP_BITS_1:
        default:
        // Default stop bit = 1 stop bit
        stopbits = US_MR_NBSTOP_1_BIT;
        break;
    }

    switch (cfg->bParityType)
    {
    case CDC_PAR_EVEN:
        parity = US_MR_PAR_EVEN;
        break;
    case CDC_PAR_ODD:
        parity = US_MR_PAR_ODD;
        break;
    case CDC_PAR_MARK:
        parity = US_MR_PAR_MARK;
        break;
    case CDC_PAR_SPACE:
        parity = US_MR_PAR_SPACE;
        break;
        default:
    case CDC_PAR_NONE:
        parity = US_MR_PAR_NO;
        break;
    }
    
    switch(cfg->bDataBits)
    {
    case 5:
    case 6:
    case 7:
        databits = cfg->bDataBits - 5;
        break;
        default:
    case 8:
        databits = US_MR_CHRL_8_BIT;
        break;
    }

    // Options for USART.  This gets called when USB is first connected AND each time the USB CDC serial port is opened by the host.
    //  sam_usart_opt_t usart_options;
    // 	usart_options.baudrate = LE32_TO_CPU(cfg->dwDTERate);
    // 	usart_options.char_length = databits;
    // 	usart_options.parity_type = parity;
    // 	usart_options.stop_bits = stopbits;
    // 	usart_options.channel_mode = US_MR_CHMODE_NORMAL;
#endif
	
    uint32_t baudrate = LE32_TO_CPU(cfg->dwDTERate);
    if (comManagerValidateBaudRate(baudrate)==0)
    {
        // Set baudrate based on USB CDC baudrate
        if (g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_UINS0))
        {
            serSyncBaudRate(EVB2_PORT_UINS0, &baudrate);
        }

        if (g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_UINS1) &&
		  !(g_flashCfg->cbOptions&EVB2_CB_OPTIONS_SPI_ENABLE))
        {
            serSyncBaudRate(EVB2_PORT_UINS1, &baudrate);
        }
    }

    if (comManagerValidateBaudRate(baudrate)==0 || baudrate==9600)
    {
        // 	    if(g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_XBEE))
        // 	    {
        // 		    serSyncBaudRate(EVB2_PORT_XBEE, &baudrate);
        // 	    }
        if (g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_XRADIO))
        {
            serSyncBaudRate(EVB2_PORT_XRADIO, &baudrate);
        }
        if (g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_BLE))
        {
            serSyncBaudRate(EVB2_PORT_BLE, &baudrate);
        }
        if (g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_SP330))
        {
            serSyncBaudRate(EVB2_PORT_SP330, &baudrate);
        }
        if (g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_GPIO_H8))
        {
            serSyncBaudRate(EVB2_PORT_GPIO_H8, &baudrate);
        }
    }
}

void callback_cdc_set_dtr(uint8_t port, bool b_enable)
{
    if (b_enable)
    {	// Host terminal has open COM
		d_usartDMA_callback_cdc_enable();
    }
    else
    {	// Host terminal has close COM
		d_usartDMA_callback_cdc_disable();
    }

    if(g_flashCfg->cbf[EVB2_PORT_USB] & (1<<EVB2_PORT_XBEE))
    {
        if (b_enable)
        {	// Assert (LOW) DTR
            ioport_set_pin_level(UART_XBEE_N_DTR_PIN, IOPORT_PIN_LEVEL_LOW);      // Assert LOW
            ioport_set_pin_level(UART_XBEE_N_RTS_PIN, IOPORT_PIN_LEVEL_LOW);      // Assert LOW
        }
        else
        {	// De-assert (HIGH) DTR
            ioport_set_pin_level(UART_XBEE_N_DTR_PIN, IOPORT_PIN_LEVEL_HIGH);     // De-assert HIGH
            ioport_set_pin_level(UART_XBEE_N_RTS_PIN, IOPORT_PIN_LEVEL_HIGH);     // De-assert HIGH
        }
    }
}

// This function never gets called because of a bug in Atmel CDC driver.  Bummer.
// void callback_cdc_set_rts(uint8_t port, bool b_enable)
// {
// 	switch(g_uins.evb.comBridgeCfg)
// 	{
//     case EVB2_CBC_USBxXBEE:
//         if (b_enable)
//         {
//             ioport_set_pin_level(UART_XBEE_N_RTS_PIN, IOPORT_PIN_LEVEL_HIGH);
//         }
//         else
//         {
//             ioport_set_pin_level(UART_XBEE_N_RTS_PIN, IOPORT_PIN_LEVEL_LOW);
//         }
//         break;
//
//     case EVB2_CBC_RS232xUSB:
//     case EVB2_CBC_RS422xUSB:
//         break;
//     }
// }


void uINS_stream_stop_all(void)
{
    int len = is_comm_stop_broadcasts_current_port(&g_commTx);
    comWrite(g_flashCfg->uinsComPort, g_commTx.buf.start, len, LED_INS_TXD_PIN);
}

void uINS_stream_enable_std(void)
{
    int len;
    
    len = is_comm_get_data(&g_commTx, DID_INS_2, 0, 0, 10);       // 20 x 4ms = 40ms
    comWrite(g_flashCfg->uinsComPort, g_commTx.buf.start, len, LED_INS_TXD_PIN);

    len = is_comm_get_data(&g_commTx, DID_DEV_INFO, 0, 0, 500);   // 500ms
    comWrite(g_flashCfg->uinsComPort, g_commTx.buf.start, len, LED_INS_TXD_PIN);
}

void uINS_stream_enable_PPD(void)
{
    rmc_t rmc;
    rmc.bits = RMC_PRESET_PPD_GROUND_VEHICLE;
    rmc.options = 0;
    int len = is_comm_set_data(&g_commTx, DID_RMC, 0, sizeof(rmc_t), &rmc);
    comWrite(g_flashCfg->uinsComPort, g_commTx.buf.start, len, LED_INS_TXD_PIN);

//     len = is_comm_get_data(&comm, DID_INS_2, 0, 0, 1);       // 1 x 4ms = 4ms
//     comWrite(EVB2_PORT_UINS0, comm.buffer, len, LED_INS_TXD_PIN);
}


void time_sync_evb_from_uINS(uint32_t week,	double timeOfWeek)
{
	g_towOffset = timeOfWeek - time_seclf();

	g_status.week = week;
	g_statusToWlocal = false;
	g_status.timeOfWeekMs = (uint32_t)round(timeOfWeek*1000.0);
	g_status.towOffset = g_towOffset;
}


void handle_data_from_uINS(p_data_hdr_t &dataHdr, uint8_t *data)
{
	uDatasets d = {0};

	if( copyDataPToStructP2(&d, &dataHdr, data, sizeof(uDatasets)) )
	{	// Invalid
		return;
	}

	// Save uINS data to global variables
	switch(dataHdr.id)
	{
	case DID_DEV_INFO:
		copyDataPToStructP2(&g_uins.uInsInfo, &dataHdr, data, sizeof(dev_info_t));
		break;

	case DID_INS_1:
		if(dataHdr.size+dataHdr.offset > sizeof(ins_1_t)){ /* Invalid */ return; }
		time_sync_evb_from_uINS(d.ins1.week, d.ins1.timeOfWeek);
		g_uins.ins1 = d.ins1;
		convertIns1ToIns2(&d.ins1, &g_uins.ins2);
		g_insUpdateTimeMs = g_comm_time_ms;
		break;
	                    
	case DID_INS_2:
		if(dataHdr.size+dataHdr.offset > sizeof(ins_2_t)){ /* Invalid */ return; }
		time_sync_evb_from_uINS(d.ins2.week, d.ins2.timeOfWeek);
		g_uins.ins2 = d.ins2;
		if(g_uins.refLlaValid)
		{
			convertIns2ToIns1(&d.ins2, &g_uins.ins1, g_uins.flashCfg.refLla);
		}
		else
		{
			convertIns2ToIns1(&d.ins2, &g_uins.ins1);
		}
		g_insUpdateTimeMs = g_comm_time_ms;
		break;

	case DID_INS_3:
		if(dataHdr.size+dataHdr.offset > sizeof(ins_3_t)){ /* Invalid */ return; }
		time_sync_evb_from_uINS(d.ins3.week, d.ins3.timeOfWeek);
		break;

	case DID_INS_4:
		if(dataHdr.size+dataHdr.offset > sizeof(ins_4_t)){ /* Invalid */ return; }
		time_sync_evb_from_uINS(d.ins4.week, d.ins4.timeOfWeek);
		break;

	case DID_INL2_STATES:
		if(dataHdr.size+dataHdr.offset > sizeof(inl2_states_t)){ /* Invalid */ return; }
		g_uins.inl2States = d.inl2States;
		break;

	case DID_PIMU:
		if(dataHdr.size+dataHdr.offset > sizeof(pimu_t)){ /* Invalid */ return; }
		g_uins.pImu = d.pImu;
		preintegratedImuToIMU(&(g_imu), &(g_uins.pImu));
		sub_Vec3_Vec3(g_imu.I.pqr, g_imu.I.pqr, g_uins.inl2States.biasPqr);	// Subtract EKF bias estimates
		sub_Vec3_Vec3(g_imu.I.acc, g_imu.I.acc, g_uins.inl2States.biasAcc);
		g_imuUpdateTimeMs = g_comm_time_ms;
		break;

	case DID_FLASH_CONFIG:	// uINS
		if(dataHdr.size+dataHdr.offset > sizeof(nvm_flash_cfg_t)){ /* Invalid */ return; }
		g_uins.flashCfg = d.flashCfg;
		g_uins.refLlaValid = (bool)llaDegValid(g_uins.flashCfg.refLla);
		break;
	}
	
	// Pass to callback
	if(s_pfnHandleUinsData)
	{
		s_pfnHandleUinsData(dataHdr, d);
	}
}


// Convert DID to message out control mask
uint64_t evbDidToErmcBit(uint32_t dataId)
{
	switch (dataId)
	{
		case DID_EVB_DEV_INFO:			return ERMC_BITS_DEV_INFO;
		case DID_EVB_FLASH_CFG:			return ERMC_BITS_FLASH_CFG;
		case DID_EVB_STATUS:			return ERMC_BITS_STATUS;
		case DID_EVB_DEBUG_ARRAY:       return ERMC_BITS_DEBUG_ARRAY;
		case DID_WHEEL_ENCODER:         return ERMC_BITS_WHEEL_ENCODER;
        default:                        break;
	}

	if (s_pfnHandleDid2Ermc)
	{	// Pass to callback
		return s_pfnHandleDid2Ermc(dataId);
	}

	return 0;
}


void broadcastRmcMessage(is_comm_instance_t *comm, uint32_t did, uint32_t size, void* data, uint32_t &time_ms, uint32_t didSendNow)
{
	if ((g_ermc.periodMultiple[did] &&
		 g_ermc.bits & evbDidToErmcBit(did) && 
		 (g_comm_time_ms-time_ms) > g_ermc.periodMultiple[did]) ||
		did == didSendNow)
	{
		time_ms = g_comm_time_ms;
		int n = is_comm_data(comm, did, 0, size, data);
		serWrite(EVB2_PORT_USB, comm->buf.start, n);
	}
}


void step_broadcast_data(is_comm_instance_t *comm, uint32_t didSendNow)
{
	static uint32_t time_ms_dev_info;
	static uint32_t time_ms_flash_cfg;
	static uint32_t time_ms_rtos;
	static uint32_t time_ms_status;
	static uint32_t time_ms_debug;
	
	// Broadcast messages
	broadcastRmcMessage(comm, DID_EVB_DEV_INFO, sizeof(dev_info_t), (void*)&g_evbDevInfo, time_ms_dev_info, didSendNow);
	broadcastRmcMessage(comm, DID_EVB_FLASH_CFG, sizeof(evb_flash_cfg_t), (void*)g_flashCfg, time_ms_flash_cfg, didSendNow);
	broadcastRmcMessage(comm, DID_EVB_RTOS_INFO, sizeof(evb_rtos_info_t), (void*)&g_rtos, time_ms_rtos, didSendNow);
	broadcastRmcMessage(comm, DID_EVB_STATUS, sizeof(evb_status_t), (void*)&g_status, time_ms_status, didSendNow);
	broadcastRmcMessage(comm, DID_EVB_DEBUG_ARRAY, sizeof(debug_array_t), (void*)&g_debug, time_ms_debug, didSendNow);

	switch (didSendNow)
	{
	case DID_EVB_RTOS_INFO:	g_enRtosStats = true; break;
	}

	if (s_pfnHandleBroadcst)
	{	// Pass to callback
		s_pfnHandleBroadcst(comm, didSendNow);
	}
}


void log_uINS_data(cISLogger &logger, is_comm_instance_t &comm)
{
	is_evb_log_stream stm = {};
	uint8_t data[MAX_DATASET_SIZE];
	p_data_hdr_t dataHdr = {};
	
    while (xStreamBufferReceive(g_xStreamBufferUINS, (void*)&stm, sizeof(is_evb_log_stream), 0) == sizeof(is_evb_log_stream))
    {	
		if (stm.marker==DATA_CHUNK_MARKER)
		{	
			switch (stm.ptype)
			{
			case _PTYPE_INERTIAL_SENSE_DATA:
				if (xStreamBufferReceive(g_xStreamBufferUINS, (void*)&dataHdr, sizeof(p_data_hdr_t), 0) == sizeof(p_data_hdr_t))
				{
					if (dataHdr.id < DID_COUNT && dataHdr.size < MAX_DATASET_SIZE)
					{
						if (xStreamBufferReceive(g_xStreamBufferUINS, (void*)data, dataHdr.size, 0) == dataHdr.size)
						{
							// Log Inertial Sense Binary data to SD Card
							if(g_loggerEnabled)
							{
								logger.LogData(0, &dataHdr, data);
							}
						}
					}	
				}
				break;
				
			case _PTYPE_NMEA:
				break;

			case _PTYPE_UBLOX:
#if UBLOX_LOG_ENABLE
				if (xStreamBufferReceive(g_xStreamBufferUINS, (void*)&data, stm.size, 0) == stm.size)
				{
extern void log_ublox_raw_to_SD(cISLogger& logger, uint8_t *dataPtr, uint32_t dataSize);
					log_ublox_raw_to_SD(logger, data, stm.size);
				}
#endif
				break;
			}	
		}
	}
}


void update_flash_cfg(evb_flash_cfg_t &newCfg)
{
    if(error_check_evb_flash_cfg(&newCfg))
    {
        return;
    }
    
    // Data is identical, no changes.
	if( !memcmp(&newCfg, g_flashCfg, sizeof(evb_flash_cfg_t)) )
    {
		return;
    }        

    bool initCbPreset = false;
    bool initIOconfig = false;
    bool reinitXBee = false;
    bool reinitWiFi = false;
	bool reinitCAN = false;
	bool reinitWheelEncoder = false;

    // Detect changes
    if (newCfg.cbPreset != g_flashCfg->cbPreset ||
		newCfg.uinsComPort != g_flashCfg->uinsComPort)
    {
        initCbPreset = true;
        initIOconfig = true;
    }
    if (newCfg.radioPID != g_flashCfg->radioPID ||
        newCfg.radioNID != g_flashCfg->radioNID ||
        newCfg.radioPowerLevel != g_flashCfg->radioPowerLevel)
    {
        reinitXBee = true;
    }    
    if (EVB_CFG_BITS_IDX_WIFI(newCfg.bits) != EVB_CFG_BITS_IDX_WIFI(g_flashCfg->bits) ||
        EVB_CFG_BITS_IDX_SERVER(newCfg.bits) != EVB_CFG_BITS_IDX_SERVER(g_flashCfg->bits))
    {   // WiFi or TCP server preset changed
        reinitWiFi = true;
    }  
    int i = EVB_CFG_BITS_IDX_WIFI(newCfg.bits);        
    if (strncmp((const char*)(newCfg.wifi[i].ssid), (const char*)(g_flashCfg->wifi[i].ssid), WIFI_SSID_PSK_SIZE)!=0 ||
        strncmp((const char*)(newCfg.wifi[i].psk),  (const char*)(g_flashCfg->wifi[i].psk),  WIFI_SSID_PSK_SIZE)!=0 ||
        newCfg.server[i].ipAddr.u32 != g_flashCfg->server[i].ipAddr.u32 ||
        newCfg.server[i].port		!= g_flashCfg->server[i].port)
    {   // WiFi or TCP settings changed
        reinitWiFi = true;            
    }
    i = EVB_CFG_BITS_IDX_SERVER(newCfg.bits);
    if (newCfg.server[i].ipAddr.u32 != g_flashCfg->server[i].ipAddr.u32 ||
        newCfg.server[i].port		!= g_flashCfg->server[i].port)
    {   // TCP settings changed
        reinitWiFi = true;
    }
    if ((newCfg.cbOptions&EVB2_CB_OPTIONS_XBEE_ENABLE) != (g_flashCfg->cbOptions&EVB2_CB_OPTIONS_XBEE_ENABLE) ||
        (newCfg.cbOptions&EVB2_CB_OPTIONS_WIFI_ENABLE) != (g_flashCfg->cbOptions&EVB2_CB_OPTIONS_WIFI_ENABLE) ||
		(newCfg.h3sp330BaudRate != g_flashCfg->h3sp330BaudRate) ||
		(newCfg.h4xRadioBaudRate != g_flashCfg->h4xRadioBaudRate) ||
		(newCfg.h8gpioBaudRate != g_flashCfg->h8gpioBaudRate) )
    {
        initIOconfig = true;
    }
	if (g_flashCfg->CANbaud_kbps != newCfg.CANbaud_kbps)
	{
		g_flashCfg->CANbaud_kbps = newCfg.CANbaud_kbps;
		reinitCAN = true;
	}
	if (g_flashCfg->can_receive_address != newCfg.can_receive_address)
	{
		g_flashCfg->can_receive_address = newCfg.can_receive_address;
		reinitCAN = true;
	}
	if (g_flashCfg->wheelCfgBits != newCfg.wheelCfgBits)
	{
		reinitWheelEncoder = true;
		initIOconfig = true;
	}
    
    // Copy data from message to working location
    *g_flashCfg = newCfg;
    
    // Apply changes
    if(initCbPreset)
    {
        com_bridge_apply_preset(g_flashCfg);
    }
    if(initIOconfig)
    {	// Update EVB IO config
        board_IO_config();
    }
    if(reinitXBee)
    {
        xbee_init();
    }
    if(reinitWiFi)
    {
        wifi_reinit();
    }
	if (reinitCAN)
	{
		CAN_init(g_flashCfg->CANbaud_kbps*1000, g_flashCfg->can_receive_address);
	}
	if (reinitWheelEncoder)
	{
		init_wheel_encoder();
	}
	evbUiRefreshLedCfg();
    
	// Enable flash write
	nvr_flash_config_write_needed();
	nvr_flash_config_write_enable();
}


void handle_data_from_host(is_comm_instance_t *comm, protocol_type_t ptype, uint32_t srcPort)
{
	uint8_t *dataPtr = comm->dataPtr + comm->dataHdr.offset;
	static uint8_t manfUnlock = false;

	switch(ptype)
	{
	case _PTYPE_INERTIAL_SENSE_DATA:
		switch(comm->dataHdr.id)
		{	// From host to EVB
		case DID_EVB_STATUS:
			is_comm_copy_to_struct(&g_status, comm, sizeof(evb_status_t));

			switch (g_status.sysCommand)
			{
			case SYS_CMD_SOFTWARE_RESET:			// Reset processor
				soft_reset_backup_register(SYS_FAULT_STATUS_USER_RESET);
				break;

			case SYS_CMD_MANF_UNLOCK:				// Unlock process for chip erase
				manfUnlock = true;
				g_status.evbStatus |= EVB_STATUS_MANF_UNLOCKED;
				break;

			case SYS_CMD_MANF_ENABLE_ROM_BOOTLOADER:	// reboot into ROM bootloader mode
				if(manfUnlock)
				{
					// Set "stay_in_bootloader" signature to require app firmware update following bootloader update.
					write_bootloader_signature_stay_in_bootloader_mode();   

					BEGIN_CRITICAL_SECTION
					flash_rom_bootloader();
					while(1);
					END_CRITICAL_SECTION
				}
				break;

			case SYS_CMD_MANF_CHIP_ERASE:		            // chip erase and reboot
				if(manfUnlock)
				{
					BEGIN_CRITICAL_SECTION
					flash_erase_chip();
					while(1);
					END_CRITICAL_SECTION
				}
				break;
			}
			g_status.sysCommand = 0;
			break;
				
		case DID_EVB_FLASH_CFG:
			evb_flash_cfg_t newCfg;
			newCfg = *g_flashCfg;
			is_comm_copy_to_struct(&newCfg, comm, sizeof(evb_flash_cfg_t));
				
			update_flash_cfg(newCfg);				
			break;
				
		case DID_EVB_DEBUG_ARRAY:
			is_comm_copy_to_struct(&g_debug, comm, sizeof(debug_array_t));
			break;
		}

		// Disable uINS bootloader if host sends IS binary data
		g_uInsBootloaderEnableTimeMs = 0;
		break;

	case _PTYPE_INERTIAL_SENSE_CMD:
		switch(comm->pkt.hdr.pid)
		{
		case PID_GET_DATA:
			// Set ERMC broadcast control bits
			setErmcBroadcastBits(comm, srcPort, evbDidToErmcBit(comm->dataHdr.id));

			// Send data now
			step_broadcast_data(comm, comm->dataHdr.id);

			// Disable uINS bootloader mode if host sends IS binary command
			g_uInsBootloaderEnableTimeMs = 0;
			break; // PID_GET_DATA

		// case PID_SET_DATA:
		// 	if(comm->dataHdr.id == DID_RMC)
		// 	{
		// 		g_ermc.bits = *((uint64_t*)(comm->dataPtr));	// RMC is not the same as ERMC (EVB RMC).  We may need to translate this if necessary.
		// 	}
			// break;

		case PID_STOP_BROADCASTS_ALL_PORTS:
		case PID_STOP_DID_BROADCAST:
		case PID_STOP_BROADCASTS_CURRENT_PORT:
			// Disable EVB broadcasts
			g_ermc.bits = 0;
			break;
		}
		break;

	case _PTYPE_NMEA:
		{
			if(comm->dataHdr.size == 10)
			{	// 4 character commands (i.e. "$STPB*14\r\n")
				switch (getNmeaMsgId(dataPtr, comm->dataHdr.size))
				{
				case NMEA_MSG_ID_BLEN: // Enable bootloader (uINS)
					g_uInsBootloaderEnableTimeMs = g_comm_time_ms;

					// Disable EVB broadcasts
					g_ermc.bits = 0;
					break;
							
				case NMEA_MSG_ID_EBLE: // Enable bootloader (EVB)
					// Disable uINS bootloader if host enables EVB bootloader
					g_uInsBootloaderEnableTimeMs = 0;
					
					enable_bootloader(PORT_SEL_USB);
					break;				

				case NMEA_MSG_ID_STPB:
				case NMEA_MSG_ID_STPC:	
					// Disable EVB communications
					g_ermc.bits = 0;
					break;
				}
				break;							
			}
			else
			{	// General NMEA							
				switch (getNmeaMsgId(dataPtr, comm->dataHdr.size))
				{
				case NMEA_MSG_ID_NELB: // SAM bootloader assistant (SAM-BA) enable
					if (comm->dataHdr.size == 22 &&
// 									(pHandle == EVB2_PORT_USB) && 
						strncmp((const char*)(&(comm->buf.start[6])), "!!SAM-BA!!", 6) == 0)
					{	// 16 character commands (i.e. "$NELB,!!SAM-BA!!\0*58\r\n")
						enable_rom_bootloader();
					}
					break;
					
				default:
					// Disable uINS bootloader if host sends larger NMEA sentence
					g_uInsBootloaderEnableTimeMs = 0;
					break;
				}				
			}
		}
		break;
	}
	
	if (s_pfnHandleHostData)
	{	// Pass to callback
		s_pfnHandleHostData(comm, ptype, srcPort);
	}
}


// Set ERMC broadcast control bits
void setErmcBroadcastBits(is_comm_instance_t *comm, uint32_t srcPort, uint64_t bits)
{
	uint32_t bc_period_multiple = *((int32_t*)(comm->dataPtr));
	if(bc_period_multiple)
	{	// Enable message
		g_ermc.bits |= bits;
	}
	else
	{	// Disable message
		g_ermc.bits &= ~bits;
		bc_period_multiple = 0;
	}
	if (comm->dataHdr.id < sizeof(g_ermc.periodMultiple))
	{
		g_ermc.periodMultiple[comm->dataHdr.id] = bc_period_multiple;
	}
}


#define CAN_COM_PORT 1
#define CAN_HDR  0xFC;
#define CAN_FTR  0xFE;

void sendRadio(uint8_t *data, int dataSize, bool sendXbee, bool sendXrad)
{
	if(g_flashCfg->portOptions == EVB2_PORT_OPTIONS_RADIO_RTK_FILTER)
	{	// Filter RTK Base Messages
		protocol_type_t ptype;

		static is_comm_instance_t comm = {};
		static uint8_t buffer[PKT_BUF_SIZE];
		if (comm.buf.start == NULL)
		{	// Init buffer
			is_comm_init(&comm, buffer, sizeof(buffer));
		}

		for(uint8_t *ptr=data; dataSize>0; ) 
		{
			// Number of bytes to copy
			int n = _MIN(dataSize, is_comm_free(&comm));

			// Copy data into buffer
			memcpy(comm.buf.tail, ptr, n);
			comm.buf.tail += n;
			dataSize -= n;
			ptr += n;
		
			while((ptype = is_comm_parse_timeout(&comm, g_comm_time_ms)) != _PTYPE_NONE)
			{			
				// Parse Data
				switch(ptype)
				{
				case _PTYPE_UBLOX:
				case _PTYPE_RTCM3:
				case _PTYPE_NMEA:
					if(sendXbee){ comWrite(EVB2_PORT_XBEE, comm.dataPtr, comm.dataHdr.size, LED_XBEE_TXD_PIN); }
					if(sendXrad){ comWrite(EVB2_PORT_XRADIO, comm.dataPtr, comm.dataHdr.size, 0); }
					break;
				}
			}
		}
	}
	else
	{
		if(sendXbee){ comWrite(EVB2_PORT_XBEE, data, dataSize, LED_XBEE_TXD_PIN); }
		if(sendXrad){ comWrite(EVB2_PORT_XRADIO, data, dataSize, 0); }		
	}		
}


// This function only forwards data after complete valid packets are received 
void com_bridge_smart_forward(uint32_t srcPort, uint32_t ledPin)
{	
	if(srcPort>=COM_RX_PORT_COUNT)
	{
		return;
	}
	
	is_comm_instance_t &comm = g_comRxPort[srcPort].comm;

	// Get available size of comm buffer
	int n = is_comm_free(&comm);

	if ((n = comRead(srcPort, comm.buf.tail, n, ledPin)) > 0)
	{
		if (g_flashCfg->cbPreset == EVB2_CB_PRESET_USB_HUB_RS422)
		{
			com_bridge_forward(srcPort, comm.buf.head, n);
			return;
		}
		if (g_uInsBootloaderEnableTimeMs)
		{	// When uINS bootloader is enabled forwarding is disabled below is_comm_parse(), so forward bootloader data here.
			switch (srcPort)
			{
				case EVB2_PORT_USB:		comWrite(EVB2_PORT_UINS0, comm.buf.tail, n, LED_INS_TXD_PIN);	break;
				case EVB2_PORT_UINS0:	comWrite(EVB2_PORT_USB, comm.buf.tail, n, 0);					break;
			}					
		}
		
		// Update comm buffer tail pointer
		comm.buf.tail += n;
				
		// Search comm buffer for valid packets
		protocol_type_t ptype;
		while((ptype = is_comm_parse_timeout(&comm, g_comm_time_ms)) != _PTYPE_NONE)
		{
			switch(srcPort)
			{
			case EVB2_PORT_USB:
			case EVB2_PORT_XBEE:
			case EVB2_PORT_SP330:
				handle_data_from_host(&comm, ptype, srcPort);
				break;
			}			

			if (ptype!=_PTYPE_NONE && 
				ptype!=_PTYPE_PARSE_ERROR &&
				g_uInsBootloaderEnableTimeMs==0)
			{	// Forward data
				uint32_t pktSize = _MIN(comm.buf.scan - comm.pktPtr, PKT_BUF_SIZE);
				com_bridge_forward(srcPort, comm.pktPtr, pktSize);

				// Send uINS data to Logging task
				if (srcPort == g_flashCfg->uinsComPort && pktSize > 0)
				{
					is_evb_log_stream stm;
					stm.marker = DATA_CHUNK_MARKER;
					stm.ptype = ptype;
					switch (ptype)
					{
					case _PTYPE_INERTIAL_SENSE_DATA:
						if (comm.dataHdr.size > 0)
						{
							handle_data_from_uINS(comm.dataHdr, comm.dataPtr);
					
							stm.size = sizeof(p_data_hdr_t) + comm.dataHdr.size;
							xStreamBufferSend(g_xStreamBufferUINS, (void*)&(stm), sizeof(is_evb_log_stream), 0);
							xStreamBufferSend(g_xStreamBufferUINS, (void*)&(comm.dataHdr), sizeof(p_data_hdr_t), 0);
							xStreamBufferSend(g_xStreamBufferUINS, (void*)(comm.dataPtr), comm.dataHdr.size, 0);							
						}
						break;
					case _PTYPE_UBLOX:
#if UBLOX_LOG_ENABLE
						stm.size = pktSize;
						xStreamBufferSend(g_xStreamBufferUINS, (void*)&(stm), sizeof(is_evb_log_stream), 0);
						xStreamBufferSend(g_xStreamBufferUINS, (void*)(comm.pktPtr), pktSize, 0);
#endif
						break;
					}
				}

			}
		}
	}	
}


void com_bridge_smart_forward_xstream(uint32_t srcPort, StreamBufferHandle_t xStreamBuffer);
void com_bridge_smart_forward_xstream(uint32_t srcPort, StreamBufferHandle_t xStreamBuffer)
{	
	if(srcPort>=COM_RX_PORT_COUNT)
	{
		return;
	}
	
	is_comm_instance_t &comm = g_comRxPort[srcPort].comm;

	// Get available size of comm buffer
	int n = is_comm_free(&comm);

	if ((n = xStreamBufferReceive(xStreamBuffer, comm.buf.tail, n, 0)) > 0)
	{
		// Update comm buffer tail pointer
		comm.buf.tail += n;
		
		// Search comm buffer for valid packets
		protocol_type_t ptype;
		while ((ptype = is_comm_parse_timeout(&comm, g_comm_time_ms)) != _PTYPE_NONE)
		{
			if (srcPort == EVB2_PORT_USB)
			{
				handle_data_from_host(&comm, ptype, srcPort);
			}
			
			if (ptype!=_PTYPE_NONE && 
				ptype!=_PTYPE_PARSE_ERROR &&
				g_uInsBootloaderEnableTimeMs==0)
			{	// Forward data
				uint32_t pktSize = _MIN(comm.buf.scan - comm.pktPtr, PKT_BUF_SIZE);
				com_bridge_forward(srcPort, comm.pktPtr, pktSize);
			}
		}
	}
}


void com_bridge_forward(uint32_t srcPort, uint8_t *buf, int len)
{
    uint32_t dstCbf = g_flashCfg->cbf[srcPort];
    
    if(dstCbf == 0 || len==0)    // None
    {
        return;
    }        

	if (g_uInsBootloaderEnableTimeMs==0)
	{	// uINS bootloader mode enabled - don't allow forwarding on these ports
		if(dstCbf & (1<<EVB2_PORT_USB))
		{
			comWrite(EVB2_PORT_USB, buf, len, 0);
		}
    
		if(dstCbf & (1<<EVB2_PORT_UINS0))
		{
			comWrite(EVB2_PORT_UINS0, buf, len, LED_INS_TXD_PIN);
		}		
	}

    if(dstCbf & (1<<EVB2_PORT_UINS1))
    {
        comWrite(EVB2_PORT_UINS1, buf, len, LED_INS_TXD_PIN);
    }
	
	bool sendXbee   = dstCbf & (1<<EVB2_PORT_XBEE) && xbee_runtime_mode();	// Disable XBee communications when being configured
	bool sendXradio = dstCbf & (1<<EVB2_PORT_XRADIO);
	if(sendXbee || sendXradio)
	{
		sendRadio(buf, len, sendXbee, sendXradio);
	}

    if(dstCbf & (1<<EVB2_PORT_BLE))
    {
        comWrite(EVB2_PORT_BLE, buf, len, LED_WIFI_TXD_PIN);
    }

    if(dstCbf & (1<<EVB2_PORT_SP330))
    {
        comWrite(EVB2_PORT_SP330, buf, len, g_flashCfg->uinsComPort==EVB2_PORT_SP330?LED_INS_TXD_PIN:0);
    }

    if(dstCbf & (1<<EVB2_PORT_GPIO_H8))
    {
        comWrite(EVB2_PORT_GPIO_H8, buf, len, g_flashCfg->uinsComPort==EVB2_PORT_GPIO_H8?LED_INS_TXD_PIN:0);
    }

#if 0   // Disabled when forwarding data directly in wifi task
    if(dstCbf & (1<<EVB2_PORT_WIFI))
    {
        xStreamBufferSend(g_xStreamBufferWiFiTx, (void*)buf, len, 0);
    }    
#endif
}


void step_com_bridge(is_comm_instance_t &comm)
{	
	// USB CDC Rx   =======================================================
	com_bridge_smart_forward(EVB2_PORT_USB, 0);

	// uINS Ser0 Rx   =======================================================
	com_bridge_smart_forward(EVB2_PORT_UINS0, LED_INS_RXD_PIN);

	// uINS Ser1 (TTL or SPI) Rx   =======================================================
	com_bridge_smart_forward(EVB2_PORT_UINS1, LED_INS_RXD_PIN);
    
#ifdef CONF_BOARD_SERIAL_XBEE           // XBee Rx   =======================================================
    xbee_step(&comm);
#endif

#ifdef CONF_BOARD_SERIAL_EXT_RADIO      // External Radio Rx   =======================================================
	com_bridge_smart_forward(EVB2_PORT_XRADIO, 0);
#endif

#ifdef CONF_BOARD_SERIAL_ATWINC_BLE     // ATWINC BLE Rx   =======================================================
	com_bridge_smart_forward(EVB2_PORT_BLE, LED_WIFI_RXD_PIN);
#endif

#ifdef CONF_BOARD_SERIAL_SP330          // SP330 RS232/RS422 converter Rx   =======================================================
	com_bridge_smart_forward(EVB2_PORT_SP330, g_flashCfg->uinsComPort==EVB2_PORT_SP330?LED_INS_RXD_PIN:0);
#endif

#ifdef CONF_BOARD_SERIAL_GPIO_H8        // H8 Header GPIO TTL Rx   =======================================================
	com_bridge_smart_forward(EVB2_PORT_GPIO_H8, g_flashCfg->uinsComPort==EVB2_PORT_GPIO_H8?LED_INS_RXD_PIN:0);
#endif

#ifdef CONF_BOARD_SPI_ATWINC_WIFI       // WiFi Rx   =======================================================
	com_bridge_smart_forward_xstream(EVB2_PORT_WIFI, g_xStreamBufferWiFiRx);
#endif

#ifdef CONF_BOARD_CAN1
	//uint8_t can_rx_message[CONF_MCAN_ELEMENT_DATA_SIZE];
	//uint32_t id;
	//uint8_t lenCAN;
	//is_can_message msg;
	//msg.startByte = CAN_HDR;
	//msg.endByte = CAN_FTR;
	//if((lenCAN = mcan_receive_message(&id, can_rx_message)) > 0)// && --timeout > 0))
	//{
		//msg.address = id;
		//msg.payload = *(is_can_payload*)can_rx_message;
		//msg.dlc = lenCAN;
		//com_bridge_forward(EVB2_PORT_CAN,(uint8_t*)&msg, sizeof(is_can_message));
	//}
	/*Test CAN*/
		//static uint8_t tx_message_1[8] = { 0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87};
		//mcan_send_message(0x100000A5, tx_message_1, CONF_MCAN_ELEMENT_DATA_SIZE);
#endif	
}


void comunications_set_uins_data_callback( pfnHandleUinsData pfn )
{
	s_pfnHandleUinsData = pfn;
}


void comunications_set_host_data_callback( pfnHandleHostData pfn )
{
	s_pfnHandleHostData = pfn;
}


void communications_init(pfnHandleBroadcst pfnBroadcst, pfnHandleDid2Ermc pfnDid2Ermc)
{
    const size_t xTriggerLevel = 1;

	g_xStreamBufferUINS = xStreamBufferCreateStatic(STREAM_BUFFER_SIZE,
													xTriggerLevel,
													s_xStreamBufferUINS_buf,
													&s_xStreamBufferUINS_struct);
	g_xStreamBufferWiFiRx = xStreamBufferCreateStatic(STREAM_BUFFER_SIZE,
													xTriggerLevel,
													s_xStreamBufferWiFiRx_buf,
													&s_xStreamBufferWiFiRx_struct);
	g_xStreamBufferWiFiTx = xStreamBufferCreateStatic(STREAM_BUFFER_SIZE,
													xTriggerLevel,
													s_xStreamBufferWiFiTx_buf,
													&s_xStreamBufferWiFiTx_struct);

	for(int i=0; i<COM_RX_PORT_COUNT; i++)
	{
		is_comm_init(&(g_comRxPort[i].comm), g_comRxPort[i].comm_buffer, PKT_BUF_SIZE);

		// Use alternate decode buffer on EVB so we can preserve and forward original packet received.
		g_comRxPort[i].comm.altDecodeBuf = s_rxDecodeBuffer;
	}

	// Broadcast callback
	s_pfnHandleBroadcst = pfnBroadcst;
	// DID to RMC
	s_pfnHandleDid2Ermc = pfnDid2Ermc;
}
