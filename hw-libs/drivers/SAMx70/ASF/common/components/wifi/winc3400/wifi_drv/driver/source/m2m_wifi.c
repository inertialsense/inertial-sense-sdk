/**
 *
 * \file
 *
 * \brief This module contains M2M Wi-Fi APIs implementation.
 *
 * Copyright (c) 2017-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#include "driver/include/m2m_wifi.h"
#include "driver/source/m2m_hif.h"
#include "driver/source/nmasic.h"

#include "ISConstants.h"
#define malloc	MALLOC
#define free	FREE

static volatile uint8 gu8ChNum;
static tpfAppWifiCb gpfAppWifiCb = NULL;


#ifdef ETH_MODE
static tpfAppEthCb  gpfAppEthCb  = NULL;
static uint8* 	        gau8ethRcvBuf=NULL;
static uint16 	        gu16ethRcvBufSize ;
#endif

//#define CONF_MGMT

#ifdef CONF_MGMT
static tpfAppMonCb  gpfAppMonCb  = NULL;
static struct _tstrMgmtCtrl
{
	uint8* pu8Buf;
	uint16 u16Offset;
	uint16 u16Sz;
}
gstrMgmtCtrl = {NULL, 0 , 0};
#endif
/**
*	@fn			m2m_wifi_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr, uint8 grp)
*	@brief		WiFi call back function
*	@param [in]	u8OpCode
*					HIF Opcode type.
*	@param [in]	u16DataSize
*					HIF data length.
*	@param [in]	u32Addr
*					HIF address.
*	@param [in]	grp
*					HIF group type.
*	@author
*	@date
*	@version	1.0
*/
static void m2m_wifi_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr)
{
	uint8 rx_buf[8];
	if (u8OpCode == M2M_WIFI_RESP_CON_STATE_CHANGED)
	{
		tstrM2mWifiStateChanged strState;
		if (hif_receive(u32Addr, (uint8*) &strState,sizeof(tstrM2mWifiStateChanged), 0) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_CON_STATE_CHANGED, &strState);
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_GET_SYS_TIME)
	{
		tstrSystemTime strSysTime;
		if (hif_receive(u32Addr, (uint8*) &strSysTime,sizeof(tstrSystemTime), 0) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_GET_SYS_TIME, &strSysTime);
		}
	}
	else if(u8OpCode == M2M_WIFI_RESP_CONN_INFO)
	{
		tstrM2MConnInfo		strConnInfo;
		if(hif_receive(u32Addr, (uint8*)&strConnInfo, sizeof(tstrM2MConnInfo), 1) == M2M_SUCCESS)
		{
			if(gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_CONN_INFO, &strConnInfo);
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_MEMORY_RECOVER)
	{
#if 0
		if (hif_receive(u32Addr, rx_buf, 4, 1) == M2M_SUCCESS)
		{
			tstrM2mWifiStateChanged strState;
			m2m_memcpy((uint8*) &strState, rx_buf,sizeof(tstrM2mWifiStateChanged));
			if (app_wifi_recover_cb)
				app_wifi_recover_cb(strState.u8CurrState);
		}
#endif
	}
	else if (u8OpCode == M2M_WIFI_REQ_DHCP_CONF)
	{
		tstrM2MIPConfig strIpConfig;
		if (hif_receive(u32Addr, (uint8 *)&strIpConfig, sizeof(tstrM2MIPConfig), 0) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_REQ_DHCP_CONF, (uint8 *)&strIpConfig);
		}
	}
	else if (u8OpCode == M2M_WIFI_REQ_WPS)
	{
		tstrM2MWPSInfo strWps;
		m2m_memset((uint8*)&strWps,0,sizeof(tstrM2MWPSInfo));
		if(hif_receive(u32Addr, (uint8*)&strWps, sizeof(tstrM2MWPSInfo), 0) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_REQ_WPS, &strWps);
		}
	}   
	else if (u8OpCode == M2M_WIFI_RESP_IP_CONFLICT)
	{
		uint32  u32ConflictedIP;
		if(hif_receive(u32Addr, (uint8 *)&u32ConflictedIP, sizeof(u32ConflictedIP), 0) == M2M_SUCCESS)
		{
			M2M_INFO("Conflicted IP \" %u.%u.%u.%u \" \n", 
				BYTE_0(u32ConflictedIP),BYTE_1(u32ConflictedIP),BYTE_2(u32ConflictedIP),BYTE_3(u32ConflictedIP));
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_IP_CONFLICT, NULL);

		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_SCAN_DONE)
	{
		tstrM2mScanDone strState;
		if(hif_receive(u32Addr, (uint8*)&strState, sizeof(tstrM2mScanDone), 0) == M2M_SUCCESS)
		{
			gu8ChNum = strState.u8NumofCh;
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_SCAN_DONE, &strState);
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_SCAN_RESULT)
	{
		tstrM2mWifiscanResult strScanResult;
		if(hif_receive(u32Addr, (uint8*)&strScanResult, sizeof(tstrM2mWifiscanResult), 0) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_SCAN_RESULT, &strScanResult);
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_CURRENT_RSSI)
	{
		if (hif_receive(u32Addr, rx_buf, 4, 0) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_CURRENT_RSSI, rx_buf);
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_CLIENT_INFO)
	{
		if (hif_receive(u32Addr, rx_buf, 4, 0) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_CLIENT_INFO, rx_buf);
		}
	}
	else if(u8OpCode == M2M_WIFI_RESP_PROVISION_INFO)
	{
		tstrM2MProvisionInfo	strProvInfo;
		if(hif_receive(u32Addr, (uint8*)&strProvInfo, sizeof(tstrM2MProvisionInfo), 1) == M2M_SUCCESS)
		{
			if(gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_PROVISION_INFO, &strProvInfo);
		}
	}
	else if(u8OpCode == M2M_WIFI_RESP_DEFAULT_CONNECT)
	{
		tstrM2MDefaultConnResp	strResp;
		if(hif_receive(u32Addr, (uint8*)&strResp, sizeof(tstrM2MDefaultConnResp), 1) == M2M_SUCCESS)
		{
			if(gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_DEFAULT_CONNECT, &strResp);
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_BLE_API_RECV)
	{
		//Read the length
		if(hif_receive(u32Addr, rx_buf, 2, 0) == M2M_SUCCESS)
		{
			uint16 u16BleMsgLen = (rx_buf[1] << 8) + rx_buf[0];			
			tstrM2mBleApiMsg* bleRx = (tstrM2mBleApiMsg*)malloc(u16BleMsgLen + sizeof(tstrM2mBleApiMsg));
			
			if(bleRx != NULL)
			{
				bleRx->u16Len = u16BleMsgLen;
				
				//Read the rest of the message
				if (hif_receive(u32Addr+2, bleRx->data, bleRx->u16Len, 1)== M2M_SUCCESS)
				{
					if(gpfAppWifiCb)
						gpfAppWifiCb(M2M_WIFI_RESP_BLE_API_RECV, bleRx);
				}
				free(bleRx);
			}
		}
	}
	else if(u8OpCode == M2M_WIFI_RESP_GET_PRNG)
	{
		tstrPrng strPrng;
		if(hif_receive(u32Addr, (uint8*)&strPrng,sizeof(tstrPrng), 0) == M2M_SUCCESS)
		{
			if(hif_receive(u32Addr + sizeof(tstrPrng),strPrng.pu8RngBuff,strPrng.u16PrngSize, 1) == M2M_SUCCESS)
			{
				if(gpfAppWifiCb) {
					gpfAppWifiCb(M2M_WIFI_RESP_GET_PRNG,&strPrng);
				}
			}
		}
	}
	else if (u8OpCode == M2M_WIFI_RESP_SET_GAIN_TABLE)
	{
		tstrM2MGainTableRsp strGainRsp;
		if (hif_receive(u32Addr, (uint8*) &strGainRsp,sizeof(tstrM2MGainTableRsp), 0) == M2M_SUCCESS)
		{
			if (gpfAppWifiCb)
				gpfAppWifiCb(M2M_WIFI_RESP_SET_GAIN_TABLE, &strGainRsp);
		}
	}
#ifdef ETH_MODE
		else if(u8OpCode == M2M_WIFI_RESP_ETHERNET_RX_PACKET)
		{
			if(hif_receive(u32Addr, rx_buf ,sizeof(tstrM2mIpRsvdPkt), 0) == M2M_SUCCESS)
			{
				tstrM2mIpRsvdPkt * pstrM2MIpRxPkt = (tstrM2mIpRsvdPkt*)rx_buf;
				tstrM2mIpCtrlBuf  strM2mIpCtrlBuf;
				uint16 u16Offset = pstrM2MIpRxPkt->u16PktOffset;
				
				strM2mIpCtrlBuf.u16RemainigDataSize = pstrM2MIpRxPkt->u16PktSz;
				if((gpfAppEthCb) &&(gau8ethRcvBuf)&& (gu16ethRcvBufSize > 0))
				{
					while (strM2mIpCtrlBuf.u16RemainigDataSize > 0)
					{
						if(strM2mIpCtrlBuf.u16RemainigDataSize > gu16ethRcvBufSize)
						{
							strM2mIpCtrlBuf.u16DataSize = gu16ethRcvBufSize ;
						}
						else
						{
							strM2mIpCtrlBuf.u16DataSize = strM2mIpCtrlBuf.u16RemainigDataSize;
						}
						if(hif_receive(u32Addr+u16Offset, gau8ethRcvBuf, strM2mIpCtrlBuf.u16DataSize, 0) == M2M_SUCCESS)
						{
							strM2mIpCtrlBuf.u16RemainigDataSize -= strM2mIpCtrlBuf.u16DataSize;
							u16Offset += strM2mIpCtrlBuf.u16DataSize;
							gpfAppEthCb(M2M_WIFI_RESP_ETHERNET_RX_PACKET, gau8ethRcvBuf, &(strM2mIpCtrlBuf));
						}
						else
						{
							break;
						}
					}
				}
			}
		}
#endif	

#ifdef CONF_MGMT
	else if(u8OpCode == M2M_WIFI_RESP_WIFI_RX_PACKET)
	{
		tstrM2MWifiRxPacketInfo		strRxPacketInfo;
		if(u16DataSize >= sizeof(tstrM2MWifiRxPacketInfo)) {
			if(hif_receive(u32Addr, (uint8*)&strRxPacketInfo, sizeof(tstrM2MWifiRxPacketInfo), 0) == M2M_SUCCESS)
			{
				u16DataSize -= sizeof(tstrM2MWifiRxPacketInfo);
				if(u16DataSize > 0 && gstrMgmtCtrl.pu8Buf != NULL)
				{
					if(u16DataSize > (gstrMgmtCtrl.u16Sz + gstrMgmtCtrl.u16Offset))
					{
						u16DataSize = gstrMgmtCtrl.u16Sz;
					}
					u32Addr += sizeof(tstrM2MWifiRxPacketInfo) + gstrMgmtCtrl.u16Offset;
					if(hif_receive(u32Addr , gstrMgmtCtrl.pu8Buf, u16DataSize, 1) != M2M_SUCCESS)
					{
						u16DataSize = 0;
					}
				}
				if(gpfAppMonCb)
					gpfAppMonCb(&strRxPacketInfo, gstrMgmtCtrl.pu8Buf,u16DataSize);
			}
		} else {
			M2M_ERR("Incorrect mon data size %u\n", u16DataSize);
		}
	}
#endif
	else
	{
		M2M_ERR("REQ Not defined %d\n",u8OpCode);
	}
}

sint8 m2m_wifi_download_mode()
{
	sint8 ret = M2M_SUCCESS;
	/* Apply device specific initialization. */
	ret = nm_drv_init_download_mode(0);
	if(ret != M2M_SUCCESS) 	goto _EXIT0;
	

	
	enable_interrupts();

_EXIT0:
	return ret;
}

static sint8 m2m_validate_ap_parameters(CONST tstrM2MAPConfig* pstrM2MAPConfig)
{
	sint8 s8Ret = M2M_SUCCESS;
	/* Check for incoming pointer */
	if(pstrM2MAPConfig == NULL)
	{
		M2M_ERR("INVALID POINTER\n");
		s8Ret = M2M_ERR_FAIL;
		goto ERR1;
	}
	/* Check for SSID */
	if((m2m_strlen((uint8 *)pstrM2MAPConfig->au8SSID) <= 0) || (m2m_strlen((uint8 *)pstrM2MAPConfig->au8SSID) >= M2M_MAX_SSID_LEN))
	{
		M2M_ERR("INVALID SSID\n");
		s8Ret = M2M_ERR_FAIL;
		goto ERR1;
	}
	/* Check for Channel */
	if(pstrM2MAPConfig->u8ListenChannel > M2M_WIFI_CH_14 || pstrM2MAPConfig->u8ListenChannel < M2M_WIFI_CH_1)
	{
		M2M_ERR("INVALID CH\n");
		s8Ret = M2M_ERR_FAIL;
		goto ERR1;
	}
	/* Check for DHCP Server IP address */
	if(!(pstrM2MAPConfig->au8DHCPServerIP[0] || pstrM2MAPConfig->au8DHCPServerIP[1]))
	{
		if(!(pstrM2MAPConfig->au8DHCPServerIP[2]))
		{
			M2M_ERR("INVALID DHCP SERVER IP\n");
			s8Ret = M2M_ERR_FAIL;
			goto ERR1;
		}
	}
	/* Check for Security */
	if(pstrM2MAPConfig->u8SecType == M2M_WIFI_SEC_OPEN)
	{
		goto ERR1;
	}
	else if(pstrM2MAPConfig->u8SecType == M2M_WIFI_SEC_WEP)
	{
		/* Check for WEP Key index */
		if((pstrM2MAPConfig->u8KeyIndx <= 0) || (pstrM2MAPConfig->u8KeyIndx > WEP_KEY_MAX_INDEX))
		{
			M2M_ERR("INVALID KEY INDEX\n");
			s8Ret = M2M_ERR_FAIL;
			goto ERR1;
		}
		/* Check for WEP Key size */
		if(	(pstrM2MAPConfig->u8KeySz != WEP_40_KEY_STRING_SIZE) &&
			(pstrM2MAPConfig->u8KeySz != WEP_104_KEY_STRING_SIZE)
		)
		{
			M2M_ERR("INVALID KEY SIZE\n");
			s8Ret = M2M_ERR_FAIL;
			goto ERR1;
		}
		/* Check for WEP Key */
		if((pstrM2MAPConfig->au8WepKey == NULL) || (m2m_strlen((uint8 *)pstrM2MAPConfig->au8WepKey) <= 0) || (m2m_strlen((uint8 *)pstrM2MAPConfig->au8WepKey) > WEP_104_KEY_STRING_SIZE))
		{
			M2M_ERR("INVALID WEP KEY\n");
			s8Ret = M2M_ERR_FAIL;
			goto ERR1;
		}
	}
	else
	{
		M2M_ERR("INVALID AUTHENTICATION MODE\n");
		s8Ret = M2M_ERR_FAIL;
		goto ERR1;
	}
	
ERR1:
	return s8Ret;
}
static sint8 m2m_validate_scan_options(tstrM2MScanOption* ptstrM2MScanOption)
{
	sint8 s8Ret = M2M_SUCCESS;
	/* Check for incoming pointer */
	if(ptstrM2MScanOption == NULL)
	{
		M2M_ERR("INVALID POINTER\n");
		s8Ret = M2M_ERR_FAIL;
	}
	else
	{
		/* Check for valid No of slots */
		 if(ptstrM2MScanOption->u8NumOfSlot == 0)
		{
			M2M_ERR("INVALID No of scan slots!\n");
			s8Ret = M2M_ERR_FAIL;
		}	
		/* Check for valid time of slots */
		 if((ptstrM2MScanOption->u8SlotTime < 10) || (ptstrM2MScanOption->u8SlotTime > 250))
		{
			M2M_ERR("INVALID scan slot time!\n");
			s8Ret = M2M_ERR_FAIL;
		}	
		/* Check for valid No of probe requests per slot */
		 if((ptstrM2MScanOption->u8ProbesPerSlot == 0) || (ptstrM2MScanOption->u8ProbesPerSlot > M2M_SCAN_DEFAULT_NUM_PROBE))
		{
			M2M_ERR("INVALID No of probe requests per scan slot\n");
			s8Ret = M2M_ERR_FAIL;
		}	
		/* Check for valid RSSI threshold */
		 if(ptstrM2MScanOption->s8RssiThresh >= 0)
		{
			M2M_ERR("INVALID RSSI threshold %d \n",ptstrM2MScanOption->s8RssiThresh);
			s8Ret = M2M_ERR_FAIL;
		}	
	}
	return s8Ret;
}

NMI_API sint8 m2m_wifi_ble_set_gain_table(uint8 table_idx)
{
	sint8 s8Ret = M2M_ERR_FAIL;
	tstrM2MGainTable strGainTable = {0};
	
	strGainTable.u8GainTable = table_idx;
	
	s8Ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_GAIN_TABLE, (uint8 *)&strGainTable, sizeof(tstrM2MGainTable), NULL, 0, 0);
	return s8Ret;
}


sint8 m2m_wifi_init_hold(void)
{
	sint8 ret = M2M_ERR_FAIL;

	/* Apply device specific initialization. */
	ret = nm_drv_init_hold(0);

	return ret;
}
sint8 m2m_wifi_init_start(tstrWifiInitParam * param)
{
	tstrM2mRev strtmp;
	sint8 ret = M2M_SUCCESS;
	
	if(param == NULL) {
		ret = M2M_ERR_FAIL;
		goto _EXIT0;
	}
	
	gpfAppWifiCb = param->pfAppWifiCb;
		
#ifdef ETH_MODE
 	gpfAppEthCb  	    = param->strEthInitParam.pfAppEthCb;
	gau8ethRcvBuf       = param->strEthInitParam.au8ethRcvBuf;
	gu16ethRcvBufSize	= param->strEthInitParam.u16ethRcvBufSize;
#endif

#ifdef CONF_MGMT
	gpfAppMonCb  = param->pfAppMonCb;
#endif

	/* Apply device specific initialization. */
	ret = nm_drv_init_start(NULL);
	if(ret != M2M_SUCCESS) 	goto _EXIT0;
	/* Initialize host interface module */
	ret = hif_init(NULL);
	if(ret != M2M_SUCCESS) 	goto _EXIT1;

	hif_register_cb(M2M_REQ_GROUP_WIFI,m2m_wifi_cb);

	M2M_INFO("Curr driver ver: %u.%u.%u\n", M2M_DRIVER_VERSION_MAJOR_NO, M2M_DRIVER_VERSION_MINOR_NO, M2M_DRIVER_VERSION_PATCH_NO);
	M2M_INFO("Curr driver HIF Level: (%u) %u.%u\n", M2M_HIF_BLOCK_VALUE, M2M_HIF_MAJOR_VALUE, M2M_HIF_MINOR_VALUE);
	ret = m2m_wifi_get_firmware_version(&strtmp);
	m2m_ota_get_firmware_version(&strtmp);

	if(ret == M2M_SUCCESS)
	{
		ret = hif_enable_access();
		if(ret == M2M_SUCCESS)
		{
			m2m_wifi_ble_set_gain_table(param->GainTableIndex);
		}
	}
	goto _EXIT0;

_EXIT1:
	nm_drv_deinit(NULL);
_EXIT0:
	return ret;
}
sint8 m2m_wifi_init(tstrWifiInitParam * param)
{
	sint8 ret = M2M_SUCCESS;

	ret = m2m_wifi_init_hold();
	if (ret == M2M_SUCCESS)
	{
		ret = m2m_wifi_init_start(param);
	}
	return ret;
}
sint8  m2m_wifi_deinit(void * arg)
{

	hif_deinit(NULL);

	nm_drv_deinit(NULL);

	return M2M_SUCCESS;
}
sint8 m2m_wifi_reinit_hold(void)
{
	m2m_wifi_deinit(NULL);
	return m2m_wifi_init_hold();
}
sint8 m2m_wifi_reinit_start(tstrWifiInitParam * param)
{
	return m2m_wifi_init_start(param);
}
sint8 m2m_wifi_reinit(tstrWifiInitParam * param)
{
	sint8 ret = M2M_ERR_FAIL;
	ret = m2m_wifi_reinit_hold();
	if(ret == M2M_SUCCESS) {
		ret = m2m_wifi_reinit_start(param);
	}
	return ret;
}

void m2m_wifi_yield(void)
{
	hif_yield();
}

sint8 m2m_wifi_handle_events(void * arg)
{
	return hif_handle_isr();
}

sint8 m2m_wifi_default_connect(void)
{
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_DEFAULT_CONNECT, NULL, 0,NULL, 0,0);
}

sint8 m2m_wifi_connect(char *pcSsid, uint8 u8SsidLen, uint8 u8SecType, void *pvAuthInfo, uint16 u16Ch)
{
	return m2m_wifi_connect_sc(pcSsid, u8SsidLen, u8SecType, pvAuthInfo,  u16Ch,0);
}
sint8 m2m_wifi_connect_sc(char *pcSsid, uint8 u8SsidLen, uint8 u8SecType, void *pvAuthInfo, uint16 u16Ch, uint8 u8NoSaveCred)
{
	sint8				ret = M2M_SUCCESS;
	tstrM2mWifiConnect	strConnect;
	tstrM2MWifiSecInfo	*pstrAuthInfo;

	if(u8SecType != M2M_WIFI_SEC_OPEN)
	{
		if(pvAuthInfo == NULL)
		{
			M2M_ERR("Key is not valid\n");
			ret = M2M_ERR_FAIL;
			goto ERR1;
		}
		if((u8SecType == M2M_WIFI_SEC_WPA_PSK) && (m2m_strlen(pvAuthInfo) == (M2M_MAX_PSK_LEN-1)))
		{
			uint8 i = 0;
			uint8* pu8Psk = (uint8*)pvAuthInfo;
			while(i < (M2M_MAX_PSK_LEN-1))
			{
				if(pu8Psk[i]<'0' || (pu8Psk[i]>'9' && pu8Psk[i] < 'A')|| (pu8Psk[i]>'F' && pu8Psk[i] < 'a') || pu8Psk[i] > 'f')
				{
					M2M_ERR("Invalid Key\n");
					ret = M2M_ERR_FAIL;
					goto ERR1;
				}
				i++;
			}
		}
	}
	if((u8SsidLen<=0)||(u8SsidLen>=M2M_MAX_SSID_LEN))
	{
		M2M_ERR("SSID LEN INVALID\n");
		ret = M2M_ERR_FAIL;
		goto ERR1;
	}

	if(u16Ch < M2M_WIFI_CH_1 || u16Ch > M2M_WIFI_CH_14)
	{
		if(u16Ch!=M2M_WIFI_CH_ALL)
		{
			M2M_ERR("CH INVALID\n");
			ret = M2M_ERR_FAIL;
			goto ERR1;
		}
	}

	m2m_memcpy(strConnect.au8SSID, (uint8*)pcSsid, u8SsidLen);
	strConnect.au8SSID[u8SsidLen]	= 0;
	strConnect.u16Ch				= NM_BSP_B_L_16(u16Ch);
	/* Credentials will be Not be saved if u8NoSaveCred is set */ 
	strConnect.u8NoSaveCred 			= u8NoSaveCred ? 1:0;
	pstrAuthInfo = &strConnect.strSec;
	pstrAuthInfo->u8SecType		= u8SecType;

	if(u8SecType == M2M_WIFI_SEC_WEP)
	{
		tstrM2mWifiWepParams	* pstrWepParams = (tstrM2mWifiWepParams*)pvAuthInfo;
		tstrM2mWifiWepParams	*pstrWep = &pstrAuthInfo->uniAuth.strWepInfo;
		pstrWep->u8KeyIndx =pstrWepParams->u8KeyIndx-1;

		if(pstrWep->u8KeyIndx >= WEP_KEY_MAX_INDEX)
		{
			M2M_ERR("Invalid Wep key index %d\n", pstrWep->u8KeyIndx);
			ret = M2M_ERR_FAIL;
			goto ERR1;
		}
		pstrWep->u8KeySz = pstrWepParams->u8KeySz-1;
		if ((pstrWep->u8KeySz != WEP_40_KEY_STRING_SIZE)&& (pstrWep->u8KeySz != WEP_104_KEY_STRING_SIZE))
		{
			M2M_ERR("Invalid Wep key length %d\n", pstrWep->u8KeySz);
			ret = M2M_ERR_FAIL;
			goto ERR1;
		}
		m2m_memcpy((uint8*)pstrWep->au8WepKey,(uint8*)pstrWepParams->au8WepKey, pstrWepParams->u8KeySz);
		pstrWep->au8WepKey[pstrWepParams->u8KeySz] = 0;

	}


	else if(u8SecType == M2M_WIFI_SEC_WPA_PSK)
	{
		uint16	u16KeyLen = m2m_strlen((uint8*)pvAuthInfo);
		if((u16KeyLen <= 0)||(u16KeyLen >= M2M_MAX_PSK_LEN))
		{
			M2M_ERR("Incorrect PSK key length\n");
			ret = M2M_ERR_FAIL;
			goto ERR1;
	}
		m2m_memcpy(pstrAuthInfo->uniAuth.au8PSK, (uint8*)pvAuthInfo, u16KeyLen + 1);
	}
	else if(u8SecType == M2M_WIFI_SEC_802_1X)
	{
		m2m_memcpy((uint8*)&pstrAuthInfo->uniAuth.strCred1x, (uint8*)pvAuthInfo, sizeof(tstr1xAuthCredentials));
	}
	else if(u8SecType == M2M_WIFI_SEC_OPEN)
	{

	}
	else
	{
		M2M_ERR("undefined sec type\n");
		ret = M2M_ERR_FAIL;
		goto ERR1;
	}

	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_CONNECT, (uint8*)&strConnect, sizeof(tstrM2mWifiConnect),NULL, 0,0);

ERR1:
	return ret;
}

sint8 m2m_wifi_disconnect(void)
{
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_DISCONNECT, NULL, 0, NULL, 0,0);
}
sint8 m2m_wifi_set_mac_address(uint8 au8MacAddress[6])
{
	tstrM2mSetMacAddress strTmp;
	m2m_memcpy((uint8*) strTmp.au8Mac, (uint8*) au8MacAddress, 6);
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_MAC_ADDRESS,
		(uint8*) &strTmp, sizeof(tstrM2mSetMacAddress), NULL, 0,0);
}

sint8 m2m_wifi_set_static_ip(tstrM2MIPConfig * pstrStaticIPConf)
{
	pstrStaticIPConf->u32DNS = NM_BSP_B_L_32(pstrStaticIPConf->u32DNS);
	pstrStaticIPConf->u32Gateway = NM_BSP_B_L_32(pstrStaticIPConf->u32Gateway);
	pstrStaticIPConf->u32StaticIP = NM_BSP_B_L_32(
		pstrStaticIPConf->u32StaticIP);
	pstrStaticIPConf->u32SubnetMask = NM_BSP_B_L_32(
		pstrStaticIPConf->u32SubnetMask);
	return hif_send(M2M_REQ_GROUP_IP, M2M_IP_REQ_STATIC_IP_CONF,
		(uint8*) pstrStaticIPConf, sizeof(tstrM2MIPConfig), NULL, 0,0);
}

sint8 m2m_wifi_request_dhcp_client(void)
{
	/*legacy API should be removed */
	return 0;
}
sint8 m2m_wifi_request_dhcp_server(uint8* addr)
{
    /*legacy API should be removed */
	return 0;
}
/*!
@fn			NMI_API sint8 m2m_wifi_set_lsn_int(tstrM2mLsnInt * pstrM2mLsnInt);
@brief		Set the Wi-Fi listen interval for power save operation. It is represented in units
			of AP Beacon periods.
@param [in]	pstrM2mLsnInt
			Structure holding the listen interval configurations.
@return		The function SHALL return 0 for success and a negative value otherwise.
@sa			tstrM2mLsnInt , m2m_wifi_set_sleep_mode
@pre		m2m_wifi_set_sleep_mode shall be called first
@warning	The Function called once after initialization. 
*/
sint8 m2m_wifi_enable_dhcp(uint8  u8DhcpEn )
{

	uint8	u8Req;
	u8Req = u8DhcpEn ? M2M_IP_REQ_ENABLE_DHCP : M2M_IP_REQ_DISABLE_DHCP;
	return hif_send(M2M_REQ_GROUP_IP, u8Req, NULL, 0, NULL, 0, 0);


}

sint8 m2m_wifi_set_lsn_int(tstrM2mLsnInt* pstrM2mLsnInt)
{
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_LSN_INT, (uint8*)pstrM2mLsnInt, sizeof(tstrM2mLsnInt), NULL, 0, 0);
}

sint8 m2m_wifi_set_cust_InfoElement(uint8* pau8M2mCustInfoElement)
{
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_CUST_INFO_ELEMENT, (uint8*)pau8M2mCustInfoElement, pau8M2mCustInfoElement[0]+1, NULL, 0, 0);
}

sint8 m2m_wifi_set_scan_options(tstrM2MScanOption* ptstrM2MScanOption)
{
	sint8	s8Ret = M2M_ERR_FAIL;
	if(m2m_validate_scan_options (ptstrM2MScanOption) == M2M_SUCCESS)
	{
		s8Ret =  hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_SCAN_OPTION, (uint8*)ptstrM2MScanOption, sizeof(tstrM2MScanOption),NULL, 0,0);
	}
	return s8Ret;
}
sint8 m2m_wifi_set_scan_region(uint16  ScanRegion)
{
	sint8	s8Ret = M2M_ERR_FAIL;
	tstrM2MScanRegion strScanRegion;
	strScanRegion.u16ScanRegion = ScanRegion;
	s8Ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_SCAN_REGION, (uint8*)&strScanRegion, sizeof(tstrM2MScanRegion),NULL, 0,0);
	return s8Ret;
}
sint8 m2m_wifi_request_scan(uint8 ch)
{
	sint8	s8Ret = M2M_SUCCESS;

	if(((ch >= M2M_WIFI_CH_1) && (ch <= M2M_WIFI_CH_14)) || (ch == M2M_WIFI_CH_ALL))
	{
		tstrM2MScan strtmp;
		strtmp.u8ChNum = ch;
		s8Ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SCAN, (uint8*)&strtmp, sizeof(tstrM2MScan),NULL, 0,0);
	}
	else
	{
		s8Ret = M2M_ERR_INVALID_ARG;
	}
	return s8Ret;
}

sint8 m2m_wifi_request_scan_passive(uint8 ch)
{
	sint8	s8Ret = M2M_SUCCESS;

	if(((ch >= M2M_WIFI_CH_1) && (ch <= M2M_WIFI_CH_14)) || (ch == M2M_WIFI_CH_ALL))
	{
		tstrM2MScan strtmp;
		strtmp.u8ChNum = ch;

		s8Ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_PASSIVE_SCAN, (uint8*)&strtmp, sizeof(tstrM2MScan),NULL, 0,0);
	}
	else
	{
		s8Ret = M2M_ERR_INVALID_ARG;
	}
	return s8Ret;
}
sint8 m2m_wifi_wps(uint8 u8TriggerType,const char  *pcPinNumber)
{
	tstrM2MWPSConnect strtmp;

	strtmp.u8TriggerType = u8TriggerType;
	/*If WPS is using PIN METHOD*/
	if (u8TriggerType == WPS_PIN_TRIGGER)
		m2m_memcpy ((uint8*)strtmp.acPinNumber,(uint8*) pcPinNumber,8);
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_WPS, (uint8*)&strtmp,sizeof(tstrM2MWPSConnect), NULL, 0,0);
}

sint8 m2m_wifi_wps_disable(void)
{
	sint8 ret = M2M_SUCCESS;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_DISABLE_WPS, NULL,0, NULL, 0, 0);
	return ret;
}

#if 0
/*
 * These two functions m2m_wifi_req_client_ctrl and m2m_wifi_req_server_init are for a mode in which two WINC ICs
 * communicate with each other via probe request and probe response frames. This mode is not supported in WINC fw.
 */
sint8 m2m_wifi_req_client_ctrl(uint8 u8Cmd)
{

	sint8 ret = M2M_SUCCESS;
#ifdef _PS_SERVER_
	tstrM2Mservercmd	strCmd;
	strCmd.u8cmd = u8Cmd;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_CLIENT_CTRL, (uint8*)&strCmd, sizeof(tstrM2Mservercmd), NULL, 0, 0);
#else
	M2M_ERR("_PS_SERVER_ is not defined\n");
#endif
	return ret;
}
sint8 m2m_wifi_req_server_init(uint8 ch)
{
	sint8 ret = M2M_SUCCESS;
#ifdef _PS_SERVER_
	tstrM2mServerInit strServer;
	strServer.u8Channel = ch;
	ret = hif_send(M2M_REQ_GROUP_WIFI,M2M_WIFI_REQ_SERVER_INIT, (uint8*)&strServer, sizeof(tstrM2mServerInit), NULL, 0, 0);
#else
	M2M_ERR("_PS_SERVER_ is not defined\n");
#endif
	return ret;
}
#endif

sint8 m2m_wifi_p2p(uint8 u8Channel)
{
	sint8 ret = M2M_SUCCESS;
	if((u8Channel == M2M_WIFI_CH_1) || (u8Channel == M2M_WIFI_CH_6) || (u8Channel == M2M_WIFI_CH_11))
	{
		tstrM2MP2PConnect strtmp;
		strtmp.u8ListenChannel = u8Channel;
		ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_ENABLE_P2P, (uint8*)&strtmp, sizeof(tstrM2MP2PConnect), NULL, 0,0);
	}
	else
	{
		M2M_ERR("Listen channel should only be 1, 6 or 11\n");
		ret = M2M_ERR_FAIL;
	}
	return ret;
}
sint8 m2m_wifi_p2p_disconnect(void)
{
	sint8 ret = M2M_SUCCESS;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_DISABLE_P2P, NULL, 0, NULL, 0, 0);
	return ret;
}
sint8 m2m_wifi_enable_ap(CONST tstrM2MAPConfig* pstrM2MAPConfig)
{
	sint8 ret = M2M_ERR_FAIL;
	if(M2M_SUCCESS == m2m_validate_ap_parameters(pstrM2MAPConfig))
	{
		ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_ENABLE_AP, (uint8 *)pstrM2MAPConfig, sizeof(tstrM2MAPConfig), NULL, 0, 0);	
	}
	return ret;
}
sint8 m2m_wifi_disable_ap(void)
{
	sint8 ret = M2M_SUCCESS;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_DISABLE_AP, NULL, 0, NULL, 0, 0);
	return ret;
}
/*!
@fn          NMI_API sint8 m2m_wifi_req_curr_rssi(void);
@brief       Request the current RSSI for the current connected AP, 
			 the response received in wifi_cb M2M_WIFI_RESP_CURRENT_RSSI	
@sa          M2M_WIFI_RESP_CURRENT_RSSI              
@return      The function shall return M2M_SUCCESS for success and a negative value otherwise.
*/
sint8 m2m_wifi_req_curr_rssi(void)
{
	sint8 ret = M2M_SUCCESS;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_CURRENT_RSSI, NULL, 0, NULL,0, 0);
	return ret;
}

sint8 m2m_wifi_req_restrict_ble(void)
{
	sint8 ret = M2M_SUCCESS;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_RESTRICT_BLE,NULL, 0, NULL, 0, 0);
	return ret;
}

sint8 m2m_wifi_req_unrestrict_ble(void)
{
	sint8 ret = M2M_SUCCESS;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_UNRESTRICT_BLE,NULL, 0, NULL, 0, 0);
	return ret;
}

sint8 m2m_wifi_send_ethernet_pkt(uint8* pu8Packet,uint16 u16PacketSize)
{
	sint8	s8Ret = -1;
	if((pu8Packet != NULL)&&(u16PacketSize>0))
	{
		tstrM2MWifiTxPacketInfo		strTxPkt;

		strTxPkt.u16PacketSize		= u16PacketSize;
		strTxPkt.u16HeaderLength	= M2M_ETHERNET_HDR_LEN;
		s8Ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SEND_ETHERNET_PACKET | M2M_REQ_DATA_PKT,
		(uint8*)&strTxPkt, sizeof(tstrM2MWifiTxPacketInfo), pu8Packet, u16PacketSize,  M2M_ETHERNET_HDR_OFFSET - M2M_HIF_HDR_OFFSET);
	}
	return s8Ret;
}
/*!
@fn          NMI_API sint8 m2m_wifi_get_otp_mac_address(uint8 *pu8MacAddr, uint8 * pu8IsValid);
@brief       Request the MAC address stored on the OTP (one time programmable) memory of the device.
			 (the function is Blocking until response received)	
@param [out] pu8MacAddr
			 Output MAC address buffer of 6 bytes size. Valid only if *pu8Valid=1.
@param [out] pu8IsValid
		     A output boolean value to indicate the validity of pu8MacAddr in OTP. 
		     Output zero if the OTP memory is not programmed, non-zero otherwise.	
@return      The function shall return M2M_SUCCESS for success and a negative value otherwise.
@sa          m2m_wifi_get_mac_address             
@pre         m2m_wifi_init required to call any WIFI/socket function
*/
sint8 m2m_wifi_get_otp_mac_address(uint8 *pu8MacAddr, uint8* pu8IsValid)
{
	sint8 ret = M2M_SUCCESS;
	ret = hif_chip_wake();
	if(ret == M2M_SUCCESS)
	{
		ret = nmi_get_otp_mac_address(pu8MacAddr, pu8IsValid);
		if(ret == M2M_SUCCESS)
		{
			ret = hif_chip_sleep();
		}
	}
	return ret;
}
/*!
@fn          NMI_API sint8 m2m_wifi_get_mac_address(uint8 *pu8MacAddr)
@brief       Request the current MAC address of the device (the working mac address).
			 (the function is Blocking until response received)	
@param [out] pu8MacAddr
			 Output MAC address buffer of 6 bytes size.	
@return      The function shall return M2M_SUCCESS for success and a negative value otherwise.
@sa          m2m_wifi_get_otp_mac_address             
@pre         m2m_wifi_init required to call any WIFI/socket function
*/
sint8 m2m_wifi_get_mac_address(uint8 *pu8MacAddr)
{
	sint8 ret = M2M_SUCCESS;
	ret = hif_chip_wake();
	if(ret == M2M_SUCCESS)
	{
		ret = nmi_get_mac_address(pu8MacAddr);
		if(ret == M2M_SUCCESS)
		{
			ret = hif_chip_sleep();
		}
	}

	return ret;
}
/*!
@fn          NMI_API sint8 m2m_wifi_req_scan_result(uint8 index);
@brief       Reads the AP information from the Scan Result list with the given index, 
			 the response received in wifi_cb M2M_WIFI_RESP_SCAN_RESULT, 
			 the response pointer should be casted with tstrM2mWifiscanResult structure 	
@param [in]  index 
			 Index for the requested result, the index range start from 0 till number of AP's found 
@sa          tstrM2mWifiscanResult,m2m_wifi_get_num_ap_found,m2m_wifi_request_scan             
@return      The function shall return @ref M2M_SUCCESS for success and a negative value otherwise
@pre         m2m_wifi_request_scan need to be called first, then m2m_wifi_get_num_ap_found 
			 to get the number of AP's found
@warning     Function used only in STA mode only. the scan result updated only if scan request called,
			 else it will be cashed in firmware for the host scan request result, 
			 which mean if large delay occur between the scan request and the scan result request, 
			 the result will not be up-to-date
*/

sint8 m2m_wifi_req_scan_result(uint8 index)
{
	sint8 ret = M2M_SUCCESS;
	tstrM2mReqScanResult strReqScanRlt;
	strReqScanRlt.u8Index = index;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SCAN_RESULT, (uint8*) &strReqScanRlt, sizeof(tstrM2mReqScanResult), NULL, 0, 0);
	return ret;
}
/*!
@fn          NMI_API uint8 m2m_wifi_get_num_ap_found(void);
@brief       Reads the number of AP's found in the last Scan Request, 
			 The function read the number of AP's from global variable which updated in the 
			 wifi_cb in M2M_WIFI_RESP_SCAN_DONE.			 
@sa          m2m_wifi_request_scan               
@return      Return the number of AP's found in the last Scan Request.
@pre         m2m_wifi_request_scan need to be called first 
@warning     That function need to be called in the wifi_cb in M2M_WIFI_RESP_SCAN_DONE, 
			 calling that function in any other place will return undefined/undated numbers.
			 Function used only in STA mode only.
*/
uint8 m2m_wifi_get_num_ap_found(void)
{
	return gu8ChNum;
}
/*!
@fn		    NMI_API uint8 m2m_wifi_get_sleep_mode(void);
@brief	    Get the current Power save mode.
@return	    The current operating power saving mode.
@sa		    tenuPowerSaveModes , m2m_wifi_set_sleep_mode
*/
uint8 m2m_wifi_get_sleep_mode(void)
{
	return hif_get_sleep_mode();
}
/*!
@fn			NMI_API sint8 m2m_wifi_set_sleep_mode(uint8 PsTyp, uint8 BcastEn);
@brief      Set the power saving mode for the WINC3400. 
@param [in]	PsTyp
			Desired power saving mode. Supported types are defined in tenuPowerSaveModes.
@param [in]	BcastEn
			Broadcast reception enable flag. 
			If it is 1, the WINC3400 must be awake each DTIM Beacon for receiving Broadcast traffic.
			If it is 0, the WINC3400 will not wakeup at the DTIM Beacon, but its wakeup depends only 
			on the the configured Listen Interval. 
@return     The function SHALL return 0 for success and a negative value otherwise.
@sa			tenuPowerSaveModes
@warning    The function called once after initialization.  
*/
sint8 m2m_wifi_set_sleep_mode(uint8 PsTyp, uint8 BcastEn)
{
	sint8 ret = M2M_SUCCESS;
	tstrM2mPsType strPs;
	strPs.u8PsType = PsTyp;
	strPs.u8BcastEn = BcastEn;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SLEEP, (uint8*) &strPs,sizeof(tstrM2mPsType), NULL, 0, 0);
	M2M_INFO("POWER SAVE %d\n",PsTyp);
	hif_set_sleep_mode(PsTyp);
	return ret;
}
/*!
@fn	        NMI_API sint8 m2m_wifi_request_sleep(void)
@brief	    Request from WINC3400 device to Sleep for specific time in the M2M_PS_MANUAL Power save mode (only).
@param [in]	u32SlpReqTime
			Request Sleep in ms 
@return     The function SHALL return M2M_SUCCESS for success and a negative value otherwise.
@sa         tenuPowerSaveModes , m2m_wifi_set_sleep_mode
@warning	This API is currently unsupported on the WINC3400
*/
sint8 m2m_wifi_request_sleep(uint32 u32SlpReqTime)
{
	sint8 ret = M2M_SUCCESS;
	uint8 psType;
	psType = hif_get_sleep_mode();
	if(psType == M2M_PS_MANUAL)
	{
		tstrM2mSlpReqTime strPs;
		strPs.u32SleepTime = u32SlpReqTime;
		ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_DOZE, (uint8*) &strPs,sizeof(tstrM2mSlpReqTime), NULL, 0, 0);
	}
	return ret;
}
/*!
@fn			NMI_API sint8 m2m_wifi_set_device_name(uint8 *pu8DeviceName, uint8 u8DeviceNameLength);
@brief		Set the WINC3400 device name which is used as P2P device name.
@param [in]	pu8DeviceName
			Buffer holding the device name.
@param [in]	u8DeviceNameLength
			Length of the device name.
@return		The function SHALL return M2M_SUCCESS for success and a negative value otherwise.
@warning	The Function called once after initialization. 
*/
sint8 m2m_wifi_set_device_name(uint8 *pu8DeviceName, uint8 u8DeviceNameLength)
{
	tstrM2MDeviceNameConfig strDeviceName;
	if(u8DeviceNameLength >= M2M_DEVICE_NAME_MAX)
	{
		u8DeviceNameLength = M2M_DEVICE_NAME_MAX;
	}
	//pu8DeviceName[u8DeviceNameLength] = '\0';
	u8DeviceNameLength ++;
	m2m_memcpy(strDeviceName.au8DeviceName, pu8DeviceName, u8DeviceNameLength);
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_DEVICE_NAME,
		(uint8*)&strDeviceName, sizeof(tstrM2MDeviceNameConfig), NULL, 0,0);
}
/*!
@fn	\
	uint32 m2m_wifi_get_chipId(void)

@brief
	Get the WINC Chip ID.

@return
	The function SHALL return chipID >0 or 0 for failure.
*/
uint32 m2m_wifi_get_chipId(void)
{
	return nmi_get_chipid();
}
/*!
@fn		sint8 m2m_wifi_get_firmware_version(tstrM2mRev* pstrRev)

@brief
	Synchronous API to obtain the firmware version currently running on the WINC IC

@param [out]	pstrRev
	pointer holds address of structure "tstrM2mRev" that contains the firmware version parameters

@return
	The function SHALL return @ref M2M_SUCCESS for success and a negative value otherwise.
*/
sint8 m2m_wifi_get_firmware_version(tstrM2mRev *pstrRev)
{
	sint8 ret = M2M_SUCCESS;
	ret = hif_chip_wake();
	if(ret == M2M_SUCCESS)
	{
		ret = nm_get_firmware_full_info(pstrRev);
		hif_chip_sleep();
	}
	return ret;
}
/*!
@fn		sint8 m2m_wifi_check_ota_rb(void);

@brief
	Synchronous API to check presence and compatibility of the WINC image that is stored in the inactive flash partition.
	This is the image that would run on the WINC IC if @ref m2m_ota_switch_firmware or @ref m2m_ota_rollback were called,
	followed by a reset of the WINC IC.

@return
	The function SHALL return @ref M2M_SUCCESS for compatible image and a negative value otherwise.
*/
sint8 m2m_wifi_check_ota_rb(void)
{
	sint8 ret = M2M_SUCCESS;
	uint16 ota_hif_info = 0;

	ret = nm_get_hif_info(NULL, &ota_hif_info);
	if(ret == M2M_SUCCESS)
	{
		ret = hif_check_compatibility(ota_hif_info);
	}
	return ret;
}
/*!
@fn	\
	NMI_API sint8 m2m_ota_get_firmware_version(tstrM2mRev *pstrRev);

@brief
	Synchronous API to obtain the firmware version of the WINC image that is stored in the inactive flash partition.
	This is the image that would run on the WINC IC if @ref m2m_ota_switch_firmware or @ref m2m_ota_rollback were called, 
	followed by a reset of the WINC IC.

@param [out]	pstrRev
	pointer holds address of structure "tstrM2mRev" that contains the ota fw version parameters
	
@return
	The function SHALL return @ref M2M_SUCCESS for success and a negative value otherwise.
*/
NMI_API sint8 m2m_ota_get_firmware_version(tstrM2mRev * pstrRev)
{
	sint8 ret = M2M_SUCCESS;
	ret = hif_chip_wake();
	if(ret == M2M_SUCCESS)
	{
		ret = nm_get_ota_firmware_info(pstrRev);
		hif_chip_sleep();
	}
	return ret;
}
#ifdef CONF_MGMT
sint8 m2m_wifi_enable_monitoring_mode(tstrM2MWifiMonitorModeCtrl *pstrMtrCtrl, uint8 *pu8PayloadBuffer,
								   uint16 u16BufferSize, uint16 u16DataOffset)
{
	sint8	s8Ret = -1;
	if((pstrMtrCtrl->u8ChannelID >= M2M_WIFI_CH_1) && (pstrMtrCtrl->u8ChannelID <= M2M_WIFI_CH_14))
	{
		gstrMgmtCtrl.pu8Buf		= pu8PayloadBuffer;
		gstrMgmtCtrl.u16Sz		= u16BufferSize;
		gstrMgmtCtrl.u16Offset	= u16DataOffset;
		s8Ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_ENABLE_MONITORING,
			(uint8*)pstrMtrCtrl, sizeof(tstrM2MWifiMonitorModeCtrl), NULL, 0,0);
	}
	return s8Ret;
}
sint8 m2m_wifi_disable_monitoring_mode(void)
{
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_DISABLE_MONITORING, NULL, 0, NULL, 0,0);
}

sint8 m2m_wifi_send_wlan_pkt(uint8 *pu8WlanPacket, uint16 u16WlanHeaderLength, uint16 u16WlanPktSize)
{
	sint8	s8Ret = -1;
	if(pu8WlanPacket != NULL)
	{
		tstrM2MWifiTxPacketInfo		strTxPkt;

		strTxPkt.u16PacketSize		= u16WlanPktSize;
		strTxPkt.u16HeaderLength	= u16WlanHeaderLength;
		s8Ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SEND_WIFI_PACKET | M2M_REQ_DATA_PKT,
		(uint8*)&strTxPkt, sizeof(tstrM2MWifiTxPacketInfo), pu8WlanPacket, u16WlanPktSize, sizeof(tstrM2MWifiTxPacketInfo));
	}
	return s8Ret;
}
#endif

sint8 m2m_wifi_start_provision_mode(tstrM2MAPConfig *pstrAPConfig, char *pcHttpServerDomainName, uint8 bEnableHttpRedirect)
{
	sint8	s8Ret = M2M_ERR_FAIL;

	if((pstrAPConfig != NULL))
	{
		tstrM2MProvisionModeConfig	strProvConfig;
		if(M2M_SUCCESS == m2m_validate_ap_parameters(pstrAPConfig))
		{
			m2m_memcpy((uint8*)&strProvConfig.strApConfig, (uint8*)pstrAPConfig, sizeof(tstrM2MAPConfig));
			if((m2m_strlen((uint8 *)pcHttpServerDomainName) <= 0) || (NULL == pcHttpServerDomainName))
			{
				M2M_ERR("INVALID DOMAIN NAME\n");
				goto ERR1;
			}
			m2m_memcpy((uint8*)strProvConfig.acHttpServerDomainName, (uint8*)pcHttpServerDomainName, 64);
			strProvConfig.u8EnableRedirect = bEnableHttpRedirect;

			s8Ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_START_PROVISION_MODE | M2M_REQ_DATA_PKT, 
						(uint8*)&strProvConfig, sizeof(tstrM2MProvisionModeConfig), NULL, 0, 0);
		}
		else
		{
			/*goto ERR1;*/
		}
	}
ERR1:
	return s8Ret;
}

sint8 m2m_wifi_stop_provision_mode(void)
{
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_STOP_PROVISION_MODE, NULL, 0, NULL, 0, 0);
}

sint8 m2m_wifi_get_connection_info(void)
{
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_GET_CONN_INFO, NULL, 0, NULL, 0, 0);
}

sint8 m2m_wifi_set_sytem_time(uint32 u32UTCSeconds)
{
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_SYS_TIME, (uint8*)&u32UTCSeconds, sizeof(tstrSystemTime), NULL, 0, 0);
}
/*!
 * @fn             NMI_API sint8 m2m_wifi_get_sytem_time(void);   
 * @see            m2m_wifi_enable_sntp
 			  		tstrSystemTime   
 * @note         get the system time from the sntp client
 *		         using the API \ref m2m_wifi_get_sytem_time.
 * @return        The function returns @ref M2M_SUCCESS for successful operations and a negative value otherwise.
 */
sint8 m2m_wifi_get_sytem_time(void)
{
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_GET_SYS_TIME, NULL,0, NULL, 0, 0);
}

sint8 m2m_wifi_enable_sntp(uint8 bEnable)
{
	uint8	u8Req;

	u8Req = bEnable ? M2M_WIFI_REQ_ENABLE_SNTP_CLIENT : M2M_WIFI_REQ_DISABLE_SNTP_CLIENT;
	return hif_send(M2M_REQ_GROUP_WIFI, u8Req, NULL, 0, NULL, 0, 0);
}
/*!
@fn			NMI_API sint8 m2m_wifi_set_power_profile(uint8 u8PwrMode);
@brief		Change the power profile mode 
@param [in]	u8PwrMode
			Change the WINC power profile to different mode 
			PWR_LOW1/PWR_LOW2/PWR_HIGH/PWR_AUTO (tenuM2mPwrMode)
@return		The function SHALL return @ref M2M_SUCCESS for success and a negative value otherwise.
@sa			tenuM2mPwrMode
@pre		m2m_wifi_init
@warning	must be called after the initializations and before any connection request and can't be changed in run time, 
*/
sint8 m2m_wifi_set_power_profile(uint8 u8PwrMode)
{
	sint8 ret = M2M_SUCCESS;
	tstrM2mPwrMode strM2mPwrMode;
	strM2mPwrMode.u8PwrMode = u8PwrMode;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_POWER_PROFILE, (uint8*)&strM2mPwrMode,sizeof(tstrM2mPwrMode), NULL, 0, 0);
	return ret;
}
/*!
@fn			NMI_API sint8 m2m_wifi_set_tx_power(uint8 u8TxPwrLevel);
@brief		set the TX power tenuM2mTxPwrLevel
@param [in]	u8TxPwrLevel
			change the TX power tenuM2mTxPwrLevel
@return		The function SHALL return @ref M2M_SUCCESS for success and a negative value otherwise.
@sa			tenuM2mTxPwrLevel
@pre		m2m_wifi_init
@warning	
*/
sint8 m2m_wifi_set_tx_power(uint8 u8TxPwrLevel)
{
	sint8 ret = M2M_SUCCESS;
	tstrM2mTxPwrLevel strM2mTxPwrLevel;
	strM2mTxPwrLevel.u8TxPwrLevel = u8TxPwrLevel;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_TX_POWER, (uint8*)&strM2mTxPwrLevel,sizeof(tstrM2mTxPwrLevel), NULL, 0, 0);
	return ret;
}

/*!
@fn			NMI_API sint8 m2m_wifi_enable_firmware_logs(uint8 u8Enable);
@brief		Enable or Disable logs in run time (Disable Firmware logs will 
			enhance the firmware start-up time and performance)
@param [in]	u8Enable
			Set 1 to enable the logs 0 for disable
@return		The function SHALL return @ref M2M_SUCCESS for success and a negative value otherwise.
@sa			__DISABLE_FIRMWARE_LOGS__ (build option to disable logs from initializations)
@pre		m2m_wifi_init
@warning	
*/
sint8 m2m_wifi_enable_firmware_logs(uint8 u8Enable)
{
	sint8 ret = M2M_SUCCESS;
	tstrM2mEnableLogs strM2mEnableLogs;
	strM2mEnableLogs.u8Enable = u8Enable;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_ENABLE_LOGS, (uint8*)&strM2mEnableLogs,sizeof(tstrM2mEnableLogs), NULL, 0, 0);
	return ret;
}

/*!
@fn			NMI_API sint8 m2m_wifi_set_battery_voltage(uint16 u16BattVoltx100);
@brief		Enable or Disable logs in run time (Disable Firmware logs will 
			enhance the firmware start-up time and performance)
@param [in]	u16BattVoltx100
			battery voltage multiplied by 100
@return		The function SHALL return @ref M2M_SUCCESS for success and a negative value otherwise.
@sa			__DISABLE_FIRMWARE_LOGS__ (build option to disable logs from initializations)
@pre		m2m_wifi_init
@warning	
*/
sint8 m2m_wifi_set_battery_voltage(uint16 u16BattVoltx100)
{
	sint8 ret = M2M_SUCCESS;
	tstrM2mBatteryVoltage strM2mBattVol = {0};
	strM2mBattVol.u16BattVolt = u16BattVoltx100;
	ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_BATTERY_VOLTAGE, (uint8*)&strM2mBattVol,sizeof(tstrM2mBatteryVoltage), NULL, 0, 0);
	return ret;
}
/*!
@fn        	 	 sint8 m2m_wifi_prng_get_random_bytes(uint8 * pu8PrngBuff,uint16 u16PrngSize)
@brief     	 Get random bytes using the PRNG bytes.	      
@param [in]    u16PrngSize
		  	 Size of the required random bytes to be generated.   	 
@param [in]    pu8PrngBuff
		        Pointer to user allocated buffer.  		            
@return           The function SHALL return M2M_SUCCESE for success and a negative value otherwise.
*/
sint8 m2m_wifi_prng_get_random_bytes(uint8 * pu8PrngBuff,uint16 u16PrngSize)
{
	sint8 ret = M2M_ERR_FAIL;
	tstrPrng   strRng = {0};
	if((u16PrngSize < (M2M_BUFFER_MAX_SIZE - sizeof(tstrPrng)))&&(pu8PrngBuff != NULL))
	{
		strRng.u16PrngSize = u16PrngSize;
		strRng.pu8RngBuff  = pu8PrngBuff;
		ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_GET_PRNG|M2M_REQ_DATA_PKT,(uint8 *)&strRng, sizeof(tstrPrng),NULL,0, 0);
	}
	else
	{
		M2M_ERR("PRNG Buffer exceeded maximum size %d or NULL Buffer\n",u16PrngSize);
	}
	return ret;
}
#ifdef ETH_MODE
/*!
@fn	\
	 NMI_API sint8 m2m_wifi_enable_mac_mcast(uint8* pu8MulticastMacAddress, uint8 u8AddRemove)

@brief
	Add MAC filter to receive Multicast packets.

@param [in]	pu8MulticastMacAddress
				Pointer to the MAC address.
@param [in] u8AddRemove
				Flag to Add/Remove MAC address.
@return		
	The function SHALL return 0 for success and a negative value otherwise.
*/

NMI_API sint8 m2m_wifi_enable_mac_mcast(uint8* pu8MulticastMacAddress, uint8 u8AddRemove)
{
	sint8 s8ret = M2M_ERR_FAIL;
	tstrM2MMulticastMac  strMulticastMac;

	if(pu8MulticastMacAddress != NULL )
	{
		strMulticastMac.u8AddRemove = u8AddRemove;
		m2m_memcpy(strMulticastMac.au8macaddress,pu8MulticastMacAddress,M2M_MAC_ADDRES_LEN);
		M2M_DBG("mac multicast: %x:%x:%x:%x:%x:%x\r\n",strMulticastMac.au8macaddress[0],strMulticastMac.au8macaddress[1],strMulticastMac.au8macaddress[2],strMulticastMac.au8macaddress[3],strMulticastMac.au8macaddress[4],strMulticastMac.au8macaddress[5]);
		s8ret = hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_SET_MAC_MCAST, (uint8 *)&strMulticastMac,sizeof(tstrM2MMulticastMac),NULL,0,0);
	}

	return s8ret;

}

/*!
@fn	\
	NMI_API sint8  m2m_wifi_set_receive_buffer(void* pvBuffer,uint16 u16BufferLen);

@brief
	set the ethernet receive buffer, should be called in the receive call back.

@param [in]	pvBuffer
				Pointer to the ethernet receive buffer.
@param [in] u16BufferLen
				Length of the buffer.

@return		
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8  m2m_wifi_set_receive_buffer(void* pvBuffer,uint16 u16BufferLen)
{
	sint8 s8ret = M2M_SUCCESS;
	if(pvBuffer != NULL)
	{
		gau8ethRcvBuf = pvBuffer;
		gu16ethRcvBufSize= u16BufferLen;
	}
	else
	{
		s8ret = M2M_ERR_FAIL;
		M2M_ERR("Buffer NULL pointer\r\n");
	}
	return s8ret;
}
#endif

/*!
@fn	\
	NMI_API sint8  m2m_wifi_ble_api_send(const uint8* const msg, const uint32 len);

@brief
	Send an encapsulated Atmel BLE API message 

@param [in]	msg
				Pointer to the Atmel BLE API message raw bytes
@param [in] len
				Length of the msg

@return		
	The function SHALL return 0 for success and a negative value otherwise.
*/
NMI_API sint8 m2m_wifi_ble_api_send(uint8* msg,  uint32 len)
{
	tstrM2mBleApiMsg bleTx;
	bleTx.u16Len = len;
	return hif_send(M2M_REQ_GROUP_WIFI, M2M_WIFI_REQ_BLE_API_SEND | M2M_REQ_DATA_PKT,
		(uint8*)&bleTx, sizeof(tstrM2mBleApiMsg), msg, len, sizeof(tstrM2mBleApiMsg));
}
