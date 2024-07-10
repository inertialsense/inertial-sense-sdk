#include "imx_defaults.h"




// Return 0 if not valid.  Zero is not a valid platform type.
int imxPlatformConfigTypeValid(uint32_t platformConfig)
{
    uint32_t platformType = platformConfig&PLATFORM_CFG_TYPE_MASK;
    return (platformType > 0) && (platformType < PLATFORM_CFG_TYPE_COUNT);
}

void imxPlatformConfigErrorCheck(uint32_t *platformConfig)
{
    uint32_t type = *platformConfig & PLATFORM_CFG_TYPE_MASK;
    uint32_t preset = (*platformConfig & PLATFORM_CFG_PRESET_MASK) >> PLATFORM_CFG_PRESET_OFFSET;

    // Error check type
    if (type > PLATFORM_CFG_TYPE_COUNT)
    {
        type = PLATFORM_CFG_TYPE_NONE;
    }

    // Error check preset
    switch (type)
    {
    case PLATFORM_CFG_TYPE_RUG2_1_G0:
    case PLATFORM_CFG_TYPE_RUG3_G0:
        if (preset > PLATFORM_CFG_RUG3_PRESET__COUNT)
        {
            preset = PLATFORM_CFG_RUG3_PRESET__G0_DEFAULT;
        }
        break;
    case PLATFORM_CFG_TYPE_RUG2_1_G1:
    case PLATFORM_CFG_TYPE_RUG2_1_G2:
    case PLATFORM_CFG_TYPE_RUG3_G1:
    case PLATFORM_CFG_TYPE_RUG3_G2:
        if (preset > PLATFORM_CFG_RUG3_PRESET__COUNT)
        {
            preset = PLATFORM_CFG_RUG3_PRESET__G0_DEFAULT;
        }
        break;
    }

    *platformConfig &= (PLATFORM_CFG_TYPE_MASK | PLATFORM_CFG_PRESET_MASK);
    *platformConfig |= type | preset << PLATFORM_CFG_PRESET_OFFSET;
}

void imxPlatformConfigToRug3FlashCfgIoConfig(uint32_t *ioConfig, uint32_t platformConfig);
void imxPlatformConfigToRug3FlashCfgIoConfig(uint32_t *ioConfig, uint32_t platformConfig)
{
    uint32_t type = platformConfig&PLATFORM_CFG_TYPE_MASK;
    uint32_t preset = (platformConfig&PLATFORM_CFG_PRESET_MASK)>>PLATFORM_CFG_PRESET_OFFSET;

    switch (type)
    {
    default:    // Not RUG-3
        return;

    case PLATFORM_CFG_TYPE_RUG2_1_G0:
    case PLATFORM_CFG_TYPE_RUG2_1_G1:
    case PLATFORM_CFG_TYPE_RUG2_1_G2:
    case PLATFORM_CFG_TYPE_RUG3_G0:
    case PLATFORM_CFG_TYPE_RUG3_G1:
    case PLATFORM_CFG_TYPE_RUG3_G2:
        break;  // RUG-3
    }

    // ioConfig - P8,P10 (G1,G2)
    *ioConfig &= ~IO_CONFIG_G1G2_MASK;
    switch (preset)
    {   // CAN
    case PLATFORM_CFG_RUG3_PRESET__1__S0_RS232_7_9___CAN_11_12______S1_GPS1:
    case PLATFORM_CFG_RUG3_PRESET__2__S0_TTL_7_9_____CAN_11_12______S1_GPS1:
    case PLATFORM_CFG_RUG3_PRESET__8_________________CAN_11_12______S1_GPS1__S0_GPS2:
        *ioConfig |= IO_CONFIG_G1G2_CAN_BUS;
        break;
        // Ser 2
    case PLATFORM_CFG_RUG3_PRESET__3__S0_TTL_7_9_____S2_TTL_8_10____S1_GPS1:
    case PLATFORM_CFG_RUG3_PRESET__5__S1_RS485_7_8_9_10_____________S2_GPS1__S0_GPS2:
    case PLATFORM_CFG_RUG3_PRESET__9__S2_TTL_8_10___________________S1_GPS1__S0_GPS2:
        *ioConfig |= IO_CONFIG_G1G2_COM2;
        break;
    }

    // ioConfig - P7-P10 (G3,G4,G5,G8)
     *ioConfig &= ~IO_CONFIG_G5G8_MASK;
    switch (preset)  
    {   // RUG-3 P7P9 (G3,G4) - SPI
    case PLATFORM_CFG_RUG3_PRESET__6__SPI_7_8_9_10__________________S2_GPS1__S0_GPS2:
        *ioConfig |= IO_CONFIG_G5G8_G6G7_SPI_ENABLE;
        break;
        // RUG-3 P7P9 (G3,G4) - No strobe available
    default:
        *ioConfig |= IO_CONFIG_G5G8_DEFAULT;
        break;
    }

    // GPS Ports
    switch (preset)
    {
    case PLATFORM_CFG_RUG3_PRESET__1__S0_RS232_7_9___CAN_11_12______S1_GPS1:
    case PLATFORM_CFG_RUG3_PRESET__2__S0_TTL_7_9_____CAN_11_12______S1_GPS1:
    case PLATFORM_CFG_RUG3_PRESET__3__S0_TTL_7_9_____S2_TTL_8_10____S1_GPS1:
    case PLATFORM_CFG_RUG3_PRESET__4__S0_RS232_7_9___S1_RS232_8_10__S2_GPS1:
        SET_IO_CFG_GPS1_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER1);
        SET_IO_CFG_GPS2_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_DISABLE);
        break;
    case PLATFORM_CFG_RUG3_PRESET__5__S1_RS485_7_8_9_10_____________S2_GPS1__S0_GPS2:
    case PLATFORM_CFG_RUG3_PRESET__6__SPI_7_8_9_10__________________S2_GPS1__S0_GPS2:
    case PLATFORM_CFG_RUG3_PRESET__7__S1_RS232_8_10_________________S2_GPS1__S0_GPS2:
        SET_IO_CFG_GPS1_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER2);
        SET_IO_CFG_GPS2_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER0);
        break;
    case PLATFORM_CFG_RUG3_PRESET__8_________________CAN_11_12______S1_GPS1__S0_GPS2:
    case PLATFORM_CFG_RUG3_PRESET__9__S2_TTL_8_10___________________S1_GPS1__S0_GPS2:
        SET_IO_CFG_GPS1_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER1);
        SET_IO_CFG_GPS2_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER0);
        break;
    }

    // Disable GPS if not available
    switch(type)
    {
    case PLATFORM_CFG_TYPE_RUG2_1_G0:
    case PLATFORM_CFG_TYPE_RUG3_G0:   
        SET_IO_CFG_GPS1_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_DISABLE);
        break;
    }
    switch(type)
    {
    case PLATFORM_CFG_TYPE_RUG2_1_G0:
    case PLATFORM_CFG_TYPE_RUG3_G0:
    case PLATFORM_CFG_TYPE_RUG2_1_G1:
    case PLATFORM_CFG_TYPE_RUG3_G1:   
        SET_IO_CFG_GPS2_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_DISABLE);
        break;
    }

    // GPS type: ZED-F9P
    SET_IO_CFG_GPS1_TYPE(*ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
    SET_IO_CFG_GPS2_TYPE(*ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
}

void imxPlatformConfigToFlashCfgIoConfig(uint32_t *ioConfig, uint32_t platformConfig)
{
    uint32_t type = platformConfig&PLATFORM_CFG_TYPE_MASK;

    switch (type)
    {
    default:    // No GPS 
    case PLATFORM_CFG_TYPE_NONE:
        SET_IO_CFG_GPS1_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_DISABLE);
        SET_IO_CFG_GPS2_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_DISABLE);
        SET_IO_CFG_GPS1_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_M8);
        SET_IO_CFG_GPS2_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_M8);
        break;

    case PLATFORM_CFG_TYPE_NONE_ONBOARD_G2:
    case PLATFORM_CFG_TYPE_RUG1:
        SET_IO_CFG_GPS1_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_ONBOARD_1);
        SET_IO_CFG_GPS2_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_ONBOARD_2);
        SET_IO_CFG_GPS1_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_M8);
        SET_IO_CFG_GPS2_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_M8);
        break;

    case PLATFORM_CFG_TYPE_EVB2_G2:
        SET_IO_CFG_GPS1_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER1);
        SET_IO_CFG_GPS2_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER2);
        SET_IO_CFG_GPS1_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        SET_IO_CFG_GPS2_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        break;

    case PLATFORM_CFG_TYPE_RUG2_0_G1:
        SET_IO_CFG_GPS1_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER1);
        SET_IO_CFG_GPS2_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_DISABLE);
        SET_IO_CFG_GPS1_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        SET_IO_CFG_GPS2_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        break;

    case PLATFORM_CFG_TYPE_RUG2_0_G2:
        SET_IO_CFG_GPS1_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER1);
        SET_IO_CFG_GPS2_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER0);
        SET_IO_CFG_GPS1_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        SET_IO_CFG_GPS2_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        break;

    case PLATFORM_CFG_TYPE_RUG2_1_G1:
    case PLATFORM_CFG_TYPE_RUG2_1_G2:
    case PLATFORM_CFG_TYPE_RUG3_G1:
    case PLATFORM_CFG_TYPE_RUG3_G2:
        imxPlatformConfigToRug3FlashCfgIoConfig(ioConfig, platformConfig);
        break;

    case PLATFORM_CFG_TYPE_IG1_G1:
        SET_IO_CFG_GPS1_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER0);
        SET_IO_CFG_GPS2_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_DISABLE);
        SET_IO_CFG_GPS1_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        SET_IO_CFG_GPS2_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        break;

    case PLATFORM_CFG_TYPE_IG1_0_G2:
    case PLATFORM_CFG_TYPE_IG1_G2:
        SET_IO_CFG_GPS1_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER0);
        SET_IO_CFG_GPS2_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER2);
        SET_IO_CFG_GPS1_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        SET_IO_CFG_GPS2_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        break;

    case PLATFORM_CFG_TYPE_IG2:
    case PLATFORM_CFG_TYPE_TBED3:
        SET_IO_CFG_GPS1_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER0);
        SET_IO_CFG_GPS2_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER0);
        SET_IO_CFG_GPS1_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_GPX);
        SET_IO_CFG_GPS2_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_GPX);
        break;

    case PLATFORM_CFG_TYPE_LAMBDA_G1:
        SET_IO_CFG_GPS1_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER1);
        SET_IO_CFG_GPS2_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_DISABLE);
        SET_IO_CFG_GPS1_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        SET_IO_CFG_GPS2_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        break;

    case PLATFORM_CFG_TYPE_LAMBDA_G2:
        SET_IO_CFG_GPS1_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER1);
        SET_IO_CFG_GPS2_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER2);
        SET_IO_CFG_GPS1_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        SET_IO_CFG_GPS2_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        break;

    case PLATFORM_CFG_TYPE_TBED2_G1_W_LAMBDA:
    case PLATFORM_CFG_TYPE_TBED2_G2_W_LAMBDA:
        SET_IO_CFG_GPS1_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_SER2);
        SET_IO_CFG_GPS2_SOURCE(*ioConfig, IO_CONFIG_GPS_SOURCE_DISABLE);
        SET_IO_CFG_GPS1_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        SET_IO_CFG_GPS2_TYPE(  *ioConfig, IO_CONFIG_GPS_TYPE_UBX_F9P);
        *ioConfig |= IO_CONFIG_GPS1_NO_INIT;
        *ioConfig |= IO_CONFIG_GPS2_NO_INIT;
        break;
    }

    // GPS timepulse source
    *ioConfig &= ~IO_CFG_GPS_TIMEPUSE_SOURCE_BITMASK;
    switch (type)
    {
        // Disabled
    case PLATFORM_CFG_TYPE_RUG2_1_G0:
    case PLATFORM_CFG_TYPE_RUG3_G0:               
    case PLATFORM_CFG_TYPE_NONE:
        break;
        // G8
    case PLATFORM_CFG_TYPE_EVB2_G2:
    case PLATFORM_CFG_TYPE_RUG2_0_G1:
    case PLATFORM_CFG_TYPE_RUG2_0_G2:
    case PLATFORM_CFG_TYPE_IG1_0_G2:
        *ioConfig |= IO_CFG_GPS_TIMEPUSE_SOURCE_STROBE_G8_PIN12<<IO_CFG_GPS_TIMEPUSE_SOURCE_OFFSET;
        break;
        // G5
    case PLATFORM_CFG_TYPE_TBED3:
        *ioConfig |= IO_CFG_GPS_TIMEPUSE_SOURCE_STROBE_G5_PIN9<<IO_CFG_GPS_TIMEPUSE_SOURCE_OFFSET;
        break;
        // G9
    case PLATFORM_CFG_TYPE_RUG2_1_G1:
    case PLATFORM_CFG_TYPE_RUG2_1_G2:
        *ioConfig |= IO_CFG_GPS_TIMEPUSE_SOURCE_STROBE_G9_PIN13<<IO_CFG_GPS_TIMEPUSE_SOURCE_OFFSET;
        break;
        // GPS1 PPS (pin 20)
    default:
        *ioConfig |= IO_CFG_GPS_TIMEPUSE_SOURCE_GNSS_PPS_PIN20<<IO_CFG_GPS_TIMEPUSE_SOURCE_OFFSET;
        break;
    }
}

void imxPlatformConfigTypeToFlashCfgIoConfig(uint32_t *ioConfig, uint32_t platformType)
{
    imxPlatformConfigToFlashCfgIoConfig(ioConfig, imxPlatformConfigTypeToDefaultPlatformConfig(platformType));
}

// Return default platformConfig based on platformType
uint32_t imxPlatformConfigTypeToDefaultPlatformConfig(uint32_t platformType)
{
    return platformType | (imxPlatformConfigTypeToDefaultPlatformPreset(platformType) << PLATFORM_CFG_PRESET_OFFSET);
}

// Return default platform preset based on platformType
uint32_t imxPlatformConfigTypeToDefaultPlatformPreset(uint32_t platformType)
{
    switch (platformType)
    {
    case PLATFORM_CFG_TYPE_RUG2_1_G0:
    case PLATFORM_CFG_TYPE_RUG3_G0:
        return PLATFORM_CFG_RUG3_PRESET__G0_DEFAULT;
        break;
    case PLATFORM_CFG_TYPE_RUG2_1_G1:
    case PLATFORM_CFG_TYPE_RUG2_1_G2:
    case PLATFORM_CFG_TYPE_RUG3_G1:
    case PLATFORM_CFG_TYPE_RUG3_G2:
        return PLATFORM_CFG_RUG3_PRESET__G2_DEFAULT;
        break;
    default:
        return 0;
    }
}

uint32_t imxMinNavOutputMs(nvm_flash_cfg_t *cfg)
{
    if (cfg->sysCfgBits & SYS_CFG_BITS_DISABLE_INS_EKF)
    {
        return 4;
    }
    else if (!(cfg->sysCfgBits & SYS_CFG_BITS_DISABLE_GPS1_FUSION) ||
             !(cfg->sysCfgBits & SYS_CFG_BITS_DISABLE_GPS2_FUSION))
    {   // Nav: GPS enabled
        return tNAV_MIN_PERIOD_MS_NAV_MODE;
    }
    else if (!(cfg->sysCfgBits & SYS_CFG_BITS_DISABLE_MAGNETOMETER_FUSION) ||
             !(cfg->sysCfgBits & SYS_CFG_BITS_DISABLE_AUTO_ZERO_ANGULAR_RATE_UPDATES))
    {   // AHRS: no GPS, magnetometer enabled
        return tNAV_MIN_PERIOD_MS_AHRS_MODE;
    }
    else
    {   // VRS: no GPS or magnetometer
        return tNAV_MIN_PERIOD_MS_VRS_MODE;
    }
}



