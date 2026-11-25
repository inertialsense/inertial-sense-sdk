/**
 * @file gpx_flash_cfg.c
 * @brief Converters between gpx_flash_cfg_t and the domains that hold its values, and compile-time checks of every
 *        member, mask and index the registry names in it.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "cbor/did/gpx_flash_cfg.h"

/** Lowest set bit of a mask: the unit of the value it holds */
#define CBOR_BIND_LSB(m)    ((uint32_t)(m) & (0u - (uint32_t)(m)))

/**
 * Fills the CONFIG_SYSTEM keys held in gpx_flash_cfg_t into the view.
 *
 * Keys of CONFIG_SYSTEM that live elsewhere are left as they are, so the view of a domain whose keys live in
 * several structs is filled by each of their mappings in turn.
 *
 * @param view  [out] the view
 * @param src   the gpx_flash_cfg_t the keys are read from
 */
void cbor_gpx_flash_cfg_to_config_system(cbor_config_system_t *view, const gpx_flash_cfg_t *src)
{
    view->brownOutLevel = (uint8_t)(((uint32_t)src->sysCfgBits & ((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK))) / CBOR_BIND_LSB(((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK))));
    view->debug = src->debug;
}

/**
 * Checks that every CONFIG_SYSTEM value held in gpx_flash_cfg_t fits its home (a packed field or a bool).
 *
 * A domain whose keys live in several structs is written all or nothing by checking each of their mappings
 * before writing any of them.
 *
 * @param view  the view
 * @return true if cbor_gpx_flash_cfg_from_config_system() would write the view
 */
bool cbor_gpx_flash_cfg_fits_config_system(const cbor_config_system_t *view)
{
    if (((uint64_t)view->brownOutLevel * CBOR_BIND_LSB(((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK)))) & ~(uint64_t)((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK))) return false;
    return true;
}

/**
 * Writes the CONFIG_SYSTEM keys held in gpx_flash_cfg_t from the view, all or nothing.
 *
 * Bits and members of gpx_flash_cfg_t that no key covers are kept. Nothing is written unless every value fits its
 * home (a packed field or a bool).
 *
 * @param dst   the gpx_flash_cfg_t the keys are written to
 * @param view  the view
 * @return false, writing nothing, if a value does not fit its home
 */
bool cbor_gpx_flash_cfg_from_config_system(gpx_flash_cfg_t *dst, const cbor_config_system_t *view)
{
    if (((uint64_t)view->brownOutLevel * CBOR_BIND_LSB(((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK)))) & ~(uint64_t)((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK))) return false;
    dst->sysCfgBits = (dst->sysCfgBits & ~((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK))) | (((uint32_t)view->brownOutLevel * CBOR_BIND_LSB(((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK)))) & ((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK)));
    dst->debug = view->debug;
    return true;
}

/**
 * Fills the CONFIG_PORTS keys held in gpx_flash_cfg_t into the view.
 *
 * Keys of CONFIG_PORTS that live elsewhere are left as they are, so the view of a domain whose keys live in
 * several structs is filled by each of their mappings in turn.
 *
 * @param view  [out] the view
 * @param src   the gpx_flash_cfg_t the keys are read from
 */
void cbor_gpx_flash_cfg_to_config_ports(cbor_config_ports_t *view, const gpx_flash_cfg_t *src)
{
    view->Port[CBOR_PORT_ID_SER0].baudRate = src->ser0BaudRate;
    view->Port[CBOR_PORT_ID_SER1].baudRate = src->ser1BaudRate;
    view->Port[CBOR_PORT_ID_SER2].baudRate = src->ser2BaudRate;
}

/**
 * Checks that every CONFIG_PORTS value held in gpx_flash_cfg_t fits its home (a packed field or a bool).
 *
 * A domain whose keys live in several structs is written all or nothing by checking each of their mappings
 * before writing any of them.
 *
 * @param view  the view
 * @return true if cbor_gpx_flash_cfg_from_config_ports() would write the view
 */
bool cbor_gpx_flash_cfg_fits_config_ports(const cbor_config_ports_t *view)
{
    (void)view;
    return true;
}

/**
 * Writes the CONFIG_PORTS keys held in gpx_flash_cfg_t from the view, all or nothing.
 *
 * Bits and members of gpx_flash_cfg_t that no key covers are kept. Nothing is written unless every value fits its
 * home (a packed field or a bool).
 *
 * @param dst   the gpx_flash_cfg_t the keys are written to
 * @param view  the view
 * @return false, writing nothing, if a value does not fit its home
 */
bool cbor_gpx_flash_cfg_from_config_ports(gpx_flash_cfg_t *dst, const cbor_config_ports_t *view)
{
    dst->ser0BaudRate = view->Port[CBOR_PORT_ID_SER0].baudRate;
    dst->ser1BaudRate = view->Port[CBOR_PORT_ID_SER1].baudRate;
    dst->ser2BaudRate = view->Port[CBOR_PORT_ID_SER2].baudRate;
    return true;
}

/**
 * Fills the CONFIG_GNSS keys held in gpx_flash_cfg_t into the view.
 *
 * Keys of CONFIG_GNSS that live elsewhere are left as they are, so the view of a domain whose keys live in
 * several structs is filled by each of their mappings in turn.
 *
 * @param view  [out] the view
 * @param src   the gpx_flash_cfg_t the keys are read from
 */
void cbor_gpx_flash_cfg_to_config_gnss(cbor_config_gnss_t *view, const gpx_flash_cfg_t *src)
{
    view->periodMs = src->startupGnssDtMs;
    view->timeSync.periodMs = src->gnssTimeSyncPeriodMs;
    view->timeSync.userDelayS = src->gnssTimeUserDelay;
    view->externalAiding.fixedLatency = ((src->sysCfgBits & ((uint32_t)(GPX_SYS_CFG_BITS_EXT_AIDING_LATENCY))) != 0u);
    view->Receivers.satSignals = src->gnssSatSigConst;
    view->Receivers.minElevation = src->gnssMinimumElevation;
    view->Receivers.cn0Min = src->gnssCn0Minimum;
    view->Receivers.cn0DynOffset = src->gnssCn0DynMinOffset;
    view->Receivers.dynamicModel = src->dynamicModel;
    view->Antennas.powerEnabled = ((src->sysCfgBits & ((uint32_t)(GPX_SYS_CFG_BITS_DISABLE_VCC_RF))) == 0u);
    memcpy(view->Antenna[CBOR_ANTENNA_ID_ANT1].offset, src->gnss1AntOffset, sizeof(view->Antenna[CBOR_ANTENNA_ID_ANT1].offset));
    memcpy(view->Antenna[CBOR_ANTENNA_ID_ANT2].offset, src->gnss2AntOffset, sizeof(view->Antenna[CBOR_ANTENNA_ID_ANT2].offset));
    view->Rtk.mode = (uint8_t)(((uint32_t)src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK))) / CBOR_BIND_LSB(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK))));
    memcpy(view->Rtk.basePosition, src->refLla, sizeof(view->Rtk.basePosition));
    view->Rtk.basePositionMoving = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING))) != 0u);
    view->Rtk.baseIdenticalToRover = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER))) != 0u);
    view->Rtk.passThrough = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH))) != 0u);
    view->Rtk.multipathMitigation = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser0 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser1 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser2 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].usb = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser0 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser1 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser2 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].usb = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser0 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser1 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser2 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].usb = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser0 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser1 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser2 = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2))) != 0u);
    view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].usb = ((src->RTKCfgBits & ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB))) != 0u);
}

/**
 * Checks that every CONFIG_GNSS value held in gpx_flash_cfg_t fits its home (a packed field or a bool).
 *
 * A domain whose keys live in several structs is written all or nothing by checking each of their mappings
 * before writing any of them.
 *
 * @param view  the view
 * @return true if cbor_gpx_flash_cfg_from_config_gnss() would write the view
 */
bool cbor_gpx_flash_cfg_fits_config_gnss(const cbor_config_gnss_t *view)
{
    if (view->externalAiding.fixedLatency > 1u) return false;
    if (view->Antennas.powerEnabled > 1u) return false;
    if (((uint64_t)view->Rtk.mode * CBOR_BIND_LSB(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)))) & ~(uint64_t)((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK))) return false;
    if (view->Rtk.basePositionMoving > 1u) return false;
    if (view->Rtk.baseIdenticalToRover > 1u) return false;
    if (view->Rtk.passThrough > 1u) return false;
    if (view->Rtk.multipathMitigation > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].usb > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].usb > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].usb > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].usb > 1u) return false;
    return true;
}

/**
 * Writes the CONFIG_GNSS keys held in gpx_flash_cfg_t from the view, all or nothing.
 *
 * Bits and members of gpx_flash_cfg_t that no key covers are kept. Nothing is written unless every value fits its
 * home (a packed field or a bool).
 *
 * @param dst   the gpx_flash_cfg_t the keys are written to
 * @param view  the view
 * @return false, writing nothing, if a value does not fit its home
 */
bool cbor_gpx_flash_cfg_from_config_gnss(gpx_flash_cfg_t *dst, const cbor_config_gnss_t *view)
{
    if (view->externalAiding.fixedLatency > 1u) return false;
    if (view->Antennas.powerEnabled > 1u) return false;
    if (((uint64_t)view->Rtk.mode * CBOR_BIND_LSB(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)))) & ~(uint64_t)((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK))) return false;
    if (view->Rtk.basePositionMoving > 1u) return false;
    if (view->Rtk.baseIdenticalToRover > 1u) return false;
    if (view->Rtk.passThrough > 1u) return false;
    if (view->Rtk.multipathMitigation > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].usb > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].usb > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].usb > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser0 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser1 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser2 > 1u) return false;
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].usb > 1u) return false;
    dst->startupGnssDtMs = view->periodMs;
    dst->gnssTimeSyncPeriodMs = view->timeSync.periodMs;
    dst->gnssTimeUserDelay = view->timeSync.userDelayS;
    if (view->externalAiding.fixedLatency) dst->sysCfgBits |= ((uint32_t)(GPX_SYS_CFG_BITS_EXT_AIDING_LATENCY)); else dst->sysCfgBits &= ~((uint32_t)(GPX_SYS_CFG_BITS_EXT_AIDING_LATENCY));
    dst->gnssSatSigConst = view->Receivers.satSignals;
    dst->gnssMinimumElevation = view->Receivers.minElevation;
    dst->gnssCn0Minimum = view->Receivers.cn0Min;
    dst->gnssCn0DynMinOffset = view->Receivers.cn0DynOffset;
    dst->dynamicModel = view->Receivers.dynamicModel;
    if (view->Antennas.powerEnabled) dst->sysCfgBits &= ~((uint32_t)(GPX_SYS_CFG_BITS_DISABLE_VCC_RF)); else dst->sysCfgBits |= ((uint32_t)(GPX_SYS_CFG_BITS_DISABLE_VCC_RF));
    memcpy(dst->gnss1AntOffset, view->Antenna[CBOR_ANTENNA_ID_ANT1].offset, sizeof(view->Antenna[CBOR_ANTENNA_ID_ANT1].offset));
    memcpy(dst->gnss2AntOffset, view->Antenna[CBOR_ANTENNA_ID_ANT2].offset, sizeof(view->Antenna[CBOR_ANTENNA_ID_ANT2].offset));
    dst->RTKCfgBits = (dst->RTKCfgBits & ~((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK))) | (((uint32_t)view->Rtk.mode * CBOR_BIND_LSB(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)))) & ((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)));
    memcpy(dst->refLla, view->Rtk.basePosition, sizeof(view->Rtk.basePosition));
    if (view->Rtk.basePositionMoving) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING));
    if (view->Rtk.baseIdenticalToRover) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER));
    if (view->Rtk.passThrough) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH));
    if (view->Rtk.multipathMitigation) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser0) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser1) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].ser2) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_UBLOX].usb) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser0) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser1) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].ser2) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS1_RTCM3].usb) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser0) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser1) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].ser2) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_UBLOX].usb) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser0) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser1) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].ser2) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2));
    if (view->Rtk.BaseOutput[CBOR_BASE_STREAM_GNSS2_RTCM3].usb) dst->RTKCfgBits |= ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)); else dst->RTKCfgBits &= ~((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB));
    return true;
}

/* Compile-time checks of every source the registry names */
_Static_assert(((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK)) != 0u, "CONFIG_SYSTEM.brownOutLevel: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_SYSTEM.brownOutLevel: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_SYSTEM.brownOutLevel: mask is wider than its member");
_Static_assert(((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK)) / CBOR_BIND_LSB(((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK))) <= UINT8_MAX, "CONFIG_SYSTEM.brownOutLevel: mask holds more than the key's type");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->debug, uint8_t: 1, default: 0), "CONFIG_SYSTEM.debug: type differs from its member");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->ser0BaudRate, uint32_t: 1, default: 0), "CONFIG_PORTS.Port.SER0.baudRate: type differs from its member");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->ser1BaudRate, uint32_t: 1, default: 0), "CONFIG_PORTS.Port.SER1.baudRate: type differs from its member");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->ser2BaudRate, uint32_t: 1, default: 0), "CONFIG_PORTS.Port.SER2.baudRate: type differs from its member");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->startupGnssDtMs, uint32_t: 1, default: 0), "CONFIG_GNSS.periodMs: type differs from its member");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnssTimeSyncPeriodMs, uint32_t: 1, default: 0), "CONFIG_GNSS.timeSync.periodMs: type differs from its member");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnssTimeUserDelay, float: 1, default: 0), "CONFIG_GNSS.timeSync.userDelayS: type differs from its member");
_Static_assert(((uint32_t)(GPX_SYS_CFG_BITS_EXT_AIDING_LATENCY)) != 0u, "CONFIG_GNSS.externalAiding.fixedLatency: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_GNSS.externalAiding.fixedLatency: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(GPX_SYS_CFG_BITS_EXT_AIDING_LATENCY)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_GNSS.externalAiding.fixedLatency: mask is wider than its member");
_Static_assert((((uint32_t)(GPX_SYS_CFG_BITS_EXT_AIDING_LATENCY)) & (((uint32_t)(GPX_SYS_CFG_BITS_EXT_AIDING_LATENCY)) - 1u)) == 0u, "CONFIG_GNSS.externalAiding.fixedLatency: a bool needs a single-bit mask");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnssSatSigConst, uint16_t: 1, default: 0), "CONFIG_GNSS.Receivers.satSignals: type differs from its member");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnssMinimumElevation, float: 1, default: 0), "CONFIG_GNSS.Receivers.minElevation: type differs from its member");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnssCn0Minimum, uint8_t: 1, default: 0), "CONFIG_GNSS.Receivers.cn0Min: type differs from its member");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnssCn0DynMinOffset, uint8_t: 1, default: 0), "CONFIG_GNSS.Receivers.cn0DynOffset: type differs from its member");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->dynamicModel, uint8_t: 1, default: 0), "CONFIG_GNSS.Receivers.dynamicModel: type differs from its member");
_Static_assert(((uint32_t)(GPX_SYS_CFG_BITS_DISABLE_VCC_RF)) != 0u, "CONFIG_GNSS.Antennas.powerEnabled: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->sysCfgBits) <= 4, "CONFIG_GNSS.Antennas.powerEnabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(GPX_SYS_CFG_BITS_DISABLE_VCC_RF)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->sysCfgBits) - 1)) == 0u, "CONFIG_GNSS.Antennas.powerEnabled: mask is wider than its member");
_Static_assert((((uint32_t)(GPX_SYS_CFG_BITS_DISABLE_VCC_RF)) & (((uint32_t)(GPX_SYS_CFG_BITS_DISABLE_VCC_RF)) - 1u)) == 0u, "CONFIG_GNSS.Antennas.powerEnabled: a bool needs a single-bit mask");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->gnss1AntOffset) == sizeof(((cbor_config_gnss_t *)0)->Antenna[CBOR_ANTENNA_ID_ANT1].offset), "CONFIG_GNSS.Antenna.ANT1.offset: array size differs from its member");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnss1AntOffset[0], float: 1, default: 0), "CONFIG_GNSS.Antenna.ANT1.offset: element type differs from its member");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->gnss2AntOffset) == sizeof(((cbor_config_gnss_t *)0)->Antenna[CBOR_ANTENNA_ID_ANT2].offset), "CONFIG_GNSS.Antenna.ANT2.offset: array size differs from its member");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->gnss2AntOffset[0], float: 1, default: 0), "CONFIG_GNSS.Antenna.ANT2.offset: element type differs from its member");
_Static_assert(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)) != 0u, "CONFIG_GNSS.Rtk.mode: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.mode: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.mode: mask is wider than its member");
_Static_assert(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)) / CBOR_BIND_LSB(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK))) <= UINT8_MAX, "CONFIG_GNSS.Rtk.mode: mask holds more than the key's type");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->refLla) == sizeof(((cbor_config_gnss_t *)0)->Rtk.basePosition), "CONFIG_GNSS.Rtk.basePosition: array size differs from its member");
_Static_assert(_Generic(((gpx_flash_cfg_t *)0)->refLla[0], double: 1, default: 0), "CONFIG_GNSS.Rtk.basePosition: element type differs from its member");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)) != 0u, "CONFIG_GNSS.Rtk.basePositionMoving: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.basePositionMoving: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.basePositionMoving: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)) & (((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.basePositionMoving: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)) != 0u, "CONFIG_GNSS.Rtk.baseIdenticalToRover: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.baseIdenticalToRover: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.baseIdenticalToRover: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)) & (((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.baseIdenticalToRover: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)) != 0u, "CONFIG_GNSS.Rtk.passThrough: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.passThrough: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.passThrough: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)) & (((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.passThrough: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)) != 0u, "CONFIG_GNSS.Rtk.multipathMitigation: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.multipathMitigation: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.multipathMitigation: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)) & (((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.multipathMitigation: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser0: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser0: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser0: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser0: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser1: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser1: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser1: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser1: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser2: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser2: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser2: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser2: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.usb: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.usb: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.usb: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.usb: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser0: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser0: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser0: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser0: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser1: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser1: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser1: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser1: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser2: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser2: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser2: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser2: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.usb: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.usb: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.usb: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.usb: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser0: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser0: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser0: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser0: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser1: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser1: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser1: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser1: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser2: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser2: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser2: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser2: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.usb: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.usb: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.usb: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.usb: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser0: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser0: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser0: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser0: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser1: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser1: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser1: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser1: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser2: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser2: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser2: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser2: a bool needs a single-bit mask");
_Static_assert(((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)) != 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.usb: mask is zero");
_Static_assert(sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) <= 4, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.usb: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)) >> 1 >> (8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits) - 1)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.usb: mask is wider than its member");
_Static_assert((((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)) & (((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)) - 1u)) == 0u, "CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.usb: a bool needs a single-bit mask");
_Static_assert(((uint64_t)((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK)) + (uint64_t)((uint32_t)(GPX_SYS_CFG_BITS_EXT_AIDING_LATENCY)) + (uint64_t)((uint32_t)(GPX_SYS_CFG_BITS_DISABLE_VCC_RF)) + (uint64_t)((uint32_t)(0xFF3FFFFC))) == (uint64_t)(((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK)) | ((uint32_t)(GPX_SYS_CFG_BITS_EXT_AIDING_LATENCY)) | ((uint32_t)(GPX_SYS_CFG_BITS_DISABLE_VCC_RF)) | ((uint32_t)(0xFF3FFFFC))), "gpx_flash_cfg.sysCfgBits: two of CONFIG_SYSTEM.brownOutLevel, CONFIG_GNSS.externalAiding.fixedLatency, CONFIG_GNSS.Antennas.powerEnabled, gpx_flash_cfg.sysCfgBits not stored/pending overlap");
_Static_assert((((uint32_t)(GPX_SYS_CFG_BITS_BOR_THRESHOLD_MASK)) | ((uint32_t)(GPX_SYS_CFG_BITS_EXT_AIDING_LATENCY)) | ((uint32_t)(GPX_SYS_CFG_BITS_DISABLE_VCC_RF)) | ((uint32_t)(0xFF3FFFFC))) == (0xFFFFFFFFu >> (32 - 8 * sizeof(((gpx_flash_cfg_t *)0)->sysCfgBits))), "gpx_flash_cfg.sysCfgBits: a bit has no key and is not listed as not_stored or pending");
_Static_assert(((uint64_t)((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_RESERVED1)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CUR_PORT_MASK)) + (uint64_t)((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CLEAR_CUR_PORT)) + (uint64_t)((uint32_t)(0xF0000000))) == (uint64_t)(((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)) | ((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)) | ((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)) | ((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)) | ((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)) | ((uint32_t)(RTK_CFG_BITS_RESERVED1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CUR_PORT_MASK)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CLEAR_CUR_PORT)) | ((uint32_t)(0xF0000000))), "gpx_flash_cfg.RTKCfgBits: two of CONFIG_GNSS.Rtk.mode, CONFIG_GNSS.Rtk.basePositionMoving, CONFIG_GNSS.Rtk.baseIdenticalToRover, CONFIG_GNSS.Rtk.passThrough, CONFIG_GNSS.Rtk.multipathMitigation, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser0, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser1, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.ser2, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_UBLOX.usb, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser0, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser1, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.ser2, CONFIG_GNSS.Rtk.BaseOutput.GNSS1_RTCM3.usb, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser0, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser1, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.ser2, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_UBLOX.usb, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser0, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser1, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.ser2, CONFIG_GNSS.Rtk.BaseOutput.GNSS2_RTCM3.usb, gpx_flash_cfg.RTKCfgBits not stored/pending, gpx_flash_cfg.RTKCfgBits not stored/pending, gpx_flash_cfg.RTKCfgBits not stored/pending, gpx_flash_cfg.RTKCfgBits not stored/pending overlap");
_Static_assert((((uint32_t)(RTK_CFG_BITS_ROVER_MODE_MASK)) | ((uint32_t)(RTK_CFG_BITS_BASE_POS_MOVING)) | ((uint32_t)(RTK_CFG_BITS_RTK_BASE_IS_IDENTICAL_TO_ROVER)) | ((uint32_t)(RTK_CFG_BITS_GNSS_PORT_PASS_THROUGH)) | ((uint32_t)(RTK_CFG_BITS_MULTIPATH_MITIGATION)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_UBLOX_USB)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS1_RTCM3_USB)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_UBLOX_USB)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER0)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_SER2)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_GNSS2_RTCM3_USB)) | ((uint32_t)(RTK_CFG_BITS_RESERVED1)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CUR_PORT_MASK)) | ((uint32_t)(RTK_CFG_BITS_BASE_OUTPUT_RTCM3_CLEAR_CUR_PORT)) | ((uint32_t)(0xF0000000))) == (0xFFFFFFFFu >> (32 - 8 * sizeof(((gpx_flash_cfg_t *)0)->RTKCfgBits))), "gpx_flash_cfg.RTKCfgBits: a bit has no key and is not listed as not_stored or pending");
