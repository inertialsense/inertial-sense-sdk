/**
 * @file can_config.c
 * @brief Converters between can_config_t and the domains that hold its values, and compile-time checks of every member,
 *        mask and index the registry names in it.
 *
 * Generated from the CBOR key registry by is-common scripts/cbor_registry.py -- do not edit.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "cbor/did/can_config.h"

/** Lowest set bit of a mask: the unit of the value it holds */
#define CBOR_BIND_LSB(m)    ((uint32_t)(m) & (0u - (uint32_t)(m)))

/**
 * Fills the CONFIG_CAN keys held in can_config_t into the view.
 *
 * Keys of CONFIG_CAN that live elsewhere are left as they are, so the view of a domain whose keys live in
 * several structs is filled by each of their mappings in turn.
 *
 * @param view  [out] the view
 * @param src   the can_config_t the keys are read from
 */
void cbor_can_config_to_config_can(cbor_config_can_t *view, const can_config_t *src)
{
    view->bitRateKbps = (uint16_t)(((uint32_t)src->can_setting & ((uint32_t)(CAN_BAUDRATE_KBPS_MASK))) / CBOR_BIND_LSB(((uint32_t)(CAN_BAUDRATE_KBPS_MASK))));
    view->fdEnabled = ((src->can_setting & ((uint32_t)(CAN_BAUDRATE_KBPS_FD_ENABLE))) != 0u);
    view->receiveAddress = src->can_receive_address;
    view->Message[CBOR_CAN_MESSAGE_INS_TIME].periodMultiple = src->can_period_mult[0];
    view->Message[CBOR_CAN_MESSAGE_INS_TIME].transmitAddress = src->can_transmit_address[0];
    view->Message[CBOR_CAN_MESSAGE_INS_STATUS].periodMultiple = src->can_period_mult[1];
    view->Message[CBOR_CAN_MESSAGE_INS_STATUS].transmitAddress = src->can_transmit_address[1];
    view->Message[CBOR_CAN_MESSAGE_INS_EULER].periodMultiple = src->can_period_mult[2];
    view->Message[CBOR_CAN_MESSAGE_INS_EULER].transmitAddress = src->can_transmit_address[2];
    view->Message[CBOR_CAN_MESSAGE_INS_QUATN2B].periodMultiple = src->can_period_mult[3];
    view->Message[CBOR_CAN_MESSAGE_INS_QUATN2B].transmitAddress = src->can_transmit_address[3];
    view->Message[CBOR_CAN_MESSAGE_INS_QUATE2B].periodMultiple = src->can_period_mult[4];
    view->Message[CBOR_CAN_MESSAGE_INS_QUATE2B].transmitAddress = src->can_transmit_address[4];
    view->Message[CBOR_CAN_MESSAGE_INS_UVW].periodMultiple = src->can_period_mult[5];
    view->Message[CBOR_CAN_MESSAGE_INS_UVW].transmitAddress = src->can_transmit_address[5];
    view->Message[CBOR_CAN_MESSAGE_INS_VE].periodMultiple = src->can_period_mult[6];
    view->Message[CBOR_CAN_MESSAGE_INS_VE].transmitAddress = src->can_transmit_address[6];
    view->Message[CBOR_CAN_MESSAGE_INS_LAT].periodMultiple = src->can_period_mult[7];
    view->Message[CBOR_CAN_MESSAGE_INS_LAT].transmitAddress = src->can_transmit_address[7];
    view->Message[CBOR_CAN_MESSAGE_INS_LON].periodMultiple = src->can_period_mult[8];
    view->Message[CBOR_CAN_MESSAGE_INS_LON].transmitAddress = src->can_transmit_address[8];
    view->Message[CBOR_CAN_MESSAGE_INS_ALT].periodMultiple = src->can_period_mult[9];
    view->Message[CBOR_CAN_MESSAGE_INS_ALT].transmitAddress = src->can_transmit_address[9];
    view->Message[CBOR_CAN_MESSAGE_INS_NORTH_EAST].periodMultiple = src->can_period_mult[10];
    view->Message[CBOR_CAN_MESSAGE_INS_NORTH_EAST].transmitAddress = src->can_transmit_address[10];
    view->Message[CBOR_CAN_MESSAGE_INS_DOWN].periodMultiple = src->can_period_mult[11];
    view->Message[CBOR_CAN_MESSAGE_INS_DOWN].transmitAddress = src->can_transmit_address[11];
    view->Message[CBOR_CAN_MESSAGE_INS_ECEF_X].periodMultiple = src->can_period_mult[12];
    view->Message[CBOR_CAN_MESSAGE_INS_ECEF_X].transmitAddress = src->can_transmit_address[12];
    view->Message[CBOR_CAN_MESSAGE_INS_ECEF_Y].periodMultiple = src->can_period_mult[13];
    view->Message[CBOR_CAN_MESSAGE_INS_ECEF_Y].transmitAddress = src->can_transmit_address[13];
    view->Message[CBOR_CAN_MESSAGE_INS_ECEF_Z].periodMultiple = src->can_period_mult[14];
    view->Message[CBOR_CAN_MESSAGE_INS_ECEF_Z].transmitAddress = src->can_transmit_address[14];
    view->Message[CBOR_CAN_MESSAGE_INS_MSL].periodMultiple = src->can_period_mult[15];
    view->Message[CBOR_CAN_MESSAGE_INS_MSL].transmitAddress = src->can_transmit_address[15];
    view->Message[CBOR_CAN_MESSAGE_PREINT_PX].periodMultiple = src->can_period_mult[16];
    view->Message[CBOR_CAN_MESSAGE_PREINT_PX].transmitAddress = src->can_transmit_address[16];
    view->Message[CBOR_CAN_MESSAGE_PREINT_QY].periodMultiple = src->can_period_mult[17];
    view->Message[CBOR_CAN_MESSAGE_PREINT_QY].transmitAddress = src->can_transmit_address[17];
    view->Message[CBOR_CAN_MESSAGE_PREINT_RZ].periodMultiple = src->can_period_mult[18];
    view->Message[CBOR_CAN_MESSAGE_PREINT_RZ].transmitAddress = src->can_transmit_address[18];
    view->Message[CBOR_CAN_MESSAGE_DUAL_PX].periodMultiple = src->can_period_mult[19];
    view->Message[CBOR_CAN_MESSAGE_DUAL_PX].transmitAddress = src->can_transmit_address[19];
    view->Message[CBOR_CAN_MESSAGE_DUAL_QY].periodMultiple = src->can_period_mult[20];
    view->Message[CBOR_CAN_MESSAGE_DUAL_QY].transmitAddress = src->can_transmit_address[20];
    view->Message[CBOR_CAN_MESSAGE_DUAL_RZ].periodMultiple = src->can_period_mult[21];
    view->Message[CBOR_CAN_MESSAGE_DUAL_RZ].transmitAddress = src->can_transmit_address[21];
    view->Message[CBOR_CAN_MESSAGE_GNSS1_POS].periodMultiple = src->can_period_mult[22];
    view->Message[CBOR_CAN_MESSAGE_GNSS1_POS].transmitAddress = src->can_transmit_address[22];
    view->Message[CBOR_CAN_MESSAGE_GNSS2_POS].periodMultiple = src->can_period_mult[23];
    view->Message[CBOR_CAN_MESSAGE_GNSS2_POS].transmitAddress = src->can_transmit_address[23];
    view->Message[CBOR_CAN_MESSAGE_GNSS1_RTK_POS_REL].periodMultiple = src->can_period_mult[24];
    view->Message[CBOR_CAN_MESSAGE_GNSS1_RTK_POS_REL].transmitAddress = src->can_transmit_address[24];
    view->Message[CBOR_CAN_MESSAGE_GNSS2_RTK_CMP_REL].periodMultiple = src->can_period_mult[25];
    view->Message[CBOR_CAN_MESSAGE_GNSS2_RTK_CMP_REL].transmitAddress = src->can_transmit_address[25];
    view->Message[CBOR_CAN_MESSAGE_ROLL_ROLLRATE].periodMultiple = src->can_period_mult[26];
    view->Message[CBOR_CAN_MESSAGE_ROLL_ROLLRATE].transmitAddress = src->can_transmit_address[26];
}

/**
 * Checks that every CONFIG_CAN value held in can_config_t fits its home (a packed field or a bool).
 *
 * A domain whose keys live in several structs is written all or nothing by checking each of their mappings
 * before writing any of them.
 *
 * @param view  the view
 * @return true if cbor_can_config_from_config_can() would write the view
 */
bool cbor_can_config_fits_config_can(const cbor_config_can_t *view)
{
    if (((uint64_t)view->bitRateKbps * CBOR_BIND_LSB(((uint32_t)(CAN_BAUDRATE_KBPS_MASK)))) & ~(uint64_t)((uint32_t)(CAN_BAUDRATE_KBPS_MASK))) return false;
    if (view->fdEnabled > 1u) return false;
    return true;
}

/**
 * Writes the CONFIG_CAN keys held in can_config_t from the view, all or nothing.
 *
 * Bits and members of can_config_t that no key covers are kept. Nothing is written unless every value fits its
 * home (a packed field or a bool).
 *
 * @param dst   the can_config_t the keys are written to
 * @param view  the view
 * @return false, writing nothing, if a value does not fit its home
 */
bool cbor_can_config_from_config_can(can_config_t *dst, const cbor_config_can_t *view)
{
    if (((uint64_t)view->bitRateKbps * CBOR_BIND_LSB(((uint32_t)(CAN_BAUDRATE_KBPS_MASK)))) & ~(uint64_t)((uint32_t)(CAN_BAUDRATE_KBPS_MASK))) return false;
    if (view->fdEnabled > 1u) return false;
    dst->can_setting = (dst->can_setting & ~((uint32_t)(CAN_BAUDRATE_KBPS_MASK))) | (((uint32_t)view->bitRateKbps * CBOR_BIND_LSB(((uint32_t)(CAN_BAUDRATE_KBPS_MASK)))) & ((uint32_t)(CAN_BAUDRATE_KBPS_MASK)));
    if (view->fdEnabled) dst->can_setting |= ((uint32_t)(CAN_BAUDRATE_KBPS_FD_ENABLE)); else dst->can_setting &= ~((uint32_t)(CAN_BAUDRATE_KBPS_FD_ENABLE));
    dst->can_receive_address = view->receiveAddress;
    dst->can_period_mult[0] = view->Message[CBOR_CAN_MESSAGE_INS_TIME].periodMultiple;
    dst->can_transmit_address[0] = view->Message[CBOR_CAN_MESSAGE_INS_TIME].transmitAddress;
    dst->can_period_mult[1] = view->Message[CBOR_CAN_MESSAGE_INS_STATUS].periodMultiple;
    dst->can_transmit_address[1] = view->Message[CBOR_CAN_MESSAGE_INS_STATUS].transmitAddress;
    dst->can_period_mult[2] = view->Message[CBOR_CAN_MESSAGE_INS_EULER].periodMultiple;
    dst->can_transmit_address[2] = view->Message[CBOR_CAN_MESSAGE_INS_EULER].transmitAddress;
    dst->can_period_mult[3] = view->Message[CBOR_CAN_MESSAGE_INS_QUATN2B].periodMultiple;
    dst->can_transmit_address[3] = view->Message[CBOR_CAN_MESSAGE_INS_QUATN2B].transmitAddress;
    dst->can_period_mult[4] = view->Message[CBOR_CAN_MESSAGE_INS_QUATE2B].periodMultiple;
    dst->can_transmit_address[4] = view->Message[CBOR_CAN_MESSAGE_INS_QUATE2B].transmitAddress;
    dst->can_period_mult[5] = view->Message[CBOR_CAN_MESSAGE_INS_UVW].periodMultiple;
    dst->can_transmit_address[5] = view->Message[CBOR_CAN_MESSAGE_INS_UVW].transmitAddress;
    dst->can_period_mult[6] = view->Message[CBOR_CAN_MESSAGE_INS_VE].periodMultiple;
    dst->can_transmit_address[6] = view->Message[CBOR_CAN_MESSAGE_INS_VE].transmitAddress;
    dst->can_period_mult[7] = view->Message[CBOR_CAN_MESSAGE_INS_LAT].periodMultiple;
    dst->can_transmit_address[7] = view->Message[CBOR_CAN_MESSAGE_INS_LAT].transmitAddress;
    dst->can_period_mult[8] = view->Message[CBOR_CAN_MESSAGE_INS_LON].periodMultiple;
    dst->can_transmit_address[8] = view->Message[CBOR_CAN_MESSAGE_INS_LON].transmitAddress;
    dst->can_period_mult[9] = view->Message[CBOR_CAN_MESSAGE_INS_ALT].periodMultiple;
    dst->can_transmit_address[9] = view->Message[CBOR_CAN_MESSAGE_INS_ALT].transmitAddress;
    dst->can_period_mult[10] = view->Message[CBOR_CAN_MESSAGE_INS_NORTH_EAST].periodMultiple;
    dst->can_transmit_address[10] = view->Message[CBOR_CAN_MESSAGE_INS_NORTH_EAST].transmitAddress;
    dst->can_period_mult[11] = view->Message[CBOR_CAN_MESSAGE_INS_DOWN].periodMultiple;
    dst->can_transmit_address[11] = view->Message[CBOR_CAN_MESSAGE_INS_DOWN].transmitAddress;
    dst->can_period_mult[12] = view->Message[CBOR_CAN_MESSAGE_INS_ECEF_X].periodMultiple;
    dst->can_transmit_address[12] = view->Message[CBOR_CAN_MESSAGE_INS_ECEF_X].transmitAddress;
    dst->can_period_mult[13] = view->Message[CBOR_CAN_MESSAGE_INS_ECEF_Y].periodMultiple;
    dst->can_transmit_address[13] = view->Message[CBOR_CAN_MESSAGE_INS_ECEF_Y].transmitAddress;
    dst->can_period_mult[14] = view->Message[CBOR_CAN_MESSAGE_INS_ECEF_Z].periodMultiple;
    dst->can_transmit_address[14] = view->Message[CBOR_CAN_MESSAGE_INS_ECEF_Z].transmitAddress;
    dst->can_period_mult[15] = view->Message[CBOR_CAN_MESSAGE_INS_MSL].periodMultiple;
    dst->can_transmit_address[15] = view->Message[CBOR_CAN_MESSAGE_INS_MSL].transmitAddress;
    dst->can_period_mult[16] = view->Message[CBOR_CAN_MESSAGE_PREINT_PX].periodMultiple;
    dst->can_transmit_address[16] = view->Message[CBOR_CAN_MESSAGE_PREINT_PX].transmitAddress;
    dst->can_period_mult[17] = view->Message[CBOR_CAN_MESSAGE_PREINT_QY].periodMultiple;
    dst->can_transmit_address[17] = view->Message[CBOR_CAN_MESSAGE_PREINT_QY].transmitAddress;
    dst->can_period_mult[18] = view->Message[CBOR_CAN_MESSAGE_PREINT_RZ].periodMultiple;
    dst->can_transmit_address[18] = view->Message[CBOR_CAN_MESSAGE_PREINT_RZ].transmitAddress;
    dst->can_period_mult[19] = view->Message[CBOR_CAN_MESSAGE_DUAL_PX].periodMultiple;
    dst->can_transmit_address[19] = view->Message[CBOR_CAN_MESSAGE_DUAL_PX].transmitAddress;
    dst->can_period_mult[20] = view->Message[CBOR_CAN_MESSAGE_DUAL_QY].periodMultiple;
    dst->can_transmit_address[20] = view->Message[CBOR_CAN_MESSAGE_DUAL_QY].transmitAddress;
    dst->can_period_mult[21] = view->Message[CBOR_CAN_MESSAGE_DUAL_RZ].periodMultiple;
    dst->can_transmit_address[21] = view->Message[CBOR_CAN_MESSAGE_DUAL_RZ].transmitAddress;
    dst->can_period_mult[22] = view->Message[CBOR_CAN_MESSAGE_GNSS1_POS].periodMultiple;
    dst->can_transmit_address[22] = view->Message[CBOR_CAN_MESSAGE_GNSS1_POS].transmitAddress;
    dst->can_period_mult[23] = view->Message[CBOR_CAN_MESSAGE_GNSS2_POS].periodMultiple;
    dst->can_transmit_address[23] = view->Message[CBOR_CAN_MESSAGE_GNSS2_POS].transmitAddress;
    dst->can_period_mult[24] = view->Message[CBOR_CAN_MESSAGE_GNSS1_RTK_POS_REL].periodMultiple;
    dst->can_transmit_address[24] = view->Message[CBOR_CAN_MESSAGE_GNSS1_RTK_POS_REL].transmitAddress;
    dst->can_period_mult[25] = view->Message[CBOR_CAN_MESSAGE_GNSS2_RTK_CMP_REL].periodMultiple;
    dst->can_transmit_address[25] = view->Message[CBOR_CAN_MESSAGE_GNSS2_RTK_CMP_REL].transmitAddress;
    dst->can_period_mult[26] = view->Message[CBOR_CAN_MESSAGE_ROLL_ROLLRATE].periodMultiple;
    dst->can_transmit_address[26] = view->Message[CBOR_CAN_MESSAGE_ROLL_ROLLRATE].transmitAddress;
    return true;
}

/* Compile-time checks of every source the registry names */
_Static_assert(((uint32_t)(CAN_BAUDRATE_KBPS_MASK)) != 0u, "CONFIG_CAN.bitRateKbps: mask is zero");
_Static_assert(sizeof(((can_config_t *)0)->can_setting) <= 4, "CONFIG_CAN.bitRateKbps: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(CAN_BAUDRATE_KBPS_MASK)) >> 1 >> (8 * sizeof(((can_config_t *)0)->can_setting) - 1)) == 0u, "CONFIG_CAN.bitRateKbps: mask is wider than its member");
_Static_assert(((uint32_t)(CAN_BAUDRATE_KBPS_MASK)) / CBOR_BIND_LSB(((uint32_t)(CAN_BAUDRATE_KBPS_MASK))) <= UINT16_MAX, "CONFIG_CAN.bitRateKbps: mask holds more than the key's type");
_Static_assert(((uint32_t)(CAN_BAUDRATE_KBPS_FD_ENABLE)) != 0u, "CONFIG_CAN.fdEnabled: mask is zero");
_Static_assert(sizeof(((can_config_t *)0)->can_setting) <= 4, "CONFIG_CAN.fdEnabled: a mask needs an integer member of at most 32 bits");
_Static_assert((((uint32_t)(CAN_BAUDRATE_KBPS_FD_ENABLE)) >> 1 >> (8 * sizeof(((can_config_t *)0)->can_setting) - 1)) == 0u, "CONFIG_CAN.fdEnabled: mask is wider than its member");
_Static_assert((((uint32_t)(CAN_BAUDRATE_KBPS_FD_ENABLE)) & (((uint32_t)(CAN_BAUDRATE_KBPS_FD_ENABLE)) - 1u)) == 0u, "CONFIG_CAN.fdEnabled: a bool needs a single-bit mask");
_Static_assert(_Generic(((can_config_t *)0)->can_receive_address, uint32_t: 1, default: 0), "CONFIG_CAN.receiveAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[0], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_TIME.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[0], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_TIME.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[1], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_STATUS.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[1], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_STATUS.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[2], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_EULER.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[2], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_EULER.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[3], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_QUATN2B.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[3], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_QUATN2B.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[4], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_QUATE2B.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[4], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_QUATE2B.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[5], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_UVW.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[5], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_UVW.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[6], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_VE.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[6], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_VE.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[7], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_LAT.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[7], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_LAT.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[8], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_LON.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[8], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_LON.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[9], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_ALT.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[9], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_ALT.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[10], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_NORTH_EAST.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[10], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_NORTH_EAST.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[11], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_DOWN.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[11], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_DOWN.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[12], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_ECEF_X.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[12], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_ECEF_X.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[13], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_ECEF_Y.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[13], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_ECEF_Y.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[14], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_ECEF_Z.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[14], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_ECEF_Z.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[15], uint16_t: 1, default: 0), "CONFIG_CAN.Message.INS_MSL.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[15], uint32_t: 1, default: 0), "CONFIG_CAN.Message.INS_MSL.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[16], uint16_t: 1, default: 0), "CONFIG_CAN.Message.PREINT_PX.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[16], uint32_t: 1, default: 0), "CONFIG_CAN.Message.PREINT_PX.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[17], uint16_t: 1, default: 0), "CONFIG_CAN.Message.PREINT_QY.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[17], uint32_t: 1, default: 0), "CONFIG_CAN.Message.PREINT_QY.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[18], uint16_t: 1, default: 0), "CONFIG_CAN.Message.PREINT_RZ.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[18], uint32_t: 1, default: 0), "CONFIG_CAN.Message.PREINT_RZ.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[19], uint16_t: 1, default: 0), "CONFIG_CAN.Message.DUAL_PX.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[19], uint32_t: 1, default: 0), "CONFIG_CAN.Message.DUAL_PX.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[20], uint16_t: 1, default: 0), "CONFIG_CAN.Message.DUAL_QY.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[20], uint32_t: 1, default: 0), "CONFIG_CAN.Message.DUAL_QY.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[21], uint16_t: 1, default: 0), "CONFIG_CAN.Message.DUAL_RZ.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[21], uint32_t: 1, default: 0), "CONFIG_CAN.Message.DUAL_RZ.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[22], uint16_t: 1, default: 0), "CONFIG_CAN.Message.GNSS1_POS.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[22], uint32_t: 1, default: 0), "CONFIG_CAN.Message.GNSS1_POS.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[23], uint16_t: 1, default: 0), "CONFIG_CAN.Message.GNSS2_POS.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[23], uint32_t: 1, default: 0), "CONFIG_CAN.Message.GNSS2_POS.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[24], uint16_t: 1, default: 0), "CONFIG_CAN.Message.GNSS1_RTK_POS_REL.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[24], uint32_t: 1, default: 0), "CONFIG_CAN.Message.GNSS1_RTK_POS_REL.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[25], uint16_t: 1, default: 0), "CONFIG_CAN.Message.GNSS2_RTK_CMP_REL.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[25], uint32_t: 1, default: 0), "CONFIG_CAN.Message.GNSS2_RTK_CMP_REL.transmitAddress: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_period_mult[26], uint16_t: 1, default: 0), "CONFIG_CAN.Message.ROLL_ROLLRATE.periodMultiple: type differs from its member");
_Static_assert(_Generic(((can_config_t *)0)->can_transmit_address[26], uint32_t: 1, default: 0), "CONFIG_CAN.Message.ROLL_ROLLRATE.transmitAddress: type differs from its member");
_Static_assert(((uint64_t)((uint32_t)(CAN_BAUDRATE_KBPS_MASK)) + (uint64_t)((uint32_t)(CAN_BAUDRATE_KBPS_FD_ENABLE))) == (uint64_t)(((uint32_t)(CAN_BAUDRATE_KBPS_MASK)) | ((uint32_t)(CAN_BAUDRATE_KBPS_FD_ENABLE))), "imx_can.can_setting: two of CONFIG_CAN.bitRateKbps, CONFIG_CAN.fdEnabled overlap");
_Static_assert((((uint32_t)(CAN_BAUDRATE_KBPS_MASK)) | ((uint32_t)(CAN_BAUDRATE_KBPS_FD_ENABLE))) == (0xFFFFFFFFu >> (32 - 8 * sizeof(((can_config_t *)0)->can_setting))), "imx_can.can_setting: a bit has no key and is not listed as not_stored or pending");
