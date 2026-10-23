/**
 * @file cltool_cbor.h
 * @brief cltool over CBOR messaging (DID_CBOR): -cbor-get / -cbor-set (the -get / -set YAML form), -cbor-out, and
 *        -flashCfg / -gpxFlashCfg through CBOR on devices that serve those DIDs.
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef CLTOOL_CBOR_H
#define CLTOOL_CBOR_H

#include <string>

#include "InertialSense.h"

/**
 * @brief Runs -cbor-get or -cbor-set on the first device and prints each reply as YAML, in the shape -get prints.
 * @param inertialSenseInterface  the host
 * @return false if the request could not be built or sent, no reply came, or a DID in a reply carries an error status
 */
bool cltool_cborGetSet(InertialSense& inertialSenseInterface);

/**
 * @brief Prints one DID_CBOR message for -cbor-out, in diagnostic notation.
 * @param direction  "rx" or "tx"
 * @param msg        the message
 * @param len        its length
 */
void cltool_cborOut(const char* direction, const uint8_t* msg, size_t len);

/**
 * @brief Has device print the CBOR messages it sends when -cbor-out is on (received ones are printed as they arrive).
 * @param device  the device
 */
void cltool_cborWatch(ISDevice& device);

/**
 * @brief -flashCfg over CBOR: reads or writes FLASH_CFG when the device serves it, with the output of the legacy path.
 * @param device  the IMX
 * @param str     the -flashCfg argument (key list, or key=value pairs separated by ',')
 * @param handled [out] false if the device does not serve FLASH_CFG over CBOR, so the caller uses the legacy DID
 * @return false on failure (a message has been printed)
 */
bool cltool_cborImxFlashCfg(ISDevice& device, const std::string& str, bool& handled);

/**
 * @brief -gpxFlashCfg over CBOR: reads GPX_FLASH_CFG from the GPX (directly, or behind an IMX) and writes it to a GPX
 *        connected directly. A GPX behind an IMX is written through the legacy DID, since the IMX owns its GPX's
 *        configuration.
 * @param device  the IMX or GPX the host is connected to
 * @param str     the -gpxFlashCfg argument
 * @param handled [out] false if the caller should use the legacy DID
 * @return false on failure (a message has been printed)
 */
bool cltool_cborGpxFlashCfg(ISDevice& device, const std::string& str, bool& handled);

#endif // CLTOOL_CBOR_H
