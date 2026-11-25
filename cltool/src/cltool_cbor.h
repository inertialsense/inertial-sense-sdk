/**
 * @file cltool_cbor.h
 * @brief cltool over CBOR messaging (DID_CBOR): -cbor-get / -cbor-set (the -get / -set YAML form), -cbor-out, and
 *        -flashCfg / -gpxFlashCfg through CBOR on devices that serve those DIDs.
 *
 * @author Kyle Mallory on 2026-10-23
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

#endif // CLTOOL_CBOR_H
