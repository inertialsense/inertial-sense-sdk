/**
 * @file ISCborFormat.h
 * @brief CBOR messages as text: requests built from the YAML form cltool's -get / -set take, replies converted to the
 *        YAML -get prints, and any message printed in CBOR diagnostic notation (RFC 8949 section 8).
 *
 * Keys are named from the CBOR key registry. In YAML a DID is its registry name (FLASH_CFG), its legacy DID name
 * (DID_FLASH_CONFIG) or its number; a field or option is its registry name or its number; message keys (MSG_*) are
 * accepted at the root. Numbers pass through unchanged, so YAML with numeric keys writes any message.
 *
 * Design: is-common/docs_internal/cbor/CBOR_PROTOCOL_DESIGN.md
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_CBOR_FORMAT_H
#define IS_CBOR_FORMAT_H

#include <cstdint>
#include <functional>
#include <string>

#include <yaml-cpp/yaml.h>

#include "ISCbor.h"

/**
 * Supplies the current values of a struct-backed DID, for a DATA entry that sets one array element (name[i]). Returns
 * the struct, or nullptr if the values are not available.
 */
typedef std::function<const void*(uint16_t did)> CborCurrentValues;

/**
 * @brief Resolves a DID written as its registry name, its legacy DID name or a number (decimal or 0x hex).
 * @param s  the text
 * @return the root DID key, or -1 if s names no DID
 */
int cborDidFromString(const std::string& s);

/**
 * @brief Adds the entries of a YAML request to req, in the form cltool's -get / -set take.
 *
 * GET: each root key is a DID, with no value (every field), a list or set of field names, or a map of nested key
 * paths, e.g. {FLASH_CFG: {ser0BaudRate, insRotation}} or {RMC_STORED: {0: {DID_INS_2}}}.
 * DATA: each root key is a DID, with a map of field values, e.g. {GPX_FLASH_CFG: {ser2BaudRate: 460800}}. A struct-backed
 * DID encodes each value as its field's type; name[i] sets one array element and needs current.
 * Message keys at the root set the envelope: MSG_TYPE, MSG_DST_DEV_FILTER (a device filter as in
 * cborDevFilterFromString()), MSG_ACK_TYPE (a name or number) and MSG_OPTION_NO_TIME set the request's options; any
 * other message key is sent as given, ahead of the DIDs. Enum values may be given by name.
 *
 * @param node     the YAML map
 * @param op       CBOR_MSG_TYPE_GET or CBOR_MSG_TYPE_DATA (a MSG_TYPE key in node overrides it)
 * @param req      [in, out] the request entries are added to
 * @param error    [out] what was wrong, when the result is false
 * @param current  current values, for name[i] in a DATA entry (optional)
 * @return false if node names an unknown DID, field or enum value, or a value does not fit its field
 */
bool cborRequestFromYaml(const YAML::Node& node, uint8_t op, CborRequest& req, std::string& error,
                         const CborCurrentValues& current = nullptr);

/**
 * @brief Converts a message's DID entries to YAML in the shape cltool's -get prints: {FLASH_CFG: {ser0BaudRate: 921600}}.
 *        Fields and DIDs are named from the registry (unknown keys stay numbers), enum values by name, and a part a
 *        device marked appears as {MSG_STATUS: RESP_TOO_LARGE}.
 * @param msg              the message
 * @param len              its length
 * @param includeEnvelope  also convert the envelope keys (MSG_*)
 * @return the YAML map, or a null node if msg is not a well-formed CBOR map
 */
YAML::Node cborToYaml(const uint8_t* msg, size_t len, bool includeEnvelope = false);

/**
 * @brief Prints CBOR in diagnostic notation. With names, each named key appears as NAME(key), enum values are named,
 *        device IDs appear in device notation and message times as GPS week / time of week or seconds, e.g.
 *        {MSG_TYPE(0xC000): REPLY|GET, MSG_SRC_DEV_ID(0xC001): IMX-5.0::62797, FLASH_CFG(12): {ser0BaudRate(2): 921600}}.
 *        Without names the output is plain diagnostic notation.
 * @param cbor   the CBOR data item (a message, or any value)
 * @param len    its length
 * @param names  annotate with registry names
 * @return the text; a part that is not well-formed CBOR ends the text with "<malformed>"
 */
std::string cborToDiag(const uint8_t* cbor, size_t len, bool names = true);

#endif // IS_CBOR_FORMAT_H
