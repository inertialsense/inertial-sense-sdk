/**
 * @file ISCbor.h
 * @brief Host-side CBOR messaging (DID_CBOR): building requests, reading replies, device-filter notation and the
 *        time carried on messages. ISDevice::cborRequest() sends a request and collects its replies.
 *
 * Design: is-common/docs_internal/cbor/CBOR_PROTOCOL_DESIGN.md
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_CBOR_H
#define IS_CBOR_H

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "cbor/cbor_envelope.h"
#include "cbor/cbor_registry.h"

/**
 * A CBOR request: envelope options and one entry per DID. Entries are encoded in the order added, after the envelope.
 */
class CborRequest
{
public:
    /**
     * @param type  MSG_TYPE operation (eCborMsgType), or 0 to leave it out (values mean DATA, keys only mean GET)
     */
    explicit CborRequest(uint8_t type = 0) : type(type) {}

    /** @brief Addresses the request (MSG_DST_DEV_FILTER); without it only the device at the other end of the link acts. */
    CborRequest& to(uint64_t dstFilter) { dst = dstFilter; return *this; }
    /** @brief Sets MSG_ACK_TYPE (eCborAckType). */
    CborRequest& ack(uint8_t ackType) { this->ackType = ackType; return *this; }
    /** @brief Asks for no time on the reply (MSG_OPTION_NO_TIME). */
    CborRequest& noTime() { omitTime = true; return *this; }

    /**
     * @brief Adds a GET entry: did: [field keys or key paths] (none = every field).
     * @param did     root DID key
     * @param fields  key paths to read
     * @return this request
     */
    CborRequest& get(uint16_t did, const std::vector<cbor_path_t>& fields = {});

    /**
     * @brief Adds a GET entry for top-level field keys.
     * @param did   root DID key
     * @param keys  field keys to read
     * @return this request
     */
    CborRequest& getKeys(uint16_t did, const std::vector<uint16_t>& keys);

    /**
     * @brief Adds a DATA entry: did: {key: value, ...} for the listed fields of a struct described by schema.
     * @param did     root DID key
     * @param schema  schema of the struct (e.g. g_cborSchema_FLASH_CFG)
     * @param values  the struct holding the values to send
     * @param keys    field keys to send (empty = every field)
     * @return this request
     */
    CborRequest& set(uint16_t did, const cbor_schema_t& schema, const void* values, const std::vector<uint16_t>& keys);

    /**
     * @brief Adds an entry whose value is already CBOR-encoded.
     * @param did   root DID key
     * @param cbor  the encoded value (e.g. a map of field values, or an array of keys)
     * @return this request
     */
    CborRequest& value(uint16_t did, std::vector<uint8_t> cbor);

    /**
     * @brief Encodes the request with the given MSG_REQ_ID.
     * @param reqId  request ID echoed by the replies
     * @return the CBOR message, empty if it could not be encoded
     */
    std::vector<uint8_t> encode(uint16_t reqId) const;

    uint8_t                                                 type = 0;       //!< MSG_TYPE operation, or 0 (inferred)
    std::optional<uint64_t>                                 dst;            //!< MSG_DST_DEV_FILTER, if set
    uint8_t                                                 ackType = CBOR_ACK_TYPE_ACK_MINIMAL;    //!< MSG_ACK_TYPE
    bool                                                    omitTime = false;   //!< MSG_OPTION_NO_TIME
    std::vector<std::pair<uint16_t, std::vector<uint8_t>>>  entries;        //!< DID key and its encoded value
};

/** A reply received for a request */
struct CborReply
{
    cbor_envelope_t         env = {};   //!< Parsed envelope (MSG_SRC_DEV_ID says which device replied)
    std::vector<uint8_t>    msg;        //!< The whole message

    /** @return true if the reply carries an entry for did */
    bool has(uint16_t did) const;

    /**
     * @return the MSG_STATUS inside did's entry (eCborMsgStatus): CBOR_MSG_STATUS_OK when the entry carries values or
     *         is absent from an acknowledgement, else the reported status
     */
    uint8_t status(uint16_t did) const;

    /** @return the key paths in did's MSG_ERR_KEYS (empty if none) */
    std::vector<cbor_path_t> errKeys(uint16_t did) const;

    /**
     * @brief Decodes did's fields onto a struct, field by field. Fields marked RESP_TOO_LARGE are skipped and listed in
     *        tooLarge; fields the schema does not know are ignored; a field whose value is invalid is skipped.
     * @param did       root DID key
     * @param schema    schema of the struct
     * @param data      the struct to write into
     * @param tooLarge  [out, optional] keys of the fields marked RESP_TOO_LARGE
     * @return number of fields written, or -1 if the reply has no values map for did
     */
    int decode(uint16_t did, const cbor_schema_t& schema, void* data, std::vector<uint16_t>* tooLarge = nullptr) const;

    /**
     * @brief Finds did's entry.
     * @param did  root DID key
     * @param it   [out] positioned at the entry's value
     * @return true if found
     */
    bool find(uint16_t did, nanocbor_value_t& it) const;
};

/**
 * @brief Formats a device filter in the SDK's device notation, with -1 for a wildcard sub-field:
 *        "IMX-5.0::62797", "GPX-1.0::-1", "GPX-1.-1::-1", "-1::-1" (every device), "host" (0).
 * @param filter  the MSG_DST_DEV_FILTER value
 * @return the notation
 */
std::string cborDevFilterToString(uint64_t filter);

/**
 * @brief Parses the notation of cborDevFilterToString() (also "*" for a wildcard, a missing version or serial as a
 *        wildcard, "all", "host" and a 0x-prefixed hex value).
 * @param str     the notation
 * @param filter  [out] the MSG_DST_DEV_FILTER value
 * @return false if str is not recognised
 */
bool cborDevFilterFromString(const std::string& str, uint64_t& filter);

/**
 * @brief Converts MSG_TIME_GPS_NS (ns since the GPS epoch) to GPS week and time of week.
 * @param gpsNs  ns since 1980-01-06
 * @param week   [out] GPS week
 * @param tow    [out] seconds into the week
 */
void cborGpsNsToWeekTow(uint64_t gpsNs, uint32_t& week, double& tow);

/**
 * @brief Converts MSG_TIME_GPS_NS to UTC seconds since the Unix epoch.
 * @param gpsNs        ns since the GPS epoch
 * @param leapSeconds  GPS - UTC offset in seconds (C_GPS_LEAP_SECONDS unless known otherwise)
 * @return UTC Unix time in seconds
 */
double cborGpsNsToUnixUtc(uint64_t gpsNs, int leapSeconds);

#endif // IS_CBOR_H
