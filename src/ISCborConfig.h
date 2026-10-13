/**
 * @file ISCborConfig.h
 * @brief Host-side helpers for DID_CBOR_CONFIG: request results, CBOR map handling, field names, and a host
 *        copy of a device's configuration bound to the CBOR field tables.
 *
 * Field keys are specific to the device type (see flash_cfg_cbor.h): an IMX serves its flash config, CAN config
 * and each port's startup RMC; a GPX serves its flash config. Field names match those used by the legacy
 * DID_FLASH_CONFIG / DID_GPX_FLASH_CFG field mappings (cltool -flashCfg / -gpxFlashCfg).
 *
 * @copyright Copyright (c) 2026 Inertial Sense, Inc.
 */

#ifndef IS_CBOR_CONFIG_H
#define IS_CBOR_CONFIG_H

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "data_sets.h"
#include "cbor/flash_cfg_cbor.h"

/** Host-side results reported in CborConfigResult::status, beside the device's eCborCfgStatus values */
enum : int
{
    CBOR_CFG_RESULT_TIMEOUT     = -1,   //!< The device did not reply in time, after retries
    CBOR_CFG_RESULT_SEND_FAILED = -2,   //!< The request could not be sent (not connected, or in bootloader)
    CBOR_CFG_RESULT_MALFORMED   = -3,   //!< A reply's CBOR could not be parsed
};

/** One configuration field: its key and the CBOR encoding of its value */
struct CborConfigField
{
    uint16_t key;                       //!< Field key (see flash_cfg_cbor.h)
    std::vector<uint8_t> value;         //!< One complete CBOR data item
};

typedef std::vector<CborConfigField> CborConfigFields;

/** Outcome of a DID_CBOR_CONFIG get or set */
struct CborConfigResult
{
    int status = CBOR_CFG_STATUS_OK;    //!< eCborCfgStatus from the device, or a CBOR_CFG_RESULT_* value
    uint16_t hdwId = 0;                 //!< Hardware ID of the device that answered
    CborConfigFields fields;            //!< GET: the fields returned. SET: the fields applied, with the values the device kept.

    bool ok() const { return status == CBOR_CFG_STATUS_OK; }
};

/** Called once when a request completes, fails or times out */
typedef std::function<void(const CborConfigResult&)> CborConfigCallback;

namespace cborcfg
{
    /**
     * @brief Appends the entries of a CBOR map. Entries with keys that are not unsigned integers below 65536 are skipped.
     * @return false if buf is not a well-formed CBOR map
     */
    bool parseMap(const uint8_t* buf, size_t size, CborConfigFields& out);

    /** @return a CBOR map holding fields, in order */
    std::vector<uint8_t> encodeMap(const CborConfigFields& fields);

    /** @return fields split, in order, into groups whose encoded maps are at most maxMapSize bytes (a field too large for one group gets a group of its own) */
    std::vector<CborConfigFields> splitFields(const CborConfigFields& fields, size_t maxMapSize);

    /**
     * @brief Name of a field, as used by the legacy flash-config mappings. CAN fields use the can_config_t member
     *        names; per-port RMC fields are "rmc[<port>].<member>".
     * @param hdwId  hardware ID of the device the key belongs to (only the type is used)
     * @return the name, or "key<N>" for a key this SDK does not know
     */
    std::string keyName(uint16_t hdwId, uint16_t key);

    /** @brief Finds a field by name (case-insensitive) or as "key<N>". @return true if found */
    bool keyFromName(uint16_t hdwId, const std::string& name, uint16_t& key);

    /** @return every key this SDK knows for the device type, in device order */
    std::vector<uint16_t> keys(uint16_t hdwId);

    /** @return a short name for a CborConfigResult::status value */
    const char* statusName(int status);
}

struct CborConfigImage;

namespace cborcfg
{
    /**
     * @brief Renders one field of image as "name = value" lines, one per array element ("name[i] = value"), formatted as
     *        the legacy flash-config listing does.
     * @return the lines, or an empty string if the image has no such key
     */
    std::string formatField(CborConfigImage& image, uint16_t key);

    /**
     * @brief Applies "name=value" or "name[i]=value" to image, parsing the value as the legacy flash-config options do
     *        (hex with or without 0x for hex-displayed fields).
     * @param key  [out] the field that was changed
     * @return false if the name is unknown, the index is out of range, or the value does not parse
     */
    bool parseAssignment(CborConfigImage& image, const std::string& assignment, uint16_t& key);

    /** @brief Finds the field addressed by "name" or "name[i]". @param index [out] the element index, or -1 for none */
    bool parseFieldRef(uint16_t hdwId, const std::string& ref, uint16_t& key, int& index);
}

/**
 * Host copy of a device's CBOR-addressable configuration. The members used depend on the device type: imx, can and
 * rmc for an IMX; gpx for a GPX.
 */
struct CborConfigImage
{
    uint16_t hdwId = 0;                 //!< Device the image describes (only the type is used)
    nvm_flash_cfg_t imx = {};           //!< IMX flash config
    can_config_t can = {};              //!< IMX CAN config
    rmci_t rmc[RMC_CFG_MAX_PORTS] = {}; //!< IMX startup RMC, per port
    gpx_flash_cfg_t gpx = {};           //!< GPX flash config

    explicit CborConfigImage(uint16_t hdwId_ = 0) : hdwId(hdwId_) {}

    /** @return the field groups for hdwId, bound to this image's members (empty for an unsupported type) */
    std::vector<flash_cfg_cbor_group_t> groups();

    /** @brief Decodes fields into the image; unknown keys are ignored. @return false if any value is invalid (image unchanged) */
    bool apply(const CborConfigFields& fields);

    /** @return the image's fields with the given keys (every field if keys is empty), in that order */
    CborConfigFields encode(const std::vector<uint16_t>& keys = {});
};

#endif // IS_CBOR_CONFIG_H
