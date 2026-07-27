/**
 * @file DataKML.h
 * @brief Extracts KML-relevant samples (position/attitude) from recognized navigation/GNSS data
 *        sets, bucketed into logical channels (`cDataKML::MyEnum`) for DeviceLogKML.
 *
 * Unlike DataCSV/DataJSON (which serialize an arbitrary DID generically via `cISDataMappings`),
 * this is a narrow, hand-coded allow-list: only the specific DIDs listed in DID_TO_KID() are
 * recognized, and WriteDataToFile() only ever extracts time/position/attitude (plus a
 * dead-reckoning flag) — never the full struct. Actual KML/XML rendering happens in
 * DeviceLogKML::CloseWriteFile(), not here; this class only buffers `sKmlLogData` samples.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2026 Inertial Sense, Inc. All rights reserved.
 */

#ifndef DATA_KML_H
#define DATA_KML_H

// #include <stdio.h>
#include <string>
#include <vector>

#include "tinyxml.h"
#include "com_manager.h"

#ifdef USE_IS_INTERNAL
#    include "../../cpp/libs/families/imx/IS_internal.h"
#endif

/** @brief One extracted position/attitude sample, buffered per-channel until DeviceLogKML::CloseWriteFile() renders it to XML. */
struct sKmlLogData
{
    double                  time;               //!< sample time in seconds (GPS time-of-week, or derived from a millisecond timestamp)
    double                  lla[3];              //!< latitude (deg), longitude (deg), altitude (m)
    float                   theta[3];            //!< Euler attitude (roll, pitch, yaw), radians; zero for position-only (GNSS) samples
    bool                    deadReckoning;       //!< true if this sample was not aided by GNSS position (see `INS_STATUS_GNSS_AIDING_POS`); unused/false for GNSS-only samples

    /** @brief Construct a zero-initialized (all-default) sample. */
    sKmlLogData() {}

    /**
     * @brief Construct a full INS-style sample (position + attitude).
     * @param _time sample time in seconds.
     * @param _lla latitude (deg), longitude (deg), altitude (m).
     * @param _theta Euler attitude (roll, pitch, yaw), radians.
     * @param _deadReckoning true if this sample was not aided by GNSS position.
     */
    sKmlLogData(double _time, double _lla[3], float _theta[3], bool _deadReckoning=false)
    {
        time = _time;
        lla[0] = _lla[0];
        lla[1] = _lla[1];
        lla[2] = _lla[2];
        theta[0] = _theta[0];
        theta[1] = _theta[1];
        theta[2] = _theta[2];
        deadReckoning = _deadReckoning;
    }

    /**
     * @brief Construct a GNSS-style, position-only sample (no attitude, never dead-reckoned).
     * @param _timeMs sample time in milliseconds; converted to seconds for @ref time.
     * @param _lla latitude (deg), longitude (deg), altitude (m).
     */
    sKmlLogData(unsigned int _timeMs, double _lla[3])
    {
        time = _timeMs*0.001;
        lla[0] = _lla[0];
        lla[1] = _lla[1];
        lla[2] = _lla[2];
        theta[0] = theta[1] = theta[2] = 0;
        deadReckoning = false;
    }
};


/** @brief Extracts KML-relevant samples from a narrow allow-list of navigation/GNSS DIDs; see the file-level documentation above. */
class cDataKML
{
public:
    /** @brief Logical KML output channels; each maps to its own file in DeviceLogKML. */
    enum MyEnum
    {
        KID_INS = 0,    //!< primary INS solution (position + attitude)
        KID_REF,        //!< reference/truth INS solution, when a reference device (serial 99999 or 10101) is present
        KID_GNSS,       //!< primary GNSS position (`DID_GNSS1_POS`)
        KID_GNSS1,      //!< receiver-reported GNSS1 position (`DID_GNSS1_RCVR_POS`)
        KID_GNSS2,      //!< GNSS2 position (`DID_GNSS2_POS`)
        KID_RTK,        //!< RTK-corrected GNSS position (`DID_GNSS1_RTK_POS`)
        MAX_NUM_KID,    //!< number of channels; not a valid channel itself
    };

    /**
     * @brief Map a recognized DID to its output channel.
     * @param did data ID to map.
     * @return the matching `MyEnum` channel, or -1 if @p did is not one of the specific DIDs this class recognizes (`DID_INS_1`, `DID_INS_2`, `DID_GNSS1_POS`, `DID_GNSS1_RCVR_POS`, `DID_GNSS2_POS`, `DID_GNSS1_RTK_POS`) — this is a narrow allow-list, not a general DID→channel mapping.
     */
    static inline int DID_TO_KID(int did)
    {
        switch (did)
        {
            default:                return -1; // Unused
            case DID_INS_1:         // FALL THROUGH
            case DID_INS_2:         return KID_INS;
            case DID_GNSS1_POS:      return KID_GNSS;
            case DID_GNSS1_RCVR_POS: return KID_GNSS1;
            case DID_GNSS2_POS:      return KID_GNSS2;
            case DID_GNSS1_RTK_POS:  return KID_RTK;
        }
    }

    /**
     * @brief Estimated bytes per buffered sample for a channel, used only to size-limit the in-memory buffer before a flush (see DeviceLogKML::WriteDateToFile()) — not a measured or exact size.
     * @param kid channel to estimate; one of `MyEnum`'s `KID_*` values.
     * @return 130 for `KID_INS`/`KID_REF` (position + attitude), 65 for the GNSS-family channels (position only), or -1 for any other value.
     */
    static inline int BYTES_PER_KID(int kid)
    {
        switch (kid)
        {
            default:
                return -1; // Unused
            case KID_INS:
            case KID_REF:
                return 130;
            case KID_GNSS:
            case KID_GNSS1:
            case KID_GNSS2:
            case KID_RTK:
                return 65;
        }
    }

    cDataKML();

    /**
     * @brief Get the short name used to build a channel's KML filename (e.g. "ins", "gnss1").
     * @param kid channel to name; one of `MyEnum`'s `KID_*` values.
     * @return the channel's short name, or an empty string for any other value.
     */
    std::string GetDatasetName(int kid);

    /**
     * @brief Extract a KML sample from one recognized data set and append it to @p list.
     *
     * Recognizes `DID_INS_1`/`DID_INS_2`/`DID_INS_3` (full position + attitude, with the
     * dead-reckoning flag derived from `INS_STATUS_GNSS_AIDING_POS`) and `DID_GNSS1_POS`/
     * `DID_GNSS1_RCVR_POS`/`DID_GNSS2_POS`/`DID_GNSS1_RTK_POS` (position only). Any other data
     * set is ignored.
     *
     * @param list the calling channel's sample buffer (typically `sKmlLog::data`); a recognized
     *        sample is appended to it, otherwise it is left unchanged.
     * @param dataHdr header identifying the data set's DID; determines whether/how it's extracted.
     * @param dataBuf the data payload described by @p dataHdr, reinterpreted per-DID.
     * @return always 0. @note the return value does not indicate whether a sample was actually
     *         appended — check @p list's size before/after if that matters to the caller.
     */
    int WriteDataToFile(std::vector<sKmlLogData>& list, const p_data_hdr_t* dataHdr, const uint8_t* dataBuf);
};

#endif // DATA_KML_H
