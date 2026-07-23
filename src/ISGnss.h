/**
 * @file ISGnss.h
 * @brief GNSS constellation and signal ID -> human-readable name/prefix helpers.
 *
 * Converts the numeric gnssId/sigId values used in satellite-tracking structures (e.g. @ref
 * gnss_sat_t, @ref gnss_sig_t) into display strings, for UI and log-summary use.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2014-2026 Inertial Sense, Inc. - http://inertialsense.com
 */

#ifndef IS_GNSS_H_
#define IS_GNSS_H_

#include <string>

/**
 * @brief Convert a GNSS constellation ID to its human-readable name.
 * @param gnssId Constellation ID (see SAT_SV_GNSS_ID_* in data_sets.h).
 * @return Constellation name (e.g. "GPS", "GLONASS"), or the numeric ID as a string if unrecognized.
 */
std::string gnssIdToGnssName(int gnssId);

/**
 * @brief Convert a GNSS constellation ID to its single-character prefix, as used in satellite ID strings.
 * @param gnssId Constellation ID (see SAT_SV_GNSS_ID_* in data_sets.h).
 * @return Constellation prefix character (e.g. 'G' for GPS, 'R' for GLONASS), or ' ' if unrecognized.
 */
char gnssIdToGnssPrefix(int gnssId);

/**
 * @brief Convert a GNSS constellation ID + signal ID pair to its human-readable signal name.
 * @param gnssId Constellation ID (see SAT_SV_GNSS_ID_* in data_sets.h).
 * @param sigId  Signal ID within that constellation (see SAT_SV_SIG_ID_* in data_sets.h).
 * @return Signal name (e.g. "L1CA", "E5aI"), or an empty string if the gnssId/sigId pair is unrecognized.
 */
std::string gnssIdSigIdToSignalName(int gnssId, int sigId);


#endif //IS_GNSS_H_
