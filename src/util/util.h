/**
 * @file util.h 
 * @brief General Utility functions. Most of these functions should be static, as we don't want/need a "Util" instance running around.
 *
 * @author Kyle Mallory on 3/14/24.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK__UTIL_H
#define IS_SDK__UTIL_H

#include <string>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <streambuf>
#include <istream>
#include <ostream>
#include <cstring>
#include <algorithm>
#include <chrono>

#include <sstream>
#include <functional>

#include <cstdarg>
#include <cstdio>

#include "ISComm.h"

namespace utils {
    /**
     * @brief Trims left-side/leading characters (whitespace by default) from the passed string; this modifies the string, in place.
     * Locates the first characters which are not within the set of ws, and removes all characters from the start upto that first
     * non-matching character.
     *
     * @param s the string to be trimmed
     * @param t a set of characters, which will be removed if they exists
     * @return a reference to the input string.
     */
    std::string& ltrim(std::string& s, const char* t = " \t\n\r\f\v");

    /**
     * @brief Trims right-side/trailing characters (whitespace by default) from the passed string; this modifies the string, in place.
     * Locates the last characters of the string which are not within the set of ws, and removes all characters from that position
     * until the end of the string non-matching character.
     *
     * @param s the string to be trimmed
     * @param t a set of characters, which will be removed if they exists
     * @return a reference to the input string.
     */
    std::string& rtrim(std::string& s, const char* t = " \t\n\r\f\v");

    /**
     * @brief Trims left-size/leading and right-side/trailing characters (whitespace by default) from the passed string; this modifies
     * the string, in place. Call both rtrim() and ltrim() in a single call.
     *
     * @param s the string to be trimmed
     * @param t a set of characters, which will be removed if they exists
     * @return a reference to the input string.
     */
    std::string& trim(std::string& s, const char* t = " \t\n\r\f\v");


    /**
     * ltrim() equivalent which makes a new copy, and does not modify the original
     * @param s the string to be trimmed
     * @param t a set of characters, which will be removed if they exists
     * @return the modified/trimmed copy of the original input string.
     */
    std::string ltrim_copy(std::string s, const char* t = " \t\n\r\f\v");

    /**
     * rtrim() equivalent which makes a new copy, and does not modify the original
     * @param s the string to be trimmed
     * @param t a set of characters, which will be removed if they exists
     * @return the modified/trimmed copy of the original input string.
     */
    std::string rtrim_copy(std::string s, const char* t = " \t\n\r\f\v");

    /**
     * trim() equivalent which makes a new copy, and does not modify the original
     * @param s the string to be trimmed
     * @param t a set of characters, which will be removed if they exists
     * @return the modified/trimmed copy of the original input string.
     */
    std::string trim_copy(std::string s, const char* t = " \t\n\r\f\v");

    /**
     * @brief Expands a shell-style GLOB (optionally a comma-separated list of them) into an equivalent
     *        regular expression.
     *
     * Exists because glob and regex are different pattern languages that are easy to conflate, and the
     * consequences are not cosmetic. Port discovery
     * (PortManager::discoverPorts()/PortFactory::locatePorts()) consumes a REGEX, while command-line port
     * specifiers are globs -- so passing one straight to the other threw std::regex_error out of a port
     * scan on the very common "*" ("*" is a quantifier with no preceding atom), aborting a 15-device
     * firmware update after every device had been reset into its bootloader.
     *
     * Translation, not escaping, so "/dev/ttyACM*" means what a user expects:
     *   *  ->  .*        any run of characters
     *   ?  ->  .         any single character
     *   everything else is regex-escaped and matched literally, so "/dev/ttyACM0" does not also match
     *   "/dev/ttyACMX" through an unescaped '.'.
     *
     * Glob character classes ("[a-z]") are deliberately NOT translated: the brackets are escaped and
     * matched literally, since silently reinterpreting them is worse than not supporting them.
     *
     * @param globs a glob, or a comma-separated list of globs
     * @return an equivalent regex; alternatives are joined with '|', and an empty input yields "(.+)"
     */
    std::string globToRegex(const std::string& globs);

    /**
     * Base case for the variadic string_format() below; returns format unchanged, since there are
     * no substitutions to perform when no additional arguments are supplied.
     * @param format the string to return as-is
     * @return a copy of format
     */
    inline std::string string_format(const std::string& format) {
        return format;
    }

    /**
     * Performs sprintf-type formatting, using a std::string for the format string, and outputting a std::string.
     * @param format a printf-style format string
     * @param args the values to substitute into format, per the usual printf conversion rules
     * @return the formatted string
     */
    template<typename ... Args>
    std::string string_format(const std::string& format, Args ... args) {
        int size_s = std::snprintf((char*)nullptr, 0, format.c_str(), args ...) + 1; // Extra space for '\0'
        if (size_s <= 0) {
            throw std::runtime_error("Error during formatting.");
        }
        auto size = static_cast<size_t>(size_s);
        std::unique_ptr<char[]> buf(new char[ size ]);
        memset(buf.get(), 0, size);
        std::snprintf(buf.get(), size, format.c_str(), args ...);
        return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
    }

    /**
     * Combine all elements of a container denoted by the start and ending iterators, to join into a
     *   single string, using the specified delimiter. The iterator type is deduced from the arguments.
     * @param begin a copy of the iterator denoting the first element to join
     * @param end reference to the iterator denoting the last element to join
     * @param delimiter a delimiter string to be placed between each element in the output string
     * @return a string of all joined elements
     */
    template <typename T>
    std::string join_to_string(T begin, const T& end, const std::string& delimiter) {
        std::ostringstream s;
        for (T cur = begin; cur != end; cur++) {
            if (cur != begin) s << delimiter.c_str();
            s << *cur;
        }
        return s.str();
    }

    /**
     * Combine all elements of a container into a single string, using the specified delimiter.
     * The container type is deduced from the argument.
     * @param v reference to the container of elements to join
     * @param delimiter a delimiter string to be placed between each element in the output string
     * @return a string of all joined elements
     */
    template <typename T>
    std::string join_to_string(const T& v, const std::string& delimiter) {
        std::ostringstream s;
        for (const auto& i : v) {
            if (&i != &*v.begin()) {
                s << delimiter;
            }
            s << i;
        }
        return s.str();
    }

    /**
     * Parses a string of delimited values (ie, x.x.x.x) and populate the values into a passed
     * std::array of type T and size N. If the string contains fewer than N numbers elements,
     * the remaining elements are not assigned (you should initialize vOut before calling this
     * function). At most N elements will be parsed. The lamba is used to convert the parsed
     * substring into the value of type T.
     *
     * This is primarily used to parse versions and ip address, and the template provides
     *  default values that support this usage (T = uint8_t, N = 4). If you wish to use it for
     *  other types, remember to set the template parameters.
     *
     * @param s the string to split
     * @param vOut a std::array<T,n> each element containing a parsed value.
     * @param d the delimiters to use when splitting (if multiple, will be separated on ANY)
     *  (defaults to '.')
     * @param lambda a lambda used to convert the parsed substring to a value of type T (defaults
     *  to stoi(), returning a decimal
     * @return returns the number of elements parsed.
     */
    template <typename T=uint8_t, int N=4>
    int split_from_string(const std::string& s, T vOut[N], const char* d = ".", std::function<T(const std::string&)> lambda = [](const std::string& ss) -> T { return static_cast<T>(std::stoul(ss)); } ) {
        size_t start = 0, n = 0, end = 0, len = s.length();
        while ( (start < len) && (n < len) && (end = s.find_first_of(d, start)) != std::string::npos) {
            vOut[n++] = lambda(s.substr(start, end - start));
            start = end + 1;
        }
        if (start < len)
            vOut[n++] = lambda(s.substr(start));
        return n;
    }


    /**
     * Splits the passed string into a vector of strings, delimited by delimiter.
     * @param str the string to be split
     * @param delimiter the substring to use as a delimiter
     * @return a vector of strings
     */
    std::vector<std::string> split_string(const std::string& str, const std::string& delimiter);


    /**
     * Returns the index of an entry (type E, default std::string) within a container (type T,
     * default std::vector<E>).
     * @param c a reference to the collection containing the element to index
     * @param e a reference to the element to locate in the collection
     * @return the index number of the element e within collection c, or -1 if not found.
     */
    template <typename E=std::string, typename T=std::vector<E>>
    int indexOf(const T& c, const E& e) {
        ptrdiff_t pos = find(c.begin(), c.end(), e) - c.begin();
        if(pos >= c.size()) return -1;
        return (int)pos;
    }

    /**
     * Formats the passed raw data as a "hexadecimal view". This can be used with any data.
     * @param raw_data a pointer to the raw byte stream
     * @param bytesLen the number of bytes following raw_data to output
     * @param bytesPerLine the number of hexadecimal bytes to print per line
     * @return a fully formatted string
     */
    std::string raw_hexdump(const char* raw_data, int bytesLen, int bytesPerLine);

    /**
     * Formats the specified DID's raw data as a "hexadecimal view". This can be used with any DID
     * that is not otherwise supported.
     * @param raw_data a pointer to the raw DID byte stream
     * @param hdr the DID header
     * @param bytesPerLine the number of hexadecimal bytes to print per line
     * @return a fully formatted string
     */
    std::string did_hexdump(const char *raw_data, const p_data_hdr_t& hdr, int bytesPerLine);

    /** Bitmask selecting which dev_info_t fields devInfoToString()/getBuildAsString() render. */
    enum dev_info_fmt_e : uint16_t {
        DV_BIT_SERIALNO         = 0x0001,        //!< serial number
        DV_BIT_FIRMWARE_VER     = 0x0002,        //!< firmware version w/ optional release type
        DV_BIT_HARDWARE_INFO    = 0x0004,        //!< hdw type & version
        DV_BIT_BUILD_KEY        = 0x0008,        //!< build key and build number
        DV_BIT_BUILD_DATE       = 0x0010,        //!< build date
        DV_BIT_BUILD_TIME       = 0x0020,        //!< build time
        DV_BIT_BUILD_COMMIT     = 0x0040,        //!< repo hash & build status (dirty)
        DV_BIT_ADDITIONAL_INFO  = 0x0100,        //!< additional info
        DV_BIT_PROTOCOL_VER     = 0x0200,        //!< protocol version
        DV_BIT_COMPACT_DATE     = 0x1000,        //!< compact date formatting
        DV_BIT_COMPACT_TIME     = 0x2000,        //!< compact time formatting
        DV_BIT_EXACT_MATCH      = 0x4000,        //!< when matching/comparing, match exactly (version & time)
    };

    /**
     * Renders the type-major-minor portion of a packed hardware id as
     * a printable string, e.g. `"IMX-5.0"`, `"GPX-1.0"`, `"UBX-5.9"`.
     *
     * Decodes via `DECODE_HDW_TYPE` / `DECODE_HDW_MAJOR` /
     * `DECODE_HDW_MINOR` and maps the type field through the same
     * type-name table `getHardwareAsString` uses. Unknown types
     * render as `"???-<major>.<minor>"`.
     *
     * The packed `uint16_t` form does not carry the sub-rev bytes
     * (`hardwareVer[2..3]`) — those come from a `dev_info_t`. Use
     * `getHardwareAsString(devInfo, true)` when you need the sub-rev.
     *
     * @param hdwId  Packed hardware id (`is_hardware_t`), typically
     *               obtained via `ENCODE_HDW_ID` or
     *               `DECODE_UNIQUE_ID_TO_HDW_ID`.
     * @return       String of the form `"<TYPE>-<major>.<minor>"`.
     */
    std::string hdwIdToString(uint16_t hdwId);

    /**
     * Composes the canonical "device-id" display string from a packed
     * hardware id and a serial number, of the form
     * `"<hdwIdString>::SN<serial>"` (e.g. `"IMX-5.0::SN519465"`).
     *
     * This is the canonical builder used by Logalyzer's `SeriesId` and
     * any other call site that wants a stable, parseable device label.
     * Inlining the format string at call sites is discouraged — call
     * this so the format stays consistent.
     *
     * @param hdwId   Packed hardware id (`is_hardware_t`).
     * @param serial  Serial number; rendered as decimal.
     * @return        `"<hdwIdString>::SN<serial>"`.
     */
    std::string deviceIdString(uint16_t hdwId, uint64_t serial);

    /**
     * Renders the hardware type-major-minor portion of devInfo, e.g. "IMX-5.0", optionally
     * appended with the sub-rev components (hardwareVer[2..3]) when showRev is true and they are
     * non-zero.
     * @param devInfo the dev_info_t supplying the hardware type and version bytes
     * @param showRev if true, append the sub-rev components when present
     * @return the rendered hardware-id string
     */
    std::string getHardwareAsString(const dev_info_t& devInfo, bool showRev = true);

    /**
     * Renders the type-major-minor portion of a packed hardware id, e.g. "IMX-5.0". This is a
     * legacy-named alias of hdwIdToString() for callers that prefer this historical name; the
     * packed form cannot carry the sub-rev bytes.
     * @param hdwId packed hardware id
     * @return the rendered hardware-id string
     */
    std::string getHardwareAsString(is_hardware_t hdwId);

    /**
     * Renders the firmware version portion of devInfo, e.g. "fw2.1.7" or "fw2.1.7-rc.5", including
     * the build-type suffix (alpha/beta/rc/devel/snap) when set.
     * @param devInfo the dev_info_t supplying the firmware version and build type
     * @param prefix  a prefix to prepend to the rendered version (defaults to "fw")
     * @return the rendered firmware-version string
     */
    std::string getFirmwareAsString(const dev_info_t& devInfo, const std::string& prefix = "fw");

    /**
     * Bits controlling getFirmwareInfoAsString(). Canonical here; ISDevice::DevInfoFormatFlags
     * aliases these so the two cannot drift, and the values are fixed because callers pass them.
     */
    enum devFirmwareInfoFlags_e : uint16_t {
        FWI_COMPACT_BUILD_TYPE = 0x0010,  //!< render the build type as one character instead of a word
        FWI_OMIT_COMMIT_HASH   = 0x0100,  //!< suppress the commit hash and dirty marker
        FWI_OMIT_BUILD_KEY     = 0x0200,  //!< suppress the build host key and number
        FWI_OMIT_BUILD_DATE    = 0x0400,  //!< suppress the build date
        FWI_OMIT_BUILD_TIME    = 0x0800,  //!< suppress the build time
        FWI_OMIT_BUILD_MILLIS  = 0x1000,  //!< suppress build milliseconds when non-zero
    };

    /**
     * Renders firmware version and build provenance: the long form behind ISDevice::getFirmwareInfo().
     *
     * The version portion is getFirmwareAsString(), so the short and long forms cannot disagree about
     * how a version is spelled.
     *
     * A device reported as running its bootloader is rendered by what is actually executing. Only an
     * Inertial Sense main MCU runs "ISbl"; a peripheral in this state is in its OWN loader -- a u-blox
     * receiver in safeboot, a CXD in its updater -- and naming that ISbl asserts something untrue,
     * while the version shown is that loader's own rather than an ISbl version.
     *
     * @param devInfo the device info to render
     * @param flags a devFirmwareInfoFlags_e bitmask
     * @return the formatted string
     */
    std::string getFirmwareInfoAsString(const dev_info_t& devInfo, uint16_t flags = 0);

    /**
     * Renders the requested build-related fields of devInfo (commit hash, build key/number,
     * build date, build time) as a single delimited string, per the DV_BIT_ flags requested.
     * @param devInfo the dev_info_t supplying the build fields
     * @param flags   a bitmask of dev_info_fmt_e DV_BIT_ values selecting which fields to render
     * @param sep     the separator placed between each rendered field
     * @return the rendered build-info string
     */
    std::string getBuildAsString(const dev_info_t& devInfo, uint16_t flags = -1, const std::string& sep = " ");

    /**
     * Parses a hardware identity string (inverse of getHardwareAsString()). Accepts
     * "<TYPE>-<major>.<minor>[.<p2>[.<p3>]]", e.g. "IMX-5.0", "GPX-1.0.2", "uINS-3.2". Populates
     * devInfo.hardwareType and hardwareVer[0..3].
     * @param s       the hardware identity string to parse
     * @param devInfo the dev_info_t to populate; left unchanged if parsing fails
     * @return false on unrecognized type prefix or malformed version, otherwise true
     */
    bool parseHardwareFromString(const std::string& s, dev_info_t& devInfo);

    /**
     * Parses a hardware identity into a packed hardware-ID MASK, allowing the version to be omitted so a
     * whole family can be named. Omitted fields are encoded all-1s, which DEV_INFO_MATCHES_HDW_ID() and
     * ISDevice::matchesHdwId() treat as "match any value in this field".
     *
     * Where parseHardwareFromString() requires "<TYPE>-<major>.<minor>" and yields one concrete identity,
     * this accepts the partial forms a selection needs:
     *   "IMX"      -> any IMX, any version
     *   "IMX-5"    -> any IMX-5.x
     *   "IMX-5.0"  -> exactly IMX-5.0
     * Type names come from the same tables parseHardwareFromString() uses, and are matched
     * case-insensitively.
     *
     * @param s          the hardware identity or family to parse
     * @param[out] hdwId the packed mask; unchanged if parsing fails
     * @return false on an unrecognized type name or malformed version, otherwise true
     */
    bool parseHardwareIdMask(const std::string& s, uint16_t& hdwId);

    /**
     * Parses a device selection into a unique-ID MASK (packed hardware ID in bits 48-63, serial number in
     * bits 0-31), leaving whichever part was omitted as a wildcard. Hardware type and serial number are two
     * masks over one identity, so a selection may name either or both:
     *   "IMX"               any IMX, any serial
     *   "IMX-5.0"           exactly IMX-5.0, any serial
     *   "SN62913", "62913"  that serial, any hardware
     *   "IMX-5.0::SN62913"  both; ':' is accepted in place of '::'
     *
     * Compare ISDevice::parseDeviceIdString(), which identifies ONE device and so requires a serial number
     * (returning 0 without one). This is the selection form, where an omitted part means "any".
     *
     * @param s           the selection to parse
     * @param[out] idMask the packed mask; unchanged if parsing fails. A fully-wild selection yields
     *                    (IS_HARDWARE_ANY << 48), which matches every device.
     * @return false if neither a hardware identity nor a serial number could be read, otherwise true
     */
    bool parseDeviceIdMask(const std::string& s, uint64_t& idMask);

    /**
     * Tests a device's identity against a unique-ID mask from parseDeviceIdMask().
     * @param devInfo the device identity to test
     * @param idMask  the mask; wildcard hardware fields and a zero serial match anything
     * @return true if devInfo satisfies every non-wildcard part of the mask
     */
    bool devInfoMatchesIdMask(const dev_info_t& devInfo, uint64_t idMask);

    /**
     * Parses a firmware version string (inverse of getFirmwareAsString()). Accepts an optional
     * "fw" prefix followed by "<M>.<m>.<p>" and an optional build-type suffix
     * "-alpha|-beta|-rc|-devel|-snap" with an optional ".<build>". Populates
     * devInfo.firmwareVer[0..3] and buildType ('a'|'b'|'c'|'d'|'s'|0 for production/no suffix).
     * The legacy "-r" suffix and unknown suffixes are normalized to 0 (production).
     * @param s       the firmware version string to parse
     * @param devInfo the dev_info_t to populate
     * @return false on malformed input, otherwise true
     */
    bool parseFirmwareFromString(const std::string& s, dev_info_t& devInfo);
    // semver::version<uint8_t, uint8_t, uint8_t> getSemanticVersion(const dev_info_t& devInfo, uint16_t flags = -1);

    /**
     * @return the current system clock as a string with millisecond precision
     */
    std::string getCurrentTimestamp();

    /**
     * Formats a string representation of devInfo, in the specific format also understood by
     * devInfoFromString(): "SN[serialNo]: [hdwType]-[hdwVer], fw[fwVersion] b[buildNum][buildType]
     * [buildDate] [buildTime] (addlInfo)", e.g. "SN102934: IMX-5.0, fw2.1.7 b83c 2024-09-18
     * 15:35:43 (p12 cmp)". Not all dev_info_t fields are rendered, but the majority are.
     * @param devInfo the dev_info_t struct that provides the values to render
     * @param flags   a bitmask of dev_info_fmt_e DV_BIT_ values selecting which fields to render
     * @return a std::string representation of devInfo
     */
    std::string devInfoToString(const dev_info_t& devInfo, uint16_t flags = -1);

    /**
     * Parses and populates a dev_info_t struct from an input string (inverse of
     * devInfoToString()). Works by attempting to parse a series of "components" from the input
     * string, removing each successfully parsed component from the string, and then trying again
     * until the string is fully consumed or fails to match any component patterns.
     * @param str     the string to parse
     * @param devInfo the dev_info_t struct to parse into
     * @return a bitmask of dev_info_fmt_e DV_BIT_ values indicating which components were parsed
     */
    uint16_t devInfoFromString(const std::string& str, dev_info_t& devInfo);

    /**
     * Converts the dev_info_t build date/time into a single uint64_t that is still
     * human-readable, but numerically significant and suitable for sorting, e.g.
     * [2024, 05, 01, 23, 18, 35] becomes (uint64_t) 20240501231835.
     * @param a         the dev_info_t whose build date/time will be used
     * @param useMillis if true, append the build milliseconds as an additional low-order digit group
     * @return the packed, sortable build date/time value
     */
    uint64_t intDateTimeFromDevInfo(const dev_info_t& a, bool useMillis = false);

    /**
     * Generates a potential/expected firmware filename based on the given devInfo. E.g. given a
     * devInfo describing an IMX-5.0 running firmware 2.5.1, built on 2025-05-31 at 20:10:07, this
     * returns something like "IS_IMX-5_v2.5.1+2025-05-31-201007.hex" -- typically the firmware
     * file that was used to load that firmware.
     * @param devInfo the dev_info_t describing the firmware; passed by value since buildMillisecond
     *                is zeroed internally before formatting
     * @return the generated firmware filename
     */
    std::string firmwareFileFromDevInfo(dev_info_t devInfo);

    /**
     * Compares the serial number and full hardware version (hardwareVer[0..3]) of two dev_info_t
     * structs for an exact match.
     * @param info1 the first dev_info_t to compare
     * @param info2 the second dev_info_t to compare
     * @return true if the serial number and hardware version match exactly, otherwise false
     */
    bool devInfoHdwMatch(const dev_info_t &info1, const dev_info_t &info2);

    /**
     * Performs a series of tests, based on the flags bitmask, to determine if two dev_info_t
     * structs are effective matches. This is not a byte-for-byte match, but rather a filter of
     * specific sets of fields within the dev_info_t struct to make a determination of equality.
     * For example, this can test whether the main firmware version matches (major, minor, patch),
     * but disregard whether the build date/time matches, etc.
     * @param info1 the first dev_info_t to compare
     * @param info2 the second dev_info_t to compare with
     * @param flags a bitmask of dev_info_fmt_e DV_BIT_ values identifying the fields to test, and
     *              other related comparison flags (e.g. DV_BIT_EXACT_MATCH)
     * @return true if the two match the specified flags, otherwise false
     */
    bool devInfoVersionMatch(const dev_info_t &info1, const dev_info_t &info2, int flags = DV_BIT_FIRMWARE_VER | DV_BIT_BUILD_COMMIT | DV_BIT_BUILD_DATE | DV_BIT_BUILD_TIME);

    /**
     * Determines if two dev_info_t structs describe compatible hardware (same hardware type,
     * major/minor version, and run state). Used primarily to determine if a firmware is
     * compatible with a target device.
     * @param a the first dev_info_t to compare
     * @param b the second dev_info_t to compare
     * @return true if the two dev_info_t structs are "compatible", otherwise false
     */
    bool isDevInfoCompatible(const dev_info_t& a, const dev_info_t& b);

    /**
     * Compares the full firmware version (firmwareVer[0..3], buildType, build date/time, and
     * build key) of two dev_info_t structs. Equivalent to compareFirmwareVersions(a, b, 0xFFFF).
     * @param a dev_info_t representing a particular firmware version
     * @param b dev_info_t representing a particular firmware version
     * @return an integer representing the difference between a and b; 0 if both are equal, >0 if
     *   a > b, and <0 if a < b. The magnitude reflects which components differ (each compared
     *   component occupies its own bit range of the result) but is not itself a meaningful distance.
     */
    int64_t compareFirmwareVersions(const dev_info_t& a, const dev_info_t& b);

    /**
     * Compares the firmware version fields of two dev_info_t structs selected by fields. A
     * comparator suitable for use by std::map<> to order dev_info_t by firmware version.
     * @param a      dev_info_t representing a particular firmware version
     * @param b      dev_info_t representing a particular firmware version
     * @param fields a bitmask of dev_info_fmt_e DV_BIT_ values selecting which components to compare
     * @return an integer representing the difference between a and b; 0 if both are equal, >0 if
     *   a > b, and <0 if a < b. The magnitude reflects which components differ (each compared
     *   component occupies its own bit range of the result) but is not itself a meaningful distance.
     */
    int64_t compareFirmwareVersions(const dev_info_t& a, const dev_info_t& b, uint16_t fields);

    // int parseStringVersion(const std::string& vIn, uint8_t vOut[4]);
    // bool devInfoFromFirmwareImage(std::string imgFilename, dev_info_t& devInfo);

    /**
     * Generates a detailed comparison, field-by-field, of two pointers of a particular DID.
     * @param did the Data ID of the data buffers to compare (A & B)
     * @param A a pointer to the first data buffer to compare
     * @param B a pointer to the second data buffer to compare
     * @param printDiff if true, print a detailed list of which fields were different, comparing their values.
     *   if false, no output it printed
     * @return true if the two data buffers match, otherwise false
     */
    bool compareDataIDs(uint32_t did, const uint8_t* A, const uint8_t* B, bool printDiff);

    /**
     * returns a string describing the portInfo parameter from a port_monitor_set_t.
     * @param portInfo the port_monitor_set_t.portInfo
     * @return a string of format "TYPE.ID"
     */
    std::string getPortMonitorDescription(uint8_t portInfo);

    /**
     * Compared two dev_info_t structs, and returns an bitmap indicating which fields match
     * @param info1
     * @param info2
     * @return a uint32_t with each bit indicating a match of a specific field in the struct
     */
    uint32_t compareDevInfo(const dev_info_t& info1, const dev_info_t& info2);

    /**
     * Checks whether domainName is a syntactically valid DNS domain name (RFC-1035-style labels,
     * with support for punycode "xn--" labels), and no longer than 254 characters.
     * @param domainName the domain name string to validate
     * @return true if domainName is a syntactically valid domain name, otherwise false
     */
    bool validDomainName(const std::string& domainName);

    /**
     * @brief Normalized components of a parsed URI, ready for direct use by the SDK.
     *
     * Wraps FIX8::uri parsing and centralizes the conventions that were previously
     * duplicated across the port factories and correction services: IPv6 literal hosts
     * are returned with their surrounding brackets stripped (e.g. "::1", not "[::1]"),
     * and the port is converted to a validated integer (1..65535) rather than a raw
     * string, with -1 indicating an absent, malformed, or out-of-range port.
     */
    struct UriParts {
        std::string scheme;     //!< URI scheme as provided (e.g. "tcp", "ntrip"); empty if absent
        std::string host;       //!< host with IPv6 brackets stripped; empty if absent
        int         port = -1;  //!< port number 1..65535, or -1 if absent/malformed/out-of-range
        std::string user;       //!< userinfo username; empty if absent
        std::string password;   //!< userinfo password; empty if absent
        std::string path;       //!< path component (e.g. an NTRIP mountpoint); empty if absent
        std::string query;      //!< query component; empty if absent

        /** @return true if a scheme was present */
        bool hasScheme() const { return !scheme.empty(); }
        /** @return true if a host was present */
        bool hasHost() const { return !host.empty(); }
        /** @return true if a valid port was present */
        bool hasPort() const { return port >= 0; }
        /** @return true if userinfo was present */
        bool hasUserinfo() const { return !user.empty() || !password.empty(); }
    };

    /**
     * @brief Parse a URI string into its normalized components.
     *
     * This is the single, codebase-wide entry point for breaking a URI into its parts; prefer it
     * over calling FIX8::uri directly so the IPv6-bracket and port-validation conventions stay
     * consistent. Note that FIX8::uri requires an authority introducer ("//") for the host/port
     * to be parsed (e.g. "tcp://host:port", not "tcp:host:port").
     *
     * @param uriStr the URI to parse (e.g. "tcp://[::1]:7777", "ntrip://user:pass@host:2101/MOUNT")
     * @return a UriParts with each present component populated; absent components are left empty (port == -1)
     */
    UriParts parseUri(const std::string& uriStr);

    /**
     * @brief Parse a URI, falling back to a second "defaults" URI for any component the primary omits.
     *
     * Each component absent from @p uriStr (scheme, host, port, user, password, path, query) is filled
     * from the corresponding component of @p defaultsUri. This lets callers express their defaults as a
     * URI string instead of patching individual fields — e.g. parseUri(arg, "tcp://127.0.0.1:7777")
     * yields the host/port the caller wants whenever @p uriStr leaves them out.
     *
     * @param uriStr      the URI to parse
     * @param defaultsUri a URI supplying default values for any component @p uriStr omits
     * @return a UriParts with present components from @p uriStr and the remainder from @p defaultsUri
     */
    UriParts parseUri(const std::string& uriStr, const std::string& defaultsUri);

    /**
     * Encodes the 3x uint32_t STM32 UID registers into the UUID format required by the calibration-db API.
     * Encoding: UID registers as LE uint32_t bytes, swapped to BE for UUID fields 1 & 2,
     * field 3 = 0x8EF4, field 4 prefix = 0x99 0x6B.
     * Example: SN522807 → "20313933-534B-8EF4-996B-5016002b0016"
     * @param uid Array of 3 uint32_t values from manufacturing_info_t.uid[0..2]
     * @return UUID string formatted as "xxxxxxxx-xxxx-8EF4-996B-xxxxxxxxxxxx"
     */
    std::string encodeSTM32UID(const uint32_t uid[3]);

    /**
     * Generates a random UUID v4 string.
     * @return UUID string formatted as "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx"
     */
    std::string generateUUIDv4();
};

/**
 * A fixed-size, in-memory std::streambuf that also tracks which byte ranges have been written
 * (via insert()), so a caller can later query whether a given range has been populated. Useful
 * for assembling a buffer out-of-order (e.g. from out-of-order network chunks).
 */
class ByteBuffer : public std::streambuf {
public:
    /**
     * Allocates a zero-initialized buffer of the given size and prepares the get/put pointers to
     * span it.
     * @param size the fixed size, in bytes, of the buffer
     */
    ByteBuffer(std::size_t size) : size_(size) {
        buffer_.resize(size_, 0); // Initialize buffer with zeros
        setg(buffer_.data(), buffer_.data(), buffer_.data() + buffer_.size());
        setp(buffer_.data(), buffer_.data() + buffer_.size());
    }

    /**
     * Copies len bytes from data into the buffer starting at pos, and records [pos, pos+len) as
     * an initialized range (merging it with any adjacent/overlapping previously-initialized ranges).
     * @param pos  the byte offset within the buffer to write to
     * @param data the bytes to copy in
     * @param len  the number of bytes to copy
     */
    void insert(std::size_t pos, const uint8_t* data, std::size_t len) {
        if (pos + len > buffer_.size()) {
            throw std::out_of_range("Insert position out of range");
        }
        std::memcpy(buffer_.data() + pos, data, len);
        initialized_ranges_.emplace_back(pos, pos + len);
        merge_initialized_ranges();
    }

    /** @return the current read position, as an offset from the start of the buffer */
    std::size_t tellg() const {
        return gptr() - eback();
    }

    /** @return the current write position, as an offset from the start of the buffer */
    std::size_t tellp() const {
        return pptr() - pbase();
    }

    /** @return the current value of the internal write-position counter */
    std::size_t data_size() const {
        return current_write_pos_;
    }

    /**
     * Moves the read position to the given offset from the start of the buffer.
     * @param pos the byte offset to seek the read position to
     */
    void seekg(std::size_t pos) {
        setg(eback(), eback() + pos, egptr());
    }

    /**
     * Checks whether the byte range [pos, pos+len) has been fully populated by one or more prior
     * insert() calls.
     * @param pos the starting byte offset of the range to check
     * @param len the length, in bytes, of the range to check
     * @return true if the entire range has been initialized, otherwise false
     */
    bool is_initialized(std::size_t pos, std::size_t len) const {
        auto end_pos = pos + len;
        for (const auto& range : initialized_ranges_) {
            if (pos >= range.first && end_pos <= range.second) {
                return true;
            }
        }
        return false;
    }

private:
    std::vector<char> buffer_;
    std::size_t size_;
    std::size_t current_write_pos_ = 0;
    std::vector<std::pair<std::size_t, std::size_t>> initialized_ranges_;

    void merge_initialized_ranges() {
        if (initialized_ranges_.empty()) return;
        std::sort(initialized_ranges_.begin(), initialized_ranges_.end());
        std::vector<std::pair<std::size_t, std::size_t>> merged;
        merged.push_back(initialized_ranges_[0]);

        for (const auto& range : initialized_ranges_) {
            if (merged.back().second >= range.first) {
                merged.back().second = (merged.back().second > range.second ? merged.back().second : range.second);
            } else {
                merged.push_back(range);
            }
        }
        initialized_ranges_ = std::move(merged);
    }
};

/**
 * A std::iostream front-end for a ByteBuffer, providing the same tellg()/tellp()/seekg()/
 * data_size()/is_initialized() convenience accessors directly on the stream.
 */
class ByteBufferStream : public std::iostream {
public:
    /**
     * Binds this stream to the given ByteBuffer, which must outlive the stream.
     * @param buffer the ByteBuffer to read/write through this stream
     */
    ByteBufferStream(ByteBuffer& buffer)
            : std::iostream(&buffer), buffer_(buffer) {}

    /** @return the current read position, as an offset from the start of the underlying buffer */
    std::size_t tellg() const {
        return buffer_.tellg();
    }

    /** @return the current write position, as an offset from the start of the underlying buffer */
    std::size_t tellp() const {
        return buffer_.tellp();
    }

    /** @return the current value of the underlying buffer's internal write-position counter */
    std::size_t data_size() const {
        return buffer_.data_size();
    }

    /**
     * Moves the read position to the given offset from the start of the underlying buffer.
     * @param pos the byte offset to seek the read position to
     */
    void seekg(std::size_t pos) {
        buffer_.seekg(pos);
    }

    /**
     * Checks whether the byte range [pos, pos+len) of the underlying buffer has been fully
     * populated by one or more prior ByteBuffer::insert() calls.
     * @param pos the starting byte offset of the range to check
     * @param len the length, in bytes, of the range to check
     * @return true if the entire range has been initialized, otherwise false
     */
    bool is_initialized(std::size_t pos, std::size_t len) const {
        return buffer_.is_initialized(pos, len);
    }

private:
    ByteBuffer& buffer_;
};


// #define FN_PROFILER_ENABLED
/**
 * RAII scoped function-timing profiler: on destruction, logs (at IS_LOG_FN_PROFILER) the elapsed
 * time since construction if it exceeds threshold, along with any intermediate mark() timestamps.
 * Compiles to a complete no-op unless FN_PROFILER_ENABLED is defined.
 */
class FnProfiler {
public:
#ifdef FN_PROFILER_ENABLED
    /**
     * Starts timing, recording the current time and an identifying label for the log output.
     * @param functionName a label (typically the function name) to identify this timing in the log output
     * @param threshold    the minimum elapsed microseconds required for the destructor to emit a log message
     */
    FnProfiler(const std::string& functionName, uint32_t threshold = 100) : m_functionName(functionName), m_threshold(threshold), m_startTime(std::chrono::high_resolution_clock::now()) { }

    // Destructor calculates and prints the duration
    ~FnProfiler() {
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - m_startTime);
        if (duration.count() < m_threshold)
            return;

        log_debug(IS_LOG_FN_PROFILER, "'%s' executed in %lluus", m_functionName.c_str(), duration.count());
        auto lastMark = m_startTime;
        for (const auto& [m, msg] : markers) {
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(m - m_startTime);
            auto span = std::chrono::duration_cast<std::chrono::microseconds>(m - lastMark);
            log_debug(IS_LOG_FN_PROFILER, "    %7uus (%7uus, %5.1f%%) ::  %s", elapsed, span, ((float)span.count() / (float)duration.count()) * 100.0f, msg.c_str());
            lastMark = m;
        }
    }

    /**
     * Records an intermediate timestamp, labeled with msg, to be reported (relative to the start
     * time and the previous mark) alongside the total duration when this FnProfiler is destroyed.
     * @param msg a label describing this checkpoint
     */
    void mark(const std::string& msg) {
        markers.emplace_back(std::make_pair(std::chrono::high_resolution_clock::now(), msg));
    }

private:
    std::string m_functionName;
    uint32_t m_threshold;
    std::chrono::high_resolution_clock::time_point m_startTime;
    std::vector<std::pair<std::chrono::high_resolution_clock::time_point, std::string>> markers;
#else
    /**
     * No-op constructor used when FN_PROFILER_ENABLED is not defined.
     * @param functionName unused
     * @param threshold    unused
     */
    FnProfiler(const std::string& functionName, uint32_t threshold = 100) { (void) functionName; (void) threshold; }

    // Destructor calculates and prints the duration
    ~FnProfiler() { }

    /**
     * No-op when FN_PROFILER_ENABLED is not defined.
     * @param msg unused
     */
    void mark(const std::string& msg) { (void) msg; }
#endif
};

/**
 * RAII helper that invokes an arbitrary callable (its type F is deduced from the constructor
 * argument) when it goes out of scope, regardless of how the scope is exited (normal return,
 * early return, or exception). Non-copyable and non-movable to prevent the wrapped callable from
 * accidentally being invoked more than once.
 */
template <typename F>
class Finalizer {
    F f;
public:
    /**
     * @param f the callable to invoke when this Finalizer is destroyed
     */
    explicit Finalizer(F f) : f(std::move(f)) {}
    ~Finalizer() { f(); }

    // Delete copy/move to prevent accidental double-execution
    Finalizer(const Finalizer&) = delete;
    Finalizer& operator=(const Finalizer&) = delete;
};
#endif //IS_SDK__UTIL_H
