/**
 * @file util.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 3/14/24.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include "util.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <regex>
#include <stdexcept>
// #include <acc_prof.h>

#include "ISDataMappings.h"


#ifdef PLATFORM_IS_WINDOWS
#include <time.h>
#include <iomanip>
#include <sstream>

extern "C" char* strptime(const char* s,
                          const char* f,
                          struct tm* tm) {
    // Isn't the C++ standard lib nice? std::get_time is defined such that its
    // format parameters are the exact same as strptime. Of course, we have to
    // create a string stream first, and imbue it with the current C locale, and
    // we also have to make sure we return the right things if it fails, or
    // if it succeeds, but this is still far simpler an implementation than any
    // of the versions in any of the C standard libraries.
    std::istringstream input(s);
    input.imbue(std::locale(setlocale(LC_ALL, nullptr)));
    input >> std::get_time(tm, f);
    if (input.fail()) {
        return nullptr;
    }
    return (char*)(s + input.tellg());
}
#endif

/**
 * @brief Trims left-side/leading characters (whitespace by default) from the passed string; this modifies the string, in place.
 * Locates the first characters which are not within the set of ws, and removes all characters from the start upto that first
 * non-matching character.
 *
 * @param s the string to be trimmed
 * @param t a set of characters, which will be removed if they exists
 * @return a reference to the input string.
 */
std::string& utils::ltrim(std::string& s, const char* t) { s.erase(0, s.find_first_not_of(t)); return s; };

/**
 * @brief Trims right-side/trailing characters (whitespace by default) from the passed string; this modifies the string, in place.
 * Locates the last characters of the string which are not within the set of ws, and removes all characters from that position
 * until the end of the string non-matching character.
 *
 * @param s the string to be trimmed
 * @param t a set of characters, which will be removed if they exists
 * @return a reference to the input string.
 */
std::string& utils::rtrim(std::string& s, const char* t) { s.erase(s.find_last_not_of(t) + 1); return s; };

/**
 * @brief Trims left-size/leading and right-side/trailing characters (whitespace by default) from the passed string; this modifies
 * the string, in place. Call both rtrim() and ltrim() in a single call.
 *
 * @param s the string to be trimmed
 * @param t a set of characters, which will be removed if they exists
 * @return a reference to the input string.
 */
std::string& utils::trim(std::string& s, const char* t) { return ltrim(rtrim(s, t), t); };


/**
 * ltrim() equivalent which makes a new copy, and does not modify the original
 * @param s the string to be trimmed
 * @param t a set of characters, which will be removed if they exists
 * @return the modified/trimmed copy of the original input string.
 */
std::string utils::ltrim_copy(std::string s, const char* t) { return ltrim(s, t); };

/**
 * rtrim() equivalent which makes a new copy, and does not modify the original
 * @param s the string to be trimmed
 * @param t a set of characters, which will be removed if they exists
 * @return the modified/trimmed copy of the original input string.
 */
std::string utils::rtrim_copy(std::string s, const char* t) { return rtrim(s, t); };

/**
 * trim() equivalent which makes a new copy, and does not modify the original
 * @param s the string to be trimmed
 * @param t a set of characters, which will be removed if they exists
 * @return the modified/trimmed copy of the original input string.
 */
std::string utils::trim_copy(std::string s, const char* t) { return trim(s, t); };



/**
 * Splits the passed string into a vector of strings, delimited by delimiter.
 * @param str the string to be split
 * @param delimiter the substring to use as a delimiter
 * @return a vector of strings
 */
std::vector<std::string> utils::split_string(const std::string& str, const std::string& delimiter) {
    std::vector<std::string> strings;

    std::string::size_type pos = 0;
    std::string::size_type prev = 0;
    while ((pos = str.find(delimiter, prev)) != std::string::npos)
    {
        strings.push_back(str.substr(prev, pos - prev));
        prev = pos + delimiter.size();
    }

    // To get the last substring (or only, if delimiter is not found)
    strings.push_back(str.substr(prev));

    return strings;
}

/**
 * @return the current system clock as a string with millisecond precision
 */
std::string utils::getCurrentTimestamp() {
    auto currentTime = std::chrono::system_clock::now();
    auto transformed = currentTime.time_since_epoch().count() / 1000000;
    auto millis = transformed % 1000;

    std::time_t tt;
    tt = std::chrono::system_clock::to_time_t(currentTime);
    auto timeinfo = localtime(&tt);

    char buffer[80];
    strftime(buffer, 80, "%F %H:%M:%S", timeinfo);
    sprintf(buffer + strlen(buffer), ".%03d", (int) millis);

    return std::string(buffer);
}

/**
 * Formats the passed raw data as a "hexadecimal view". This can be used with any data.
 * @param raw_data a pointer to the raw byte stream
 * @param bytesLen the number of bytes following raw_data to output
 * @param bytesPerLine the number of hexadecimal bytes to print per line.
 * @return returns a fully formatted string
 */

std::string utils::raw_hexdump(const char* raw_data, int bytesLen, int bytesPerLine) {
    char buf[2048];
    char* ptrEnd = buf + 2048;
    char* ptr = buf;

#if DISPLAY_DELTA_TIME==1
    static double lastTime[2] = { 0 };
    double dtMs = 1000.0*(wheel.timeOfWeek - lastTime[i]);
    lastTime[i] = wheel.timeOfWeek;
    ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
#endif
    int lines = bytesLen / bytesPerLine;
    for (int j = 0; j < lines; j++) {
        int linelen = (j == lines-1) ? bytesLen % bytesPerLine : bytesPerLine;
        ptr += SNPRINTF(ptr, ptrEnd - ptr, "    ");
        for (int i = 0; i < linelen; i++) {
            ptr += SNPRINTF(ptr, ptrEnd - ptr, "%02x ", (uint8_t)raw_data[(j * bytesPerLine) + i]);
        }
        ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
    }

    return std::string(buf);
}

/**
 * Formats the specified DID's raw data as a "hexadecimal view". This can be used with any DID that is not
 * otherwise supported.
 * @param raw_data a pointer to the raw DID byte stream
 * @param hdr the DID header
 * @param bytesPerLine the number of hexadecimal bytes to print per line.
 * @return returns a fully formatted string
 */
std::string utils::did_hexdump(const char *raw_data, const p_data_hdr_t& hdr, int bytesPerLine)
{
    (void)hdr;
    char buf[2048];
    char* ptrEnd = buf + 2048;
    char* ptr = buf;

    ptr += SNPRINTF(ptr, ptrEnd - ptr, "(%d) %s (RAW):", hdr.id, cISDataMappings::DataName(hdr.id));

#if DISPLAY_DELTA_TIME==1
    static double lastTime[2] = { 0 };
    double dtMs = 1000.0*(wheel.timeOfWeek - lastTime[i]);
    lastTime[i] = wheel.timeOfWeek;
    ptr += SNPRINTF(ptr, ptrEnd - ptr, " %4.1lfms", dtMs);
#else
#endif
    ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
    int lines = hdr.size / bytesPerLine;
    for (int j = 0; j < lines; j++) {
        int linelen = (j == lines-1) ? hdr.size % bytesPerLine : bytesPerLine;
        ptr += SNPRINTF(ptr, ptrEnd - ptr, "\t");
        for (int i = 0; i < linelen; i++) {
            ptr += SNPRINTF(ptr, ptrEnd - ptr, "%02x ", (uint8_t)raw_data[(j * bytesPerLine) + i]);
        }
        ptr += SNPRINTF(ptr, ptrEnd - ptr, "\n");
    }

    return std::string(buf);
}

std::string utils::getHardwareAsString(const dev_info_t& devInfo) {
    // hardware type & version
    const char *typeName = "\?\?\?";
    switch (devInfo.hardwareType) {
        case IS_HARDWARE_TYPE_UINS: typeName = "uINS"; break;
        case IS_HARDWARE_TYPE_IMX: typeName = "IMX"; break;
        case IS_HARDWARE_TYPE_GPX: typeName = "GPX"; break;
        default: typeName = "\?\?\?"; break;
    }
    std::string out = utils::string_format("%s-%u.%u", typeName, devInfo.hardwareVer[0], devInfo.hardwareVer[1]);
    if ((devInfo.hardwareVer[2] != 0) || (devInfo.hardwareVer[3] != 0)) {
        out += utils::string_format(".%u", devInfo.hardwareVer[2]);
        if (devInfo.hardwareVer[3] != 0)
            out += utils::string_format(".%u", devInfo.hardwareVer[3]);
    }
    return out;
}

std::string utils::getFirmwareAsString(const dev_info_t& devInfo, const std::string& prefix) {
    std::string out = utils::string_format("%s%u.%u.%u", prefix.c_str(), devInfo.firmwareVer[0], devInfo.firmwareVer[1], devInfo.firmwareVer[2]);
    switch(devInfo.buildType) {
        case 'a': out +="-alpha";       break;
        case 'b': out +="-beta";        break;
        case 'c': out +="-rc";          break;
        case 'd': out +="-devel";       break;
        case 's': out +="-snap";        break;
        case '*': out +="-snap";        break;
        default : out +="";             break;
    }
    if (devInfo.firmwareVer[3] != 0)
        out += utils::string_format(".%u", devInfo.firmwareVer[3]);

    return out;
}

std::string utils::getBuildAsString(const dev_info_t &devInfo, uint16_t flags, const std::string& sep) {
    std::string out;

    if ((flags & DV_BIT_BUILD_COMMIT) && devInfo.repoRevision) {
        out += utils::string_format("%08x", devInfo.repoRevision);
        if (devInfo.buildType == '*') {
            out += "*";
        }
    }

    if (flags & DV_BIT_BUILD_KEY) {  // include build key/number
        // build number/type
        if (!out.empty()) out += sep;
        if (((devInfo.buildNumber >> 12) & 0xFFFFF) > 0) {
            out += utils::string_format("%05x.%d", ((devInfo.buildNumber >> 12) & 0xFFFFF), (devInfo.buildNumber & 0xFFF));
        } else {
            if (devInfo.buildNumber & 0xFFF) {
                out += utils::string_format("b%d", (devInfo.buildNumber & 0xFFF));
            }
        }
    }

    if (flags & DV_BIT_BUILD_DATE) {
        out += (out.empty() ? "" : sep) + utils::string_format( (flags & DV_BIT_COMPACT_DATE ? "%04u%02u%02u" : "%04u-%02u-%02u"), devInfo.buildYear + 2000, devInfo.buildMonth, devInfo.buildDay);
    }

    if (flags & DV_BIT_BUILD_TIME) {
        out += (out.empty() ? "" : sep) + utils::string_format( (flags & DV_BIT_COMPACT_TIME ?  "%02u%02u%02u" : "%02u:%02u:%02u"), devInfo.buildHour, devInfo.buildMinute, devInfo.buildSecond);

        if (devInfo.buildMillisecond)
            out += utils::string_format(".%03u", devInfo.buildMillisecond);
    }

    return out;
}

/**
 * A convenience function to format and return a string representation
 * of (in a very specific format, that matching devInfoFromString())
 * the contents of dev_info_t devInfo. Note that this does not format/
 * print all dev_info_t fields, but it does handle the majority of them.
 *
 * SN[serialNo]: [hdwType]-[hdwVer], fw[fwVersion] b[buildNum][buildType] [buildDate] [buildTime] (addlInfo)
 * SN102934: IMX-5.0, fw2.1.7 b83c 2024-09-18 15:35:43 (p12 cmp)
 *
 * @param devInfo the dev_info_t struct that provides the values to print
 * @return a std::string of the resulting devInfo;
 */
std::string utils::devInfoToString(const dev_info_t& devInfo, uint16_t flags) {
    std::string out;

    // device serial no

    if (flags & DV_BIT_SERIALNO)
        out += utils::string_format("SN%06d:", devInfo.serialNumber);
    if (flags & DV_BIT_HARDWARE_INFO)
        out += (out.empty() ? "" : " ") + utils::getHardwareAsString(devInfo);
    if (flags & DV_BIT_FIRMWARE_VER)
        out += (out.empty() ? "" : " ") + utils::getFirmwareAsString(devInfo);
    if (flags & (DV_BIT_BUILD_DATE | DV_BIT_BUILD_TIME | DV_BIT_BUILD_KEY | DV_BIT_BUILD_COMMIT))
        out += (out.empty() ? "" : " ") + utils::getBuildAsString(devInfo, flags);
    if ((flags & DV_BIT_ADDITIONAL_INFO) && devInfo.addInfo[0])
        out += (out.empty() ? "" : " ") + utils::string_format("(%s)", devInfo.addInfo);

    return out;
}

/**
 * @brief A convenience function to parse and populate a dev_info_t struct from an input string.
 * This function works by attempting to parse a series of "components" from the input string,
 * removing each successfully parsed component from the string, and then trying again until
 * the string is fully consumed, or the string fails to match any component patterns.
 *
 * @param str the string to parse
 * @param devInfo the dev_info_t struct to parse into
 * @return a bitmask of which components were parsed from the input string
 */
uint16_t utils::devInfoFromString(const std::string& str, dev_info_t& devInfo) {
    // first, see what we can derive from the filename
    // REGEX Patterns:
    // SN[serialNo]: [hdwType]-[hdwVer], fw[fwVersion] b[buildNum][buildType] [buildDate] [buildTime] (addlInfo)
    // SN102934: IMX-5.0 fw2.1.7-rc.83 1739c.5 2024-09-18 15:35:43 (p12 cmp)
    // SN102934: IMX-5.0 fw2.1.7 b83c 2024-09-18 15:35:43 (p12 cmp)
    // SN23914: GPX-1.0 fw0.1.12 b0b 2023-11-03 9:09:13 (Some Info)

    // These are a list of Regex patterns that match different components of a firmware string;
    // these MUST be in order of the most restrictive first, and least restrictive last, otherwise the least restrictive will consume everything.
    static std::vector<std::regex> componentPatterns = {
        std::regex(R"(SN(\d+)[:]?)"),                           //!< 0, serial number
        std::regex(R"((fw|v)([\d.]+)(-([\w]+)\.([\d]+))?)"),    //!< 1, firmware version w/ optional release type
        std::regex(R"((\d){4}-(\d){1,2}-(\d){1,2})"),           //!< 2, build date
        std::regex(R"((\d){1,2}:(\d){2}:(\d){2}|[_-](\d{6}))"), //!< 3, build time
        std::regex(R"((IS_)?([\w]+)-([\d\.]+)[,]?)"),           //!< 4, hdw type & version
        std::regex(R"(([0-9A-F]{5})(\.(\d+)))"),                //!< 5, build key and build number
        std::regex(R"(b(\d{1}[\d.]*)([a-z]?))"),                //!< 6, legacy build number and type -- note that this MUST be tested before 5, because 5 CAN catch it...
        std::regex(R"(\(([^\)]*)\))"),                          //!< 7, additional info
        std::regex(R"(([0-9a-f]{6,8})(\*)?)"),                  //!< 8, repo hash & build status (dirty)
    };

    int ii = 0;
    std::tm tm = {};
    uint16_t componentsParsed = 0;
    std::string local_str = str;    // make a copy that we can destroy
    bool making_progress = true;      // we'll keep trying, as long as we keep making progress..
    devInfo = {};                   // reinitialize dev_info_t

    while (!local_str.empty() && making_progress) {
        making_progress = false;

        for ( int i = 0, nc = componentPatterns.size(); i < nc; i++ ) {
            std::smatch match;
            if (std::regex_search(local_str, match, componentPatterns[i])) {
                making_progress = true;         // we've matched and consumed something, let's make note of it.
                switch (i) {
                    case 0: // parse SN
                        // serial number
                        devInfo.serialNumber = stoi(match[1].str());
                        componentsParsed |= DV_BIT_SERIALNO;
                        break;
                    case 1: // firmware version w/ optional release type
                        // firmware version - typically 3 digits, but could be 4, seperated by decimal
                        devInfo.buildType = 0;
                        for (auto& e : devInfo.firmwareVer) e = 0;
                        split_from_string<uint8_t, 4>(match[2].str(), devInfo.firmwareVer);
                        if (match[3].matched) {
                            if (match[4].str() == "alpha") devInfo.buildType = 'a';
                            else if (match[4].str() == "beta") devInfo.buildType = 'b';
                            else if (match[4].str() == "rc") devInfo.buildType = 'c';
                            else if (match[4].str() == "devel") devInfo.buildType = 'd';
                            else if (match[4].str() == "snap") devInfo.buildType = 's';

                            if (match[5].matched) {
                                devInfo.firmwareVer[3] = std::stoi(match[5].str());
                            }
                        }
                        componentsParsed |= DV_BIT_FIRMWARE_VER;
                        break;
                    case 2: // build date
                        strptime(match[0].str().c_str(), "%Y-%m-%d", &tm);
                        devInfo.buildYear = tm.tm_year - 100;
                        devInfo.buildMonth = tm.tm_mon + 1;
                        devInfo.buildDay = tm.tm_mday;
                        componentsParsed |= DV_BIT_BUILD_DATE;
                        break;
                    case 3: // build time
                        if (match[4].matched) strptime(match[4].str().c_str(), "%H%M%S", &tm);
                        else strptime(match[0].str().c_str(), "%H:%M:%S", &tm);
                        devInfo.buildHour = tm.tm_hour;
                        devInfo.buildMinute = tm.tm_min;
                        devInfo.buildSecond = tm.tm_sec;
                        devInfo.buildMillisecond = 0;
                        componentsParsed |= DV_BIT_BUILD_TIME;
                        break;
                    case 4: // parse HDW type & version
                        // hardware type
                        for (ii = 0; ii < IS_HARDWARE_TYPE_COUNT; ii++) {
                            if (match[2].str() == g_isHardwareTypeNames[ii]) {
                                devInfo.hardwareType = ii;
                                break;
                            }
                            if ((str.find("bootloader") != std::string::npos) || (str.find("mcuboot") != std::string::npos)) {
                                devInfo.hdwRunState = 1;    // Bootloader firmware?
                            } else {
                                devInfo.hdwRunState = 2;    // APP firmware??
                            }
                        }
                        // hardware version
                        for (auto& e : devInfo.hardwareVer) e = 0;
                        split_from_string<uint8_t, 4>(match[3].str(), devInfo.hardwareVer);
                        componentsParsed |= DV_BIT_HARDWARE_INFO;
                        break;
                    case 5: // build key and build number
                        {
                            int build_key = std::stoi(match[1].str(), NULL, 16);
                            int build_num = std::stoi(match[3].str());
                            devInfo.buildNumber = ((build_key << 12) & 0xFFFFF000) | (build_num & 0xFFF);
                        }
                        componentsParsed |= DV_BIT_BUILD_KEY;
                        break;
                    case 6:  // legacy build number and type
                        devInfo.buildNumber = std::stol(match[1].str());
                        devInfo.buildType = match[2].str()[0];
                        componentsParsed |= DV_BIT_BUILD_KEY;
                        break;
                    case 7: // additional info
                        strncpy(devInfo.addInfo, trim_copy(match[1].str(), "() ").c_str(), DEVINFO_ADDINFO_STRLEN-1);
                        devInfo.addInfo[DEVINFO_ADDINFO_STRLEN-1] = 0;
                        componentsParsed |= DV_BIT_ADDITIONAL_INFO;
                        break;
                    case 8:  // repo hash & build status
                        devInfo.repoRevision = std::stol(match[1].str(), NULL, 16);
                        if (match[2].matched && (match[2].str()[0] == '*')) {
                            devInfo.buildType = '*';
                        }
                        componentsParsed |= DV_BIT_BUILD_COMMIT;
                        break;
                }

                // don't remove the match string from the local_str until AFTER we parse it-- match[]es are an iterator into the searched string.
                int pos = match.position(0);
                int len = match.length(0);
                local_str.erase(pos, len);
                utils::trim(local_str);
            }
        }
    }

    return componentsParsed;
}

/**
 * simple check if two dev_info_t structs indicate equivelent/compatible hardware
 * Used primarily to determine if a firmware is compatible with a target device.
 * @param a
 * @param b
 * @return true if the two dev_info_t structs are "compatible", otherwise false
 */
bool utils::isDevInfoCompatible(const dev_info_t& a, const dev_info_t& b) {
    if (a.hardwareType != b.hardwareType)
        return false;
    if (a.hardwareVer[0] != b.hardwareVer[0])
        return false;
    if (a.hardwareVer[1] != b.hardwareVer[1])
        return false;
    if (a.hdwRunState != b.hdwRunState)         // don't forget to also confirm the run state - APP firmware is NOT THE SAME as Bootloader Firmware
        return false;
    return true;
}

/**
 * A simple convenience function converts the dev_info_t build date/time into a single uint64_t
 * that is still human-readable, but numerically significant, and suitable for sorting.
 * ie "[2024, 05, 01, 23, 18, 35]" = (uint64_t) 20240501231835
 * @param a the input dev_info_t from which the build date/time will be used
 * @return
 */
uint64_t utils::intDateTimeFromDevInfo(const dev_info_t& a, bool useMillis) {
    uint64_t aDateTime = 0;
    aDateTime = (a.buildYear < 100 ? 2000 : 1900) + a.buildYear;
    aDateTime = (aDateTime * 100) + a.buildMonth;
    aDateTime = (aDateTime * 100) + a.buildDay;
    aDateTime = (aDateTime * 100) + a.buildHour;
    aDateTime = (aDateTime * 100) + a.buildMinute;
    aDateTime = (aDateTime * 100) + a.buildSecond;
    if (useMillis)
        aDateTime = (aDateTime * 10000) + a.buildMillisecond;
    return aDateTime;
}


bool utils::devInfoHdwMatch(const dev_info_t &info1, const dev_info_t &info2)
{
    if (info1.serialNumber != info2.serialNumber)
    {
        return false;
    }

    for (int i = 0; i < 4; i++)
    {
        if (info1.hardwareVer[i] != info2.hardwareVer[i])
        {
            return false;
        }
    }

    return true;
}

bool utils::devInfoVersionMatch(const dev_info_t &info1, const dev_info_t &info2, bool checkTime) {
    if ((info1.firmwareVer[0] != info2.firmwareVer[0]) ||
        (info1.firmwareVer[1] != info2.firmwareVer[1]) ||
        (info1.firmwareVer[2] != info2.firmwareVer[2]) ||
        (info1.buildYear != info2.buildYear) ||
        (info1.buildMonth != info2.buildMonth) ||
        (info1.buildDay != info2.buildDay) ||
        (info1.buildNumber != info2.buildNumber) ||
        (info1.buildType != info2.buildType) ||
        (info1.protocolVer[0] != info2.protocolVer[0]) ||
        (info1.protocolVer[1] != info2.protocolVer[1]) ||
        (info1.protocolVer[2] != info2.protocolVer[2]) ||
        (info1.protocolVer[3] != info2.protocolVer[3]) ||
        (info1.repoRevision != info2.repoRevision)) {
        return false;
    }

    if (checkTime) {
        int minutes1 = info1.buildHour * 60 + info1.buildMinute;
        int minutes2 = info2.buildHour * 60 + info2.buildMinute;
        int dtMinutes = abs(minutes1 - minutes2);
        if (dtMinutes > 30) {
            return false;
        }
    }

    return true;
}

/**
 * A comparitor function which can be used by std::map<> to order a map in reverse order by dev_info_t (key)
 * @param a dev_info_t representing a particular firmware version
 * @param b dev_info_t representing a particular firmware version
 * @return returns true if A is greater/newer than B.
 */
bool utils::compareFirmwareVersions(const dev_info_t& a, const dev_info_t& b) {
    if (a.firmwareVer[0] != b.firmwareVer[0])
        return a.firmwareVer[0] > b.firmwareVer[0];
    if (a.firmwareVer[1] != b.firmwareVer[1])
        return a.firmwareVer[1] > b.firmwareVer[1];
    if (a.firmwareVer[2] != b.firmwareVer[2])
        return a.firmwareVer[2] > b.firmwareVer[2];
    if (a.firmwareVer[3] != b.firmwareVer[3])
        return a.firmwareVer[3] > b.firmwareVer[3];

    uint64_t aDateTime = intDateTimeFromDevInfo(a);
    uint64_t bDateTime = intDateTimeFromDevInfo(b);
    if (aDateTime != bDateTime)
        return aDateTime > bDateTime;

    if (a.buildNumber != b.buildNumber)
        return a.buildNumber > b.buildNumber;

    return a.buildType > b.buildType;
}

/**
 * Generates a detailed comparison, field-by-field, of two pointers of a particular DID.
 * @param did the Data ID of the data buffers to compare (A & B)
 * @param A a pointer to the first data buffer to compare
 * @param B a pointer to the second data buffer to compare
 * @param printDiff if true, print a detailed list of which fields were different, comparing their values.
 *   if false, no output it printed
 * @return true if the two data buffers match, otherwise false
 */
bool utils::compareDataIDs(uint32_t did, const uint8_t* A, const uint8_t* B, bool printDiff) {
    const data_set_t& dataInfo = *(cISDataMappings::DataSet(did));
    bool match = true;

    for (uint32_t i = 0; i < dataInfo.size; i++) {
        if (A[i] != B[i]) {
            // get the field Info given the current position; we should be able to determine the start and end of this field
            const data_info_t* fieldInfo = cISDataMappings::FieldInfoByOffset(did, i);
            if (fieldInfo) {
                if (printDiff) {
                    char valueBuff[32] = {};
                    std::string valueA = "<\?\?>", valueB = "<\?\?>";
                    std::string fieldName = (fieldInfo ? fieldInfo->name.c_str() : "<UNKNOWN>");
                    if (fieldInfo) {
                        cISDataMappings::DataToString(*fieldInfo, NULL, A, valueBuff, 0, false);
                        valueA = std::string(valueBuff);

                        cISDataMappings::DataToString(*fieldInfo, NULL, B, valueBuff, 0, false);
                        valueB = std::string(valueBuff);
                    }
                    printf("%s field '%s' mismatch: %s (A) != %s (B) (offset %d, size %d)\n",cISDataMappings::DataName(did), fieldName.c_str(), valueA.c_str(), valueB.c_str(), fieldInfo->offset, fieldInfo->size);
                    match = false;
                }
                i = fieldInfo->offset + fieldInfo->size;
            }
        }
    }
    return match;
}

std::string utils::getPortMonitorDescription(uint8_t portInfo) {
    static std::vector<std::string> portTypeNames = { "???", "SER", "USB", "SPI", "I2C", "CAN" };
    return utils::string_format("%s.%d", portTypeNames[(portInfo >> 4) & 0x0F].c_str(), portInfo & 0x0F);
}


/**
 * Compared two dev_info_t structs, and returns an bitmap indicating which fields match
 * @param info1
 * @param info2
 * @return a uint32_t with each bit indicating a match of a specific field in the struct
 */
uint32_t utils::compareDevInfo(const dev_info_t& info1, const dev_info_t& info2) {
    uint32_t match = 0;

    match |= (((info1.reserved          == info2.reserved)          & 1) << 0);

    match |= (((info1.hardwareType      == info2.hardwareType)      & 1) << 1);
    match |= (((info1.hdwRunState       == info2.hdwRunState)       & 1) << 2);

    match |= (((info1.serialNumber      == info2.serialNumber)      & 1) << 3);

    match |= (((info1.hardwareVer[0]    == info2.hardwareVer[0])    & 1) << 4);
    match |= (((info1.hardwareVer[1]    == info2.hardwareVer[1])    & 1) << 5);
    match |= (((info1.hardwareVer[2]    == info2.hardwareVer[2])    & 1) << 6);
    match |= (((info1.hardwareVer[3]    == info2.hardwareVer[3])    & 1) << 7);

    match |= (((info1.firmwareVer[0]    == info2.firmwareVer[0])    & 1) << 8);
    match |= (((info1.firmwareVer[1]    == info2.firmwareVer[1])    & 1) << 9);
    match |= (((info1.firmwareVer[2]    == info2.firmwareVer[2])    & 1) << 10);
    match |= (((info1.firmwareVer[3]    == info2.firmwareVer[3])    & 1) << 11);

    match |= (((info1.buildNumber       == info2.buildNumber)       & 1) << 12);

    match |= (((info1.protocolVer[0]    == info2.protocolVer[0])    & 1) << 13);
    match |= (((info1.protocolVer[1]    == info2.protocolVer[1])    & 1) << 14);
    match |= (((info1.protocolVer[2]    == info2.protocolVer[2])    & 1) << 15);
    match |= (((info1.protocolVer[3]    == info2.protocolVer[3])    & 1) << 16);

    match |= (((!strncmp(info1.manufacturer, info2.manufacturer, DEVINFO_MANUFACTURER_STRLEN))    & 1) << 17);

    match |= (((info1.buildType         == info2.buildType)         & 1) << 18);

    match |= (((info1.buildYear         == info2.buildYear)         & 1) << 19);
    match |= (((info1.buildMonth        == info2.buildMonth)        & 1) << 20);
    match |= (((info1.buildDay          == info2.buildDay)          & 1) << 21);

    match |= (((info1.buildHour         == info2.buildHour)         & 1) << 22);
    match |= (((info1.buildMinute       == info2.buildMinute)       & 1) << 23);
    match |= (((info1.buildSecond       == info2.buildSecond)       & 1) << 24);
    match |= (((info1.buildMillisecond  == info2.buildMillisecond)  & 1) << 25);

    match |= (((!strncmp(info1.addInfo, info2.addInfo, DEVINFO_ADDINFO_STRLEN))    & 1) << 26);

    return match;
}
