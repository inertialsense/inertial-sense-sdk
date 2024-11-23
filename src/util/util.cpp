/**
 * @file util.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 3/14/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#include "util.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <regex>
#include <string>
#include <stdexcept>

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

std::string utils::getCurrentTimestamp() {
    using std::chrono::system_clock;
    auto currentTime = std::chrono::system_clock::now();
    char buffer[80];

    auto transformed = currentTime.time_since_epoch().count() / 1000000;

    auto millis = transformed % 1000;

    std::time_t tt;
    tt = system_clock::to_time_t(currentTime);
    auto timeinfo = localtime(&tt);
    strftime(buffer, 80, "%F %H:%M:%S", timeinfo);
    sprintf(buffer + strlen(buffer), "%03d", (int) millis);

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
std::string utils::devInfoToString(const dev_info_t& devInfo) {
    std::string out;

    // device serial no
    out += utils::string_format("SN%06d: ", devInfo.serialNumber);

    // hardware type & version
    const char *typeName = "\?\?\?";
    switch (devInfo.hardwareType) {
        case IS_HARDWARE_TYPE_UINS: typeName = "uINS"; break;
        case IS_HARDWARE_TYPE_IMX: typeName = "IMX"; break;
        case IS_HARDWARE_TYPE_GPX: typeName = "GPX"; break;
        default: typeName = "\?\?\?"; break;
    }
    out += utils::string_format("%s-%u.%u", typeName, devInfo.hardwareVer[0], devInfo.hardwareVer[1]);
    if ((devInfo.hardwareVer[2] != 0) || (devInfo.hardwareVer[3] != 0)) {
        out += utils::string_format(".%u", devInfo.hardwareVer[2]);
        if (devInfo.hardwareVer[3] != 0)
            out += utils::string_format(".%u", devInfo.hardwareVer[3]);
    }

    // firmware version
    out += utils::string_format(" fw%u.%u.%u", devInfo.firmwareVer[0], devInfo.firmwareVer[1], devInfo.firmwareVer[2]);
    if (devInfo.firmwareVer[3] != 0)
        out += utils::string_format(".%u", devInfo.firmwareVer[3]);

    // build number/type
    out += utils::string_format(" b%u", devInfo.buildNumber);
    if ((devInfo.buildType != 0) && (devInfo.buildType != 'r'))
        out += utils::string_format("%c", devInfo.buildType);

    // build date/time
    out += utils::string_format(" %04u-%02u-%02u", devInfo.buildYear + 2000, devInfo.buildMonth, devInfo.buildDay);
    out += utils::string_format(" %02u:%02u:%02u", devInfo.buildHour, devInfo.buildMinute, devInfo.buildSecond);
    if (devInfo.buildMillisecond)
        out += utils::string_format(".%03u", devInfo.buildMillisecond);

    // additional info
    out += utils::string_format(" (%s)", devInfo.addInfo);
    return out;
}

/**
 * A convenience function to parse and populate a dev_info_t struct
 * from an input string of a specific format (the format matching
 * devInfoToString()). Note that this does not parse all dev_info_t
 * fields, but it does handle the majority of them. This function
 * used a regex pattern to match/parse the string contents.
 * @param str the string to parse
 * @param devInfo the dev_info_t struct to parse into
 * @return true if the string was successfully parsed
 */
bool utils::devInfoFromString(const std::string& str, dev_info_t& devInfo) {
    // first, see what we can derive from the filename
    // REGEX Patterns:
    // SN[serialNo]: [hdwType]-[hdwVer], fw[fwVersion] b[buildNum][buildType] [buildDate] [buildTime] (addlInfo)
    // SN102934: IMX-5.0, fw2.1.7 b83c 2024-09-18 15:35:43 (p12 cmp)
    std::regex fwPattern("SN(\\d+): ([\\w]+)-([\\d\\.]+), fw([\\d.]+) b([\\d.]+)([a-z]{0,1}) ([\\d-]+ [\\d:]{8}) \\([^\\)]+\\)$");
    std::smatch match;

    if (!std::regex_match(str, match, fwPattern))
        return false;

    // match components are: (0) serial-number, (1) device-type, (2) hdw-version, (3) image-type, (4) firmware version, (5) build-number, (6) build-type, (7) build-date, (8) build-time, (9) addl info

    // hardware type
    std::string hdwType = match[1];
    for (int i = 0; i < IS_HARDWARE_TYPE_COUNT; i++) {
        if (hdwType == g_isHardwareTypeNames[i]) {
            devInfo.hardwareType = i;
            break;
        }
    }

    // hardware version - either 1 or 2 digits, seperated by decimal
    parseStringVersion(match[2].str(), devInfo.hardwareVer);

    // image type (we don't really care)
    std::string imgType = match[3];

    // firmware version - typically 3 digits, but could be 4, seperated by decimal
    parseStringVersion(match[4].str(), devInfo.firmwareVer);

    // build number
    devInfo.buildNumber = std::stol(match[5].str());

    // build/release type
    devInfo.buildType = match[6].str()[0];

    std::tm tm = {};
    std::string fwBuildDateTime = match[7].str() + " " + match[8].str();
    strptime(fwBuildDateTime.c_str(), "%Y-%m-%d %H:%M:%S", &tm);
    devInfo.buildYear = tm.tm_year;
    devInfo.buildMonth = tm.tm_mon + 1;
    devInfo.buildDay = tm.tm_mday;
    devInfo.buildHour = tm.tm_hour;
    devInfo.buildMinute = tm.tm_min;
    devInfo.buildSecond = tm.tm_sec;
    devInfo.buildMillisecond = 0;

    return true;
}

/**
 * simple utility to parse a version string of x.x.x.x (upto 4) and return descrete
 * values into an array of 4 bytes. If the string contains fewer than 4 numbers,
 * the remaining values are not assigned (you should initialize vOut before calling
 * this function).
 * @param vIn the string "1.2.3.4" to parse
 * @param vOut an array of 4 bytes, each bytes containing the parsed version number.
 * @return returns the number of elements parsed.
 */
int utils::parseStringVersion(const std::string& vIn, uint8_t vOut[4]) {
    long unsigned int start = 0, n = 0, end = 0;
    while ((n < 3) && ((end = vIn.find_first_of('.', start)) != std::string::npos)){
        vOut[n++] = stoi(vIn.substr(start, end - start));
        start = end+1;
    }
    vOut[n++] = stoi(vIn.substr(start));
    return n;
}

/**
 * Attempts to populate a dev_info_t struct with the firmware version and build info parsed
 * from either the filename or the file contents.
 * @param imageFile
 * @param devInfo
 * @return true if dev_info_t was populated, otherwise false
 */
bool utils::fillDevInfoFromFirmwareImage(std::string imgFilename, dev_info_t& devInfo) {
    // first, see what we can derive from the filename
    // REGEX Patterns:
    // -- IMX BL Firmware: IS_(\w+)-([\d\.]+)_(\w+)_(v[\d.]+)_(b[\d.]+)_([\d-]+)_([\d]+).*(\.hex)
    // -- IMX/GPX App Firmware:  IS_(\w+)-([\d\.]+)_(\w+_|)(v[\d.]+)_(b[\d.]+)_([\d-]+)_([\d]+).*(\.[\w]+)$
    // -- Firmware Package:  IS-(\w+)_(r[\d.]+)_([\d-]+)_([\d]+).*(\.[\w]+)$
    std::regex fwPattern("IS_(\\w+)-([\\d\\.]+)_(\\w+_|)v([\\d.]+)_b([\\d.]+)([a-z]{0,1})_([\\d-]+)_([\\d]{6}).*(\\.[\\w]+)$");
    std::smatch match;

    if (!std::regex_match(imgFilename, match, fwPattern))
        return false;

    // match components are: (0) device-type, (1) hdw-version, (2) image-type, (3) firmware version, (4) build-number, (5) build-type, (6) build-date, (7) build-time, (8) file-type

    // hardware type
    const std::string hdwType = match[1];
    for (int i = 0; i < IS_HARDWARE_TYPE_COUNT; i++) {
        if (hdwType == g_isHardwareTypeNames[i]) {
            devInfo.hardwareType = i;
            break;
        }
    }

    // hardware version - either 1 or 2 digits, seperated by decimal
    const std::string hdwStr = match[2];
    parseStringVersion(hdwStr, devInfo.hardwareVer);

    // image type (we don't really care)
    const std::string imgType = match[3];

    // firmware version - typically 3 digits, but could be 4, seperated by decimal
    const std::string fwVerStr = match[4];
    parseStringVersion(match[4], devInfo.firmwareVer);

    // build number
    const std::string buildNumStr = match[5];
    devInfo.buildNumber = std::stol(buildNumStr);

    // build/release type
    const std::string buildTypeStr = match[6];
    if (buildTypeStr[0] == 0) devInfo.buildType = 'r';
    else devInfo.buildType = buildTypeStr[0];

    const std::string buildDateStr = match[7];
    const std::string buildTimeStr = match[8];

    std::tm tm = {};
    std::string fwBuildDateTime = buildDateStr + " " + buildTimeStr;
    strptime(fwBuildDateTime.c_str(), "%Y-%m-%d %H%M%S", &tm);
    devInfo.buildYear = tm.tm_year - 100; // we use 2-digit years starting 2000, not 1900.
    devInfo.buildMonth = tm.tm_mon + 1;
    devInfo.buildDay = tm.tm_mday;
    devInfo.buildHour = tm.tm_hour;
    devInfo.buildMinute = tm.tm_min;
    devInfo.buildSecond = tm.tm_sec;
    devInfo.buildMillisecond = 0;

/*
    int fileType = -1;
    const std::string filenameExtStr = match[9];
    if (filenameExtStr == ".hex") fileType = 0;
    else if (filenameExtStr == ".bin") fileType = 1;
    else if (filenameExtStr == ".fpk") fileType = 2;
*/

    // second, parse the file, and see if we can find any additional details -- we do this second because A) its expensive, and B) is more reliable, so it will overwrite values parsed from the filename
    return true;
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
