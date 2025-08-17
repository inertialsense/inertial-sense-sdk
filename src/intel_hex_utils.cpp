#include <cctype>
#include <cstdint>      // uint8_t, uint16_t, uint32_t
#include <cstddef>      // size_t
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>
#include "intel_hex_utils.h"




bool fileExists(const std::string& filename) {
    std::ifstream file(filename);
    return file.good();
}

// Convert two hex characters from a string to a byte
static uint8_t parseHexByte(const std::string& str, size_t pos) {
    return static_cast<uint8_t>(std::stoi(str.substr(pos, 2), nullptr, 16));
}

// Helper: trim trailing newline and whitespace
void trimTrailing(std::string& line) {
    line.erase(line.find_last_not_of(" \r\n") + 1);
}

// Helper: validate hex character
bool isHexChar(char c) {
    return std::isxdigit(static_cast<unsigned char>(c));
}

// Helper: validate Intel HEX line checksum
bool validateLineChecksum(const std::string& line) {
    uint8_t sum = 0;
    for (size_t i = 1; i < line.length(); i += 2) {
        sum += parseHexByte(line, i);
    }
    return sum == 0;
}

// Parse the HEX file and return a map of absolute address → data byte
static std::map<uint32_t, uint8_t> parseIntelHex(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    std::map<uint32_t, uint8_t> flashData;
    std::string line;
    uint32_t extendedLinearAddress = 0;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] != ':') {
            continue;
        }

        uint8_t byteCount = parseHexByte(line, 1);
        uint16_t address = static_cast<uint16_t>(std::stoi(line.substr(3, 4), nullptr, 16));
        uint8_t recordType = parseHexByte(line, 7);

        if (recordType == 0x00) { // Data record
            uint32_t absoluteAddress = (extendedLinearAddress << 16) | address;
            for (uint8_t i = 0; i < byteCount; ++i) {
                uint8_t dataByte = parseHexByte(line, 9 + i * 2);
                flashData[absoluteAddress + i] = dataByte;
            }
        } else if (recordType == 0x04) { // Extended linear address
            extendedLinearAddress = static_cast<uint16_t>(std::stoi(line.substr(9, 4), nullptr, 16));
        } else if (recordType == 0x01) { // End of file
            break;
        }
    }

    return flashData;
}

// Compute number of flash pages used, given parsed flash data and page size
static size_t computePagesUsed(const std::map<uint32_t, uint8_t>& flashData, size_t pageSize) {
    if (flashData.empty()) {
        return 0;
    }

    uint32_t minAddr = flashData.begin()->first;
    uint32_t maxAddr = flashData.rbegin()->first;

    size_t firstPage = minAddr / pageSize;
    size_t lastPage  = maxAddr / pageSize;

    return lastPage - firstPage + 1;
}

// Top-level function to call from other programs
size_t calculateFlashPagesUsed(const std::string& hexFilename, size_t flashPageSize) {
    auto flashData = parseIntelHex(hexFilename);
    return computePagesUsed(flashData, flashPageSize);
}


///////////////////////////////////////////////////////////////////////////
// Intel HEX validation functions
///////////////////////////////////////////////////////////////////////////

/**
 * @brief Validates the format and contents of an Intel HEX file.
 *
 * This function performs a comprehensive set of checks on the provided Intel HEX file
 * to ensure it adheres to the Intel HEX specification and does not contain overlapping data.
 * 
 * @note The function stops at the first error found and sets \p errorOut accordingly.
 *
 * **Validation checks performed:**
 *  - File access:
 *    - Ensures the file can be opened for reading.
 *  - Line-level structure:
 *    - Each line must begin with a colon (`:`).
 *    - All characters after the initial colon must be valid hexadecimal digits (`0-9`, `A-F`, `a-f`).
 *    - Line must be at least 11 characters long (minimum valid record length).
 *  - Data consistency:
 *    - Byte count field is parsed and used to compute the expected line length; actual line length must match.
 *    - Record type field must be in the range `0x00` to `0x05` (valid Intel HEX record types).
 *    - Line checksum must be valid according to the Intel HEX specification.
 *  - Record-specific rules:
 *    - End-of-file (EOF) record (`recordType == 0x01`):
 *      - Only one EOF record is allowed; multiple EOF records are rejected.
 *    - Data record (`recordType == 0x00`):
 *      - Computes the absolute address using any active extended linear address.
 *      - Ensures no byte of data overlaps with any previously written address.
 *    - Extended linear address record (`recordType == 0x04`):
 *      - Updates the high-order 16 bits of the absolute address for subsequent data records.
 *  - File-level structure:
 *    - At least one EOF record must be present before the end of file.
 * 
 * @param hexFilename The path to the HEX file.
 * @param errorOut Output parameter for error messages.
 * @return true If the file is valid.
 * @return false If the file is invalid.
 */
bool validateHexFile(const std::string& hexFilename, std::string& errorOut) {
    std::ifstream file(hexFilename);
    if (!file.is_open()) {
        errorOut = "Failed to open file: " + hexFilename;
        return false;
    }

    std::string line;
    size_t lineNum = 0;
    bool eofSeen = false;
    uint32_t extAddr = 0;
    std::unordered_set<uint32_t> writtenAddresses;

    while (std::getline(file, line)) {
        ++lineNum;

        trimTrailing(line);

        if (line.empty() || line[0] != ':') {
            errorOut = "Line " + std::to_string(lineNum) + " does not start with ':'";
            return false;
        }

        // Check for valid hex characters only (excluding colon at start)
        for (size_t i = 1; i < line.size(); ++i) {
            if (!isHexChar(line[i])) {
                errorOut = "Invalid hex character at line " + std::to_string(lineNum);
                return false;
            }
        }

        if (line.size() < 11) {
            errorOut = "Line " + std::to_string(lineNum) + " too short.";
            return false;
        }

        uint8_t byteCount = parseHexByte(line, 1);
        uint16_t address = static_cast<uint16_t>(std::stoi(line.substr(3, 4), nullptr, 16));
        uint8_t recordType = parseHexByte(line, 7);

        // Check expected length (1 ':' + 2 chars/byte * (byteCount + 4 fields) + 2 for checksum)
        size_t expectedLength = 9 + byteCount * 2 + 2; // 9 = 1B count + 2B addr + 1B type (all as hex)
        if (line.length() != expectedLength) {
            errorOut = "Incorrect line length at line " + std::to_string(lineNum);
            return false;
        }

        // Validate checksum
        if (!validateLineChecksum(line)) {
            errorOut = "Checksum mismatch at line " + std::to_string(lineNum);
            return false;
        }

        // Validate record type
        if (recordType > 0x05) {
            errorOut = "Unknown record type at line " + std::to_string(lineNum);
            return false;
        }

        // Check for multiple EOFs
        if (recordType == 0x01) {
            if (eofSeen) {
                errorOut = "Multiple EOF records detected.";
                return false;
            }
            eofSeen = true;
        }

        // Optional: check for overlapping addresses
        if (recordType == 0x00) {
            uint32_t absAddr = (extAddr << 16) | address;
            for (uint8_t i = 0; i < byteCount; ++i) {
                uint32_t a = absAddr + i;
                if (writtenAddresses.count(a)) {
                    { std::stringstream ss; ss << std::hex << std::uppercase << a; errorOut = "Overlapping data at address 0x" + ss.str(); }
                    return false;
                }
                writtenAddresses.insert(a);
            }
        } else if (recordType == 0x04) { // Extended linear address
            extAddr = static_cast<uint16_t>(std::stoi(line.substr(9, 4), nullptr, 16));
        }
    }

    if (!eofSeen) {
        errorOut = "Missing EOF record.";
        return false;
    }

    return true;
}


//////////////////////////////////////////////////////////////////////////////
// Extract the bootloader version numbers from an STM32 Intel HEX file.
//////////////////////////////////////////////////////////////////////////////

// ----- Bootloader signature from linker script -----
static constexpr uint8_t kBootSig[] = {
    0x20, 0x0F, 0xF9, 0xA7, 0x17, 0x7D, 0x4E, 0x99,
    0xDB, 0x53, 0xA2, 0x72, 0xE7, 0xC3, 0xE1, 0xFA
};
static constexpr size_t kBootSigLen = sizeof(kBootSig);

static inline uint8_t hexByte(const std::string& s, size_t pos) {
    return static_cast<uint8_t>(std::stoi(s.substr(pos, 2), nullptr, 16));
}
static inline void rtrim(std::string& s) {
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ' || s.back() == '\t'))
        s.pop_back();
}

// Parse Intel HEX into sparse map
static bool parseIntelHexToMap(const std::string& path, std::map<uint32_t, uint8_t>& mem) {
    std::ifstream f(path);
    if (!f) return false;

    std::string line;
    uint32_t extLinear = 0;
    bool sawEOF = false;
    size_t lineNum = 0;

    while (std::getline(f, line)) {
        ++lineNum;
        rtrim(line);
        if (line.empty()) continue;
        if (line[0] != ':') return false;

        for (size_t i = 1; i < line.size(); ++i) {
            if (!isHexChar(line[i])) return false;
        }
        if (line.size() < 11) return false;

        const uint8_t  byteCount = hexByte(line, 1);
        const uint16_t addr16    = static_cast<uint16_t>(std::stoi(line.substr(3, 4), nullptr, 16));
        const uint8_t  recType   = hexByte(line, 7);

        // expected length calculation
        const size_t expectedLen = 1 + 2 * (byteCount + 5);
        if (line.size() != expectedLen) return false;

        if (!validateLineChecksum(line)) return false;

        if (recType == 0x00) { // Data record
            const uint32_t base = (extLinear << 16) | addr16;
            for (uint8_t i = 0; i < byteCount; ++i) {
                const uint8_t b = hexByte(line, 9 + i * 2);
                mem[base + i] = b;
            }
        } else if (recType == 0x04) { // Extended linear address
            if (byteCount != 2) return false;
            extLinear = static_cast<uint16_t>(std::stoi(line.substr(9, 4), nullptr, 16));
        } else if (recType == 0x01) { // EOF
            sawEOF = true;
            break;
        } else {
            // ignore other record types (02,03,05)
        }
    }

    return sawEOF;
}

// Search for contiguous pattern
static bool findPatternAddress(const std::map<uint32_t, uint8_t>& mem,
                               const uint8_t* pat, size_t patLen,
                               uint32_t& foundAddr) {
    if (mem.size() < patLen) return false;

    for (auto it = mem.begin(); it != mem.end(); ++it) {
        uint32_t addr = it->first;
        if (it->second != pat[0]) continue;

        bool ok = true;
        for (size_t i = 1; i < patLen; ++i) {
            auto jt = mem.find(addr + static_cast<uint32_t>(i));
            if (jt == mem.end() || jt->second != pat[i]) { ok = false; break; }
        }
        if (ok) { foundAddr = addr; return true; }
    }
    return false;
}

/**
 * @brief Extracts the bootloader version numbers from an STM32 Intel HEX file.
 *
 * @param hexPath Path to the Intel HEX file.
 * @param major   Output: major version number (binary value, e.g., 0x06 for "6").
 * @param minor   Output: minor version value (ASCII code, e.g., 0x68 for 'h').
 * @return true if the version was successfully found and extracted; false on any error.
 */
bool extractBootloaderVersionFromHex(const std::string& hexPath,
                                     uint8_t& major,
                                     uint8_t& minor)
{
    std::map<uint32_t, uint8_t> mem;
    if (!parseIntelHexToMap(hexPath, mem)) return false;

    uint32_t sigAddr = 0;
    if (!findPatternAddress(mem, kBootSig, kBootSigLen, sigAddr)) return false;

    const uint32_t versionAddr = sigAddr + static_cast<uint32_t>(kBootSigLen);
    auto itMajor = mem.find(versionAddr);
    auto itMinor = mem.find(versionAddr + 1);
    if (itMajor == mem.end() || itMinor == mem.end()) return false;

    major = itMajor->second;
    minor = itMinor->second;

    // Optional checksum byte (major + minor) at versionAddr + 2
    // Keep non-fatal:
    // auto itChk = mem.find(versionAddr + 2);
    // if (itChk != mem.end() && itChk->second != uint8_t(major + minor)) return false;

    return true;
}

