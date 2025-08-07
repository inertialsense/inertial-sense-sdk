#include "intel_hex_utils.h"

#include <iostream>
#include <sstream>
#include <unordered_set>
#include <iomanip>
#include <cctype>
#include <fstream>
#include <string>
#include <map>
#include <stdexcept>


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

