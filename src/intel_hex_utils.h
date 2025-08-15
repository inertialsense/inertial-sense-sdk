#pragma once

#include <string>
#include <cstddef>   // for size_t

/**
 * @brief Check if a file exists.
 * @param filename Path to the file
 * @return true if the file exists, false otherwise
 */
bool fileExists(const std::string& filename);

/**
 * @brief Calculate the number of flash pages used by the given Intel HEX file. 
 * Throws std::runtime_error if the file cannot be read or parsed.
 * 
 * @param hexFilename Path to the .hex file
 * @param flashPageSize Flash page size in bytes (e.g., 2048 for STM32)
 * @return size_t Number of flash pages used
 */
size_t calculateFlashPagesUsed(const std::string& hexFilename, size_t flashPageSize);

/**
 * @brief Validate the contents of an Intel HEX file.
 * 
 * @param hexFilename Path to the .hex file
 * @param errorOut Output parameter for error messages
 * @return true if the file is valid, false otherwise
 */
bool validateHexFile(const std::string& hexFilename, std::string& errorOut);

/**
 * @brief Extracts the bootloader version numbers from an STM32 Intel HEX file.
 *
 * This function scans the HEX file for a known bootloader signature (defined
 * in the linker script) and reads the major and minor version bytes that follow it.
 *
 * @param hexPath Path to the Intel HEX file.
 * @param major   Output: major version number (binary value, e.g., 0x06 for "6").
 * @param minor   Output: minor version value (ASCII code, e.g., 0x68 for 'h').
 * @return true if the version was successfully found and extracted; false on any error.
 */
bool extractBootloaderVersionFromHex(const std::string& hexPath, uint8_t& major, uint8_t& minor);