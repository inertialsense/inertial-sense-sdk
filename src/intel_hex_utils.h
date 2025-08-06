#pragma once

#include <string>
#include <cstddef>   // for size_t

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
