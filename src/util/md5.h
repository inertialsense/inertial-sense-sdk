/**
 * @file md5.h
 * @brief MD5 message-digest implementation (md5_init()/md5_update()/md5_final(), plus the
 *        md5_hash()/md5_stream_details()/md5_file_details() convenience wrappers), the
 *        md5hash_t/md5Context_t data types, string/char-array conversion helpers, and a
 *        deprecated alternate implementation (altMD5_*) retained only for legacy compatibility.
 *
 * @author Inertial Sense, Inc.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef INERTIALSENSE_SDK__MD5_H
#define INERTIALSENSE_SDK__MD5_H

#include <stdint.h>
#include <stdlib.h>
#include <istream>

#include "ISConstants.h"
#include <string>

/** 128-bit MD5 digest, viewable as bytes, 16-bit words, 32-bit double-words, or 64-bit long-words. */
typedef union {
    uint8_t         bytes[16];     //!< digest as 16 individual bytes
    uint16_t        words[8];      //!< digest as 8 16-bit words
    uint32_t        dwords[4];     //!< digest as 4 32-bit double-words
    uint64_t        ldwords[2];    //!< digest as 2 64-bit long-words
} md5hash_t;

/** Running-state context for an in-progress MD5 computation; passed to md5_init()/md5_update()/md5_final(). */
typedef struct {
    md5hash_t       state;         //!< current 128-bit intermediate hash state
    uint32_t        count[2];      //!< number of bits processed so far, as a 64-bit little-endian pair
    unsigned char   buffer[64];    //!< buffered input bytes not yet forming a complete 512-bit block
} md5Context_t;

/**
 * Initializes an MD5 context for a new computation. Follow with one or more calls to
 * md5_update(), then call md5_final() to retrieve the resulting digest.
 * @param context the MD5 context to initialize
 */
void md5_init(md5Context_t& context);

/**
 * Feeds the specified data into the running MD5 hash held by context. May be called multiple
 * times to hash data incrementally, in which case md5_final() should only be called once, after
 * the last chunk has been fed in.
 * @param context  the MD5 context previously initialized by md5_init()
 * @param input    the bytes to consume into the hash
 * @param inputLen the number of bytes in input
 */
void md5_update(md5Context_t& context, const unsigned char *input, size_t inputLen);

/**
 * Finalizes the MD5 computation held by context and writes the resulting 128-bit digest to hash.
 * @param context the MD5 context previously initialized by md5_init() and fed via md5_update()
 * @param hash    receives the final 128-bit MD5 digest
 */
void md5_final(md5Context_t& context, md5hash_t& hash);

// Hash generation functions
/**
 * Computes the MD5 hash for the specified in-memory buffer.
 * @param md5hash  receives the resulting 128-bit MD5 digest
 * @param data_len the number of bytes in data
 * @param data     the buffer to hash
 */
void md5_hash(md5hash_t& md5hash, uint32_t data_len, uint8_t* data);

/**
 * Computes the size and MD5 checksum of the given input stream. Reads the stream in full, then
 * restores the stream's read position back to the beginning before returning.
 * @param is       the input stream to hash
 * @param filesize [out] receives the number of bytes read from the stream
 * @param md5      [out] receives the resulting 128-bit MD5 digest
 * @return 0 on success, or a negative -errno value on error
 */
int md5_stream_details(std::istream& is, size_t& filesize, md5hash_t& md5);

/**
 * Computes the size and MD5 checksum of the named file.
 * @param filename the path of the file to hash
 * @param filesize [out] receives the size of the file, in bytes
 * @param md5      [out] receives the resulting 128-bit MD5 digest
 * @return 0 on success, or a negative -errno value on error
 */
int md5_file_details(const std::string& filename, size_t& filesize, md5hash_t& md5);

/**
 * Compares two MD5 digests for equality.
 * @param a the first digest to compare
 * @param b the second digest to compare
 * @return true if a and b represent the same 128-bit digest, otherwise false
 */
inline bool md5_matches(const md5hash_t &a, const md5hash_t &b) {
    return (
        (a.dwords[0] == b.dwords[0]) &&
        (a.dwords[1] == b.dwords[1]) &&
        (a.dwords[2] == b.dwords[2]) &&
        (a.dwords[3] == b.dwords[3]));
}

// Helper functions
/**
 * Converts a 32-character MD5 hex-digest string (no null terminator required beyond the 32
 * hex characters consumed) into its binary md5hash_t representation.
 * @param md5    receives the decoded 128-bit digest
 * @param hashStr the hex-digest characters to decode; at least 32 characters are read
 */
void md5_from_char_array(md5hash_t& md5, const char hashStr[]);

/**
 * Converts a binary md5hash_t digest into its 32-character hexadecimal string representation.
 * @param md5          the digest to convert
 * @param hashStr      buffer that receives the hex-digest characters (plus a null terminator)
 * @param hashStrMaxLen the size of hashStr, in bytes; must be greater than 32
 * @return true on success, or false if hashStrMaxLen is too small
 */
bool md5_to_char_array(md5hash_t& md5, char hashStr[], int hashStrMaxLen);

#define USE_ALTERNATE_MD5_IMPL
#ifdef USE_ALTERNATE_MD5_IMPL

/********************** NOTICE **********************************************
 * The following MD5 implementation is incorrect and needs to be removed, but
 * is retained for legacy purposes (as there are some pre-production firmware
 * versions which use it).  We will eventually remove this code, but for now
 * it needs to stay here.  DO NOT USE IT, unless you are very sure about why
 * you need to.  You have been warned.
 ****************************************************************************/

/** Resets the legacy alternate-implementation's global running hash state. Do not use (see NOTICE above). */
void altMD5_reset();

/**
 * Feeds data into the legacy alternate implementation's global running hash and returns the
 * current digest. Do not use (see NOTICE above).
 * @param data_len the number of bytes in data
 * @param data     the bytes to consume into the hash
 * @return a pointer to the global running digest (not thread-safe; do not retain across calls)
 */
md5hash_t* altMD5_hash(size_t data_len, uint8_t* data);

/**
 * Copies the legacy alternate implementation's current global running digest. Do not use (see
 * NOTICE above).
 * @param hash receives a copy of the current 128-bit digest
 */
void altMD5_getHash(md5hash_t &hash);

/**
 * Computes the size and (legacy, incorrect) MD5 checksum of the given input stream, using the
 * alternate implementation. Do not use (see NOTICE above).
 * @param is       the input stream to hash
 * @param filesize [out] receives the number of bytes read from the stream
 * @param md5      [out] receives the resulting digest
 * @return 0 on success, or a negative -errno value on error
 */
int altMD5_file_details(std::istream* is, size_t& filesize, md5hash_t& md5);
#endif // USE_ALTERNATE_MD5_IMPL

/**
 * Parses a 32-character MD5 hex-digest string into its binary md5hash_t representation.
 * @param hashStr the hex-digest string to parse; must be at least 32 characters
 * @return the decoded 128-bit digest, or a zeroed md5hash_t if hashStr is too short
 */
md5hash_t md5_from_string(std::string hashStr);

/**
 * Converts a binary md5hash_t digest into its 32-character hexadecimal string representation.
 * @param md5 the digest to convert
 * @return the 32-character lowercase hex-digest string
 */
std::string md5_to_string(const md5hash_t& md5);

/**
 * Converts an MD5 digest, given as a raw uint32_t[4] array, into its 32-character hexadecimal
 * string representation.
 * @param hash the digest, as 4 uint32_t words (aliased directly to an md5hash_t)
 * @return the 32-character lowercase hex-digest string
 */
std::string md5_to_string_u32(uint32_t hash[4]);
#ifndef ARM
/**
 * Prints the hex-digest string representation of md5 to stdout (no trailing newline).
 * @param md5 the digest to print
 */
void md5_print(md5hash_t& md5);
#endif

#endif // INERTIALSENSE_SDK__MD5_H