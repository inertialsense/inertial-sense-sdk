#ifndef INERTIALSENSE_SDK__MD5_H
#define INERTIALSENSE_SDK__MD5_H

#include <stdint.h>
#include <stdlib.h>
#include <istream>

#include "../ISConstants.h"
#include <string>

typedef union {
    uint8_t         bytes[16];
    uint16_t        words[8];
    uint32_t        dwords[4];
    uint64_t        ldwords[2];
} md5hash_t;

// MD5 context structure
typedef struct {
    md5hash_t       state;
    uint32_t        count[2];
    unsigned char   buffer[64];
} md5Context_t;

/**
 * Initializes the MD5 hash. Don't forget to call hashMd5() afterwards to actually get your hash
 */
void md5_init(md5Context_t& context);

/**
 * Adds the specified data into the running MD5 hash
 * @param len the number of bytes to consume into the hash
 * @param data the bytes to consume into the hash
 * @return a static buffer of 16 unsigned bytes which represent the 128 total bits of the MD5 hash
 */
void md5_update(md5Context_t& context, const unsigned char *input, unsigned int inputLen);

/**
 * updates the passed reference to an array, the current running md5 sum.
 * @param md5sum the reference to an array of uint32_t[4] where the md5 sum will be stored
 */
void md5_final(md5Context_t& context, md5hash_t& hash);

// Hash generation functions
void md5_hash(md5hash_t& md5hash, uint32_t data_len, uint8_t* data);
int md5_file_details(std::istream* is, size_t& filesize, md5hash_t& md5);
int md5_file_details(const std::string& filename, size_t& filesize, md5hash_t& md5);

// Hash match check
inline bool md5_matches(const md5hash_t &a, const md5hash_t &b) {
    return (
        (a.dwords[0] == b.dwords[0]) && 
        (a.dwords[1] == b.dwords[1]) && 
        (a.dwords[2] == b.dwords[2]) && 
        (a.dwords[3] == b.dwords[3])); 
}

// Helper functions
void md5_from_char_array(md5hash_t& md5, const char hashStr[]);
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

void altMD5_reset();
md5hash_t* altMD5_hash(size_t data_len, uint8_t* data);
void altMD5_getHash(md5hash_t &hash);
int altMD5_file_details(std::istream* is, size_t& filesize, md5hash_t& md5);
#endif // USE_ALTERNATE_MD5_IMPL

md5hash_t md5_from_string(std::string hashStr);
std::string md5_to_string(const md5hash_t& md5);
std::string md5_to_string_u32(uint32_t hash[4]);
#ifndef ARM
void md5_print(md5hash_t& md5);
#endif

#endif // INERTIALSENSE_SDK__MD5_H