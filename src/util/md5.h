#ifndef INERTIALSENSE_SDK__MD5_H
#define INERTIALSENSE_SDK__MD5_H

#include <stdint.h>
#include <stdlib.h>

#include "ISConstants.h"
#ifndef ARM
#include <string>
#endif

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

// These functions must be called as a group in sequence to generate the hash (i.e. md5_init, md5_update, md5_update, ..., md5_final)
void md5_init(md5Context_t *context);
void md5_update(md5Context_t *context, const unsigned char *input, unsigned int inputLen);
void md5_final(unsigned char digest[16], md5Context_t *context);

// Hash generation functions
void md5_hash(md5hash_t& md5hash, uint32_t data_len, uint8_t* data);
int md5_file_details(const char *filename, size_t& filesize, uint32_t md5hash[4]);

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
#ifndef ARM
md5hash_t md5_from_string(std::string hashStr);
std::string md5_to_string(md5hash_t& md5);
std::string md5_to_string_u32(uint32_t hash[4]);
void md5_print(md5hash_t& md5);
#endif

#endif // INERTIALSENSE_SDK__MD5_H
