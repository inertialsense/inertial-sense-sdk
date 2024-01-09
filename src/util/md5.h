#ifndef INERTIALSENSE_SDK__MD5_H
#define INERTIALSENSE_SDK__MD5_H

#include <stdint.h>
#include <stdlib.h>

#include "ISConstants.h"
#if !defined(PLATFORM_IS_EMBEDDED) 
#include <string.h>
#endif

typedef union {
    uint8_t bytes[16];
    uint16_t words[8];
    uint32_t dwords[4];
    uint64_t ldwords[2];
} md5hash_t;

// MD5 context structure
typedef struct {
    md5hash_t state;
    uint32_t count[2];
    unsigned char buffer[64];
} MD5_CTX_t;

void MD5Init(MD5_CTX_t *context);
void MD5Update(MD5_CTX_t *context, const unsigned char *input, unsigned int inputLen);
void MD5Final(unsigned char digest[16], MD5_CTX_t *context);

void md5_hash(md5hash_t& md5hash, uint32_t data_len, uint8_t* data);
int md5_file_details(const char *filename, size_t& filesize, uint32_t(&md5hash)[4]);

inline bool md5_matches(md5hash_t a, md5hash_t b) { 
    return (
        (a.dwords[0] == b.dwords[0]) && 
        (a.dwords[1] == b.dwords[1]) && 
        (a.dwords[2] == b.dwords[2]) && 
        (a.dwords[3] == b.dwords[3])); 
}

#if !defined(PLATFORM_IS_EMBEDDED) 
std::string md5_string(md5hash_t& md5hash);
void md5_print(md5hash_t& md5hash);
#endif

#endif // INERTIALSENSE_SDK__MD5_H