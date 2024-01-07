#ifndef INERTIALSENSE_SDK__MD5_H
#define INERTIALSENSE_SDK__MD5_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef union {
    uint8_t bytes[16];
    uint16_t words[8];
    uint32_t dwords[4];
    uint64_t ldwords[2];
} md5hash_t;

void md5_reset(md5hash_t& hash);
uint8_t* md5_hash(md5hash_t& md5hash, uint32_t data_len, uint8_t* data);

inline bool md5_matches(md5hash_t a, md5hash_t b) { return ((a.ldwords[0] == b.ldwords[0]) && (a.ldwords[1] == b.ldwords[1])); }

int md5_file_details(std::string filename, size_t& filesize, uint32_t(&md5hash)[4]);

std::string md5_hash_string(md5hash_t& md5hash);

void md5_print_hash(md5hash_t& md5hash);



// MD5 context structure
typedef struct {
    uint32_t state[4];
    uint32_t count[2];
    unsigned char buffer[64];
} MD5_CTX_t;

void MD5Init(MD5_CTX_t *context);
void MD5Update(MD5_CTX_t *context, const unsigned char *input, unsigned int inputLen);
void MD5Final(unsigned char digest[16], MD5_CTX_t *context);


#define MD5HASH_MATCHES(fp1, fp2)  ((fp1.ldwords[0] == fp2.ldwords[0]) && (fp1.ldwords[1] == fp2.ldwords[1]))

#endif // INERTIALSENSE_SDK__MD5_H