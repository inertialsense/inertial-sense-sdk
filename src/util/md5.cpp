#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include "md5.h"
using namespace std;


// Constants for MD5Transform routine
#define S11 7
#define S12 12
#define S13 17
#define S14 22
#define S21 5
#define S22 9
#define S23 14
#define S24 20
#define S31 4
#define S32 11
#define S33 16
#define S34 23
#define S41 6
#define S42 10
#define S43 15
#define S44 21

// Function Prototypes
void MD5Transform(uint32_t state[4], const unsigned char block[64]);
void Encode(unsigned char *output, const uint32_t *input, unsigned int len);
void Decode(uint32_t *output, const unsigned char *input, unsigned int len);
void MD5_memcpy(unsigned char *output, const unsigned char *input, unsigned int len);
void MD5_memset(unsigned char *output, int value, unsigned int len);

// F, G, H and I are basic MD5 functions
#define F(x, y, z) (((x) & (y)) | ((~x) & (z)))
#define G(x, y, z) (((x) & (z)) | ((y) & (~z)))
#define H(x, y, z) ((x) ^ (y) ^ (z))
#define I(x, y, z) ((y) ^ ((x) | (~z)))

// ROTATE_LEFT rotates x left n bits
#define ROTATE_LEFT(x, n) (((x) << (n)) | ((x) >> (32-(n))))

// FF, GG, HH, and II transformations
#define FF(a, b, c, d, x, s, ac) { \
 (a) += F ((b), (c), (d)) + (x) + (uint32_t)(ac); \
 (a) = ROTATE_LEFT ((a), (s)); \
 (a) += (b); \
  }
#define GG(a, b, c, d, x, s, ac) { \
 (a) += G ((b), (c), (d)) + (x) + (uint32_t)(ac); \
 (a) = ROTATE_LEFT ((a), (s)); \
 (a) += (b); \
  }
#define HH(a, b, c, d, x, s, ac) { \
 (a) += H ((b), (c), (d)) + (x) + (uint32_t)(ac); \
 (a) = ROTATE_LEFT ((a), (s)); \
 (a) += (b); \
  }
#define II(a, b, c, d, x, s, ac) { \
 (a) += I ((b), (c), (d)) + (x) + (uint32_t)(ac); \
 (a) = ROTATE_LEFT ((a), (s)); \
 (a) += (b); \
  }

// MD5 initialization
void md5_init(md5Context_t *context) {
    context->count[0] = context->count[1] = 0;

    // Load magic initialization constants
    context->state.dwords[0] = 0x67452301;
    context->state.dwords[1] = 0xefcdab89;
    context->state.dwords[2] = 0x98badcfe;
    context->state.dwords[3] = 0x10325476;
}

// MD5 block update operation
void md5_update(md5Context_t *context, const unsigned char *input, unsigned int inputLen) {
    unsigned int i, index, partLen;

    // Compute number of bytes mod 64
    index = (unsigned int)((context->count[0] >> 3) & 0x3F);

    // Update number of bits
    if ((context->count[0] += ((uint32_t)inputLen << 3)) < ((uint32_t)inputLen << 3)) {
        context->count[1]++;
    }
    context->count[1] += ((uint32_t)inputLen >> 29);

    partLen = 64 - index;

    // Transform as many times as possible
    if (inputLen >= partLen) {
        MD5_memcpy((unsigned char *)&context->buffer[index], (unsigned char *)input, partLen);
        MD5Transform(context->state.dwords, context->buffer);

        for (i = partLen; i + 63 < inputLen; i += 64) {
            MD5Transform(context->state.dwords, &input[i]);
        }

        index = 0;
    } else {
        i = 0;
    }

    // Buffer remaining input
    MD5_memcpy((unsigned char *)&context->buffer[index], (unsigned char *)&input[i], inputLen - i);
}

// MD5 finalization
void md5_final(unsigned char digest[16], md5Context_t *context) {
    unsigned char bits[8];
    unsigned int index, padLen;
    unsigned char PADDING[64] = {}; 
    PADDING[0] = 0x80;

    // Save number of bits
    Encode(bits, context->count, 8);

    // Pad out to 56 mod 64
    index = (unsigned int)((context->count[0] >> 3) & 0x3f);
    padLen = (index < 56) ? (56 - index) : (120 - index);
    md5_update(context, PADDING, padLen);

    // Append length (before padding)
    md5_update(context, bits, 8);

    // Store state in digest
    Encode(digest, context->state.dwords, 16);

    // Zeroize sensitive information
    // MD5_memset((unsigned char *)context, 0, sizeof(*context));
}

// MD5 basic transformation
void MD5Transform(uint32_t state[4], const unsigned char block[64]) {
    uint32_t a = state[0], b = state[1], c = state[2], d = state[3], x[16];

    Decode(x, block, 64);

    // Round 1
    FF(a, b, c, d, x[ 0], S11, 0xd76aa478); // 1
    FF(d, a, b, c, x[ 1], S12, 0xe8c7b756); // 2
    FF(c, d, a, b, x[ 2], S13, 0x242070db); // 3
    FF(b, c, d, a, x[ 3], S14, 0xc1bdceee); // 4
    FF(a, b, c, d, x[ 4], S11, 0xf57c0faf); // 5
    FF(d, a, b, c, x[ 5], S12, 0x4787c62a); // 6
    FF(c, d, a, b, x[ 6], S13, 0xa8304613); // 7
    FF(b, c, d, a, x[ 7], S14, 0xfd469501); // 8
    FF(a, b, c, d, x[ 8], S11, 0x698098d8); // 9
    FF(d, a, b, c, x[ 9], S12, 0x8b44f7af); // 10
    FF(c, d, a, b, x[10], S13, 0xffff5bb1); // 11
    FF(b, c, d, a, x[11], S14, 0x895cd7be); // 12
    FF(a, b, c, d, x[12], S11, 0x6b901122); // 13
    FF(d, a, b, c, x[13], S12, 0xfd987193); // 14
    FF(c, d, a, b, x[14], S13, 0xa679438e); // 15
    FF(b, c, d, a, x[15], S14, 0x49b40821); // 16

    // Round 2
    GG(a, b, c, d, x[ 1], S21, 0xf61e2562); // 17
    GG(d, a, b, c, x[ 6], S22, 0xc040b340); // 18
    GG(c, d, a, b, x[11], S23, 0x265e5a51); // 19
    GG(b, c, d, a, x[ 0], S24, 0xe9b6c7aa); // 20
    GG(a, b, c, d, x[ 5], S21, 0xd62f105d); // 21
    GG(d, a, b, c, x[10], S22,  0x2441453); // 22
    GG(c, d, a, b, x[15], S23, 0xd8a1e681); // 23
    GG(b, c, d, a, x[ 4], S24, 0xe7d3fbc8); // 24
    GG(a, b, c, d, x[ 9], S21, 0x21e1cde6); // 25
    GG(d, a, b, c, x[14], S22, 0xc33707d6); // 26
    GG(c, d, a, b, x[ 3], S23, 0xf4d50d87); // 27
    GG(b, c, d, a, x[ 8], S24, 0x455a14ed); // 28
    GG(a, b, c, d, x[13], S21, 0xa9e3e905); // 29
    GG(d, a, b, c, x[ 2], S22, 0xfcefa3f8); // 30
    GG(c, d, a, b, x[ 7], S23, 0x676f02d9); // 31
    GG(b, c, d, a, x[12], S24, 0x8d2a4c8a); // 32

    // Round 3
    HH(a, b, c, d, x[ 5], S31, 0xfffa3942); // 33
    HH(d, a, b, c, x[ 8], S32, 0x8771f681); // 34
    HH(c, d, a, b, x[11], S33, 0x6d9d6122); // 35
    HH(b, c, d, a, x[14], S34, 0xfde5380c); // 36
    HH(a, b, c, d, x[ 1], S31, 0xa4beea44); // 37
    HH(d, a, b, c, x[ 4], S32, 0x4bdecfa9); // 38
    HH(c, d, a, b, x[ 7], S33, 0xf6bb4b60); // 39
    HH(b, c, d, a, x[10], S34, 0xbebfbc70); // 40
    HH(a, b, c, d, x[13], S31, 0x289b7ec6); // 41
    HH(d, a, b, c, x[ 0], S32, 0xeaa127fa); // 42
    HH(c, d, a, b, x[ 3], S33, 0xd4ef3085); // 43
    HH(b, c, d, a, x[ 6], S34,  0x4881d05); // 44
    HH(a, b, c, d, x[ 9], S31, 0xd9d4d039); // 45
    HH(d, a, b, c, x[12], S32, 0xe6db99e5); // 46
    HH(c, d, a, b, x[15], S33, 0x1fa27cf8); // 47
    HH(b, c, d, a, x[ 2], S34, 0xc4ac5665); // 48

    // Round 4
    II(a, b, c, d, x[ 0], S41, 0xf4292244); // 49
    II(d, a, b, c, x[ 7], S42, 0x432aff97); // 50
    II(c, d, a, b, x[14], S43, 0xab9423a7); // 51
    II(b, c, d, a, x[ 5], S44, 0xfc93a039); // 52
    II(a, b, c, d, x[12], S41, 0x655b59c3); // 53
    II(d, a, b, c, x[ 3], S42, 0x8f0ccc92); // 54
    II(c, d, a, b, x[10], S43, 0xffeff47d); // 55
    II(b, c, d, a, x[ 1], S44, 0x85845dd1); // 56
    II(a, b, c, d, x[ 8], S41, 0x6fa87e4f); // 57
    II(d, a, b, c, x[15], S42, 0xfe2ce6e0); // 58
    II(c, d, a, b, x[ 6], S43, 0xa3014314); // 59
    II(b, c, d, a, x[13], S44, 0x4e0811a1); // 60
    II(a, b, c, d, x[ 4], S41, 0xf7537e82); // 61
    II(d, a, b, c, x[11], S42, 0xbd3af235); // 62
    II(c, d, a, b, x[ 2], S43, 0x2ad7d2bb); // 63
    II(b, c, d, a, x[ 9], S44, 0xeb86d391); // 64

    // Update state
    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;

    // Zeroize sensitive information
    MD5_memset((unsigned char *)x, 0, sizeof(x));
}

// Encodes input (uint32_t) into output (unsigned char)
void Encode(unsigned char *output, const uint32_t *input, unsigned int len) {
    unsigned int i, j;

    for (i = 0, j = 0; j < len; i++, j += 4) {
        output[j] = (unsigned char)(input[i] & 0xff);
        output[j+1] = (unsigned char)((input[i] >> 8) & 0xff);
        output[j+2] = (unsigned char)((input[i] >> 16) & 0xff);
        output[j+3] = (unsigned char)((input[i] >> 24) & 0xff);
    }
}

// Decodes input (unsigned char) into output (uint32_t)
void Decode(uint32_t *output, const unsigned char *input, unsigned int len) {
    unsigned int i, j;

    for (i = 0, j = 0; j < len; i++, j += 4) {
        output[i] = ((uint32_t)input[j]) | (((uint32_t)input[j+1]) << 8) |
                    (((uint32_t)input[j+2]) << 16) | (((uint32_t)input[j+3]) << 24);
    }
}

// Copies memory
void MD5_memcpy(unsigned char *output, const unsigned char *input, unsigned int len) {
    unsigned int i;

    for (i = 0; i < len; i++) {
        output[i] = input[i];
    }
}

// Sets memory
void MD5_memset(unsigned char *output, int value, unsigned int len) {
    unsigned int i;

    for (i = 0; i < len; i++) {
        ((char *)output)[i] = (char)value;
    }
}

/**
 * Compute the MD5 hash for the specified data.
 * @param md5 the md5 hash output
 * @param data_len the number of bytes of the data
 * @param data the data
 */
void md5_hash(md5hash_t& md5, uint32_t data_len, uint8_t* data) 
{
    md5Context_t context;
    unsigned char digest[16];
    md5_init(&context);
    md5_update(&context, (const unsigned char *)data, data_len);
    md5_final(digest, &context);

    // Copy output
    md5 = context.state;
}

/**
 * Gets the file size and calculated md5sum of the specified file.
 * @param filename the file to validate/fetch details for
 * @param filesize [OUT] the size of the file, as read from the scan
 * @param md5result [OUT] the MD5 checksum calculated for the file
 * @return 0 on success, errno (negative) if error
 */
int md5_file_details(const char *filename, size_t& filesize, uint32_t md5hash[4]) 
{
    ifstream ifs(filename, ios::binary);
    if (!ifs.good()) return errno;

    md5Context_t context;
    unsigned char digest[16];
    md5_init(&context);

    filesize = 0;
    while (ifs)
    {
        uint8_t buff[64] = {};
        ifs.read((char *)buff, sizeof(buff));
        size_t len = ifs.gcount();

        md5_update(&context, (const unsigned char *)buff, len);

        if (ifs.eof())
        {
            filesize += len;
        }
        else
        {
            filesize = ifs.tellg();
        }
    }

    md5_final(digest, &context);

    // Copy hash
    for (int i=0; i<4; i++)
    {
        md5hash[i] = context.state.dwords[i];
    }

    return 0;
}

// Converts md5 hexadecimal char array to binary integer 
md5hash_t md5_from_char_array(const char hashStr[])
{
    md5hash_t md5 = {};

    for (int i=0; i<16; i++)
    {
        int j = i*2;
        uint8_t a = hashStr[j];
        uint8_t b = hashStr[j+1];
        md5.bytes[i] = ((a <= '9') ? a - '0' : (a & 0x7) + 9);
        md5.bytes[i] <<= 4;
        md5.bytes[i] |= ((b <= '9') ? b - '0' : (b & 0x7) + 9);
    }

    return md5;
}

// Converts md5 binary integer to char array hexadecimal 
bool md5_to_char_array(md5hash_t& md5, char hashStr[], int hashStrMaxLen)
{
    if (hashStrMaxLen <= 32)
    {
        return false;
    }

    for (int i=0; i<16; i++)
    {
        int j = i*2;
        snprintf(&(hashStr[j]), hashStrMaxLen-j, "%02x", md5.bytes[i]);
    }

    return true;
}

#ifndef ARM

// Converts md5 hexadecimal string to binary integer 
md5hash_t md5_from_string(string hashStr)
{
    if (hashStr.size() < 32)
        return {};

    return md5_from_char_array(hashStr.c_str());
}

// Converts md5 binary integer to char string hexadecimal 
string md5_to_string(md5hash_t& md5hash)
{
    char array[33]; // Include extra byte for null termination
    md5_to_char_array(md5hash, array, sizeof(array));

    string str;
    return str.assign(array, 32);
}

// Converts md5 binary integer to char string hexadecimal 
string md5_to_string_u32(uint32_t hash[4]) 
{ 
    return md5_to_string(*(md5hash_t*)hash); 
}

void md5_print(md5hash_t& md5)
{
#if 1
    cout << md5_to_string(md5);
#else
    for (int i=0; i<16; i++)
    {
        printf("%02x", md5.bytes[i]);
    }
#endif
}

#endif  // #ifndef ARM

