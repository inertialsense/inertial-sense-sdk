#include <fstream>
#include <iostream>
#include "md5.h"
using namespace std;

#define MD5_BUFF_SIZE  512

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
void md5_init(md5Context_t& context) {
    context.count[0] = context.count[1] = 0;

    // Load magic initialization constants
    context.state.dwords[0] = 0x67452301;
    context.state.dwords[1] = 0xefcdab89;
    context.state.dwords[2] = 0x98badcfe;
    context.state.dwords[3] = 0x10325476;
}

// MD5 block update operation
void md5_update(md5Context_t& context, const unsigned char *input, unsigned int inputLen) {
    unsigned int i, index, partLen;

    if (!inputLen)
        return; // don't do ANY processing if there is no data passed.

    // Compute number of bytes mod 64
    index = (unsigned int)((context.count[0] >> 3) & 0x3F);

    // Update number of bits
    if ((context.count[0] += ((uint32_t)inputLen << 3)) < ((uint32_t)inputLen << 3)) {
        context.count[1]++;
    }
    context.count[1] += ((uint32_t)inputLen >> 29);

    partLen = 64 - index;

    // Transform as many times as possible
    if (inputLen >= partLen) {
        memcpy(&context.buffer[index], input, partLen);
        MD5Transform(context.state.dwords, context.buffer);

        for (i = partLen; i + 63 < inputLen; i += 64) {
            MD5Transform(context.state.dwords, &input[i]);
        }

        index = 0;
    } else {
        i = 0;
    }

    // Buffer remaining input
    memcpy(&context.buffer[index], &input[i], inputLen - i);
}

// MD5 finalization
void md5_final(md5Context_t& context, md5hash_t& hash) {
    unsigned char bits[8];
    unsigned int index, padLen;
    static unsigned char PADDING[64] = {}; 
    PADDING[0] = 0x80;

    // Save number of bits
    Encode(bits, context.count, 8);

    // Pad out to 56 mod 64
    index = (unsigned int)((context.count[0] >> 3) & 0x3f);
    padLen = (index < 56) ? (56 - index) : (120 - index);
    md5_update(context, PADDING, padLen);

    // Append length (before padding)
    md5_update(context, bits, 8);

    // Store state in digest
    hash = context.state;
    // Zeroize sensitive information
    // memset(context, 0, sizeof(*context));
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
    memset(x, 0, sizeof(x));
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

/**
 * Compute the MD5 hash for the specified data.
 * @param md5 the md5 hash output
 * @param data_len the number of bytes of the data
 * @param data the data
 */
void md5_hash(md5hash_t& md5, uint32_t data_len, uint8_t* data) 
{
    md5Context_t context;
    md5_init(context);
    md5_update(context, (const unsigned char *)data, data_len);
    md5_final(context, md5);
}

/**
 * Gets the file size and calculated md5sum of the specified file.
 * @param filename the file to validate/fetch details for
 * @param filesize [OUT] the size of the file, as read from the scan
 * @param md5result [OUT] the MD5 checksum calculated for the file
 * @return 0 on success, errno (negative) if error
 */
int md5_file_details(std::istream* is, size_t& filesize, md5hash_t& md5)
{
    if (!is) return -EINVAL;
    if (!is->good()) return -errno;

    md5Context_t context;
    md5_init(context);

    filesize = 0;
    is->seekg(ios_base::beg);
    while (is && (is->tellg() != -1))
    {
        uint8_t buff[MD5_BUFF_SIZE] = {};
        is->read((char *)buff, sizeof(buff));
        int len = (int)is->gcount();

        md5_update(context, (const unsigned char *)buff, (unsigned int)len);

        if (is->eof()) //  || (len != sizeof(buff)))
        {
            filesize += len;
            break;
        }
        else
        {
            int32_t tell = (int32_t)is->tellg();
            if (tell != -1) {
                filesize = tell;
            }
        }
    }

    md5_final(context, md5);

    // reset back to the start of the stream before returning
    if (is->fail())
        is->clear();
    is->seekg(ios_base::beg);
    return 0;
}

/**
 * Gets the file size and calculated md5sum of the specified file.
 * @param filename the file to validate/fetch details for
 * @param filesize [OUT] the size of the file, as read from the scan
 * @param md5result [OUT] the MD5 checksum calculated for the file
 * @return 0 on success, errno (negative) if error
 */
int md5_file_details(const std::string& filename, size_t& filesize, md5hash_t& md5) {
    ifstream file(filename);
    return md5_file_details(&file, filesize, md5);
}


// Converts md5 hexadecimal char array to binary integer 
void md5_from_char_array(md5hash_t& md5, const char hashStr[])
{
    for (int i=0; i<16; i++)
    {
        int j = i*2;
        uint8_t a = hashStr[j];
        uint8_t b = hashStr[j+1];
        md5.bytes[i] = ((a <= '9') ? a - '0' : (a & 0x7) + 9);
        md5.bytes[i] <<= 4;
        md5.bytes[i] |= ((b <= '9') ? b - '0' : (b & 0x7) + 9);
    }
}

// Converts md5 binary integer to char array hexadecimal 
bool md5_to_char_array(const md5hash_t& md5, char hashStr[], int hashStrMaxLen)
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

#ifdef USE_ALTERNATE_MD5_IMPL
/********************** NOTICE **********************************************
 * The following MD5 implementation is incorrect and needs to be removed, but
 * is retained for legacy purposes (as there are some pre-production firmware
 * versions which use it).  We will eventually remove this code, but for now
 * it needs to stay here.  DO NOT USE IT, unless you are very sure about why
 * you need to.  You have been warned.
 ****************************************************************************/

md5hash_t g_md5hash;
/**
 * Initializes the MD5 hash. Don't forget to call hashMd5() afterwards to actually get your hash
 */
void altMD5_reset() {
    // seed the hash
    g_md5hash.dwords[0] = 0x67452301;
    g_md5hash.dwords[1] = 0xefcdab89;
    g_md5hash.dwords[2] = 0x98badcfe;
    g_md5hash.dwords[3] = 0x10325476;
}

/**
 * Adds the specified data into the running MD5 hash
 * @param len the number of bytes to consume into the hash
 * @param data the bytes to consume into the hash
 * @return a static buffer of 16 unsigned bytes which represent the 128 total bits of the MD5 hash
 *
 * TODO: This function uses dynamic memory to allocate memory for the data buffer. Since our implementation
 * will generally be using the fixed size of the session_chunk_size, we can probably do this allocation once and
 * reuse the buffer, instead of allocating and then freeing with each call.  Likewise, we maybe able to
 * define a static buffer of MAX_CHUNK_SIZE and go that route as well.
 */
#define MD5_LEFTROTATE(x, c) (((x) << (c)) | ((x) >> (32 - (c))))
md5hash_t* altMD5_hash(size_t data_len, uint8_t* data) {
    // Message (to prepare)
    uint8_t *msg = NULL;

    // Note: All variables are unsigned 32 bit and wrap modulo 2^32 when calculating

    // r specifies the per-round shift amounts

    static uint32_t r[] = {7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
                           5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20,
                           4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
                           6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};

    // Use binary integer part of the sines of integers (in radians) as constants// Initialize variables:
    static uint32_t k[] = {
            0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee,
            0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
            0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be,
            0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
            0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa,
            0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
            0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed,
            0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
            0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c,
            0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
            0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05,
            0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
            0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039,
            0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
            0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1,
            0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391};


    // Pre-processing: adding a single 1 bit
    //append "1" bit to message
    /* Notice: the input bytes are considered as bits strings,
       where the first bit is the most significant bit of the byte.[37] */

    // Pre-processing: padding with zeros
    //append "0" bit until message length in bit ≡ 448 (mod 512)
    //append length mod (2 pow 64) to message

    // If data has not length return
    if (data_len == 0)
        return &g_md5hash;

    int new_len = (int)(((((data_len + 8) / 64) + 1) * 64) - 8);

    msg = static_cast<uint8_t *>(calloc(new_len + 64, 1)); // also appends "0" bits
    // (we alloc also 64 extra bytes...)
    memcpy(msg, data, data_len);
    msg[data_len] = 128; // write the "1" bit

    uint32_t bits_len = (uint32_t)(8*data_len); // note, we append the len
    memcpy(msg + new_len, &bits_len, 4);           // in bits at the end of the buffer

    // Process the message in successive 512-bit chunks:
    //for each 512-bit chunk of message:
    int offset;
    for(offset=0; offset<new_len; offset += (512/8)) {

        // break chunk into sixteen 32-bit words w[j], 0 ≤ j ≤ 15
        uint32_t *w = (uint32_t *) (msg + offset);

        // Initialize hash value for this chunk:
        uint32_t a = g_md5hash.dwords[0];
        uint32_t b = g_md5hash.dwords[1];
        uint32_t c = g_md5hash.dwords[2];
        uint32_t d = g_md5hash.dwords[3];

        // Main loop:
        uint32_t i;
        for(i = 0; i<64; i++) {
            uint32_t f, g;

            if (i < 16) {
                f = (b & c) | ((~b) & d);
                g = i;
            } else if (i < 32) {
                f = (d & b) | ((~d) & c);
                g = (5*i + 1) % 16;
            } else if (i < 48) {
                f = b ^ c ^ d;
                g = (3*i + 5) % 16;
            } else {
                f = c ^ (b | (~d));
                g = (7*i) % 16;
            }

            uint32_t temp = d;
            d = c;
            c = b;
            b = b + MD5_LEFTROTATE((a + f + k[i] + w[g]), r[i]);
            a = temp;
        }

        // Add this chunk's hash to result so far:

        g_md5hash.dwords[0] += a;
        g_md5hash.dwords[1] += b;
        g_md5hash.dwords[2] += c;
        g_md5hash.dwords[3] += d;
    }

    // cleanup
    free(msg);

    return &g_md5hash;
}

/**
 * updates the passed reference to an array, the current running md5 sum.
 * @param md5sum the reference to an array of uint32_t[4] where the md5 sum will be stored
 */
void altMD5_getHash(md5hash_t &hash) {
    hash = g_md5hash;
}

/**
 * Gets the file size and calculated md5sum of the specified file.
 * @param filename the file to validate/fetch details for
 * @param filesize [OUT] the size of the file, as read from the scan
 * @param md5result [OUT] the MD5 checksum calculated for the file
 * @return 0 on success, errno (negative) if error
 */
int altMD5_file_details(std::istream* is, size_t& filesize, md5hash_t& md5)
{
    if (!is) return -EINVAL;
    if (!is->good()) return -errno;

    altMD5_reset();

    filesize = 0;
    is->seekg(ios_base::beg);
    while (is && (is->tellg() != -1))
    {
        uint8_t buff[MD5_BUFF_SIZE] = {};
        is->read((char *)buff, sizeof(buff));
        int len = (int)(is->gcount());

        altMD5_hash(len, (uint8_t *)buff);

        if (is->eof()) //  || (len != sizeof(buff)))
        {
            filesize += len;
            break;
        }
        else
        {
            int32_t tell = (int32_t)(is->tellg());
            if (tell != -1) {
                filesize = tell;
            }
        }
    }

    altMD5_getHash(md5);

    // reset back to the start of the stream before returning
    if (is->fail())
        is->clear();
    is->seekg(ios_base::beg);

    return 0;
}

#endif // USE_ALTERNATE_MD5_IMPL


// Converts md5 hexadecimal string to binary integer
md5hash_t md5_from_string(string hashStr)
{
    if (hashStr.size() < 32)
        return {};

    md5hash_t md5;
    md5_from_char_array(md5, hashStr.c_str());
    return md5;
}

// Converts md5 binary integer to char string hexadecimal 
string md5_to_string(const md5hash_t& md5hash)
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

#ifndef ARM
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
