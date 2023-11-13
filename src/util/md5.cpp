#include "md5.h"

/**
 * Initializes the MD5 hash. Don't forget to call hashMd5() afterwards to actually get your hash
 */
void md5_reset(md5hash_t& hash) {
    // seed the hash
    hash.dwords[0] = 0x67452301;
    hash.dwords[1] = 0xefcdab89;
    hash.dwords[2] = 0x98badcfe;
    hash.dwords[3] = 0x10325476;
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
uint8_t* md5_hash(md5hash_t& md5hash, uint16_t data_len, uint8_t* data) {
    // Message (to prepare)
    uint8_t *msg = (uint8_t *)0;

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

    int new_len = ((((data_len + 8) / 64) + 1) * 64) - 8;

    msg = (uint8_t *)(calloc(new_len + 64, 1)); // also appends "0" bits
    // (we alloc also 64 extra bytes...)
    memcpy(msg, data, data_len);
    msg[data_len] = 128; // write the "1" bit

    uint32_t bits_len = 8*data_len; // note, we append the len
    memcpy(msg + new_len, &bits_len, 4);           // in bits at the end of the buffer

    // Process the message in successive 512-bit chunks:
    //for each 512-bit chunk of message:
    int offset;
    for(offset=0; offset<new_len; offset += (512/8)) {

        // break chunk into sixteen 32-bit words w[j], 0 ≤ j ≤ 15
        uint32_t *w = (uint32_t *) (msg + offset);

        // Initialize hash value for this chunk:
        uint32_t a = md5hash.dwords[0];
        uint32_t b = md5hash.dwords[1];
        uint32_t c = md5hash.dwords[2];
        uint32_t d = md5hash.dwords[3];

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
        md5hash.dwords[0] += a;
        md5hash.dwords[1] += b;
        md5hash.dwords[2] += c;
        md5hash.dwords[3] += d;
    }

    // cleanup
    free(msg);

    return (uint8_t *)&md5hash.dwords[0];
}
