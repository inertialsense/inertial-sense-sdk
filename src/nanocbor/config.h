/*
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * @defgroup    nanocbor_config NanoCBOR configuration header
 * @brief       Provides compile-time configuration for nanocbor
 *
 * @{
 *
 * @file
 *
 * @author      Koen Zandberg <koen@bergzand.net>
 */

#ifndef NANOCBOR_CONFIG_H
#define NANOCBOR_CONFIG_H

#include <stdint.h>

/* MSVC doesn't define __SIZEOF_DOUBLE__ / __SIZEOF_FLOAT__. Without these,
 * nanocbor_fmt_double treats double as equivalent to float and converts
 * via nanocbor_fmt_float, losing precision. Define them explicitly for MSVC. */
#if defined(_MSC_VER) && !defined(__SIZEOF_DOUBLE__)
#define __SIZEOF_DOUBLE__ 8
#endif
#if defined(_MSC_VER) && !defined(__SIZEOF_FLOAT__)
#define __SIZEOF_FLOAT__ 4
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Recursion limit when using @ref nanocbor_skip.
 */
#ifndef NANOCBOR_RECURSION_MAX
#define NANOCBOR_RECURSION_MAX 10
#endif

/**
 * @brief library providing htonll, be64toh or equivalent. Must also provide
 * the reverse operation (ntohll, htobe64 or equivalent)
 *
 * Platform-specific: GCC and Clang use the __builtin_bswap* intrinsics (below),
 * so no byte-order header is needed; endian.h is absent on bare-metal targets
 * and macOS. Fall back to endian.h only for other POSIX compilers.
 */
#if defined(__ZEPHYR__)
#define NANOCBOR_BYTEORDER_HEADER <zephyr/sys/byteorder.h>
#elif defined(__GNUC__)
#define NANOCBOR_BYTEORDER_HEADER <stdint.h>
#elif defined(_MSC_VER)
/* MSVC on Windows — no endian.h, use _byteswap_* intrinsics from stdlib.h */
#define NANOCBOR_BYTEORDER_HEADER <stdlib.h>
#else
#ifndef NANOCBOR_BYTEORDER_HEADER
#define NANOCBOR_BYTEORDER_HEADER "endian.h"
#endif
#endif

/**
 * @brief call providing htonll or be64toh or equivalent functionality
 *
 * must take a uint64_t big endian and return it in host endianess
 */
#ifndef NANOCBOR_BE64TOH_FUNC
#if defined(__ZEPHYR__)
#define NANOCBOR_BE64TOH_FUNC(be) sys_be64_to_cpu(be)
#elif defined(__GNUC__)
#define NANOCBOR_BE64TOH_FUNC(be) __builtin_bswap64(be)
#elif defined(_MSC_VER)
#define NANOCBOR_BE64TOH_FUNC(be) _byteswap_uint64(be)
#else
#define NANOCBOR_BE64TOH_FUNC(be) (be64toh(be))
#endif
#endif

/**
 * @brief call providing htonll or htobe64 or equivalent functionality
 *
 * must take a uint64_t in host endianess and return the big endian value
 */
#ifndef NANOCBOR_HTOBE64_FUNC
#if defined(__ZEPHYR__)
#define NANOCBOR_HTOBE64_FUNC(he) sys_cpu_to_be64(he)
#elif defined(__GNUC__)
#define NANOCBOR_HTOBE64_FUNC(he) __builtin_bswap64(he)
#elif defined(_MSC_VER)
#define NANOCBOR_HTOBE64_FUNC(he) _byteswap_uint64(he)
#else
#define NANOCBOR_HTOBE64_FUNC(he) htobe64(he)
#endif
#endif

/**
 * @brief call providing htonl or htobe32 or equivalent functionality
 *
 * must take a uint32_t in host endianess and return the big endian value
 */
#ifndef NANOCBOR_HTOBE32_FUNC
#if defined(__ZEPHYR__)
#define NANOCBOR_HTOBE32_FUNC(he) sys_cpu_to_be32(he)
#elif defined(__GNUC__)
#define NANOCBOR_HTOBE32_FUNC(he) __builtin_bswap32(he)
#elif defined(_MSC_VER)
#define NANOCBOR_HTOBE32_FUNC(he) _byteswap_ulong(he)
#else
#define NANOCBOR_HTOBE32_FUNC(he) htobe32(he)
#endif
#endif

/**
 * @brief configuration for size_t SIZE_MAX equivalent
 */
#ifndef NANOCBOR_SIZE_SIZET
#if (SIZE_MAX == UINT16_MAX)
#define NANOCBOR_SIZE_SIZET NANOCBOR_SIZE_SHORT
#elif (SIZE_MAX == UINT32_MAX)
#define NANOCBOR_SIZE_SIZET NANOCBOR_SIZE_WORD
#elif (SIZE_MAX == UINT64_MAX)
#define NANOCBOR_SIZE_SIZET NANOCBOR_SIZE_LONG
#else
#error ERROR: unable to determine maximum size of size_t
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif /* NANOCBOR_CONFIG_H */
/** @} */
