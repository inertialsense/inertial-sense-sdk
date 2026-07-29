/**
 * @file natsort.h
 * @brief "Natural order" string comparison (strnatcmp()/natcmp() and case-insensitive variants),
 *        adapted from Martin Pool's public-domain strnatcmp.c, plus std::sort/%std::map-ready
 *        comparator functors (nat_cmp, nat_case_cmp).
 *
 * @author Kyle Mallory on 4/27/24.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

/* -*- mode: c; c-file-style: "k&r" -*-

  strnatcmp.c -- Perform 'natural order' comparisons of strings in C.
  Copyright (C) 2000, 2004 by Martin Pool <mbp sourcefrog net>

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#ifndef INERTIALSENSE_SDK__NATSORT_H
#define INERTIALSENSE_SDK__NATSORT_H

#include <string>

namespace utils {

#ifdef __cplusplus
    extern "C" {
#endif

    /* CUSTOMIZATION SECTION
     *
     * You can change this typedef, but must then also change the inline
     * functions in strnatcmp.c */
    typedef char nat_char;   //!< character type used by the strnatcmp()/strnatcasecmp() implementation

    /**
     * Performs a case-sensitive "natural order" comparison of two null-terminated C strings
     * (runs of digits are compared numerically rather than character-by-character, so "img2" is
     * ordered before "img10").
     * @param a the first null-terminated string to compare
     * @param b the second null-terminated string to compare
     * @return <0 if a sorts before b, 0 if they are equivalent, >0 if a sorts after b
     */
    int strnatcmp(nat_char const *a, nat_char const *b);

    /**
     * Case-insensitive variant of strnatcmp().
     * @param a the first null-terminated string to compare
     * @param b the second null-terminated string to compare
     * @return <0 if a sorts before b, 0 if they are equivalent, >0 if a sorts after b
     */
    int strnatcasecmp(nat_char const *a, nat_char const *b);

    /**
     * std::string convenience wrapper around strnatcmp().
     * @param a the first string to compare
     * @param b the second string to compare
     * @return <0 if a sorts before b, 0 if they are equivalent, >0 if a sorts after b
     */
    int natcmp(const std::string& a, const std::string& b);

    /**
     * std::string convenience wrapper around strnatcasecmp().
     * @param a the first string to compare
     * @param b the second string to compare
     * @return <0 if a sorts before b, 0 if they are equivalent, >0 if a sorts after b
     */
    int natcasecmp(const std::string& a, const std::string& b);

    /** Case-sensitive "natural order" std::string comparator functor, e.g. for std::sort()/%std::map<>. */
    struct nat_cmp {
        /**
         * @param s1 the first string to compare
         * @param s2 the second string to compare
         * @return true if s1 sorts before s2 in natural order, otherwise false
         */
        bool operator()(const std::string& s1, const std::string& s2) const {
            return (utils::natcmp(s1, s2) < 0);
        }
    };

    /** Case-insensitive "natural order" std::string comparator functor, e.g. for std::sort()/%std::map<>. */
    struct nat_case_cmp {
        /**
         * @param s1 the first string to compare
         * @param s2 the second string to compare
         * @return true if s1 sorts before s2 in natural order (case-insensitive), otherwise false
         */
        bool operator()(const std::string& s1, const std::string& s2) const {
            return (utils::natcasecmp(s1, s2) < 0);
        }
    };


#ifdef __cplusplus
    }
#endif
} // utils

#endif //INERTIALSENSE_SDK__NATSORT_H
