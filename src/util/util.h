/**
 * @file util.h 
 * @brief General Utility functions. Most of these functions should be static, as we don't want/need a "Util" instance running around.
 *
 * @author Kyle Mallory on 3/14/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_GPX_UTIL_H
#define IS_GPX_UTIL_H

#include <string>

namespace utils {

    std::string getCurrentTimestamp();

};


#endif //IS_GPX_UTIL_H
