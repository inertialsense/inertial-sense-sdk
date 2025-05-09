/**
 * @file types.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 7/3/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_CORE_TYPES_H
#define IS_CORE_TYPES_H

#include <inttypes.h>

typedef enum {
    IS_LOG_LEVEL_NONE  = 0,
    IS_LOG_LEVEL_ERROR = 1,
    IS_LOG_LEVEL_WARN  = 2,
    IS_LOG_LEVEL_INFO  = 3,
    IS_LOG_LEVEL_MORE_INFO = 4,
    IS_LOG_LEVEL_DEBUG = 5,
    IS_LOG_LEVEL_MORE_DEBUG = 6,
    IS_LOG_LEVEL_SILLY = 7
} eLogLevel;

typedef enum {
    IS_BL_TYPE_NONE = 0,
    IS_BL_TYPE_SAMBA,
    IS_BL_TYPE_ISB,
    IS_BL_TYPE_APP,
    IS_BL_TYPE_DFU,
} eBootLoaderType;


#endif //IS_CORE_TYPES_H
