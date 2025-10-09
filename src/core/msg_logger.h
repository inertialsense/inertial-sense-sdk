/**
 * @file msg_logger.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 4/7/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_CORE__MSG_LOGGER_H
#define IS_CORE__MSG_LOGGER_H

// #define DEBUG_LOGGING
#ifndef DEBUG_LOGGING
    #define debug_message(...)
#else
    #ifndef debug_message
        #define debug_message printf
    #endif
#endif

#endif //IS_CORE__MSG_LOGGER_H
