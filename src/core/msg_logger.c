#include "msg_logger.h"

eLogLevel log_level = IS_LOG_LEVEL;

#if defined(PLATFORM_IS_WINDOWS) || defined(PLATFORM_IS_LINUX)
FILE* log_file = NULL;
#endif
