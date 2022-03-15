/**
 * @file ISBootloaderLog.c
 * @author Dave Cutting (davidcutting42@gmail.com)
 * @brief Inertial Sense logging functions for bootloaders
 * @version 0.1
 * @date 2022-03-15
 * 
 * @copyright Copyright (c) 2022 Inertial Sense, Inc
 * 
 */

/*
MIT LICENSE

Copyright (c) 2014-2021 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ISBootloaderLog.h"

void uinsLog(
	const is_device_interface_log_level message_level,
	const is_device_context const * context,
	const int error_code,
	const char * error_message
)
{
	char* prefix;

	if (context->interface->log_level >= message_level)
	{
		switch (message_level)
		{
			case IS_LOG_LEVEL_ERROR: prefix = "ERROR"; break;
			case IS_LOG_LEVEL_WARN: prefix = "WARN"; break;
			case IS_LOG_LEVEL_INFO: prefix = "INFO"; break;
			case IS_LOG_LEVEL_DEBUG: prefix = "DEBUG"; break;
			case IS_LOG_LEVEL_SILLY: prefix = "SILLY"; break;
			default: prefix = "";
		}
		printf("%s: %s\n", prefix, error_message);
		if (error_code)
		{
			printf("libusb (%d) %s\n", error_code, libusb_error_name(error_code));
		}
	}

	if (context->error_callback
		&& (message_level == IS_LOG_LEVEL_ERROR || message_level == IS_LOG_LEVEL_WARN))
	{
		context->error_callback(context->interface, context->user_data, error_code, error_message);
	}
}

void uinsLogError(const is_device_context const * context, const int error_code, const char * error_message)
{
	uinsLog(IS_LOG_LEVEL_ERROR, context, error_code, error_message);
}

void uinsLogWarn(const is_device_context const * context, const int error_code, const char * error_message)
{
	uinsLog(IS_LOG_LEVEL_WARN, context, error_code, error_message);
}

void uinsLogDebug(const is_device_context const * context, const char *format, ...)
{
	if (context->interface->log_level >= IS_LOG_LEVEL_DEBUG)
	{
		va_list args;
		va_start(args, format);
		vprintf(format, args);
		va_end(args);
	}
}
