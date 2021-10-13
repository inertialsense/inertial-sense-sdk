#include "uins_log.h"

void uinsLog(
	const uins_device_interface_log_level message_level,
	const uins_device_context const * context,
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

void uinsLogError(const uins_device_context const * context, const int error_code, const char * error_message)
{
	uinsLog(IS_LOG_LEVEL_ERROR, context, error_code, error_message);
}

void uinsLogWarn(const uins_device_context const * context, const int error_code, const char * error_message)
{
	uinsLog(IS_LOG_LEVEL_WARN, context, error_code, error_message);
}

void uinsLogDebug(const uins_device_context const * context, const char *format, ...)
{
	if (context->interface->log_level >= IS_LOG_LEVEL_DEBUG)
	{
		va_list args;
		va_start(args, format);
		vprintf(format, args);
		va_end(args);
	}
}
