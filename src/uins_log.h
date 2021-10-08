#ifndef __UINS_LOG_H
#define __UINS_LOG_H

#include <stdio.h>
#include <stdarg.h>

#include <libusb.h>

#include "uins_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void uinsLog(const uins_device_interface_log_level message_level, uins_device_context* context, const int error_code, const char * error_message);
void uinsLogError(uins_device_context* context, const int error_code, const char * error_message);
void uinsLogWarn(uins_device_context* context, const int error_code, const char * error_message);

void uinsLogDebug(const uins_device_interface const * interface, const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif	// __UINS_LOG_H

