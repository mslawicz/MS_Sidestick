#ifndef __LOGGER_H
#define __LOGGER_H

#include "main.h"
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif

enum LOG_LEVEL
{
    LOG_LEVEL_ERROR,    
    LOG_LEVEL_WARNING,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG
};

#define LOG_DEBUG(fmt, ...) logMessage(LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) logMessage(LOG_LEVEL_INFO, fmt, ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...) logMessage(LOG_LEVEL_WARNING, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) logMessage(LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)

extern UART_HandleTypeDef* pLoggerUART;

void logMessage(enum LOG_LEVEL logLevel, const char* fmt, ...);


#ifdef __cplusplus
}
#endif
#endif /* __LOGGER_H__ */