#include "logger.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define LOG_MAX_LEN 256

UART_HandleTypeDef* pLoggerUART;

void logMessage(enum LOG_LEVEL logLevel, const char* fmt, ...)
{
    static char str[LOG_MAX_LEN];
    va_list args;
    char* logLevelStr;

    switch(logLevel)
    {
        case LOG_LEVEL_ERROR:
        logLevelStr = "error: ";
        break;

        case LOG_LEVEL_WARNING:
        logLevelStr = "warning: ";
        break;

        default:
        case LOG_LEVEL_INFO:
        logLevelStr = "info: ";
        break;        

        case LOG_LEVEL_DEBUG:
        logLevelStr = "debug: ";
        break;
    }

    strcpy(str, logLevelStr);
    size_t logLevelStrSize = strlen(str);

    va_start(args, fmt);
    vsprintf(str + logLevelStrSize, fmt, args);
    va_end(args);
    strcat(str, "\r\n");

    HAL_UART_Transmit(pLoggerUART, (const uint8_t*)str, strlen(str), 100);
}
