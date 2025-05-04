#pragma once

#include <string.h>

#include <Arduino.h>

#ifndef LOG_LEVEL
#define LOG_LEVEL (LOG_LEVEL_NONE)
#endif

#define LOG_LEVEL_NONE (0)
#define LOG_LEVEL_ERROR (1)
#define LOG_LEVEL_WARNING (2)
#define LOG_LEVEL_INFO (3)
#define LOG_LEVEL_DEBUG (4)
#define LOG_LEVEL_TRACE (5)
#define LOG_LEVEL_COUNT (LOG_LEVEL_TRACE)

#if (LOG_LEVEL == LOG_LEVEL_NONE)

#define LOG_ERROR(format, ...)
#define LOG_WARNING(format, ...)
#define LOG_INFO(format, ...)
#define LOG_DEBUG(format, ...)
#define LOG_TRACE(format, ...)

#define LOG_SET_LEVEL(loglevel)
#define LOG_GET_LEVEL() (LOG_LEVEL)

#else // (LOG_LEVEL == LOG_LEVEL_NONE)

#define LOG_ERROR(format, ...) __LOG_SAVE__(LOG_LEVEL_ERROR, format, ##__VA_ARGS__)
#define LOG_WARNING(format, ...) __LOG_SAVE__(LOG_LEVEL_WARNING, format, ##__VA_ARGS__)
#define LOG_INFO(format, ...) __LOG_SAVE__(LOG_LEVEL_INFO, format, ##__VA_ARGS__)
#define LOG_DEBUG(format, ...) __LOG_SAVE__(LOG_LEVEL_DEBUG, format, ##__VA_ARGS__)
#define LOG_TRACE(format, ...) __LOG_SAVE__(LOG_LEVEL_TRACE, format, ##__VA_ARGS__)

#define LOG_SET_LEVEL(loglevel) (LogsOutput::setLogLevel(logLevel))
#define LOG_GET_LEVEL() (LogsOutput::getLogLevel())

#define __FILE_NAME__ strrchr("\\" __FILE__, '\\') + 1
#define __LOG_SAVE__(loglevel, format, ...)                                                        \
    do                                                                                             \
    {                                                                                              \
        LogsOutput::print(loglevel, __FILE_NAME__, __LINE__, __FUNCTION__, format, ##__VA_ARGS__); \
    } while (0)

#define TAG_ERROR "E"
#define TAG_WARNING "W"
#define TAG_INFO "I"
#define TAG_DEBUG "D"
#define TAG_TRACE "T"

class LogsOutput
{
    constexpr static size_t debugStringLength = 200;

public:
    static void print(uint8_t logLevel, const char *file, int line, const char *func, const char *format, ...)
    {
        const char *tags[LOG_LEVEL_COUNT] = {TAG_ERROR, TAG_WARNING, TAG_INFO, TAG_DEBUG, TAG_TRACE};
        static char debugString[debugStringLength];

        if (logLevel > logLevelMax)
        {
            return;
        }

        va_list arp;

        size_t timeMs = millis();
        const char *tag = tags[logLevel - LOG_LEVEL_ERROR];
        char *module = (char *)file;
        while (char *offset = strchr(module, '/'))
        {
            module = offset + 1;
        }

        snprintf(debugString, sizeof(debugString), "[%6u][%s][%s:%u]: ", timeMs, tag, module, line);

        size_t offset = strlen(debugString);

        va_start(arp, format);
        vsnprintf(&debugString[offset], sizeof(debugString) - offset, format, arp);
        va_end(arp);

        Serial.println(debugString);
    }

    static void setLogLevel(uint8_t logLevel)
    {
        logLevelMax = logLevel;
    }

    static uint8_t getLogLevel()
    {
        return logLevelMax;
    }

private:
    static uint8_t logLevelMax;
};

#endif // (LOG_LEVEL > LOG_LEVEL_NONE)