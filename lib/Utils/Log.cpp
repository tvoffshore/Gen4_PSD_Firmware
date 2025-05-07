/**
 * @file Log.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Log module implementation
 * @version 0.1
 * @date 2025-05-07
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "Log.hpp"

#if (LOG_LEVEL > LOG_LEVEL_NONE)

using namespace Log;

#include <stdint.h>
#include <string.h>

#include <Arduino.h>

#include <Settings.hpp>

#define LOG_STRING_LENGTH (256)

#define COLOR_BLACK "30"
#define COLOR_RED "31"
#define COLOR_GREEN "32"
#define COLOR_BROWN "33"
#define COLOR_BLUE "34"
#define COLOR_PURPLE "35"
#define COLOR_CYAN "36"

#define PLAIN_COLOR(COLOR) "\033[0;" COLOR "m"
#define BOLD_COLOR(COLOR) "\033[1;" COLOR "m"
#define RESET_COLOR "\033[0m"

#define COLOR_ERROR PLAIN_COLOR(COLOR_RED)
#define COLOR_WARNING PLAIN_COLOR(COLOR_BROWN)
#define COLOR_INFO PLAIN_COLOR(COLOR_GREEN)
#define COLOR_DEBUG PLAIN_COLOR(COLOR_CYAN)
#define COLOR_TRACE PLAIN_COLOR(COLOR_PURPLE)

#define TAG_ERROR "E"
#define TAG_WARNING "W"
#define TAG_INFO "I"
#define TAG_DEBUG "D"
#define TAG_TRACE "T"

#define SOURCE_DELIM_CHAR '/'
#define SOURCE_EXT_CHAR '.'
#define LOG_STRING_END RESET_COLOR "\r\n"

namespace
{
    // Default max log level
    constexpr uint8_t maxLevelDefault = LOG_LEVEL;
    // Settings identifier in internal storage
    constexpr auto settingsId = Settings::Id::LogModule;

#pragma pack(push, 1)
    /**
     * @brief Non volatile settings structure for log module
     */
    struct LogSettings
    {
        uint8_t maxLevel; // The max log level
    };
#pragma pack(pop)

    // Settings for log module
    LogSettings settings = {.maxLevel = maxLevelDefault};
} // namespace

/**
 * @brief Initialize LOG module
 */
void Log::initialize()
{
    Settings::read(settingsId, settings);
}

/**
 * @brief Print log message according formatted string with header and new line characters
 *
 * @param[in] logLevel Level of log message
 * @param[in] file Source of message
 * @param[in] format Formatted string
 */
void Log::println(uint8_t logLevel, const char *source, const char *format, ...)
{
    assert(logLevel < LOG_LEVEL_COUNT);
    assert(source);
    assert(format);

    if (logLevel > settings.maxLevel)
    {
        return;
    }

    const char *colors[LOG_LEVEL_COUNT] = {COLOR_ERROR, COLOR_WARNING, COLOR_INFO, COLOR_DEBUG, COLOR_TRACE};
    const char *tags[LOG_LEVEL_COUNT] = {TAG_ERROR, TAG_WARNING, TAG_INFO, TAG_DEBUG, TAG_TRACE};

    const char *color = colors[logLevel - LOG_LEVEL_ERROR];
    const char *tag = tags[logLevel - LOG_LEVEL_ERROR];
    const size_t timestamp = millis();
    const char *delimChar = strrchr(source, SOURCE_DELIM_CHAR);
    const char *extChar = strrchr(source, SOURCE_EXT_CHAR);
    const char *startSourceChar = delimChar ? delimChar + 1 : source;
    const size_t sourceLength = extChar ? extChar - startSourceChar : strlen(startSourceChar);

    char string[LOG_STRING_LENGTH + strlen(LOG_STRING_END)]; // Leave space for string end chars
    size_t length = LOG_STRING_LENGTH;
    // Add header to string
    snprintf(string, length, "%s%s (%d) %.*s: ", color, tag, timestamp, sourceLength, startSourceChar);

    va_list arp;
    va_start(arp, format);
    size_t offset = strlen(string);
    length -= offset;
    // Add message with arguments to string
    vsnprintf(&string[offset], length, format, arp);
    va_end(arp);

    offset = strlen(string);
    length = sizeof(string) - offset;
    // Add string end chars
    snprintf(&string[offset], length, LOG_STRING_END);

    Serial.print(string);
}

/**
 * @brief Print log message according formatted string without any extra characters
 *
 * @param[in] format Formatted string
 */
void Log::write(const char *format, ...)
{
    assert(format);

    char string[LOG_STRING_LENGTH];

    va_list arp;
    va_start(arp, format);
    vsnprintf(string, sizeof(string), format, arp);
    va_end(arp);

    Serial.print(string);
}

/**
 * @brief Set the new maximum printed log level
 *
 * @param maxLevel Maximum log level
 */
void Log::setMaxLevel(uint8_t maxLevel)
{
    if (settings.maxLevel != maxLevel && maxLevel <= LOG_LEVEL)
    {
        settings.maxLevel = maxLevel;
        Settings::update(settingsId, settings);
    }
}

/**
 * @brief Get the current maximum printed log level
 *
 * @return Maximum log level
 */
uint8_t Log::getMaxLevel()
{
    return settings.maxLevel;
}

#else // (LOG_LEVEL > LOG_LEVEL_NONE)

/**
 * @brief Initialize LOG module
 */
void Log::initialize() {}

/**
 * @brief Print log message according formatted string with header and new line characters
 *
 * @param[in] logLevel Level of log message
 * @param[in] file Source of message
 * @param[in] format Formatted string
 */
void Log::println(uint8_t logLevel, const char *source, const char *format, ...)
{
    (void)logLevel;
    (void)source;
    (void)format;
}

/**
 * @brief Print log message according formatted string without any extra characters
 *
 * @param[in] format Formatted string
 */
void Log::write(const char *format, ...)
{
    (void)format;
}

/**
 * @brief Set the new maximum printed log level
 *
 * @param level Maximum log level
 */
void Log::setMaxLevel(uint8_t level)
{
    (void)level;
}

/**
 * @brief Get the current maximum printed log level
 *
 * @return Maximum log level
 */
uint8_t Log::getMaxLevel()
{
    return LOG_LEVEL_NONE;
}

#endif // (LOG_LEVEL > LOG_LEVEL_NONE)
