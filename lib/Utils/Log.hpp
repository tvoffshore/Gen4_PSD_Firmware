/**
 * @file Log.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Log module API
 * @version 0.1
 * @date 2025-05-07
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <stdint.h>
#include <string.h>

#define LOG_LEVEL_NONE (0)
#define LOG_LEVEL_ERROR (1)
#define LOG_LEVEL_WARNING (2)
#define LOG_LEVEL_INFO (3)
#define LOG_LEVEL_DEBUG (4)
#define LOG_LEVEL_TRACE (5)
#define LOG_LEVEL_COUNT (6)

#ifndef LOG_LEVEL
#ifdef DEBUG
#define LOG_LEVEL LOG_LEVEL_DEBUG
#else
#define LOG_LEVEL LOG_LEVEL_NONE
#endif // DEBUG
#endif // LOG_LEVEL

#define LOG_PRINT(loglevel, format, ...) Log::println(loglevel, __FILE__, format, ##__VA_ARGS__)

#if (LOG_LEVEL >= LOG_LEVEL_ERROR)
#define LOG_ERROR(format, ...) LOG_PRINT(LOG_LEVEL_ERROR, format, ##__VA_ARGS__)
#else
#define LOG_ERROR(format, ...)
#endif // LOG_LEVEL >= LOG_LEVEL_ERROR

#if (LOG_LEVEL >= LOG_LEVEL_WARNING)
#define LOG_WARNING(format, ...) LOG_PRINT(LOG_LEVEL_WARNING, format, ##__VA_ARGS__)
#else
#define LOG_WARNING(format, ...)
#endif // LOG_LEVEL >= LOG_LEVEL_WARNING

#if (LOG_LEVEL >= LOG_LEVEL_INFO)
#define LOG_INFO(format, ...) LOG_PRINT(LOG_LEVEL_INFO, format, ##__VA_ARGS__)
#else
#define LOG_INFO(format, ...)
#endif // LOG_LEVEL >= LOG_LEVEL_INFO

#if (LOG_LEVEL >= LOG_LEVEL_DEBUG)
#define LOG_DEBUG(format, ...) LOG_PRINT(LOG_LEVEL_DEBUG, format, ##__VA_ARGS__)
#else
#define LOG_DEBUG(format, ...)
#endif // LOG_LEVEL >= LOG_LEVEL_DEBUG

#if (LOG_LEVEL >= LOG_LEVEL_TRACE)
#define LOG_TRACE(format, ...) LOG_PRINT(LOG_LEVEL_TRACE, format, ##__VA_ARGS__)
#else
#define LOG_TRACE(format, ...)
#endif // LOG_LEVEL >= LOG_LEVEL_TRACE

namespace Log
{
    /**
     * @brief Initialize LOG module
     */
    void initialize(void);

    /**
     * @brief Print log message according formatted string with header and new line characters
     *
     * @param[in] logLevel Level of log message
     * @param[in] file Source of message
     * @param[in] format Formatted string
     */
    void println(uint8_t logLevel, const char *source, const char *format, ...);

    /**
     * @brief Print log message according formatted string without any extra characters
     *
     * @param[in] format Formatted string
     */
    void write(const char *format, ...);

    /**
     * @brief Set the new maximum printed log level
     *
     * @param maxLevel Maximum log level
     */
    void setMaxLevel(uint8_t maxLevel);

    /**
     * @brief Get the current maximum printed log level
     *
     * @return Maximum log level
     */
    uint8_t getMaxLevel();
} // namespace Log
