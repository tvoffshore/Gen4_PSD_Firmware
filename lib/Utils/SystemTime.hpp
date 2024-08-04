/**
 * @file SystemTime.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief System time API
 * @version 0.1
 * @date 2024-07-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <Wire.h>

namespace SystemTime
{
    // Date: 4 year + 2 month + 2 day = 8 symbols
    typedef char DateString[8 + 1];
    // Time: 2 hour + 2 minute + 2 second = 6 symbols
    typedef char TimeString[6 + 1];
    // Timestamp: 4 year + 2 month + 2 day + 1 "T" + 2 hour + 2 minute + 2 second = 15 symbols
    typedef char TimestampString[15 + 1];

    /**
     * @brief Date and time structure
     */
    struct DateTime
    {
        uint8_t Second;
        uint8_t Minute;
        uint8_t Hour;
        uint8_t Day;
        uint8_t Month;
        uint8_t Year;
    };

    /**
     * @brief Initialize system time interface
     *
     * @param[in] wire Reference to the wire interface object for RTC
     */
    bool initialize(TwoWire &wire);

    /**
     * @brief Get current epoch time (number of seconds that have elapsed since January 1, 1970)
     *
     * @param[out] time Epoch time
     * @return true if operation succeed, false otherwise
     */
    bool getEpochTime(time_t &time);

    /**
     * @brief Set new epoch time (number of seconds that have elapsed since January 1, 1970)
     *
     * @param[in] time Epoch time
     * @return true if operation succeed, false otherwise
     */
    bool setEpochTime(time_t time);

    /**
     * @brief Get current date packed to human-readable string
     *
     * @param[out] string String with date
     */
    void getStringDate(char *string);

    /**
     * @brief Get current time packed to human-readable string
     *
     * @param[out] string String with time
     */
    void getStringTime(char *string);

    /**
     * @brief Set new date from human-readable string
     *
     * @param[in] string String with date
     * @return true if date was set successfully, false otherwise
     */
    bool setStringDate(const char *string);

    /**
     * @brief Set new time from human-readable string
     *
     * @param[in] string String with time
     * @return true if date was set successfully, false otherwise
     */
    bool setStringTime(const char *string);

    /**
     * @brief Get current date and time
     *
     * @param dateTime Current date and time
     */
    void getDateTime(DateTime &dateTime);

    /**
     * @brief Get current date and time packed to human-readable timestamp string
     *
     * @param[in] string String with timestamp
     */
    void getTimestamp(TimestampString &string);

    /**
     * @brief Convert epoch time to human-readable timestamp string
     *
     * @param epochTime Epoch time
     * @param string human-readable string with time
     */
    void epochToTimestamp(time_t epochTime, TimestampString &string);
} // namespace SystemTime
