#pragma once

#include <Wire.h>

namespace SystemTime
{
// Date: 4 year + 2 month + 2 day = 8 symbols
typedef char DateString[8 + 1];
// Time: 2 hour + 2 minute + 2 second = 6 symbols
typedef char TimeString[6 + 1];
// Timestamp: 4 year + 2 month + 2 day + 1 "T" + 2 hour + 2 minute + 2 second + 3 milliseconds = 18 symbols
typedef char TimestampString[18 + 1];

/**
 * @brief Initialize system time interface
 */
bool initialize(TwoWire &wire);

/**
 * @brief Get current epoch time (number of seconds that have elapsed since January 1, 1970)
 *
 * @return Epoch time
 */
bool getEpochTime(time_t &time);

/**
 * @brief Set new epoch time (number of seconds that have elapsed since January 1, 1970)
 *
 * @param time Epoch time to set
 */
bool setEpochTime(time_t time);

/**
 * @brief Get current date packed to human-readable string
 *
 * @return String with date
 */
void getStringDate(char *string);

/**
 * @brief Get current time packed to human-readable string
 *
 * @return String with time
 */
void getStringTime(char *string);

/**
 * @brief Set new date from human-readable string
 *
 * @param string String with date
 * @return True if date was set successfully, false otherwise
 */
bool setStringDate(const char *string);

/**
 * @brief Set new time from human-readable string
 *
 * @param string String with time
 * @return True if date was set successfully, false otherwise
 */
bool setStringTime(const char *string);

/**
 * @brief Get current datetime and milliseconds packed to human-readable timestamp string
 *
 * @return String with timestamp
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
