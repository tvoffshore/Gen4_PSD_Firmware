#include "SystemTime.hpp"

#include <assert.h>

#include <Arduino.h>
#include <TimeLib.h>

#include <Debug.hpp>
#include <Rtc.hpp>

namespace SystemTime
{
namespace
{
constexpr uint32_t epochYear = 1970;
constexpr uint8_t monthCount = 12;

TwoWire *pWire = nullptr; // Reference to I2C device

/**
 * @brief Provide time from RTC for synchronization
 *
 * @return RTC time
 */
time_t syncProvider()
{
    if (pWire == nullptr)
    {
        return false;
    }

    time_t time;

    bool result = RTC::getRtcTime(*pWire, time);
    if (result == false)
    {
        LOG_ERROR("Get RTC time failed!");
        time = 0;
    }

    return time;
}

/**
 * @brief Create a timestamp
 *
 * @param tm Time elements structure
 * @param millisSinceSec Milliseconds since second
 * @param string Timestamp string
 */
void createTimestamp(const tmElements_t &tm, size_t millisSinceSec, TimestampString &string)
{
    assert(string);

    snprintf((char *)string, sizeof(TimestampString), "%4d%02d%02dT%02d%02d%02d%03d", tmYearToCalendar(tm.Year),
             tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second, millisSinceSec);
}

/**
 * @brief Return count of day in specified month
 *
 * @param month Specified month
 * @return Count of day
 */
uint8_t monthDayCount(uint8_t month)
{
    uint8_t monthDays[monthCount] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    assert(month < monthCount);

    return monthDays[month];
}

/**
 * @brief Check whether year leap or not
 *
 * @param year Year to check
 * @return True is year is leap, false otherwise
 */
constexpr bool isLeapYear(uint32_t year)
{
    return (!(year % 4) && ((year % 100) || !(year % 400)));
}
} // namespace

/**
 * @brief Initialize system time interface
 */
bool initialize(TwoWire &wire)
{
    LOG_INFO("Initialize system time...");

    pWire = &wire;

    time_t time;
    bool result = getEpochTime(time);
    if (result == true)
    {
        setSyncProvider(syncProvider);

        TimestampString timestamp;
        epochToTimestamp(time, timestamp);
        LOG_INFO("System time initialized: time since epoch %d - %s", time, timestamp);
    }

    return result;
}

/**
 * @brief Get current epoch time (number of seconds that have elapsed since January 1, 1970)
 *
 * @return Epoch time
 */
bool getEpochTime(time_t &time)
{
    if (pWire == nullptr)
    {
        return false;
    }

    // Load RTC time
    bool result = RTC::getRtcTime(*pWire, time);
    if (result == true)
    {
        char timeString[32];
        ctime_r(&time, timeString);
        LOG_INFO("RTC time is %.*s", strlen(timeString) - 1, timeString);
    }
    else
    {
        LOG_ERROR("Get RTC time failed!");
    }

    return result;
}

/**
 * @brief Set new epoch time (number of seconds that have elapsed since January 1, 1970)
 *
 * @param time Epoch time to set
 */
bool setEpochTime(time_t time)
{
    if (pWire == nullptr)
    {
        return false;
    }

    // Update RTC time
    bool result = RTC::setRtcTime(*pWire, time);
    if (result == true)
    {
        char timeString[32];
        ctime_r(&time, timeString);
        LOG_INFO("Set RTC time to %.*s", strlen(timeString) - 1, timeString);
    }
    else
    {
        LOG_ERROR("Set RTC time failed!");
    }

    // Update system time
    setTime(time);

    return result;
}

/**
 * @brief Get current date packed to human-readable string
 *
 * @return String with date
 */
void getStringDate(char *string)
{
    assert(string);

    snprintf((char *)string, sizeof(DateString), "%04d%02d%02d", year(), month(), day());
}

/**
 * @brief Get current time packed to human-readable string
 *
 * @return String with time
 */
void getStringTime(char *string)
{
    assert(string);

    snprintf((char *)string, sizeof(TimeString), "%02d%02d%02d", hour(), minute(), second());
}

/**
 * @brief Set new date from human-readable string
 *
 * @param string String with date
 * @return True if date was set successfully, false otherwise
 */
bool setStringDate(const char *string)
{
    bool result = false;
    tmElements_t tm;
    size_t value = atoi(string);

    LOG_INFO("Set new date: %s", string);

    tm.Day = value % 100;
    value /= 100;
    tm.Month = value % 100;
    value /= 100;
    uint32_t calendarYear = value % 10000;

    if (calendarYear > epochYear && (tm.Month > 0 && tm.Month <= monthCount))
    {
        uint8_t maxDay = monthDayCount(tm.Month - 1);
        if (tm.Month == 2 && isLeapYear(calendarYear))
        {
            // February month and leap year - additional day
            maxDay++;
        }

        if (tm.Day > 0 && tm.Day <= maxDay)
        {
            tm.Year = CalendarYrToTm(calendarYear);
            tm.Hour = hour();
            tm.Minute = minute();
            tm.Second = second();

            result = setEpochTime(makeTime(tm));
        }
    }

    return result;
}

/**
 * @brief Set new time from human-readable string
 *
 * @param string String with time
 * @return True if date was set successfully, false otherwise
 */
bool setStringTime(const char *string)
{
    bool result = false;
    tmElements_t tm;
    size_t value = atoi(string);

    LOG_INFO("Set new time: %s", string);

    tm.Second = value % 100;
    value /= 100;
    tm.Minute = value % 100;
    value /= 100;
    tm.Hour = value % 100;

    if (tm.Hour < 24 && tm.Minute < 60 && tm.Second < 60)
    {
        tm.Day = day();
        tm.Month = month();
        tm.Year = CalendarYrToTm(year());

        result = setEpochTime(makeTime(tm));
    }

    return result;
}

/**
 * @brief Get current datetime and milliseconds packed to human-readable timestamp string
 *
 * @return String with timestamp
 */
void getTimestamp(TimestampString &string)
{
    static uint32_t millisOffset = 0;    // Milliseconds offset within second
    static unsigned long lastMillis = 0; // Last milliseconds reading
    static time_t lastTime = 0;          // Last time reading
    tmElements_t tm;

    // Read curret millis
    uint32_t millisTime = millis();

    // Read current epoch time
    time_t time = now();
    if (time != lastTime)
    {
        // New seconds
        lastTime = time;
        millisOffset = 0;
    }
    else
    {
        millisOffset += (millisTime - lastMillis);
    }

    // Save current millis
    lastMillis = millisTime;

    breakTime(time, tm);

    createTimestamp(tm, millisOffset, string);
}

/**
 * @brief Convert epoch time to human-readable timestamp string
 *
 * @param epochTime Epoch time
 * @param string human-readable string with time
 */
void epochToTimestamp(time_t epochTime, TimestampString &string)
{
    tmElements_t tm;

    breakTime(epochTime, tm);

    createTimestamp(tm, 0, string);
}
} // namespace SystemTime
