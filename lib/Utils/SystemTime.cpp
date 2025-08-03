#include "SystemTime.hpp"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include <Arduino.h>

#include <Log.hpp>
#include <Rtc.hpp>
#include <TimeLib.h>

namespace SystemTime
{
    namespace
    {
        constexpr uint32_t epochYear = 1970;
        constexpr uint8_t monthCount = 12;

        // Interval to sync system time with RTC, seconds
        constexpr time_t rtcSyncIntervalSec = 5 * 60;

        TwoWire *pWire = nullptr; // Reference to I2C device

        /**
         * @brief Get current RTC time (number of seconds that have elapsed since January 1, 1970)
         *
         * @return RTC time if operation succeed, 0 otherwise
         */
        time_t getRtcTime()
        {
            if (pWire == nullptr)
            {
                return 0;
            }

            time_t time = 0;

            // Load RTC time
            bool result = RTC::getTime(*pWire, time);
            if (result == true)
            {
                char timeString[32];
                ctime_r(&time, timeString);
                LOG_DEBUG("Get RTC time: %.*s", strlen(timeString) - 1, timeString);
            }
            else
            {
                LOG_ERROR("Get RTC time failed!");
            }

            return time;
        }

        /**
         * @brief Set new time to RTC
         *
         * @param[in] time Epoch time (number of seconds that have elapsed since January 1, 1970)
         * @return true if operation succeed, false otherwise
         */
        bool setRtcTime(time_t time)
        {
            if (pWire == nullptr)
            {
                return false;
            }

            // Update RTC time
            bool result = RTC::setTime(*pWire, time);
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

            bool result = RTC::getTime(*pWire, time);
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
         * @param string Timestamp string
         */
        void createTimestamp(const tmElements_t &tm, DateTimeString &string)
        {
            assert(string);

            snprintf((char *)string, sizeof(DateTimeString), "%4d%02d%02dT%02d%02d%02d", tmYearToCalendar(tm.Year), tm.Month,
                     tm.Day, tm.Hour, tm.Minute, tm.Second);
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
     *
     * @param[in] wire Reference to the wire interface object for RTC
     */
    void initialize(TwoWire &wire)
    {
        LOG_INFO("Initialize system time");

        pWire = &wire;

        setSyncProvider(getRtcTime);
        setSyncInterval(rtcSyncIntervalSec);
    }

    /**
     * @brief Get current date packed to human-readable string
     *
     * @param[out] string String with date
     */
    void getStringDate(DateString &string)
    {
        snprintf((char *)string, sizeof(DateString), "%04d%02d%02d", year(), month(), day());
    }

    /**
     * @brief Get current time packed to human-readable string
     *
     * @param[out] string String with time
     */
    void getStringTime(TimeString &string)
    {
        snprintf((char *)string, sizeof(TimeString), "%02d%02d%02d", hour(), minute(), second());
    }

    /**
     * @brief Set new date from human-readable string
     *
     * @param[in] string String with date
     * @return true if date was set successfully, false otherwise
     */
    bool setStringDate(const DateString &string)
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

                result = setRtcTime(makeTime(tm));
            }
        }

        return result;
    }

    /**
     * @brief Set new time from human-readable string
     *
     * @param[in] string String with time
     * @return true if date was set successfully, false otherwise
     */
    bool setStringTime(const TimeString &string)
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

            result = setRtcTime(makeTime(tm));
        }

        return result;
    }

    /**
     * @brief Get current epoch time
     *
     * @return Epoch time (number of seconds that have elapsed since January 1, 1970)
     */
    time_t getEpochTime()
    {
        // Read current epoch time
        time_t time = now();

        return time;
    }

    /**
     * @brief Get current date and time structure
     *
     * @param dateTime Date and time structure
     * @return Epoch time (number of seconds that have elapsed since January 1, 1970)
     */
    time_t getDateTime(DateTime &dateTime)
    {
        tmElements_t tm;

        // Read current epoch time
        time_t time = now();
        // Convert to date and time
        breakTime(time, tm);

        dateTime.Second = tm.Second;
        dateTime.Minute = tm.Minute;
        dateTime.Hour = tm.Hour;
        dateTime.Day = tm.Day;
        dateTime.Month = tm.Month;
        dateTime.Year = tm.Year + epochYear;

        return time;
    }

    /**
     * @brief Get current human-readable timestamp string
     *
     * @param[in] string Human-readable timestamp string
     * @return Epoch time (number of seconds that have elapsed since January 1, 1970)
     */
    time_t getTimestamp(DateTimeString &string)
    {
        tmElements_t tm;

        // Read current epoch time
        time_t time = now();
        // Convert to date and time
        breakTime(time, tm);

        createTimestamp(tm, string);

        return time;
    }

    /**
     * @brief Convert epoch time to human-readable timestamp string
     *
     * @param epochTime Epoch time (number of seconds that have elapsed since January 1, 1970)
     * @param string Human-readable timestamp string
     */
    void epochToTimestamp(time_t epochTime, DateTimeString &string)
    {
        tmElements_t tm;

        breakTime(epochTime, tm);

        createTimestamp(tm, string);
    }
} // namespace SystemTime
