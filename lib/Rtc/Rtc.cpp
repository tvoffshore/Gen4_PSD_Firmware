#include "Rtc.hpp"

#include <Debug.hpp>

namespace RTC
{
namespace
{
// I2C device address
constexpr int address = 0x51;

/**
 * @brief Registers list
 */
enum class Register : uint8_t
{
    Control1,     // 0x00
    Control2,     // 0x01
    Offset,       // 0x02
    RAM,          // 0x03
    Seconds,      // 0x04
    Minutes,      // 0x05
    Hours,        // 0x06
    Date,         // 0x07
    Weekday,      // 0x08
    Month,        // 0x09
    Year,         // 0x0A
    AlarmSeconds, // 0x0B
    AlarmMinutes, // 0x0C
    AlarmHours,   // 0x0D
    AlarmDate,    // 0x0E
    AlarmWeekday, // 0x0F
    TimerValue,   // 0x10
    TimerMode     // 0x11
};

struct Control1Register
{
    uint8_t CAP : 1;    // 0 - Must always be written with logic 0
    uint8_t H12_24 : 1; // 0 - 24 hour mode is selected (0 to 23), 1 - 12 hour mode is selected (1 to 12)
    uint8_t CIE : 1;    // 0 - No compensation interrupt, 1 - Compensation interrupt pulses will be generated
    uint8_t SR2 : 2;    // 0 - 00 - No software reset, 11 - Initiate software reset
    uint8_t STOP : 1;   // 0 - RTC clock runs, 1 - RTC clock is stopped
    uint8_t SR1 : 1;    // 0 - No software reset, 1 - Initiate software reset
    uint8_t TEST : 1;   // 0 - Normal mode, 1 - External clock test mode. Do not use
};

/**
 * @brief RTC time stuct
 */
union RtcTime
{
    struct
    {
        uint8_t Second;
        uint8_t Minute;
        uint8_t Hour;
        uint8_t Date;
        uint8_t Weekday;
        uint8_t Month;
        uint8_t Year;
    } Fields;
    uint8_t rawData[7];
};

/**
 * @brief Repack byte from 0 to 99 into bcd mode
 *
 * @param[in] from Number to convert from 00 to 99
 * @return Converted number
 */
uint8_t decToBcd(uint8_t from)
{
    return (from / 10) * 16 + from % 10;
}

/**
 * @brief Convert BCD packed number into standard dec
 *
 * @param[in] from Number to convert
 * @return Converted dumber
 */
uint8_t bcdToDec(uint8_t from)
{
    return (from / 16) * 10 + from % 16;
}

/**
 * @brief Write a single register
 *
 * @param[in] regID Register identifier to write
 * @param[in] value Register value
 * @return true if writting succeed, false otherwise
 */
bool writeRegister(TwoWire &wire, Register regId, uint8_t value)
{
    wire.beginTransmission(address);
    wire.write(static_cast<uint8_t>(regId));
    wire.write(value);
    uint8_t status = wire.endTransmission();

    return (status == 0);
}

/**
 * @brief Write several registers
 *
 * @param[in] regID The start register identifier to write
 * @param[in] data Data to write
 * @param[in] size Size of data
 * @return true if writting succeed, false otherwise
 */
bool writeRegisters(TwoWire &wire, Register regId, const uint8_t *data, size_t size)
{
    wire.beginTransmission(address);
    wire.write(static_cast<uint8_t>(regId));
    for (size_t i = 0; i < size; i++)
    {
        wire.write(data[i]);
    }
    uint8_t status = wire.endTransmission();

    return (status == 0);
}

/**
 * @brief Read a single register
 *
 * @param[in] regId Register identifier to read from
 * @param[out] value Register value
 * @return true if reading succeed, false otherwise
 */
bool readRegister(TwoWire &wire, Register regId, uint8_t &value)
{
    wire.beginTransmission(address);
    wire.write(static_cast<uint8_t>(regId));
    uint8_t status = wire.endTransmission();
    if (status != 0)
    {
        LOG_ERROR("Write ERROR: %d", status);
        return false;
    }

    bool result = false;
    uint8_t readBytes = wire.requestFrom(address, 1);
    if (readBytes == 1)
    {
        int readValue = wire.read();
        if (readValue >= 0)
        {
            value = static_cast<uint8_t>(readValue);
            result = true;
        }
    }

    return result;
}

/**
 * @brief Read several register
 *
 * @param[in] regId The start register identifier to read from
 * @param[out] data Register data
 * @param[in] size Size of data
 * @return true if reading succeed, false otherwise
 */
bool readRegisters(TwoWire &wire, Register regId, uint8_t *data, size_t size)
{
    wire.beginTransmission(address);
    wire.write(static_cast<uint8_t>(regId));
    uint8_t status = wire.endTransmission();
    if (status != 0)
    {
        LOG_ERROR("Write ERROR: %d", status);
        return false;
    }

    bool result = false;
    uint8_t readBytes = wire.requestFrom(address, size);
    if (readBytes == size)
    {
        for (size_t i = 0; i < size; i++)
        {
            int readValue = wire.read();
            if (readValue < 0)
            {
                break;
            }
            data[i] = static_cast<uint8_t>(readValue);
        }
        result = true;
    }

    return result;
}

} // namespace

/**
 * @brief Get the Teensy3/4 RTC time
 *
 * @param time Reference to get time
 * @return true if getting time succeed, false otherwise
 */
bool getRtcTime(TwoWire &wire, time_t &time)
{
    tm tm = {0};
    RtcTime rtcTime;

    bool result = readRegisters(wire, Register::Seconds, rtcTime.rawData, sizeof(rtcTime));
    if (result == true)
    {
        tm.tm_sec = bcdToDec(rtcTime.Fields.Second);
        tm.tm_min = bcdToDec(rtcTime.Fields.Minute);
        tm.tm_hour = bcdToDec(rtcTime.Fields.Hour);
        tm.tm_mday = bcdToDec(rtcTime.Fields.Date);
        tm.tm_mon = bcdToDec(rtcTime.Fields.Month) - 1;   // month noumber in RV-8263-C7 is counting from 1
        tm.tm_year = bcdToDec(rtcTime.Fields.Year) + 100; // tm caclulates year from 1900, RV-8263-C7 from 2000

        time = mktime(&tm);
    }

    return result;
}

/**
 * @brief Set the Teensy3/4 time to RTC
 *
 * @param time Time to set
 * @return true if setting time succeed, false otherwise
 */
bool setRtcTime(TwoWire &wire, time_t time)
{
    RtcTime rtcTime;
    tm tm = {0};
    gmtime_r(&time, &tm);

    rtcTime.Fields.Second = decToBcd(tm.tm_sec);
    rtcTime.Fields.Minute = decToBcd(tm.tm_min);
    rtcTime.Fields.Hour = decToBcd(tm.tm_hour);
    rtcTime.Fields.Date = decToBcd(tm.tm_mday);
    rtcTime.Fields.Month = decToBcd(tm.tm_mon + 1);   // month noumber in RV-8263-C7 is counting from 1
    rtcTime.Fields.Year = decToBcd(tm.tm_year - 100); // tm caclulates year from 1900, RTC from 2000
    rtcTime.Fields.Weekday = decToBcd(tm.tm_wday);

    bool result = writeRegisters(wire, Register::Seconds, rtcTime.rawData, sizeof(rtcTime));

    return result;
}
} // namespace RTC
