#pragma once

#include <ctime>

#include <Wire.h>

namespace RTC
{
/**
 * @brief Get the Teensy3/4 RTC time
 *
 * @param wire Reference to I2C device
 * @param time Reference to get time
 * @return true if getting time succeed, false otherwise
 */
bool getRtcTime(TwoWire &wire, time_t &time);

/**
 * @brief Set the Teensy3/4 time to RTC
 *
 * @param wire Reference to I2C device
 * @param time Time to set
 * @return true if setting time succeed, false otherwise
 */
bool setRtcTime(TwoWire &wire, time_t time);

} // namespace RTC
