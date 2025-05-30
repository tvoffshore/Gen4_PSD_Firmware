/**
 * @file Rtc.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Real time clock module RV-8263-C7 API
 * @version 0.1
 * @date 2025-05-15
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <ctime>
#include <stdbool.h>

#include <Wire.h>

namespace RTC
{
/**
 * @brief Get the RTC time
 *
 * @param wire Reference to I2C device
 * @param time Reference to UNIX time to get
 * @return true if getting time succeed, false otherwise
 */
bool getTime(TwoWire &wire, time_t &time);

/**
 * @brief Set the time to RTC
 *
 * @param wire Reference to I2C device
 * @param time UNIX time to set
 * @return true if setting time succeed, false otherwise
 */
bool setTime(TwoWire &wire, time_t time);
} // namespace RTC
