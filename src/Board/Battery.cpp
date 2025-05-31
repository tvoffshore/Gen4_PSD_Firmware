/**
 * @file Battery.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Battery reader implementation
 * @version 0.1
 * @date 2024-07-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "Board/Battery.hpp"

#include <stddef.h>

#include <Log.hpp>

#include "Board/Board.hpp"

using namespace Battery;

namespace
{
    // Analog control pin active level
    constexpr uint8_t controlActiveLevel = 0;
    // Analog control pin inactive level
    constexpr uint8_t controlInactiveLevel = 1 - controlActiveLevel;

    // Output divider resistance, kOhm
    constexpr uint32_t outputOhm = 100;
    // Full divider resistance, kOhm
    constexpr uint32_t fullOhm = outputOhm + 390;

    // Number of voltage readings to average the result
    constexpr size_t voltageReadNum = 10;

    /**
     * @brief Battery discharge rate profile
     */
    Status dischargeProfile[] = {
        {.voltage = 3400, .level = 100},
        {.voltage = 3350, .level = 90},
        {.voltage = 3320, .level = 80},
        {.voltage = 3300, .level = 70},
        {.voltage = 3270, .level = 60},
        {.voltage = 3260, .level = 50},
        {.voltage = 3250, .level = 40},
        {.voltage = 3220, .level = 30},
        {.voltage = 3200, .level = 20},
        {.voltage = 3000, .level = 10},
        {.voltage = 2500, .level = 0},
    };
    constexpr size_t dischargeProfileSize = sizeof(dischargeProfile) / sizeof(*dischargeProfile);

    // The last read battery status
    Status status = {.voltage = 0, .level = 255};

    // Functions prototypes
    void controlMeasure(bool enable);
    uint32_t measureVoltage();
    uint8_t voltageToLevel(uint16_t voltage);

    /**
     * @brief Control battery measurements
     *
     * @param enable true - enable, false - disable
     */
    void controlMeasure(bool enable)
    {
        if (enable)
        {
            // Detach SPI pins and stop bus
            Board::stopSPI();

            // Configure analog input pin
            pinMode(ADC_IN, INPUT);
            digitalWrite(ADC_CTRL, controlActiveLevel);
        }
        else
        {
            digitalWrite(ADC_CTRL, controlInactiveLevel);

            // Attach SPI pins and start bus
            Board::setupSPI();
        }
    }

    /**
     * @brief Measure battery voltage
     *
     * @return Battery voltage, millivolts
     */
    uint32_t measureVoltage()
    {
        uint32_t inputVoltageMv = analogReadMilliVolts(ADC_IN);

        // Vbat = Vin * fullOhm / outputOhm
        uint32_t battVoltageMv = inputVoltageMv * fullOhm / outputOhm;

        LOG_TRACE("ADC input voltage %u mV, BATT voltage %u mV", inputVoltageMv, battVoltageMv);

        return battVoltageMv;
    }

    /**
     * @brief Convert voltage to battery level
     *
     * @param voltage Battery voltage, millivolts
     * @return Battery level, percents
     */
    uint8_t voltageToLevel(uint16_t voltage)
    {
        const auto &highestPoint = dischargeProfile[0];
        if (voltage >= highestPoint.voltage)
        {
            LOG_DEBUG("Measured voltage above highest point");
            return highestPoint.level;
        }

        const auto &lowestPoint = dischargeProfile[dischargeProfileSize - 1];
        if (voltage < lowestPoint.voltage)
        {
            LOG_DEBUG("Measured voltage below lowest point");
            return lowestPoint.level;
        }

        size_t idx;
        for (idx = 1; idx < dischargeProfileSize - 1; idx++)
        {
            if (voltage >= dischargeProfile[idx].voltage)
            {
                break;
            }
        }

        const auto &belowPoint = dischargeProfile[idx];
        const auto &abovePoint = dischargeProfile[idx - 1];

        // Linear interpolation between below and above points
        uint8_t level = belowPoint.level + (abovePoint.level - belowPoint.level) * (voltage - belowPoint.voltage) /
                                               (abovePoint.voltage - belowPoint.voltage);

        return level;
    }
} // namespace

/**
 * @brief Initialize battery reader
 */
void Battery::initialize()
{
    pinMode(ADC_CTRL, OUTPUT);
    digitalWrite(ADC_CTRL, controlInactiveLevel);
}

/**
 * @brief Read battery status
 *
 * @return Battery status
 */
const Status &Battery::readStatus()
{
    uint32_t voltageSumMv = 0;

    // Enable measurements
    controlMeasure(true);

    for (size_t i = 0; i < voltageReadNum; i++)
    {
        delay(1);
        voltageSumMv += measureVoltage();
    }

    status.voltage = voltageSumMv / voltageReadNum;
    status.level = voltageToLevel(status.voltage);

    LOG_DEBUG("Read battery voltage %u mV, level %u", status.voltage, status.level);

    // Disable measurements
    controlMeasure(false);

    return status;
}

/**
 * @brief Get the last read battery status
 *
 * @return The last battery status
 */
const Status &Battery::lastStatus()
{
    return status;
}
