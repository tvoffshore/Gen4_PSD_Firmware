/**
 * @file VddController.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Analog sensor voltage controller implementation
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "Analog/VddController.hpp"

#include <stdbool.h>
#include <stdint.h>

#include <Log.hpp>
#include <Settings.hpp>

#include "IoExpander/IoExpander.hpp"
#include "Pot/Pot.hpp"

using namespace Analog;

namespace
{
    // Sensor voltage pin (ADC_1_CH5)
    constexpr auto pinVddSens = GPIO_NUM_6;

    // The minimum acceptable voltage, Volt
    constexpr float voltageMin = 9.2;
    // The maximum acceptable voltage, Volt
    constexpr float voltageMax = 24.0;
    // The minimum acceptable voltage, Volt
    constexpr float voltageDefault = 10;

    // Output divider resistance, kOhm
    constexpr uint32_t outputOhm = 20;
    // Full divider resistance, kOhm
    constexpr uint32_t fullOhm = outputOhm + 130;

    // The threshold for error between target and measured voltage allowed, Volt
    constexpr float voltageStableThreshold = 0.1;
    // The threshold for voltage error at which voltage is considered accurate, Volt
    constexpr float voltageErrorThreshold = 0.01;
    // The Maximum number of POT resistance consecutive adjusting
    constexpr size_t potAdjustCountMax = 10;
    // The threshold for number of consecutive voltage stable reading to finish adjusting
    constexpr size_t stableCountThreshold = 3;
    // Delay steps to wait for voltage level stabilization
    constexpr size_t stableDelayStepsMs[] = {10, 10, 100, 10, 200, 10, 500, 10,
                                             1000, 100, 2000, 100, 5000, 100, 10000, 100};
    // The maximum delay to wait for voltage level stabilization
    constexpr size_t stableDelayMaxMs = 30000;

    // Settings identifier in internal storage
    constexpr auto settingsId = Settings::Id::VddController;

#pragma pack(push, 1)
    /**
     * @brief Non volatile settings structure for voltage controller
     */
    struct VddSettings
    {
        float targetVoltage; // The target voltage to apply to ADC sensor, Volts
    };
#pragma pack(pop)

    // Settings for voltage controller
    VddSettings settings = {.targetVoltage = voltageDefault};

    // Module initialization flag
    bool isInitialized = false;

    /**
     * @brief Convert DCDC voltage to POT resistance that is needed to generate this voltage
     *
     * @param voltage Voltage level, Volt
     * @return Resistavce value, Ohm
     */
    constexpr uint32_t voltageToResistance(float voltage)
    {
        // Vout = ((1MOhm / (22kOhm + R)) + 1) * 1V
        return ((1000000 / (voltage - 1)) - 22000);
    }

    /**
     * @brief Enable DCDC
     *
     * @return true if enabling succeeds, false otherwise
     */
    bool enableDCDC()
    {
        // Enable DCDC
        bool result = IoExpander::setPin(IoExpander::Pin::DCDC_EN);
        if (result == true)
        {
            LOG_INFO("DCDC is enabled");
        }

        return result;
    }

    /**
     * @brief Read actual voltage level on sensors
     *
     * @return Actual voltage level, Volts
     */
    float readVoltage()
    {
        uint32_t inputVoltageMv = analogReadMilliVolts(pinVddSens);

        // Vsens = Vadc * fullOhm / outputOhm
        float sensorVoltageV = static_cast<float>(inputVoltageMv) * fullOhm / outputOhm / 1000;

        LOG_TRACE("ADC input voltage = %u mV, SENSOR voltage = %.2f V", inputVoltageMv, sensorVoltageV);

        return sensorVoltageV;
    }

    /**
     * @brief Wait until voltage is stable for sertain time
     *
     * @return true if voltage is stable, false otherwise
     */
    bool waitStableVoltage()
    {
        bool result = false;
        size_t stableCount = 0;
        size_t delayStepInd = 0;
        size_t delayTotalMs = 0;
        float currentReading = readVoltage();

        // Continue while max delay isn't reached or readings are stable
        while (delayTotalMs < stableDelayMaxMs || stableCount != 0)
        {
            size_t delayMs = stableDelayStepsMs[delayStepInd];
            LOG_TRACE("Delay %dms", delayMs);
            delay(delayMs);
            delayTotalMs += delayMs;
            LOG_TRACE("Total delay is %dms", delayTotalMs);

            float newReading = readVoltage();
            // Calculate the new delta value between readings
            float deltaReading = newReading - currentReading;
            if (abs(deltaReading) < voltageStableThreshold)
            {
                stableCount++;
                LOG_TRACE("Stable #%d, delta is %.2fV", stableCount, deltaReading);
                if (stableCount == stableCountThreshold)
                {
                    result = true;
                    break;
                }
                delayStepInd = 0;
            }
            else
            {
                delayStepInd++;
                LOG_TRACE("Delay step #%d, delta is %.2fV", delayStepInd, deltaReading);
                if (delayStepInd == sizeof(stableDelayStepsMs) / sizeof(*stableDelayStepsMs))
                {
                    result = false;
                    break;
                }
                stableCount = 0;
            }
            // Update current reading value
            currentReading = newReading;
        }

        return result;
    }

    /**
     * @brief Change resistance by one step in specified direction and wait voltage stabilization
     *
     * @param[in] direction true - increase resistance, false - decrease
     * @return true if resistance is changed and voltage is stable, false otherwise
     */
    bool changeResistance(bool direction)
    {
        bool result = direction ? POT::increaseResistance() : POT::decreaseResistance();
        if (result == false)
        {
            LOG_ERROR("POT adjusting error!");
            return false;
        }

        // Wait new stable voltage level
        result = waitStableVoltage();
        if (result == false)
        {
            LOG_ERROR("Stable voltage waiting fail!");
        }

        return result;
    }

    /**
     * @brief Set Pot resistance to calculated value according to target voltage
     *
     * @param[in] targetVoltage Target voltage level, Volts
     * @return true if calculated value is set, false otherwise
     */
    bool setVoltage(float targetVoltage)
    {
        // Convert target voltage to initial POT resistance
        uint32_t resistance = voltageToResistance(targetVoltage);

        LOG_DEBUG("Target voltage %.1fV, POT resistance %dOhm", targetVoltage, resistance);

        // Setup initial POT resistance
        bool result = POT::setResistance(resistance);
        if (result == true)
        {
            result = POT::getResistance(resistance);
        }

        if (result == true)
        {
            LOG_DEBUG("POT resistance is set to %dOhm", resistance);
        }

        return result;
    }

    /**
     * @brief Adjust POT resistance according to measured voltage to reach target voltage
     *
     * @param[in] targetVoltage Target voltage level, Volts
     * @return true if target voltage is reached, false otherwise
     */
    bool adjustVoltage(float targetVoltage)
    {
        size_t adjustCount = 0;

        // Wait for initial stable voltage level
        bool result = waitStableVoltage();
        if (result == false)
        {
            LOG_ERROR("Initial stable voltage waiting fail!");
            return false;
        }

        // Calculate initial voltage error value
        float error = readVoltage() - targetVoltage;
        while (abs(error) > voltageErrorThreshold)
        {
            LOG_TRACE("Adjust #%d, voltage error %.2fV", adjustCount, error);
            if (adjustCount > potAdjustCountMax)
            {
                LOG_ERROR("Maximum POT adjust count is reached!");
                break;
            }

            // Increase resistance to decrease voltage (error is positive) and vice versa
            bool changeDirection = error > 0;
            result = changeResistance(changeDirection);
            if (result == false)
            {
                LOG_ERROR("Change POT resistance fail!");
                break;
            }

            // Calculate new voltage error value
            float errorNew = readVoltage() - targetVoltage;
            if (abs(errorNew) > abs(error))
            {
                LOG_TRACE("Previous voltage error %.2f was better than new %.2f", error, errorNew);
                // Change resistance one step back because previous voltage was closer
                result = changeResistance(!changeDirection);
                if (result == false)
                {
                    LOG_ERROR("Change POT resistance fail!");
                }
                // Update voltage error value
                error = readVoltage() - targetVoltage;
                break;
            }

            // Update voltage error value
            error = errorNew;
            // Increment total count of adjusting
            adjustCount++;
        }

        LOG_DEBUG("Target voltage %.1f %s reached at adjust #%d with error %.2f", targetVoltage,
                  result ? "was" : "wasn't", adjustCount, error);

        return result;
    }
} // namespace

/**
 * @brief Initialize the voltage controller and its components
 * @note IO expander device should be initialized before this
 *
 * @return true if initialization succeeds, false otherwise
 */
bool VddController::initialize()
{
    LOG_DEBUG("Initialize voltage controller...");

    Settings::read(settingsId, settings);

    setVoltage(settings.targetVoltage);

    pinMode(pinVddSens, INPUT);

    bool result = IoExpander::initializePin(IoExpander::Pin::DCDC_EN, IoExpander::PinConfig::Output);
    if (result == false)
    {
        LOG_ERROR("IO expander pin initialization fail!");
        return false;
    }

    // Disable DCDC first
    result = cutOffVoltage();
    if (result == false)
    {
        LOG_ERROR("Initial sensor voltage disabling fail!");
        return false;
    }

    // Initialize digital POT with maximum resistance and W and B terminals connected
    result = POT::initialize();
    if (result == false)
    {
        LOG_ERROR("POT initialization failed!");
        return false;
    }

#if 0
    result = IoExpander::selfTest(IoExpander::PinMask::DCDC_EN);
    if (result == false)
    {
        LOG_ERROR("IO expander pin self-test failed!");
        return false;
    }
#endif

    isInitialized = true;

    LOG_INFO("Voltage controller initialized");

    return isInitialized;
}

/**
 * @brief Cut off voltage from sensor
 * @note Disable DCDC
 *
 * @return true if disabling succeeds, false otherwise
 */
bool VddController::cutOffVoltage()
{
    // Disable DCDC
    bool result = IoExpander::resetPin(IoExpander::Pin::DCDC_EN);
    if (result == true)
    {
        LOG_INFO("DCDC is disabled");
    }

    return result;
}

/**
 * @brief Apply the target voltage level to sensor
 * @note Enable DCDC if it is needed
 *
 * @return true if applying succeeds, false otherwise
 */
bool VddController::applyVoltage()
{
    bool result = false;

    if (isInitialized == false)
    {
        LOG_WARNING("VddController isn't initialized yet");
        goto end;
    }

    // Disable DCDC first
    result = cutOffVoltage();
    if (result == false)
    {
        LOG_ERROR("Disable DCDC fail!");
        goto end;
    }

    result = setVoltage(settings.targetVoltage);
    if (result == false)
    {
        LOG_ERROR("Set POT resistance fail!");
        goto end;
    }

    // Enable DCDC back
    result = enableDCDC();
    if (result == false)
    {
        LOG_ERROR("Enable DCDC fail!");
        goto end;
    }

    // Adjust POT resistance according to measured voltage
    result = adjustVoltage(settings.targetVoltage);
    if (result == false)
    {
        LOG_ERROR("Adjust POT resistance fail!");
        goto end;
    }

end:
    return result;
}

/**
 * @brief Measure the actual voltage level applied to sensor
 *
 * @return Actual voltage, Volt
 */
float VddController::measureVoltage()
{
    // Wait for stable voltage level
    bool result = waitStableVoltage();
    if (result == false)
    {
        LOG_ERROR("Stable voltage waiting fail!");
    }

    float value = readVoltage();

    LOG_DEBUG("Measured voltage is %.2fV", value);

    return value;
}

/**
 * @brief Get target voltage level
 *
 * @return Target voltage, Volt
 */
float VddController::targetVoltage()
{
    return settings.targetVoltage;
}

/**
 * @brief Set new target voltage level
 *
 * @param voltage New target voltage, Volt
 */
void VddController::setTargetVoltage(float voltage)
{
    if (voltage < voltageMin)
    {
        voltage = voltageMin;
    }
    else if (voltage > voltageMax)
    {
        voltage = voltageMax;
    }

    LOG_DEBUG("Set sensor target voltage to %.1fV", voltage);

    if (settings.targetVoltage != voltage)
    {
        settings.targetVoltage = voltage;
        Settings::update(settingsId, settings);
    }
}
