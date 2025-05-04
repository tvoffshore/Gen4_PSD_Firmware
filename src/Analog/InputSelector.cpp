/**
 * @file InputSelector.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Analog channel 2 input type selector implementation
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "Analog/InputSelector.hpp"

#include <stdint.h>

#include <Debug.hpp>

#include "InternalStorage.hpp"
#include "IoExpander/IoExpander.hpp"

using namespace Analog;

namespace
{
    // Input type representation in a human-readable format
    const char *inputTypeString[] = {"IN2-/4-20mA", "IN2+/0-5V", "OUT_IN2", "GND"};
    static_assert(sizeof(inputTypeString) / sizeof(*inputTypeString) == static_cast<size_t>(InputType::Count));

    // Analog switch logic control input A0 pin
    constexpr auto pinA0 = IoExpander::Pin::SW2_A0;
    // Analog switch logic control input A1 pin
    constexpr auto pinA1 = IoExpander::Pin::SW2_A1;
    // Analog switch enable pin
    constexpr auto pinEn = IoExpander::Pin::SW2_EN;

    // Analog switch logic control input A0 pin mask
    constexpr auto pinMaskA0 = IoExpander::PinMask::SW2_A0;
    // Analog switch logic control input A1 pin mask
    constexpr auto pinMaskA1 = IoExpander::PinMask::SW2_A1;
    // Analog switch enable pin mask
    constexpr auto pinMaskEn = IoExpander::PinMask::SW2_EN;

    // Default gain selection
    constexpr auto inputTypeDefault = InputType::In2Out;
    // Settings identifier in internal storage
    constexpr auto settingsId = SettingsModules::InputSelector;

#pragma pack(push, 1)
    /**
     * @brief Non volatile settings structure for input selector
     */
    struct Settings
    {
        InputType inputType; // The selected input
    };
#pragma pack(pop)

    // Settings for input selector
    Settings _settings = {.inputType = inputTypeDefault};

    // Module initialization flag
    bool _isInitialized = false;
} // namespace

/**
 * @brief Initialize module
 *
 * @return true if initialization succeeds, false otherwise
 */
bool InputSelector::initialize()
{
    LOG_DEBUG("Initialize input type selector...");

    InternalStorage::readSettings(settingsId, _settings);

    // Initialize A0, A1 and EN pins as output
    bool result = IoExpander::initializePins(pinMaskA0 | pinMaskA1 | pinMaskEn, IoExpander::PinConfig::Output);
    if (result == true)
    {
        // Perform a self-test of output pins
        result = IoExpander::selfTest(pinMaskA0 | pinMaskA1 | pinMaskEn);
    }

    if (result == false)
    {
        LOG_ERROR("IO expander pins initialization fail!");
        return false;
    }

    // Enable analog switch
    result = IoExpander::setPin(pinEn);
    if (result == false)
    {
        LOG_ERROR("IO expander pin set fail!");
        return false;
    }

    _isInitialized = true;

    result = setInputType(_settings.inputType);
    if (result == false)
    {
        LOG_ERROR("Initial input type set fail!");
        return false;
    }

    LOG_INFO("Input type selector initialized, input type %s",
             inputTypeString[static_cast<size_t>(_settings.inputType)]);

    return result;
}

/**
 * @brief Get the current input type
 *
 * @return The current input type option
 */
InputType InputSelector::inputType()
{
    return _settings.inputType;
}

/**
 * @brief Set new input type
 *
 * @param[in] inputType New input type option
 * @return true if setting succeeds, false otherwise
 */
bool InputSelector::setInputType(InputType inputType)
{
    if (_isInitialized == false)
    {
        LOG_ERROR("InputSelector isn't initialized!");
        return false;
    }

    if (inputType >= InputType::Count)
    {
        LOG_WARNING("Unknown input type option %d, force set to default %d", inputType, inputTypeDefault);
        inputType = inputTypeDefault;
    }

    bool result = false;
    LOG_DEBUG("Setup input type %s", inputTypeString[static_cast<size_t>(inputType)]);

    switch (inputType)
    {
    case InputType::In2Neg4_20mA:
        // A0 | 0
        // A1 | 0
        result = IoExpander::resetPins(pinMaskA0 | pinMaskA1);
        break;
    case InputType::In2Pos0_5V:
        // A0 | 1
        // A1 | 0
        result = IoExpander::resetPin(pinA1);
        if (result == true)
        {
            result = IoExpander::setPin(pinA0);
        }
        break;
    case InputType::In2Out:
        // A0 | 0
        // A1 | 1
        result = IoExpander::resetPin(pinA0);
        if (result == true)
        {
            result = IoExpander::setPin(pinA1);
        }
        break;
    case InputType::Ground:
        // A0 | 1
        // A1 | 1
        result = IoExpander::setPins(pinMaskA0 | pinMaskA1);
        break;
    }

    if (result == false)
    {
        LOG_ERROR("IO expander pins set fail!");
    }

    if (_settings.inputType != inputType)
    {
        _settings.inputType = inputType;
        InternalStorage::updateSettings(settingsId, _settings);
    }

    return result;
}
