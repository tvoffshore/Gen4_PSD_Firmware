/**
 * @file IoExpander.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief I/O Expander implementation
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "IoExpander/IoExpander.hpp"

#include <Log.hpp>

#include "IoExpander/Tca9534.hpp"

using namespace IoExpander;

namespace
{
    // Pins mask array
    constexpr uint8_t pinMask[] = {
        PinMask::SW2_A0,        // Pin SW2_A0 mask
        PinMask::SW2_A1,        // Pin SW2_A1 mask
        PinMask::SW2_EN,        // Pin SW2_EN mask
        PinMask::RS232_FORCEON, // Pin RS232_FORCEON mask
        PinMask::RS232_FLT,     // Pin RS232_FLT mask
        PinMask::IMU_INT,       // Pin IMU_INT mask
        PinMask::VSENS_PG,      // Pin VSENS_PG mask
        PinMask::DCDC_EN,       // Pin DCDC_EN mask
    };
    static_assert(sizeof(pinMask) / sizeof(*pinMask) == static_cast<size_t>(Pin::Count));

    // Module initialization flag
    bool isInitialized = false;
} // namespace

/**
 * @brief Initialize IMU device
 *
 * @return True if initialization succeed, false otherwise
 */
bool IoExpander::initialize()
{
    LOG_DEBUG("Initialize IO expander...");

    bool result = Tca9534::configure();
    if (result == true)
    {
        LOG_INFO("IO expander initialized");
        isInitialized = true;
    }

    return result;
}

/**
 * @brief Initialize pin with specified config
 *
 * @param[in] pin Pin identifier to initialize
 * @param[in] config Pins configuration
 * @return true if configuration succeeds, false otherwise
 */
bool IoExpander::initializePin(Pin pin, PinConfig config)
{
    uint8_t mask = pinMask[static_cast<size_t>(pin)];

    bool result = initializePins(mask, config);

    return result;
}

/**
 * @brief Initialize pins with specified config
 *
 * @param[in] mask Pin mask to initialize
 * @param[in] config Pins configuration
 * @return true if configuration succeeds, false otherwise
 */
bool IoExpander::initializePins(uint8_t mask, PinConfig config)
{
    if (isInitialized == false)
    {
        LOG_WARNING("IO expander isn't initialized");
        return false;
    }

    // Read current config value
    uint8_t configValue;
    bool result = Tca9534::readConfig(configValue);
    if (result == false)
    {
        return false;
    }

    // Check if the current state of the pins is already the same as desired
    if ((configValue & mask) ^ mask)
    {
        LOG_WARNING("Pins 0x%02X are already initialized as %s", mask,
                    config == PinConfig::Output ? "OUTPUT" : "INPUT");
        return true;
    }

    LOG_DEBUG("Initialize pins 0x%02X as %s", mask, config == PinConfig::Output ? "OUTPUT" : "INPUT");

    if (config == PinConfig::Output)
    {
        configValue &= ~mask;
    }
    else
    {
        configValue |= mask;
    }

    // Write new config value
    result = Tca9534::writeConfig(configValue);

    return result;
}

/**
 * @brief Set specified output pin
 *
 * @param[in] pin Pin identifier to set
 * @return true if setting succeed, false otherwise
 */
bool IoExpander::setPin(Pin pin)
{
    uint8_t mask = pinMask[static_cast<size_t>(pin)];

    bool result = setPins(mask);

    return result;
}

/**
 * @brief Set specified output pins at once
 * @note It is more effective to change several pins in one IO expander transaction
 *
 * @param[in] mask Pin mask to set
 * @return true if setting succeed, false otherwise
 */
bool IoExpander::setPins(uint8_t mask)
{
    if (isInitialized == false)
    {
        LOG_WARNING("IO expander isn't initialized");
        return false;
    }

    // Read current input port value
    uint8_t portValue;
    bool result = Tca9534::readPort(portValue);
    if (result == false)
    {
        return false;
    }

    LOG_TRACE("Set pins 0x%02X", mask);

    // Modify port value
    portValue |= mask;

    // Write new value to output port
    result = Tca9534::writePort(portValue);

    return result;
}

/**
 * @brief Reset specified output pin
 *
 * @param[in] pin Pin identifier to reset
 * @return true if resetting succeed, false otherwise
 */
bool IoExpander::resetPin(Pin pin)
{
    uint8_t mask = pinMask[static_cast<size_t>(pin)];

    bool result = resetPins(mask);

    return result;
}

/**
 * @brief Reset specified output pins
 * @note It is more effective to change several pins in one IO expander transaction
 *
 * @param[in] mask Pin mask to reset
 * @return true if resetting succeed, false otherwise
 */
bool IoExpander::resetPins(uint8_t mask)
{
    if (isInitialized == false)
    {
        LOG_WARNING("IO expander isn't initialized");
        return false;
    }

    // Read current input port value
    uint8_t portValue;
    bool result = Tca9534::readPort(portValue);
    if (result == false)
    {
        return false;
    }

    LOG_TRACE("Reset pins 0x%02X", mask);

    // Modify port value
    portValue &= ~mask;

    // Write new value to output port
    result = Tca9534::writePort(portValue);

    return result;
}

/**
 * @brief Toggle specified output pin
 *
 * @param[in] pin Pin identifier to toggle
 * @return true if toggling succeed, false otherwise
 */
bool IoExpander::togglePin(Pin pin)
{
    uint8_t mask = pinMask[static_cast<size_t>(pin)];

    bool result = togglePins(mask);

    return result;
}

/**
 * @brief Toggle specified output pins
 * @note It is more effective to change several pins in one IO expander transaction
 *
 * @param[in] mask Pin mask to toggle
 * @return true if toggling succeed, false otherwise
 */
bool IoExpander::togglePins(uint8_t mask)
{
    if (isInitialized == false)
    {
        LOG_WARNING("IO expander isn't initialized");
        return false;
    }

    // Read current input port value
    uint8_t portValue;
    bool result = Tca9534::readPort(portValue);
    if (result == false)
    {
        return false;
    }

    LOG_TRACE("Toggle pins 0x%02X", mask);

    // Modify port value
    portValue ^= mask;

    // Write new value to output port
    result = Tca9534::writePort(portValue);

    return result;
}

/**
 * @brief Read specified input pin
 *
 * @param pin Pin identifier to read
 * @param state Reference to pin input state
 * @return true if reading succeed, false otherwise
 */
bool IoExpander::readPin(Pin pin, bool &state)
{
    if (isInitialized == false)
    {
        LOG_WARNING("IO expander isn't initialized");
        return false;
    }

    LOG_TRACE("Read pin %d", pin);

    // Read current input port value
    uint8_t portValue;
    bool result = Tca9534::readPort(portValue);
    if (result == false)
    {
        return false;
    }

    state = (portValue & pinMask[static_cast<size_t>(pin)]);

    LOG_TRACE("Pin %d state is %s", pin, state ? "HIGH" : "LOW");

    return result;
}

/**
 * @brief Perform a self-test
 * @note The specified output pins change their state one by one and the input value of these pins is checked
 * @warning Self-test may set any of the output pins to different states! Enable only if you know what you do!!!
 *
 * @param[in] outputMask Mask of output pins to do self-test with
 * @return true if self-test passed, false otherwise
 */
bool IoExpander::selfTest(uint8_t outputMask)
{
    bool result = true;

    LOG_DEBUG("Run self-test, mask 0x%02X", outputMask);

    uint8_t restoreValue;
    // Save previous value to restore after self-test
    result = Tca9534::readPort(restoreValue);
    if (result == false)
    {
        return false;
    }

    for (size_t pin = 0; pin < static_cast<size_t>(Pin::Count); pin++)
    {
        if (pinMask[pin] & outputMask)
        {
            LOG_TRACE("Test output pin %d", pin);

            result = setPin(static_cast<Pin>(pin));
            if (result == false)
            {
                break;
            }

            bool pinState;
            result = readPin(static_cast<Pin>(pin), pinState);
            if (result == false || pinState != true)
            {
                result = false;
                break;
            }

            result = resetPin(static_cast<Pin>(pin));
            if (result == false)
            {
                break;
            }

            result = readPin(static_cast<Pin>(pin), pinState);
            if (result == false || pinState != false)
            {
                result = false;
                break;
            }
        }
    }

    if (result == true)
    {
        LOG_DEBUG("Self-test passed, restore output port value to 0x%02X", restoreValue);

        // Restore output port value
        result = Tca9534::writePort(restoreValue);
    }

    return result;
}
