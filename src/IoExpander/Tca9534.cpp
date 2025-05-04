/**
 * @file Tca9534.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief TCA9534 Low Voltage 8-Bit I2C and SMBUS Low-Power I/O Expander driver implementation
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "IoExpander/Tca9534.hpp"

#include <stdbool.h>
#include <stdint.h>

#include <Debug.hpp>
#include <Wire.h>

using namespace IoExpander;

namespace
{
    // I2C transport
    constexpr auto &wire = Wire;
    // I2C device address
    constexpr int address = 0x20;
    // Port width in bits
    constexpr size_t portWidth = 8;

    // Default IO config value (all pins are configured as input)
    constexpr uint8_t configValue = 0xFF;
    // Default IO polarity value (all pins' polarity is not inverted)
    constexpr uint8_t polarityValue = 0x00;

    /**
     * @brief Registers list
     */
    enum class Register : uint8_t
    {
        Input,    // Input Port register reflects the incoming logic levels of the pins (input or output)
        Output,   // Output Port register shows the outgoing logic levels of output pins
        Polarity, // Polarity Inversion register allows polarity inversion of input pins
        Config    // Configuration register configures the directions of the I/O pins
    };

    /**
     * @brief Write device register
     *
     * @param[in] regId Register identifier
     * @param[in] value Value to write
     * @return true if writting succeed, false otherwise
     */
    bool writeRegister(Register regId, uint8_t value)
    {
        wire.beginTransmission(address);
        wire.write(static_cast<uint8_t>(regId));
        wire.write(value);
        uint8_t status = wire.endTransmission();
        if (status != 0)
        {
            LOG_ERROR("Write ERROR: %d", status);
            return false;
        }

        return true;
    }

    /**
     * @brief Read device register
     *
     * @param[in] regId Register identifier
     * @param[out] value Reference to read value
     * @return true if reading succeed, false otherwise
     */
    bool readRegister(Register regId, uint8_t &value)
    {
        // Write register address
        wire.beginTransmission(address);
        wire.write(static_cast<uint8_t>(regId));
        uint8_t status = wire.endTransmission();
        if (status != 0)
        {
            LOG_ERROR("Write ERROR: %d", status);
            return false;
        }

        bool result = false;
        // Read data from specified register
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
} // namespace

/**
 * @brief Configure IO port
 *
 * @return true if configuration succeed, false otherwise
 */
bool Tca9534::configure()
{
    bool result = false;
    uint8_t readValue;

    // Configure all IO pins as inputs
    result = writeRegister(Register::Config, configValue);
    if (result == false)
    {
        goto end;
    }

    // Check the state of config register
    result = readRegister(Register::Config, readValue);
    if (result == false || readValue != configValue)
    {
        result = false;
        goto end;
    }

    // Set normal input polarity (not inverted)
    result = writeRegister(Register::Polarity, polarityValue);
    if (result == false)
    {
        goto end;
    }

    // Check the state of input polarity register
    result = readRegister(Register::Polarity, readValue);
    if (result == false || readValue != polarityValue)
    {
        result = false;
        goto end;
    }

    if (result == true)
    {
        LOG_DEBUG("Tca9534 is configured");
    }

end:
    return result;
}

/**
 * @brief Write pin config
 *
 * @param[in] value Value to write
 * @return true if writing succeed, false otherwise
 */
bool Tca9534::writeConfig(uint8_t value)
{
    bool result = writeRegister(Register::Config, value);

    LOG_TRACE("Config -> 0x%02X, %s", value, result ? "OK" : "FAIL");

    return result;
}

/**
 * @brief Read pin config
 *
 * @param[out] value Reference to read value
 * @return true if reading succeed, false otherwise
 */
bool Tca9534::readConfig(uint8_t &value)
{
    bool result = readRegister(Register::Config, value);

    LOG_TRACE("Config <- 0x%02X, %s", value, result ? "OK" : "FAIL");

    return result;
}

/**
 * @brief Write output port
 *
 * @param[in] value Value to write
 * @return true if writing succeed, false otherwise
 */
bool Tca9534::writePort(uint8_t value)
{
    bool result = writeRegister(Register::Output, value);

    LOG_TRACE("Output -> 0x%02X, %s", value, result ? "OK" : "FAIL");

    return result;
}

/**
 * @brief Read input port
 *
 * @param[out] value Reference to read value
 * @return true if reading succeed, false otherwise
 */
bool Tca9534::readPort(uint8_t &value)
{
    bool result = readRegister(Register::Input, value);

    LOG_TRACE("Input <- 0x%02X, %s", value, result ? "OK" : "FAIL");

    return result;
}
