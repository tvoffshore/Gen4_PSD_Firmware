/**
 * @file Mcp45hv.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief 7/8-Bit Single, +36V (±18V) Digital POT driver implementation
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "Pot/Mcp45hv.hpp"

#include <stdbool.h>
#include <stdint.h>

#include <Wire.h>

#include <Log.hpp>

using namespace POT;

namespace
{
    // I2C transport
    constexpr auto &wire = Wire;
    // I2C device address
    constexpr int address = 0x3C;

    // Wiper position register address (operations: write, read, increment, decrement)
    constexpr uint8_t wiperAddress = 0x00;
    // Control register address (operations: write, read)
    constexpr uint8_t controlAddress = 0x04;

    struct ControlRegister
    {
        uint8_t R0B : 1;
        uint8_t R0W : 1;
        uint8_t R0A : 1;
        uint8_t R0HW : 1;
        uint8_t reserved : 4;
    };

    /**
     * @brief Registers list
     */
    enum class Register : uint8_t
    {
        Wiper,   // Wiper position register
        Control, // Control register
    };

    /**
     * @brief Write data operation
     *
     * @param[in] regAddr Register address
     * @param[in] value Value to write
     * @return true if writting succeed, false otherwise
     */
    bool writeData(uint8_t regAddr, uint8_t value)
    {
        // Write data action
        uint8_t command = 0x00;
        command |= ((regAddr & 0x0F) << 4);

        wire.beginTransmission(address);
        wire.write(static_cast<uint8_t>(command));
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
     * @brief Read data operation
     *
     * @param[in] regAddr Register address
     * @param[out] value Read value
     * @return true if reading succeed, false otherwise
     */
    bool readData(uint8_t regAddr, uint8_t &value)
    {
        // Read data action
        uint8_t command = 0x03 << 2;
        command |= ((regAddr & 0x0F) << 4);

        // Write register address
        wire.beginTransmission(address);
        wire.write(static_cast<uint8_t>(command));
        uint8_t status = wire.endTransmission();
        if (status != 0)
        {
            LOG_ERROR("Write ERROR: %d", status);
            return false;
        }

        bool result = false;
        // Read data from specified register
        uint8_t readBytes = wire.requestFrom(address, 2);
        if (readBytes == 2)
        {
            // First read an empty byte
            int readValue = wire.read();
            // Check if it is empty
            if (readValue != 0x00)
            {
                return false;
            }
            // Next read target value
            readValue = wire.read();
            if (readValue >= 0)
            {
                value = static_cast<uint8_t>(readValue);
                result = true;
            }
        }

        return result;
    }

    bool incrementData(uint8_t regAddr)
    {
        // Write data action
        uint8_t command = 0x01 << 2;
        command |= ((regAddr & 0x0F) << 4);

        wire.beginTransmission(address);
        wire.write(static_cast<uint8_t>(command));
        uint8_t status = wire.endTransmission();
        if (status != 0)
        {
            LOG_ERROR("Write ERROR: %d", status);
            return false;
        }

        return true;
    }

    bool decrementData(uint8_t regAddr)
    {
        // Write data action
        uint8_t command = 0x02 << 2;
        command |= ((regAddr & 0x0F) << 4);

        wire.beginTransmission(address);
        wire.write(static_cast<uint8_t>(command));
        uint8_t status = wire.endTransmission();
        if (status != 0)
        {
            LOG_ERROR("Write ERROR: %d", status);
            return false;
        }

        return true;
    }
} // namespace

/**
 * @brief Configure POT with the terminal config
 *
 * @param[in] config Terminal config
 * @return true if configuration succeed, false otherwise
 */
bool Mcp45hv::configure(TerminalConfig config)
{
    uint8_t value;

    // First read the current control register value
    bool result = readData(controlAddress, value);
    if (result == false)
    {
        LOG_ERROR("Can not read control register value!");
        return false;
    }

    LOG_TRACE("Current control register value = 0x%02X", value);

    // Next modify the control register value
    ControlRegister *pControl = static_cast<ControlRegister *>(static_cast<void *>(&value));
    switch (config)
    {
    case TerminalConfig::Disconnect:
        // The current states of R0W, R0A, R0B are ignored when R0HW == 0
        pControl->R0HW = 0;
        break;
    case TerminalConfig::ConnectBack:
        // The current states of R0W, R0A, R0B are active back when R0HW == 1
        pControl->R0HW = 1;
        break;
    case TerminalConfig::ConnectWA:
        pControl->R0HW = 1;
        pControl->R0W = 1;
        pControl->R0A = 1;
        pControl->R0B = 0;
        break;
    case TerminalConfig::ConnectWB:
        pControl->R0HW = 1;
        pControl->R0W = 1;
        pControl->R0A = 0;
        pControl->R0B = 1;
        break;
    case TerminalConfig::ConnectWAB:
        pControl->R0HW = 1;
        pControl->R0W = 1;
        pControl->R0A = 1;
        pControl->R0B = 1;
        break;
    }

    LOG_DEBUG("New control register value = 0x%02X", value);

    return writeData(controlAddress, value);
}

/**
 * @brief Set new wiper position
 *
 * @param[in] code Code value for wiper position
 * @return true if setting succeed, false otherwise
 */
bool Mcp45hv::setWiper(uint8_t code)
{
    bool result = writeData(wiperAddress, code);

    LOG_TRACE("Set -> 0x%02X, %s", code, result ? "OK" : "FAIL");

    return result;
}

/**
 * @brief Get the current wiper position
 *
 * @param[out] code Reference to a code value of wiper position
 * @return true if setting succeed, false otherwise
 */
bool Mcp45hv::getWiper(uint8_t &code)
{
    bool result = readData(wiperAddress, code);

    LOG_TRACE("Get <- 0x%02X, %s", code, result ? "OK" : "FAIL");

    return result;
}

/**
 * @brief Increment wiper position
 *
 * @return true if incrementing succeed, false otherwise
 */
bool Mcp45hv::incrementWiper()
{
    bool result = incrementData(wiperAddress);

    LOG_TRACE("Increment %s", result ? "OK" : "FAIL");

    return result;
}

/**
 * @brief Decrement wiper position
 *
 * @return true if decrementing succeed, false otherwise
 */
bool Mcp45hv::decrementWiper()
{
    bool result = decrementData(wiperAddress);

    LOG_TRACE("Decrement %s", result ? "OK" : "FAIL");

    return result;
}
