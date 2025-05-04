/**
 * @file Tca9534.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief TCA9534 Low Voltage 8-Bit I2C and SMBUS Low-Power I/O Expander driver API
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

namespace IoExpander::Tca9534
{
    /**
     * @brief Configure I/O Expander interface
     *
     * @return true if configuration succeed, false otherwise
     */
    bool configure();

    /**
     * @brief Write pin config
     *
     * @param[in] value Value to write
     * @return true if writing succeed, false otherwise
     */
    bool writeConfig(uint8_t value);

    /**
     * @brief Read pin config
     *
     * @param[out] value Reference to read value
     * @return true if reading succeed, false otherwise
     */
    bool readConfig(uint8_t &value);

    /**
     * @brief Write output port
     *
     * @param[in] value Value to write
     * @return true if writing succeed, false otherwise
     */
    bool writePort(uint8_t value);

    /**
     * @brief Read input port
     *
     * @param[out] value Reference to read value
     * @return true if reading succeed, false otherwise
     */
    bool readPort(uint8_t &value);
} // namespace IoExpander::Tca9534
