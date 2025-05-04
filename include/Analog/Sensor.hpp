/**
 * @file Sensor.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Analog sensor API
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <stdint.h>

namespace Analog
{
    /**
     * @brief Analog sensor class
     */
    class Sensor
    {
    public:
        /**
         * @brief Construct a new analog sensor object
         *
         * @param inputPin Pin to read raw analog values
         * @param enablePin Enable sensor pin (active HIGH)
         */
        Sensor(uint8_t inputPin, uint8_t enablePin);

        /**
         * @brief Initialize analog sensor
         */
        void initialize();

        /**
         * @brief Enable analog sensor
         */
        void enable();

        /**
         * @brief Disable analog sensor
         */
        void disable();

        /**
         * @brief Read current value from analog sensor
         *
         * @return Analog value
         */
        uint16_t read();

    private:
        uint8_t _inputPin; // Pin to read raw analog values
        uint8_t _enablePin; // Enable sensor pin (active HIGH)
    };
} // namespace Analog
