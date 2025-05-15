/**
 * @file GainSelector.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Analog channel 1 gain selector API
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
     * @brief Gain options
     */
    enum class Gain : uint8_t
    {
        Gain100, // 0: Gain (Rg = 2.1kOhm) = (200kOhm / 2.1kOhm) + 5 = 100.24
        Gain150, // 1: Gain (Rg = 1.37kOhm) = (200kOhm / 1.37kOhm) + 5 = 150.99
        Gain200, // 2: Gain (Rg = 1.02kOhm) = (200kOhm / 1.02kOhm) + 5 = 201.08
        Gain250, // 3: Gain (Rg = 806Ohm) = (200kOhm / 806Ohm) + 5 = 253.14

        Count // Total count of gains
    };

    namespace GainSelector
    {
        /**
         * @brief Initialize module
         * Initialize output pins and set gain from the settings
         *
         * @return true if initialization succeeds, false otherwise
         */
        bool initialize();

        /**
         * @brief Get the current gain
         *
         * @return The current gain option
         */
        Gain gain();

        /**
         * @brief Set new gain
         *
         * @param[in] selection New gain option
         */
        void setGain(Gain gain);
    } // namespace GainSelector
} // namespace Analog
