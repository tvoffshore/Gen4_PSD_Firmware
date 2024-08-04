/**
 * @file Battery.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Battery controller API
 * @version 0.1
 * @date 2024-08-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <stdint.h>

namespace Battery
{
    /**
     * @brief Battery status data structure
     */
    struct Status
    {
        uint16_t voltage; // Battery voltage, millivolts
        uint8_t level;    // Battery level, percents
    };

    namespace Controller
    {
        /**
         * @brief Initialize battery controller
         */
        void initialize();

        /**
         * @brief Read battery status
         *
         * @return Battery status
         */
        const Status &readStatus();

        /**
         * @brief Get the last read battery status
         *
         * @return The last battery status
         */
        const Status &lastStatus();
    } // namespace Controller
} // namespace Battery