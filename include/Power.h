/**
 * @file Power.h
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Power management API
 * @version 0.1
 * @date 2024-06-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <stddef.h>

namespace Power
{
    /**
     * @brief Setup board power
     */
    void setup();

    /**
     * @brief Goes into deep sleep mode and wait wake up events
     * @warning This function never returns
     *
     * @param sleepDuration Time to sleep, seconds
     */
    void deepSleep(size_t sleepDuration);
} // namespace Power
