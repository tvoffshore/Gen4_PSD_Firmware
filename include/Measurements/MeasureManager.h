/**
 * @file MeasureManager.h
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Sensor measurements manager API
 * @version 0.1
 * @date 2024-07-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <stdbool.h>

namespace Measurements::Manager
{
    /**
     * @brief Initialize sensor measurements
     *
     * @return true if initialization succeed, false otherwise
     */
    bool initialize();

    /**
     * @brief Perform sensor input data processing
     */
    void process();
} // namespace Measurements::Manager
