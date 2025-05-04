/**
 * @file VddController.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Analog sensor voltage controller API
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <stdbool.h>

namespace Analog::VddController
{
    /**
     * @brief Initialize the voltage controller and its components
     * @note IO expander device should be initialized before this
     *
     * @return true if initialization succeeds, false otherwise
     */
    bool initialize();

    /**
     * @brief Cut off voltage from sensor
     * @note Disable DCDC
     *
     * @return true if disabling succeeds, false otherwise
     */
    bool cutOffVoltage();

    /**
     * @brief Apply the target voltage level to sensor
     * @note Enable DCDC if it is needed
     *
     * @return true if applying succeeds, false otherwise
     */
    bool applyVoltage();

    /**
     * @brief Measure the actual voltage level applied to sensor
     *
     * @return Actual voltage, Volt
     */
    float measureVoltage();

    /**
     * @brief Get target voltage level
     *
     * @return Target voltage, Volt
     */
    float targetVoltage();

    /**
     * @brief Set new target voltage level
     *
     * @param voltage New target voltage, Volt
     */
    void setTargetVoltage(float voltage);
} // namespace Analog::VddController
