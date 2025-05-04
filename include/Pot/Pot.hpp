/**
 * @file Pot.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Digital POT API
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

namespace POT
{
    /**
     * @brief Initialize POT device
     * @note Setup W and B terminals and maximum resistance
     *
     * @return true if initialization succeed, false otherwise
     */
    bool initialize();

    /**
     * @brief Set specified resistance to Ohm value
     *
     * @param[in] resistanceOhm Resistance to set
     * @return true if setting succeed, false otherwise
     */
    bool setResistance(uint32_t resistanceOhm);

    /**
     * @brief Get the current resistance value
     *
     * @param[out] resistanceOhm Reference to the current resistance value
     * @return true if getting succeed, false otherwise
     */
    bool getResistance(uint32_t &resistanceOhm);

    /**
     * @brief Increase resistance value by one step
     *
     * @return true if increasing succeed, false otherwise
     */
    bool increaseResistance();

    /**
     * @brief Decrease resistance value by one step
     *
     * @return true if decreasing succeed, false otherwise
     */
    bool decreaseResistance();
} // namespace POT
