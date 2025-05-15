/**
 * @file InputSelector.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Analog channel 2 input type selector API
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
     * @brief Input type options
     */
    enum class InputType : uint8_t
    {
        In2Neg4_20mA, // 0: IN2-/4-20mA signal
        In2Pos0_5V,   // 1: IN2+/0-5V signal
        In2Out,       // 2: IN2 output signal
        Ground,       // 3: Ground plate

        Count // Total count of input types
    };

    namespace InputSelector
    {
        /**
         * @brief Initialize module
         *
         * @return true if initialization succeeds, false otherwise
         */
        bool initialize();

        /**
         * @brief Get the current input type
         *
         * @return The current input type option
         */
        InputType inputType();

        /**
         * @brief Set new input type
         *
         * @param[in] inputType New input type option
         * @return true if setting succeeds, false otherwise
         */
        bool setInputType(InputType inputType);
    } // namespace InputSelector
} // namespace Analog
