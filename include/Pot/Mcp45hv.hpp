/**
 * @file Mcp45hv.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief 7/8-Bit Single, +36V (±18V) Digital POT driver API
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * Digital Potentiometer contains three terminal pins: A, B, W.
 * A and B terminals a static, and W terminal is adjustable
 * and can be moved between A and B according to the Wiper position:
 *
 *  A
 *  |
 *  /
 *  \ - W
 *  /
 *  |
 *  B
 *
 * The resistance between A and B is constant, so moving W
 * between A and B leads to changing resistance between W-A and W-B
 */

namespace POT
{
    /**
     * @brief Terminal pins configuration
     */
    enum class TerminalConfig
    {
        Disconnect,  // All terminal pins are disconnected
        ConnectBack, // All terminal pins' states are restored
        ConnectWA,   // Terminal pins W and A are connected
        ConnectWB,   // Terminal pins W and B are connected
        ConnectWAB,  // Terminal pins W, A and B are connected
    };

    namespace Mcp45hv
    {
        /**
         * @brief Configure POT with the terminal config
         *
         * @param[in] config Terminal config
         * @return true if configuration succeed, false otherwise
         */
        bool configure(TerminalConfig config);

        /**
         * @brief Set new wiper position
         *
         * @param[in] code Code value for wiper position
         * @return true if setting succeed, false otherwise
         */
        bool setWiper(uint8_t code);

        /**
         * @brief Get the current wiper position
         *
         * @param[out] code Reference to a code value of wiper position
         * @return true if setting succeed, false otherwise
         */
        bool getWiper(uint8_t &code);

        /**
         * @brief Increment wiper position
         *
         * @return true if incrementing succeed, false otherwise
         */
        bool incrementWiper();

        /**
         * @brief Decrement wiper position
         *
         * @return true if decrementing succeed, false otherwise
         */
        bool decrementWiper();
    } // namespace Mcp45hv
} // namespace POT
