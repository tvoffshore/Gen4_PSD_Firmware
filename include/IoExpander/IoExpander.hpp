/**
 * @file IoExpander.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief I/O Expander API
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

namespace IoExpander
{
    /**
     * @brief Pin configuration
     */
    enum class PinConfig
    {
        Output, //< Pin configured as output
        Input   //< Pin configured as input
    };

    /**
     * @brief I/O Expander pin identifiers
     */
    enum class Pin
    {
        SW2_A0,        // Output
        SW2_A1,        // Output
        SW2_EN,        // Output
        RS232_FORCEON, // Output
        RS232_FLT,     // Input
        IMU_INT,       // Input
        VSENS_PG,      // Input
        DCDC_EN,       // Output

        Count // Total count of pins
    };

    /**
     * @brief I/O Expander pin masks
     */
    namespace PinMask
    {
        constexpr uint8_t SW2_A0 = 1 << 0;
        constexpr uint8_t SW2_A1 = 1 << 1;
        constexpr uint8_t SW2_EN = 1 << 2;
        constexpr uint8_t RS232_FORCEON = 1 << 3;
        constexpr uint8_t RS232_FLT = 1 << 4;
        constexpr uint8_t IMU_INT = 1 << 5;
        constexpr uint8_t VSENS_PG = 1 << 6;
        constexpr uint8_t DCDC_EN = 1 << 7;
    } // namespace PinMask

    /**
     * @brief Initialize I/O Expander
     *
     * @return true if initialization succeed, false otherwise
     */
    bool initialize();

    /**
     * @brief Initialize pin with specified config
     *
     * @param[in] pin Pin identifier to initialize
     * @param[in] config Pins configuration
     * @return true if configuration succeeds, false otherwise
     */
    bool initializePin(Pin pin, PinConfig config);

    /**
     * @brief Initialize pins with specified config
     *
     * @param[in] mask Pin mask to initialize
     * @param[in] config Pins configuration
     * @return true if configuration succeeds, false otherwise
     */
    bool initializePins(uint8_t mask, PinConfig config);

    /**
     * @brief Set specified output pin
     *
     * @param[in] pin Pin identifier to set
     * @return true if setting succeed, false otherwise
     */
    bool setPin(Pin pin);

    /**
     * @brief Set specified output pins at once
     * @note It is more effective to change several pins in one I/O Expander transaction
     *
     * @param[in] mask Pin mask to set
     * @return true if setting succeed, false otherwise
     */
    bool setPins(uint8_t mask);

    /**
     * @brief Reset specified output pin
     *
     * @param[in] pin Pin identifier to reset
     * @return true if resetting succeed, false otherwise
     */
    bool resetPin(Pin pin);

    /**
     * @brief Reset specified output pins
     * @note It is more effective to change several pins in one I/O Expander transaction
     *
     * @param[in] mask Pin mask to reset
     * @return true if resetting succeed, false otherwise
     */
    bool resetPins(uint8_t mask);

    /**
     * @brief Toggle specified output pin
     *
     * @param[in] pin Pin identifier to toggle
     * @return true if toggling succeed, false otherwise
     */
    bool togglePin(Pin pin);

    /**
     * @brief Toggle specified output pins
     * @note It is more effective to change several pins in one I/O Expander transaction
     *
     * @param[in] mask Pin mask to toggle
     * @return true if toggling succeed, false otherwise
     */
    bool togglePins(uint8_t mask);

    /**
     * @brief Read specified input/output pin
     *
     * @param pin Pin identifier to read
     * @param state Reference to pin input state
     * @return true if reading succeed, false otherwise
     */
    bool readPin(Pin pin, bool &state);

    /**
     * @brief Perform a self-test
     * @note The specified output pins change their state one by one and the input value of these pins is checked
     * @warning Self-test may set any of the output pins to different states! Enable only if you know what you do!!!
     *
     * @param[in] outputMask Mask of output pins to do self-test with
     * @return true if self-test passed, false otherwise
     */
    bool selfTest(uint8_t outputMask);

} // namespace IoExpander
