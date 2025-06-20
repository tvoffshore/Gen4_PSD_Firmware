/**
 * @file Board.h
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Board management API
 * @version 0.1
 * @date 2024-06-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <Arduino.h>

namespace Board
{
    // USB serial interface configuration
    namespace UsbConfig
    {
        constexpr unsigned long baudrate = 115200;
    } // namespace UART

    // I2C interface configuration
    namespace I2cConfig
    {
        constexpr uint32_t frequency = 100000; // 100kHz
        constexpr auto pinSda = GPIO_NUM_34;   // I2C SDA pin
        constexpr auto pinScl = GPIO_NUM_33;   // I2C SCL pin
    } // namespace I2C

    // SPI interface configuration
    namespace SpiConfig
    {
        // Maximum SPI SCK frequency
        constexpr uint32_t frequency = 20000000; // 20MHz
        constexpr auto pinSck = GPIO_NUM_1;      // SPI clock pin
        constexpr auto pinMosi = GPIO_NUM_47;    // SPI MOSI pin
        constexpr auto pinMiso = GPIO_NUM_48;    // SPI MISO pin
        constexpr auto pinCsSd = GPIO_NUM_26;    // SPI chip select for SD pin
    } // namespace SpiConfig

    /**
     * @brief Setup board power, all interfaces and pins
     */
    void setup();

    /**
     * @brief Setup USB serial interface
     */
    void setupUSB();

    /**
     * @brief Setup I2C interface
     */
    void setupI2C();

    /**
     * @brief Setup SPI interface
     */
    void setupSPI();

    /**
     * @brief Stop SPI interface
     */
    void stopSPI();

    /**
     * @brief Setup build-in LED
     */
    void setupLED();

    /**
     * @brief Setup board power
     */
    void powerUp();

    /**
     * @brief Shut down board power
     */
    void powerDown();

    /**
     * @brief Enable gpio pad hold function for particular pins that need to keep their states
     */
    void holdPins();

    /**
     * @brief Disable gpio pad hold function for particular pins that need to change their states
     */
    void releasePins();
} // namespace Power
