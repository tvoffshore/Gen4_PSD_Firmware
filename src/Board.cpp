/**
 * @file Board.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Board management implementation
 * @version 0.1
 * @date 2024-06-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "Board.h"

#include <stddef.h>
#include <stdint.h>

#include <Debug.hpp>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <SPI.h>
#include <Wire.h>

#include "Serial/Interfaces/Max3221.hpp"
#include "Serial/Interfaces/St3485.hpp"

using namespace Board;

namespace
{
    // Photo diode pin (ADC_1_CH6)
    constexpr auto pinPhotoDiode = GPIO_NUM_7;

    /**
     * @brief Convert seconds to microseconds
     *
     * @param seconds Time in seconds
     * @return Microseconds
     */
    constexpr uint64_t secondsToMicros(size_t seconds)
    {
        return seconds * 1000 * 1000;
    }

    // Functions prototypes
    void moveLoraToSleep();
    void holdPinsDeepSleep();

    /**
     * @brief Move LoRa chip into sleep mode
     */
    void moveLoraToSleep()
    {
        // Initialize SPI bus and slave select pin to communicate with LoRa chip
        SPI.end();
        SPI.setBitOrder(MSBFIRST);
        SPI.setDataMode(SPI_MODE0);
        SPI.setFrequency(2000000);
        SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI);
        pinMode(LoRa_NSS, OUTPUT);

        // Wake up LoRa chip
        digitalWrite(LoRa_NSS, LOW);
        SPI.transfer(0xC0);
        SPI.transfer(0);
        digitalWrite(LoRa_NSS, HIGH);

        // Wait until LoRa chip is ready
        size_t timeoutMs = 1000;
        pinMode(LoRa_BUSY, INPUT);
        while (digitalRead(LoRa_BUSY))
        {
            delay(1);
            timeoutMs--;
            if (timeoutMs == 0)
            {
                LOG_ERROR("Waiting LoRa ready failed");
                return;
            }
        }

        // Move LoRa chip into sleep mode
        digitalWrite(LoRa_NSS, LOW);
        SPI.transfer(0x84);
        SPI.transfer(0);
        digitalWrite(LoRa_NSS, HIGH);

        delay(2);

        // Deinitialize SPI bus and slave select pin
        SPI.end();
        pinMode(LoRa_NSS, INPUT);
    }

    /**
     * @brief Enable gpio pad hold function for particular pins that need to keep their states
     */
    void holdPinsDeepSleep()
    {
        // Workaround to keep RS232 RX state while ESP is in deep sleep
        esp_err_t error = gpio_hold_en(Serials::Max3221::pinRxEnable);
        LOG_TRACE("Hold RS232 RE pin state, error: %d", error);

        // Workaround to keep RS485 RX state while ESP is in deep sleep
        error = gpio_hold_en(Serials::St3485::pinRxEnable);
        LOG_TRACE("Hold RS485 RE pin state, error: %d", error);
    }
} // namespace

/**
 * @brief Setup board
 */
void Board::setup()
{
    // Set LoRa chip into sleep mode
    moveLoraToSleep();

    // Setup USB serial port for debug messages
    setupUSB();
    // Setup I2C interface to communicate with IMU, RTC
    setupI2C();
    // Setup SPI interface to communicate with SD, Display
    setupSPI();
    // Setup build-in LED (turn off)
    setupLED();

    // Power up the board
    powerUp();
    delay(10);

    LOG_INFO("Board is powered up");
}

/**
 * @brief Setup USB serial interface
 */
void Board::setupUSB()
{
    Serial.begin(UsbConfig::baudrate);
}

/**
 * @brief Setup I2C interface
 */
void Board::setupI2C()
{
    Wire.begin(I2cConfig::pinSda, I2cConfig::pinScl, I2cConfig::frequency);
}

/**
 * @brief Setup SPI interface
 */
void Board::setupSPI()
{
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setFrequency(SpiConfig::frequency);
    SPI.begin(SpiConfig::pinSck, SpiConfig::pinMiso, SpiConfig::pinMosi);
}

/**
 * @brief Stop SPI interface
 */
void Board::stopSPI()
{
    SPI.end();
}

/**
 * @brief Setup build-in LED
 */
void Board::setupLED()
{
    pinMode(BUILTIN_LED, OUTPUT);
    // Turn off build-in LED (LOW - off, HIGH - on)
    digitalWrite(BUILTIN_LED, LOW);
}

/**
 * @brief Setup board power
 */
void Board::powerUp()
{
    pinMode(Vext_CTRL, OUTPUT);
    // LOW - power up, HIGH - power down
    digitalWrite(Vext_CTRL, LOW);
}

/**
 * @brief Shut down board power
 */
void Board::powerDown()
{
    // LOW - power up, HIGH - power down
    digitalWrite(Vext_CTRL, HIGH);
}

/**
 * @brief Goes into deep sleep mode and wait wake up events
 * @warning This function never returns
 *
 * @param sleepDuration Time to sleep, seconds
 */
void Board::deepSleep(size_t sleepDuration)
{
    LOG_INFO("Entering into sleep mode, wake up in %u seconds", sleepDuration);

    esp_sleep_enable_ext1_wakeup(1 << pinPhotoDiode, ESP_EXT1_WAKEUP_ANY_HIGH);
    esp_sleep_enable_ext0_wakeup(Serials::Max3221::pinRx, LOW);
    esp_sleep_enable_timer_wakeup(secondsToMicros(sleepDuration));

    // Power down the board
    powerDown();

    // Hold gpio pins
    holdPinsDeepSleep();

    esp_deep_sleep_start();
    // Never reachable
}
