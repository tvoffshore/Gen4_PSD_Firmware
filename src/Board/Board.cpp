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

#include "Board/Board.hpp"

#include <stddef.h>
#include <stdint.h>

#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <SPI.h>
#include <Wire.h>

#include <Log.hpp>

#include "Serial/Interfaces/Max3221.hpp"
#include "Serial/Interfaces/St3485.hpp"

using namespace Board;

namespace
{
    // Max time to wait LoRa is ready
    constexpr size_t loraReadyWaitTimeMs = 1000;

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

        // Initialize LoRa pins
        pinMode(LoRa_RST, OUTPUT);
        pinMode(LoRa_NSS, OUTPUT);
        pinMode(LoRa_BUSY, INPUT);
        digitalWrite(LoRa_RST, HIGH);

        // Wake up LoRa chip
        digitalWrite(LoRa_NSS, LOW);
        SPI.transfer(0xC0);
        SPI.transfer(0);
        digitalWrite(LoRa_NSS, HIGH);

        // Wait until LoRa chip is ready
        size_t expiredMs = 0;
        bool isReady = digitalRead(LoRa_BUSY) == 0;
        while (isReady == false && expiredMs < loraReadyWaitTimeMs)
        {
            delay(1);
            expiredMs++;
            isReady = digitalRead(LoRa_BUSY) == 0;
        }
        LOG_DEBUG("LoRa %s ready in %d ms", isReady ? "is" : "isn't", expiredMs);
        if (isReady == false)
        {
            LOG_ERROR("Waiting LoRa ready failed");
            return;
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
} // namespace

/**
 * @brief Setup the board
 */
void Board::setup()
{
    // Disable the pin hold function to have a chance to change their states
    releasePins();

    // Setup USB serial port for log messages
    setupUSB();

    LOG_DEBUG("Setup board...");

    // Set LoRa chip into sleep mode
    moveLoraToSleep();

    // Setup I2C interface to communicate with IMU, RTC
    setupI2C();
    // Setup SPI interface to communicate with SD, Display
    setupSPI();
    // Setup build-in LED (turn off)
    setupLED();

    // Initialize power pin
    pinMode(Vext_CTRL, OUTPUT);
    // Power down the board initially
    powerDown();

    LOG_INFO("Board setup done");
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
    // LOW - power up
    digitalWrite(Vext_CTRL, LOW);
    delay(10);
}

/**
 * @brief Shut down board power
 */
void Board::powerDown()
{
    // HIGH - power down
    digitalWrite(Vext_CTRL, HIGH);
}

/**
 * @brief Enable gpio pad hold function for particular pins that need to keep their states
 */
void Board::holdPins()
{
    // Workaround to keep UART TX HIGH +3.3V (RS232 DOUT LOW -5V) while ESP is in deep sleep
    gpio_hold_en(Serials::Max3221::pinTx);

    // Workaround to keep RS232 RX state ENABLED to wake up on serial command while ESP is in deep sleep
    gpio_hold_en(Serials::Max3221::pinRxEnable);

    // Workaround to keep RS485 RX state ENABLED to wake up on serial command while ESP is in deep sleep
    gpio_hold_en(Serials::St3485::pinRxEnable);
    // Workaround to keep RS485 TX state DISABLED while ESP is in deep sleep
    gpio_hold_en(Serials::St3485::pinTxEnable);

    // Workaround to keep LoRa reset pin HIGH while ESP is in deep sleep (not to restart LoRa occasionally)
    gpio_hold_en(static_cast<gpio_num_t>(LoRa_RST));
}

/**
 * @brief Disable gpio pad hold function for particular pins that need to change their states
 */
void Board::releasePins()
{
    // Release RS232 TX enable pin after deep sleep
    gpio_hold_dis(Serials::Max3221::pinTx);

    // Release RS232 RX enable pin after deep sleep
    gpio_hold_dis(Serials::Max3221::pinRxEnable);

    // Release RS485 RX enable pin after deep sleep
    gpio_hold_dis(Serials::St3485::pinRxEnable);
    // Release RS485 TX enable pin after deep sleep
    gpio_hold_dis(Serials::St3485::pinTxEnable);

    // Release LoRa reset pin after deep sleep
    gpio_hold_dis(static_cast<gpio_num_t>(LoRa_RST));
}
