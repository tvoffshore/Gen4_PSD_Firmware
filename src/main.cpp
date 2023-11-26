#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <Debug.hpp>
#include <SystemTime.hpp>

namespace WirePin
{
    constexpr auto sda = GPIO_NUM_34;
    constexpr auto scl = GPIO_NUM_33;
} // namespace WirePin

#if (LOG_LEVEL > LOG_LEVEL_NONE)
// Log level declaration
uint8_t LogsOutput::logLevelMax = LOG_LEVEL;
#endif // #if (LOG_LEVEL > LOG_LEVEL_NONE)

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

void setup()
{
    // Setup USB serial port for debug messages (115200 baudrate)
    Serial.begin(115200);

    // Setup I2C interface to communicate with IMU, RTC (100kHz frequency)
    Wire.begin(WirePin::sda, WirePin::scl, 100000);

    // Set LoRa chip into sleep mode
    moveLoraToSleep();

    // Turn off build-in LED (LOW - off, HIGH - on)
    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, LOW);

    // Power up the board (LOW - power up, HIGH - power down)
    pinMode(Vext_CTRL, OUTPUT);
    digitalWrite(Vext_CTRL, LOW);
    delay(10);
    LOG_INFO("Board is powered up");

    // Initialize system time with RTC value
    SystemTime::initialize(Wire);

    LOG_INFO("Setup done");
}

void loop()
{
}