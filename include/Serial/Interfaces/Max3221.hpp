#pragma once

#include <stdint.h>

#include <HardwareSerial.h>

#include "SerialInterface.hpp"

namespace Serials
{
    /**
     * @brief MAX3221 transceiver device class
     */
    class Max3221 : public SerialInterface
    {
    public:
        // Serial RX pin
        static constexpr auto pinRx = GPIO_NUM_18;
        // Serial TX pin
        static constexpr auto pinTx = GPIO_NUM_21;
        // Receiver enable (active low)
        static constexpr auto pinRxEnable = GPIO_NUM_19;

        // Baud rate for serial device
        static constexpr size_t serialBaudrate = 9600;

        /**
         * @brief Construct a new Max3221 object
         */
        Max3221()
            : _serial(Serial1)
        {
        }

        /**
         * @brief Initialize hardware peripheral (pins, interfaces)
         */
        virtual void initialize() override
        {
            pinMode(pinRxEnable, OUTPUT);
            // Disable receiver
            digitalWrite(pinRxEnable, HIGH);
        }

        /**
         * @brief Get information string about serial interface
         *
         * @return const char* Information string
         */
        virtual const char *getInfo() override
        {
            return "RS232-MAX3221";
        }

        /**
         * @brief Start hardware (apply power, begine communication)
         */
        virtual void start() override
        {
            _serial.begin(serialBaudrate);
            _serial.setPins(pinRx, pinTx);

            // Esp32 resets rxEnablePin pin mode after uart restart
            pinMode(pinRxEnable, OUTPUT);
            // Enable receiver
            digitalWrite(pinRxEnable, LOW);
        }

        /**
         * @brief Stop hardware (remove power, stop communication)
         */
        virtual void stop() override
        {
            _serial.end();

            // Disable receiver
            digitalWrite(pinRxEnable, HIGH);
        }

        /**
         * @brief Check availability of received data
         *
         * @return True if at least 1 byte is available, or false if nothing has been received
         */
        virtual int available() override
        {
            return _serial.available();
        }

        /**
         * @brief Read 1 byte
         *
         * @return 0 to 255 if available, or -1 if nothing available
         */
        virtual int read() override
        {
            return _serial.read();
        }

        /**
         * @brief Transmit buffer
         *
         * @param str Pointer to string
         * @param size Size of transmitted data
         */
        virtual size_t write(const char *buffer, size_t size) override
        {
            return _serial.write(buffer, size);
        }

    private:
        HardwareSerial &_serial; // Reference to uart hardware serial interface object
    };
} // namespace Serials
