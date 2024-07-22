#pragma once

#include <stdint.h>

#include <HardwareSerial.h>

#include "SerialInterface.hpp"

namespace Serials
{
    /**
     * @brief ST3485 transceiver device class
     */
    class St3485 : public SerialInterface
    {
    public:
        // Serial Rx pin
        static constexpr auto pinRx = GPIO_NUM_18;
        // Serial Tx Pin
        static constexpr auto pinTx = GPIO_NUM_21;
        // Receiver output enable (RO is enabled when RE is low)
        static constexpr auto pinRxEnable = GPIO_NUM_17;
        // Driver output enable (DO is enabled when DE is high)
        static constexpr auto pinTxEnable = GPIO_NUM_40;

        // Baud rate for serial device
        static constexpr size_t serialBaudrate = 9600;

        /**
         * @brief Construct a new St3485 object
         */
        St3485()
            : _serial(Serial1)
        {
        }

        /**
         * @brief Initialize hardware peripheral (pins, interfaces)
         */
        virtual void initialize() override
        {
            pinMode(pinRxEnable, OUTPUT);
            pinMode(pinTxEnable, OUTPUT);

            // Switch RS485 transceiver into low-power mode
            digitalWrite(pinRxEnable, HIGH); // Set RE to HIGH
            digitalWrite(pinTxEnable, LOW);  // Reset DE to LOW
        }

        /**
         * @brief Get information string about serial interface
         *
         * @return const char* Information string
         */
        virtual const char *getInfo() override
        {
            return "RS485-ST3485";
        }

        /**
         * @brief Start hardware (apply power, begine communication)
         */
        virtual void start() override
        {
            _serial.begin(serialBaudrate);
            _serial.setPins(pinRx, pinTx);

            // Switch RS485 transceiver into receiver mode
            digitalWrite(pinRxEnable, LOW); // Set RE to LOW
        }

        /**
         * @brief Stop hardware (remove power, stop communication)
         */
        virtual void stop() override
        {
            _serial.end();

            // Switch RS485 transceiver into low-power mode
            digitalWrite(pinRxEnable, HIGH); // Set RE to HIGH
            digitalWrite(pinTxEnable, LOW);  // Reset DE to LOW
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
            // Switch RS485 transceiver into driver mode
            digitalWrite(pinTxEnable, HIGH); // Set DE to HIGH

            size_t txSize = _serial.write(buffer, size);
            _serial.flush();

            // Switch RS485 transceiver into receiver mode
            digitalWrite(pinTxEnable, LOW); // Reset DE to LOW

            return txSize;
        }

    private:
        HardwareSerial &_serial; ///< Reference to uart hardware serial interface object
    };
} // namespace Serials
