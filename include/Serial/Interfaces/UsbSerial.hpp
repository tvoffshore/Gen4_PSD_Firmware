#pragma once

#include <stdint.h>

#include <HardwareSerial.h>

#include "SerialInterface.hpp"

namespace Serials
{
    /**
     * @brief Usb serial device class
     */
    class UsbSerial : public SerialInterface
    {
        // Baudrate for serial device
        static constexpr size_t serialBaudrate = 115200;

    public:
        /**
         * @brief Construct a new Usb Serial object
         */
        UsbSerial()
            : _serial(Serial)
        {
        }

        /**
         * @brief Initialize hardware peripheral (pins, interfaces)
         */
        virtual void initialize() override
        {
        }

        /**
         * @brief Get information string about serial interface
         *
         * @return const char* Information string
         */
        virtual const char *getInfo() override
        {
            return "USB-SERIAL";
        }

        /**
         * @brief Start hardware (apply power, begine communication)
         */
        virtual void start() override
        {
            _serial.begin(serialBaudrate);
        }

        /**
         * @brief Stop hardware (remove power, stop communication)
         */
        virtual void stop() override
        {
            _serial.end();
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
        HardwareSerial &_serial; ///< Reference to usb serial interface object
    };
} // namespace Serials
