#pragma once

#include <stdint.h>

#include "SerialInterface.hpp"

namespace Serials
{
/**
 * @brief Mock serial device class
 */
class SerialMock : public SerialInterface
{
public:
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
        return "SERIAL-MOCK";
    }

    /**
     * @brief Start hardware (apply power, begine communication)
     */
    virtual void start() override
    {
    }

    /**
     * @brief Stop hardware (remove power, stop communication)
     */
    virtual void stop() override
    {
    }

    /**
     * @brief Check availability of received data
     *
     * @return True if at least 1 byte is available, or false if nothing has been received
     */
    virtual int available() override
    {
        return 0;
    }

    /**
     * @brief Read 1 byte
     *
     * @return 0 to 255 if available, or -1 if nothing available
     */
    virtual int read() override
    {
        return -1;
    }

    /**
     * @brief Transmit buffer
     *
     * @param str Pointer to string
     * @param size Size of transmitted data
     */
    virtual size_t write(const char *buffer, size_t size) override
    {
        return 0;
    }
};
} // namespace Serials
