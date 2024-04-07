#pragma once

#include <stddef.h>

namespace Serials
{
/**
 * @brief Serial interface
 */
struct SerialInterface
{
    /**
     * @brief Initialize serial interface (pins, interfaces)
     */
    virtual void initialize() = 0;

    /**
     * @brief Get information string about serial interface
     *
     * @return const char* Information string
     */
    virtual const char *getInfo() = 0;

    /**
     * @brief Start serial interface (apply power, begine communication)
     */
    virtual void start() = 0;

    /**
     * @brief Stop serial interface (remove power, stop communication)
     */
    virtual void stop() = 0;

    /**
     * @brief Check availability of received data from the serial interface
     *
     * @return True if at least 1 byte is available, or false if nothing has been received
     */
    virtual int available() = 0;

    /**
     * @brief Read 1 byte from the serial interface
     *
     * @return 0 to 255 if available, or -1 if nothing available
     */
    virtual int read() = 0;

    /**
     * @brief Transmit buffer to the serial interface
     *
     * @param str Pointer to string
     * @param size Size of transmitted data
     */
    virtual size_t write(const char *buffer, size_t size) = 0;
};
} // namespace Serials
