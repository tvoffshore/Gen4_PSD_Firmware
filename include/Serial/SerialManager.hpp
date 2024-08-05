#pragma once

#include <array>
#include <functional>

#include "SerialCommands.hpp"
#include "SerialDevice.hpp"

namespace Serials
{
    /**
     * @brief Serial interfaces select options
     */
    enum class SerialSelect
    {
        RS232, // RS232 serial interface
        RS485, // RS485 serial interface
        Count  // Serial interfaces count
    };

    /**
     * @brief Serial command types
     */
    enum class CommandType
    {
        Read,   ///< Read command type
        Write,  ///< Write command type
        Execute ///< Execute command type
    };

    /**
     * @brief The read serial command handler function type
     */
    using ReadCommandHandler = std::function<void(const char **)>;

    /**
     * @brief The write serial command handler function type
     */
    using WriteCommandHandler = std::function<void(const char *)>;

    /**
     * @brief Command notify handler function type
     */
    using CommandNotifyHandler = std::function<void(CommandType)>;

    namespace Manager
    {
        /**
         * @brief Initialize serial manager
         *
         * @return True if initialization succeeds, false otherwise
         */
        void initialize();

        /**
         * @brief Perform serial devices input data processing
         */
        void process();

        /**
         * @brief Subscribe to specified read command to provide read data
         * May be only one subscriber that provides read data
         *
         * @param commandId Command identifier
         * @param handler Handler function
         */
        void subscribeToRead(CommandId commandId, ReadCommandHandler &&handler);

        /**
         * @brief Subscribe to specified write command to handle write data
         * May be only one subscriber that handle write data
         *
         * @param commandId Command identifier
         * @param handler Handler function
         */
        void subscribeToWrite(CommandId commandId, WriteCommandHandler &&handler);

        /**
         * @brief Subscribe to notification about specified command received
         * May be up to notifiersMaxCount subscribers for command notifications
         *
         * @param commandId Command identifier
         * @param handler Handler function
         */
        void subscribeToNotify(CommandId commandId, CommandNotifyHandler &&handler);

        /**
         * @brief Get the current serial command source device
         *
         * @return Pointer to serial device
         */
        SerialDevice *getCommandSourceDevice();
    } // namespace Manager
} // namespace Serials
