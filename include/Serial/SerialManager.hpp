#pragma once

#include <array>
#include <functional>

#include "SerialCommands.hpp"
#include "SerialDevice.hpp"

namespace Serials
{
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
     * @brief Serial manager class
     * Perform incoming data handling from serial devices
     */
    class SerialManager
    {
#pragma pack(push, 1)
        /**
         * @brief Non volatile settings structure for serial manager
         */
        struct Settings
        {
            int slaveAddress;     // Slave address of the serial devices
            uint8_t serialSelect; // Serial interface selection @ref SerialInterface
        };
#pragma pack(pop)

        // Maximum number of notifiers to one serial command
        constexpr static size_t notifiersMaxCount = 5;

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

    public:
        /**
         * @brief Serial interfaces
         */
        enum class SerialInterface
        {
            RS232, // RS232 serial interface
            RS485, // RS485 serial interface
            Count  // Serial interfaces count
        };

        /**
         * @brief Construct a new Serial Manager object
         */
        SerialManager();

        /**
         * @brief Initialize serial manager
         *
         * @return True if initialization succeeds, false otherwise
         */
        bool initialize();

        /**
         * @brief Perform serial devices input data processing
         */
        void process();

        /**
         * @brief Register handlers for read serial commands
         */
        void registerSerialReadHandlers();

        /**
         * @brief Register handlers for write serial commands
         */
        void registerSerialWriteHandlers();

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

    private:
        /**
         * @brief Switch between selected serial interfaces
         *
         * @param serialSelect Serial interface selection
         */
        void selectSerialInterface(uint8_t serialSelect);

        /**
         * @brief The read serial command handler function type
         *
         * @param device Pointer to the serial device that received the command
         * @param commandId Command identifier
         * @param responseString Response string with reading data
         * @return True if handling successfully, false otherwise
         */
        bool readHandler(SerialDevice *device, CommandId commandId, const char **responseString);

        /**
         * @brief The write serial command handler function type
         *
         * @param device Pointer to the serial device that received the command
         * @param commandId Command identifier
         * @param dataString String with data to write
         * @return True if handling successfully, false otherwise
         */
        bool writeHandler(SerialDevice *device, CommandId commandId, const char *dataString);

        /**
         * @brief The execute serial command handler function type
         *
         * @param device Pointer to the serial device that received the command
         * @param commandId Command identifier
         * @return True if handling successfully, false otherwise
         */
        bool executeHandler(SerialDevice *device, CommandId commandId);

        /**
         * @brief Notify about command received
         *
         * @param id Command identifier
         * @param type Received command type
         * @return True if at least one notifier exist, false otherwise
         */
        bool notifyCommand(size_t id, CommandType type);

        Settings _settings; // Settings of the serial manager

        std::array<ReadCommandHandler, static_cast<size_t>(CommandId::Commands)> _readHandlers;   // Read handlers
        std::array<WriteCommandHandler, static_cast<size_t>(CommandId::Commands)> _writeHandlers; // Write handlers

        std::array<std::array<CommandNotifyHandler, notifiersMaxCount>, static_cast<size_t>(CommandId::Commands)>
            _commandsNotifiers; // Commands notifiers
    };
} // namespace Serials
