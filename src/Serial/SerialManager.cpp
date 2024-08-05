#include "Serial/SerialManager.hpp"

#include <assert.h>
#include <Debug.hpp>

#include "InternalStorage.hpp"
#include "Serial/SerialCommands.hpp"
#include "Serial/SerialDevice.hpp"
#include "Serial/Interfaces/Max3221.hpp"
#include "Serial/Interfaces/SerialMock.hpp"
#include "Serial/Interfaces/St3485.hpp"
#include "Serial/Interfaces/UsbSerial.hpp"

using namespace Serials;

namespace
{
    // Maximum number of notifiers to one serial command
    constexpr size_t notifiersMaxCount = 5;

    // Default serial device slave address
    constexpr int defaultSlaveAddress = 123;
    // Default serial interface selection
    constexpr uint8_t defaultSerialSelect = static_cast<uint8_t>(SerialSelect::RS232);

    // Settings identifier in internal storage
    constexpr auto settingsId = SettingsModules::SerialManager;

    /**
     * @brief Serial device identifiers
     */
    enum class SerialDeviceId
    {
        Rs232,     // RS232 device identifier
        Rs485,     // RS485 device identifier
        UsbSerial, // USB serial device identifier

        Count // Total number of device identifiers
    };

#pragma pack(push, 1)
    /**
     * @brief Non volatile settings structure
     */
    struct Settings
    {
        int slaveAddress;     // Slave address of the serial devices
        uint8_t serialSelect; // Serial interface selection @ref SerialSelect
    };
#pragma pack(pop)

    // MAX3221 RS232 transceiver
    Max3221 max3221;
    // ST3485 RS485 transceiver
    St3485 st3485;
    // USB serial transceiver
    UsbSerial usbSerial;

    // Serial devices
    std::array<SerialDevice, static_cast<size_t>(SerialDeviceId::Count)> serialDevices = {
        SerialDevice(max3221),   // RS232
        SerialDevice(st3485),    // RS485
        SerialDevice(usbSerial), // USB serial
    };

    // Current serial command source device
    SerialDevice *pCommandSourceDevice = nullptr;

    std::array<ReadCommandHandler, static_cast<size_t>(CommandId::Commands)> readHandlers;   // Read handlers
    std::array<WriteCommandHandler, static_cast<size_t>(CommandId::Commands)> writeHandlers; // Write handlers

    std::array<std::array<CommandNotifyHandler, notifiersMaxCount>, static_cast<size_t>(CommandId::Commands)> commandsNotifiers; // Commands notifiers

    // Serial manager settings
    Settings settings = {.slaveAddress = defaultSlaveAddress, .serialSelect = defaultSerialSelect};

    // Function prototypes
    void selectSerialInterface(uint8_t serialSelect);
    bool readHandler(SerialDevice *device, CommandId commandId, const char **responseString);
    bool writeHandler(SerialDevice *device, CommandId commandId, const char *dataString);
    bool executeHandler(SerialDevice *device, CommandId commandId);
    bool notifyCommand(size_t id, CommandType type);
    void registerSerialReadHandlers();
    void registerSerialWriteHandlers();

    /**
     * @brief Switch between selected serial interfaces
     *
     * @param serialSelect Serial interface selection
     */
    void selectSerialInterface(uint8_t serialSelect)
    {
        if (serialSelect >= static_cast<uint8_t>(SerialSelect::Count))
        {
            serialSelect = static_cast<uint8_t>(SerialSelect::RS232);
        }

        if (serialSelect != settings.serialSelect)
        {
            settings.serialSelect = serialSelect;
            InternalStorage::updateSettings(settingsId, settings);
        }

        if (serialSelect == static_cast<uint8_t>(SerialSelect::RS232))
        {
            LOG_INFO("RS232 serial interface selected");
            // Stop RS485 serial device
            serialDevices[static_cast<size_t>(SerialDeviceId::Rs485)].stop();
            // Start RS232 serial device
            serialDevices[static_cast<size_t>(SerialDeviceId::Rs232)].start();
        }
        else
        {
            LOG_INFO("RS485 serial interface selected");
            // Stop RS232 serial device
            serialDevices[static_cast<size_t>(SerialDeviceId::Rs232)].stop();
            // Start RS485 serial device
            serialDevices[static_cast<size_t>(SerialDeviceId::Rs485)].start();
        }
    }

    /**
     * @brief The read serial command handler function type
     *
     * @param device Pointer to the serial device that received the command
     * @param commandId Command identifier
     * @param responseString Response string with reading data
     * @return True if handling successfully, false otherwise
     */
    bool readHandler(SerialDevice *device, CommandId commandId, const char **responseString)
    {
        bool result = false;
        size_t id = static_cast<size_t>(commandId);

        LOG_INFO("Read command received, ID: %d", id);

        if (id < readHandlers.size())
        {
            auto &handler = readHandlers[id];

            // Perform read handler for current command
            if (handler != nullptr)
            {
                handler(responseString);

                result = true;
            }
        }

        notifyCommand(id, CommandType::Read);

        return result;
    }

    /**
     * @brief The write serial command handler function type
     *
     * @param device Pointer to the serial device that received the command
     * @param commandId Command identifier
     * @param dataString String with data to write
     * @return True if handling successfully, false otherwise
     */
    bool writeHandler(SerialDevice *device, CommandId commandId, const char *dataString)
    {
        bool result = false;
        size_t id = static_cast<size_t>(commandId);

        LOG_INFO("Write command received, ID: %d", id);

        if (id < writeHandlers.size() && dataString != nullptr)
        {
            auto &handler = writeHandlers[id];

            // Perform write handler for current command
            if (handler != nullptr)
            {
                handler(dataString);

                result = true;
            }
        }

        notifyCommand(id, CommandType::Write);

        return result;
    }

    /**
     * @brief The execute serial command handler function type
     *
     * @param device Pointer to the serial device that received the command
     * @param commandId Command identifier
     * @return True if handling successfully, false otherwise
     */
    bool executeHandler(SerialDevice *device, CommandId commandId)
    {
        size_t id = static_cast<size_t>(commandId);

        LOG_INFO("Execute command received, ID: %d", id);

        bool result = notifyCommand(id, CommandType::Execute);
        return result;
    }

    /**
     * @brief Notify about command received
     *
     * @param id Command identifier
     * @param type Received command type
     * @return True if at least one notifier exist, false otherwise
     */
    bool notifyCommand(size_t id, CommandType type)
    {
        bool result = false;

        if (id < commandsNotifiers.size())
        {
            auto &notifiers = commandsNotifiers[id];

            // Notify all subscribers about received command
            for (auto &notifier : notifiers)
            {
                if (notifier != nullptr)
                {
                    notifier(type);

                    result = true;
                }
            }
        }

        return result;
    }

    /**
     * @brief Register handlers for read serial commands
     */
    void registerSerialReadHandlers()
    {
        static char dataString[Serials::SerialDevice::dataMaxLength];

        // Subscribe to provide serial devices address
        Manager::subscribeToRead(CommandId::SlaveAddress,
                                 [](const char **responseString)
                                 {
                                     snprintf(dataString, sizeof(dataString), "%d", settings.slaveAddress);

                                     *responseString = dataString;
                                 });

        // Subscribe to provide serial interface selection
        Manager::subscribeToRead(CommandId::SerialSelect,
                                 [](const char **responseString)
                                 {
                                     snprintf(dataString, sizeof(dataString), "%d", settings.serialSelect);

                                     *responseString = dataString;
                                 });
    }

    /**
     * @brief Register handlers for write serial commands
     */
    void registerSerialWriteHandlers()
    {
        // Subscribe to handle write serial devices address
        Manager::subscribeToWrite(CommandId::SlaveAddress,
                                  [](const char *dataString)
                                  {
                                      int address = atoi(dataString);

                                      if (address > SerialDevice::broadcastAddress && address <= SerialDevice::maxAddress &&
                                          address != settings.slaveAddress)
                                      {
                                          LOG_INFO("Set new address: %d", address);

                                          settings.slaveAddress = address;
                                          InternalStorage::updateSettings(settingsId, settings);

                                          for (auto &device : serialDevices)
                                          {
                                              device.setSlaveAddress(address);
                                          }
                                      }
                                  });

        // Subscribe to handle write serial interface selection
        Manager::subscribeToWrite(CommandId::SerialSelect,
                                  [](const char *dataString)
                                  {
                                      int serialSelect = atoi(dataString);

                                      selectSerialInterface(serialSelect);
                                  });
    }
} // namespace

/**
 * @brief Initialize serial manager
 */
void Manager::initialize()
{
    // Reset read handlers
    readHandlers.fill(nullptr);

    // Reset write handlers
    writeHandlers.fill(nullptr);

    // Reset commands notifiers
    for (auto &notifiers : commandsNotifiers)
    {
        notifiers.fill(nullptr);
    }

    InternalStorage::readSettings(settingsId, settings);

    // Initialize serial devices
    for (auto &device : serialDevices)
    {
        // Setup read/write/execute handlers
        device.initialize(
            [](SerialDevice *device, CommandId commandId, const char **responseString)
            {
                pCommandSourceDevice = device;
                bool result = readHandler(device, commandId, responseString);
                pCommandSourceDevice = nullptr;

                return result;
            },
            [](SerialDevice *device, CommandId commandId, const char *dataString)
            {
                pCommandSourceDevice = device;
                bool result = writeHandler(device, commandId, dataString);
                pCommandSourceDevice = nullptr;

                return result;
            },
            [](SerialDevice *device, CommandId commandId)
            {
                pCommandSourceDevice = device;
                bool result = executeHandler(device, commandId);
                pCommandSourceDevice = nullptr;

                return result;
            });

        device.setSlaveAddress(settings.slaveAddress);
    }

    // Start USB serial device
    serialDevices[static_cast<size_t>(SerialDeviceId::UsbSerial)].start();

    // Activate selected serial interface
    selectSerialInterface(settings.serialSelect);

    // Register local serial handlers
    registerSerialReadHandlers();
    registerSerialWriteHandlers();
}

/**
 * @brief Perform serial devices input data processing
 */
void Manager::process()
{
    for (auto &device : serialDevices)
    {
        // Receive and handle serial commands
        device.process();
    }
}

/**
 * @brief Subscribe to specified read command to provide read data
 * May be only one subscriber that provides read data
 *
 * @param commandId Command identifier
 * @param handler Handler function
 */
void Manager::subscribeToRead(CommandId commandId, ReadCommandHandler &&handler)
{
    size_t id = static_cast<size_t>(commandId);

    assert(id < readHandlers.size());
    assert(handler);
    assert(commandsList[id].accessMask & AccessMask::read);

    auto &readHandler = readHandlers[id];
    if (readHandler == nullptr)
    {
        readHandler = std::move(handler);

        LOG_TRACE("Read handler for command %d is registered", id);
    }
    else
    {
        LOG_WARNING("Read handler for command %d is already registered", id);
    }
}

/**
 * @brief Subscribe to specified write command to handle write data
 * May be only one subscriber that handle write data
 *
 * @param commandId Command identifier
 * @param handler Handler function
 */
void Manager::subscribeToWrite(CommandId commandId, WriteCommandHandler &&handler)
{
    size_t id = static_cast<size_t>(commandId);

    assert(id < writeHandlers.size());
    assert(handler);
    assert(commandsList[id].accessMask & AccessMask::write);

    auto &writeHandler = writeHandlers[id];
    if (writeHandler == nullptr)
    {
        writeHandler = std::move(handler);

        LOG_TRACE("Write handler for command %d is registered", id);
    }
    else
    {
        LOG_WARNING("Write handler for command %d is already registered", id);
    }
}

/**
 * @brief Subscribe to notification about specified command received
 * May be up to subscribersMaxCount subscribers for command notifications
 *
 * @param commandId Command identifier
 * @param handler Handler function
 */
void Manager::subscribeToNotify(CommandId commandId, CommandNotifyHandler &&handler)
{
    size_t id = static_cast<size_t>(commandId);

    assert(id < commandsNotifiers.size());
    assert(handler);

    size_t notifierNumber = 0;
    auto &notifiers = commandsNotifiers[id];

    // Search the free slot for a new notifier
    for (auto &notifier : notifiers)
    {
        if (notifier == nullptr)
        {
            notifier = std::move(handler);

            LOG_TRACE("Notifier %d for command %d is registered", notifierNumber, id);

            break;
        }

        notifierNumber++;
    }

    if (notifierNumber == notifiersMaxCount)
    {
        LOG_WARNING("Max number of notifiers %d for command %d are already registered", notifierNumber, id);
    }
}

/**
 * @brief Get the current serial command source device
 *
 * @return Pointer to serial device
 */
SerialDevice *Manager::getCommandSourceDevice()
{
    return pCommandSourceDevice;
}
