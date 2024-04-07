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

    // Default serial device slave address
    constexpr int defaultSlaveAddress = 123;
    // Default serial interface selection
    constexpr uint8_t defaultSerialSelect = static_cast<uint8_t>(SerialManager::SerialInterface::RS232);
    // Settings identifier in internal storage
    constexpr auto settingsId = SettingsModules::SerialManager;
} // namespace

/**
 * @brief Construct a new Serial Manager object
 */
SerialManager::SerialManager()
    : _settings{
          .slaveAddress = defaultSlaveAddress,
          .serialSelect = defaultSerialSelect,
      }
{
}

/**
 * @brief Initialize serial manager
 *
 * @return True if initialization succeeds, false otherwise
 */
bool SerialManager::initialize()
{
    // Reset read handlers
    _readHandlers.fill(nullptr);

    // Reset write handlers
    _writeHandlers.fill(nullptr);

    // Reset commands notifiers
    for (auto &notifiers : _commandsNotifiers)
    {
        notifiers.fill(nullptr);
    }

    InternalStorage::readSettings(settingsId, _settings);

    // Initialize serial devices
    for (auto &device : serialDevices)
    {
        // Setup read/write/execute handlers
        device.initialize(
            [this](SerialDevice *device, CommandId commandId, const char **responseString)
            {
                pCommandSourceDevice = device;
                bool result = readHandler(device, commandId, responseString);
                pCommandSourceDevice = nullptr;

                return result;
            },
            [this](SerialDevice *device, CommandId commandId, const char *dataString)
            {
                pCommandSourceDevice = device;
                bool result = writeHandler(device, commandId, dataString);
                pCommandSourceDevice = nullptr;

                return result;
            },
            [this](SerialDevice *device, CommandId commandId)
            {
                pCommandSourceDevice = device;
                bool result = executeHandler(device, commandId);
                pCommandSourceDevice = nullptr;

                return result;
            });

        device.setSlaveAddress(_settings.slaveAddress);
    }

    // Start USB serial device
    serialDevices[static_cast<size_t>(SerialDeviceId::UsbSerial)].start();

    // Activate selected serial interface
    selectSerialInterface(_settings.serialSelect);

    registerSerialReadHandlers();
    registerSerialWriteHandlers();

    return true;
}

/**
 * @brief Perform serial devices input data processing
 */
void SerialManager::process()
{
    for (auto &device : serialDevices)
    {
        // Receive and handle serial commands
        device.process();
    }
}

/**
 * @brief Register handlers for read serial commands
 */
void SerialManager::registerSerialReadHandlers()
{
    static char dataString[Serials::SerialDevice::dataMaxLength];

    // Subscribe to provide serial devices address
    subscribeToRead(CommandId::SlaveAddress,
                    [this](const char **responseString)
                    {
                        snprintf(dataString, sizeof(dataString), "%d", _settings.slaveAddress);

                        *responseString = dataString;
                    });

    // Subscribe to provide serial interface selection
    subscribeToRead(CommandId::SerialSelect,
                    [this](const char **responseString)
                    {
                        snprintf(dataString, sizeof(dataString), "%d", _settings.serialSelect);

                        *responseString = dataString;
                    });
}

/**
 * @brief Register handlers for write serial commands
 */
void SerialManager::registerSerialWriteHandlers()
{
    // Subscribe to handle write serial devices address
    subscribeToWrite(CommandId::SlaveAddress,
                     [this](const char *dataString)
                     {
                         int address = atoi(dataString);

                         if (address > SerialDevice::broadcastAddress && address <= SerialDevice::maxAddress &&
                             address != _settings.slaveAddress)
                         {
                             LOG_INFO("Set new address: %d", address);

                             _settings.slaveAddress = address;
                             InternalStorage::updateSettings(settingsId, _settings);

                             for (auto &device : serialDevices)
                             {
                                 device.setSlaveAddress(address);
                             }
                         }
                     });

    // Subscribe to handle write serial interface selection
    subscribeToWrite(CommandId::SerialSelect,
                     [this](const char *dataString)
                     {
                         int serialSelect = atoi(dataString);

                         selectSerialInterface(serialSelect);
                     });
}

/**
 * @brief Subscribe to specified read command to provide read data
 * May be only one subscriber that provides read data
 *
 * @param commandId Command identifier
 * @param handler Handler function
 */
void SerialManager::subscribeToRead(CommandId commandId, ReadCommandHandler &&handler)
{
    size_t id = static_cast<size_t>(commandId);

    assert(id < _readHandlers.size());
    assert(handler);
    assert(commandsList[id].accessMask & AccessMask::read);

    auto &readHandler = _readHandlers[id];
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
void SerialManager::subscribeToWrite(CommandId commandId, WriteCommandHandler &&handler)
{
    size_t id = static_cast<size_t>(commandId);

    assert(id < _writeHandlers.size());
    assert(handler);
    assert(commandsList[id].accessMask & AccessMask::write);

    auto &writeHandler = _writeHandlers[id];
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
void SerialManager::subscribeToNotify(CommandId commandId, CommandNotifyHandler &&handler)
{
    size_t id = static_cast<size_t>(commandId);

    assert(id < _commandsNotifiers.size());
    assert(handler);

    size_t notifierNumber = 0;
    auto &notifiers = _commandsNotifiers[id];

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
SerialDevice *SerialManager::getCommandSourceDevice()
{
    return pCommandSourceDevice;
}

/**
 * @brief Switch between selected serial interfaces
 *
 * @param serialSelect Serial interface selection
 */
void SerialManager::selectSerialInterface(uint8_t serialSelect)
{
    if (serialSelect >= static_cast<uint8_t>(SerialInterface::Count))
    {
        serialSelect = static_cast<uint8_t>(SerialInterface::RS232);
    }

    if (serialSelect != _settings.serialSelect)
    {
        _settings.serialSelect = serialSelect;
        InternalStorage::updateSettings(settingsId, _settings);
    }

    if (serialSelect == static_cast<uint8_t>(SerialInterface::RS232))
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
bool SerialManager::readHandler(SerialDevice *device, CommandId commandId, const char **responseString)
{
    bool result = false;
    size_t id = static_cast<size_t>(commandId);

    LOG_INFO("Read command received, ID: %d", id);

    if (id < _readHandlers.size())
    {
        auto &handler = _readHandlers[id];

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
bool SerialManager::writeHandler(SerialDevice *device, CommandId commandId, const char *dataString)
{
    bool result = false;
    size_t id = static_cast<size_t>(commandId);

    LOG_INFO("Write command received, ID: %d", id);

    if (id < _writeHandlers.size() && dataString != nullptr)
    {
        auto &handler = _writeHandlers[id];

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
bool SerialManager::executeHandler(SerialDevice *device, CommandId commandId)
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
bool SerialManager::notifyCommand(size_t id, CommandType type)
{
    bool result = false;

    if (id < _commandsNotifiers.size())
    {
        auto &notifiers = _commandsNotifiers[id];

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