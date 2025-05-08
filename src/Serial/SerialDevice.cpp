#include "Serial/SerialDevice.hpp"

#include <Log.hpp>

using namespace Serials;

// Comment it to disable printing start string to the serial port
#define PRINT_START_STRING_TO_SERIAL

namespace
{
    constexpr size_t addressLength = 3;    // Length of slave address field
    constexpr size_t commandMaxLength = 4; // Maximum length of command identifier field
    constexpr size_t printMaxLength = 100; // Maximum length of print string

    constexpr char startMsgChar = '!';    // Character is used to signal the start of a new message
    constexpr char separatorChar = ':';   // Separator for extra check on message validity
    constexpr char accessReadChar = '?';  // Read data from slave access code
    constexpr char accessWriteChar = '='; // Write data to slave access code
    constexpr char endOfMsgChar = '\r';   // Indicate the end of the message

    constexpr const char *ackString = "\r";   // Acknowledge string (to accept host command)
    constexpr const char *nackString = "?\r"; // Not acknowledge string (to reject host command)

    constexpr unsigned long comingMaxTimeMs = 500; // Maximum time to wait for the rest of the message coming

}; // namespace

/**
 * @brief Construct a new Serial Device object
 *
 * @param serialInterface Reference to Serial interface object
 */
SerialDevice::SerialDevice(SerialInterface &serialInterface)
    : _serialInterface(serialInterface)
{
}

/**
 * @brief Initialize Serial Device object
 *
 * @param readHandler The read serial command handler
 * @param writeHandler The write serial command handler
 * @param executeHandler The execute serial command handler
 */
void SerialDevice::initialize(ReadHandler &&readHandler, WriteHandler &&writeHandler, ExecuteHandler &&executeHandler)
{
    assert(readHandler);
    assert(writeHandler);
    assert(executeHandler);

    LOG_DEBUG("Initialize serial device...");

    _readHandler = std::move(readHandler);
    _writeHandler = std::move(writeHandler);
    _executeHandler = std::move(executeHandler);

    _serialInterface.initialize();

    LOG_INFO("Serial device initialized, interface: %s", _serialInterface.getInfo());
}

/**
 * @brief Start serial device
 */
void SerialDevice::start()
{
    if (_isActive == false)
    {
        _isActive = true;
        _serialInterface.start();

#ifdef PRINT_START_STRING_TO_SERIAL
        print("Serial interface started: %s", _serialInterface.getInfo());
#endif // PRINT_START_MSG_TO_SERIAL

        LOG_INFO("Serial device started, interface: %s", _serialInterface.getInfo());

        setState(State::Start);
    }
    else
    {
        LOG_WARNING("Serial device has already been started, interface: %s", _serialInterface.getInfo());
    }
}

/**
 * @brief Stop serial device
 */
void SerialDevice::stop()
{
    if (_isActive == true)
    {
        _serialInterface.stop();
        _isActive = false;

        LOG_INFO("Serial device is suspended, interface: %s", _serialInterface.getInfo());
    }
    else
    {
        LOG_WARNING("Serial device has already been suspended, interface: %s", _serialInterface.getInfo());
    }
}

/**
 * @brief Print formatted string to serial interface
 *
 * @param format Format string
 * @param ... Arguments
 * @return true if printing succeed, false otherwise
 */
bool SerialDevice::print(const char *format, ...)
{
    // String with data to write to the serial device
    static char printString[printMaxLength] = {0};
    va_list arp;

    va_start(arp, format);
    // Left 1 symbol for \n
    vsnprintf(printString, sizeof(printString) - 1, format, arp);
    va_end(arp);

    strcat(printString, "\n");
    size_t printLength = strlen(printString);

    size_t writtenSize = _serialInterface.write(printString, printLength);
    return (writtenSize == printLength);
}

/**
 * @brief Write binary data to serial interface
 *
 * @param data Data buffer
 * @param size Size of buffer, bytes
 * @return true if writing succeed, false otherwise
 */
bool SerialDevice::write(const char *data, size_t size)
{
    size_t writtenSize = _serialInterface.write(data, size);
    return (writtenSize == size);
}

/**
 * @brief Serial device input data process
 *
 * @return Status of message input
 */
SerialDevice::InputStatus SerialDevice::process()
{
    InputStatus inputStatus = InputStatus::Continue;

    if (_isActive == true)
    {
        // Try to get new character from serial input
        while (_serialInterface.available() && inputStatus == InputStatus::Continue)
        {
            _elapsed = 0;

            char inputChar = _serialInterface.read();

            inputStatus = parseInput(inputChar);
        }

        // Message parsing failed
        if (inputStatus == InputStatus::Failed)
        {
            sendNack();
        }

        // Check incoming message receiving timeout
        if (_state != State::Start && _elapsed > comingMaxTimeMs)
        {
            // Reset state to Start by timeout
            setState(State::Start);
        }
    }

    return inputStatus;
}

/**
 * @brief Get current slave address of the device
 *
 * @return Slave address
 */
int SerialDevice::slaveAddress()
{
    return _slaveAddress;
}

/**
 * @brief Set the new slave address of the device
 *
 * @param address New address to set
 */
void SerialDevice::setSlaveAddress(int address)
{
    _slaveAddress = address;
}

/**
 * @brief Add new character to input string
 *
 * @param newChar New character to add
 * @param maxLength Maximum length of string
 * @return True if character successfully added to string, false otherwise
 */
bool SerialDevice::addToInput(char newChar, size_t maxLength)
{
    if (_inputOffset < maxLength)
    {
        _inputString[_inputOffset] = newChar;
        // Add zero after last character
        _inputString[_inputOffset + 1] = 0;

        _inputOffset++;

        return true;
    }

    return false;
}

/**
 * @brief Set new state of finite machine and reset input string offset value
 *
 * @param newState New state to set
 */
void SerialDevice::setState(State newState)
{
    _state = newState;

    // Reset input string
    _inputOffset = 0;
    _inputString[_inputOffset] = 0;
}

/**
 * @brief Recognise command identifier from input string
 *
 * @param accessMask Access bitmask of new command
 * @return True if command is recognised and exists in commands list, false otherwise
 */
bool SerialDevice::recogniseCommandId(uint8_t accessMask)
{
    for (size_t id = 0; id < sizeof(commandsList) / sizeof(*commandsList); id++)
    {
        const Command *pCommand = &commandsList[id];

        if (strcmp(_inputString, pCommand->string) == 0 && (accessMask & pCommand->accessMask))
        {
            _commandId = pCommand->id;

            LOG_DEBUG("%s: new command is recognized: %s", _serialInterface.getInfo(), pCommand->string);

            // Command identifier found
            return true;
        }
    }

    // Command identifier not found
    return false;
}

/**
 * @brief Receive slave address from serial input
 *
 * @param inputChar New received character
 * @return Status of address input
 */
SerialDevice::InputStatus SerialDevice::receiveAddress(char inputChar)
{
    if (inputChar == separatorChar)
    {
        // Address must consist of "addressLength" count of digits
        if (_inputOffset != addressLength)
        {
            return InputStatus::Failed;
        }

        // Try to recognize address number from input string
        int addr = invalidAddress;
        int recognized = sscanf(_inputString, "%d", &addr);

        // Check if address is recognized and corresponds to a slave or broadcast address
        if (recognized == 1 && (addr == _slaveAddress || addr == broadcastAddress))
        {
            _isBroadcastMessage = (addr == broadcastAddress) ? true : false;

            return InputStatus::Complete;
        }
        else
        {
            return InputStatus::Failed;
        }
    }
    else
    {
        // Address string accumulation
        bool result = addToInput(inputChar, addressLength);
        if (result == false)
        {
            return InputStatus::Failed;
        }
    }

    return InputStatus::Continue;
}

/**
 * @brief Receive command identifier from serial input
 *
 * @param inputChar New received character
 * @return Status of command identifier input
 */
SerialDevice::InputStatus SerialDevice::receiveCommand(char inputChar)
{
    uint8_t accessMask = AccessMask::none;

    // Determine if an access code has been received
    if (inputChar == accessReadChar)
    {
        accessMask = AccessMask::read;
    }
    else if (inputChar == accessWriteChar)
    {
        accessMask = AccessMask::write;
    }
    else if (inputChar == endOfMsgChar)
    {
        accessMask = AccessMask::execute;
    }

    if (accessMask != AccessMask::none)
    {
        bool result = recogniseCommandId(accessMask);
        if (result == true)
        {
            return InputStatus::Complete;
        }
        else
        {
            return InputStatus::Failed;
        }
    }
    else
    {
        // Command string accumulation
        bool result = addToInput(inputChar, commandMaxLength);
        if (result == false)
        {
            return InputStatus::Failed;
        }
    }

    return InputStatus::Continue;
}

/**
 * @brief Receive data from serial input
 *
 * @param inputChar New received character
 * @return Status of data input
 */
SerialDevice::InputStatus SerialDevice::receiveData(char inputChar)
{
    if (inputChar == endOfMsgChar)
    {
        return InputStatus::Complete;
    }
    else
    {
        // Command string accumulation
        bool result = addToInput(inputChar, dataMaxLength);
        if (result == false)
        {
            return InputStatus::Failed;
        }
    }

    return InputStatus::Continue;
}

/**
 * @brief Parse new input character received from serial interface
 *
 * @param inputChar New input character
 * @return Status of message input
 */
SerialDevice::InputStatus SerialDevice::parseInput(char inputChar)
{
    InputStatus parseStatus = InputStatus::Continue;

    switch (_state)
    {
    case State::Start:
        if (inputChar == startMsgChar)
        {
            setState(State::Address);
        }
        break;
    case State::Address:
    {
        InputStatus status = receiveAddress(inputChar);
        if (status == InputStatus::Complete)
        {
            setState(State::Command);
        }
        else if (status == InputStatus::Failed)
        {
            setState(State::Start);
        }
    }
    break;
    case State::Command:
    {
        InputStatus status = receiveCommand(inputChar);
        if (status == InputStatus::Complete)
        {
            // Execute command message ends with an endOfMsgChar character instead of access code
            if (inputChar == endOfMsgChar)
            {
                bool result = handleExecuteCommand();
                parseStatus = (result == true) ? InputStatus::Complete : InputStatus::Failed;

                setState(State::Start);
            }
            // Write command message should contain data field after access code
            else if (inputChar == accessWriteChar)
            {
                setState(State::Data);
            }
            // Read command message should contain endOfMsgChar character after access code
            else
            {
                setState(State::End);
            }
        }
        else if (status == InputStatus::Failed)
        {
            // Command identifier not recognised
            parseStatus = InputStatus::Failed;

            setState(State::Start);
        }
    }
    break;
    case State::Data:
    {
        InputStatus status = receiveData(inputChar);
        if (status == InputStatus::Complete)
        {
            bool result = handleWriteCommand();
            parseStatus = (result == true) ? InputStatus::Complete : InputStatus::Failed;

            setState(State::Start);
        }
        else if (status == InputStatus::Failed)
        {
            parseStatus = InputStatus::Failed;

            setState(State::Start);
        }
    }
    break;
    case State::End:
    {
        if (inputChar == endOfMsgChar)
        {
            bool result = handleReadCommand();
            parseStatus = (result == true) ? InputStatus::Complete : InputStatus::Failed;
        }
        else
        {
            parseStatus = InputStatus::Failed;
        }

        setState(State::Start);
    }
    break;
    default:
        break;
    }

    return parseStatus;
}

/**
 * @brief Send not acknowledge response to the host
 */
inline void SerialDevice::sendNack()
{
    // Don't send NACK to broadcast messages
    if (_isBroadcastMessage == false)
    {
        LOG_TRACE("%s: send NACK", _serialInterface.getInfo());

        // Send not acknowledge response to the host
        _serialInterface.write(nackString, strlen(nackString));
    }
}

/**
 * @brief Send not acknowledge response to the host
 *
 * @param data Data string for response or null pointer (default) if nothing to send
 */
inline void SerialDevice::sendAck(const char *dataString)
{
    // Don't send ACK to broadcast messages with one exception: request of the slave address
    if (_isBroadcastMessage == false || (_commandId == CommandId::SlaveAddress && dataString != nullptr))
    {
        // Flush _outputString length
        _outputString[0] = '\0';

        // Copy data if it exists at first
        if (dataString != nullptr)
        {
            size_t copyMaxLength = dataMaxLength - strlen(ackString);
            strncpy(_outputString, dataString, copyMaxLength);

            /* No null-character is implicitly appended by strncpy
             * at the end of _outputString if dataString is longer than copyMaxLength */
            if (strlen(dataString) > copyMaxLength)
            {
                LOG_WARNING("%s: response data string is cropped from %d to %d symbols", _serialInterface.getInfo(),
                            strlen(dataString), copyMaxLength);

                // Append null-character at the end for this case
                _outputString[copyMaxLength] = '\0';
            }
        }

        LOG_TRACE("%s: send ACK, response data length %d", _serialInterface.getInfo(), strlen(_outputString));

        // Append ACK string at the end
        strcat(_outputString, ackString);

        // Send output string to the serial
        _serialInterface.write(_outputString, strlen(_outputString));
    }
}

/**
 * @brief Handle read command message. Pending response with read data + acknowledge
 *
 * @return True if handling succeed, false otherwise
 */
inline bool SerialDevice::handleReadCommand()
{
    bool result = false;

    if (_readHandler != nullptr)
    {
        const char *responseString = nullptr;

        LOG_DEBUG("%s: handle read command, id %d", _serialInterface.getInfo(), _commandId);

        // Read response string
        result = _readHandler(this, _commandId, &responseString);

        if (responseString != nullptr)
        {
            LOG_DEBUG("%s: read command response: %s", _serialInterface.getInfo(), responseString);
        }

        if (result == true)
        {
            // Send acknowledge response
            sendAck(responseString);
        }
        else
        {
            LOG_WARNING("%s: not supported command, id %d", _serialInterface.getInfo(), _commandId);
        }
    }

    return result;
}

/**
 * @brief Handle write command message. Pending response with acknowledge
 *
 * @return True if handling succeed, false otherwise
 */
inline bool SerialDevice::handleWriteCommand()
{
    bool result = false;

    if (_writeHandler != nullptr)
    {
        LOG_DEBUG("%s: handle write command, id %d, data: %s", _serialInterface.getInfo(), _commandId, _inputString);

        result = _writeHandler(this, _commandId, _inputString);

        if (result == true)
        {
            // Send acknowledge response
            sendAck();
        }
        else
        {
            LOG_WARNING("%s: not supported command, id %d", _serialInterface.getInfo(), _commandId);
        }
    }

    return result;
}

/**
 * @brief Handle execute command message. Pending response with acknowledge
 *
 * @return True if handling succeed, false otherwise
 */
inline bool SerialDevice::handleExecuteCommand()
{
    bool result = false;

    if (_executeHandler != nullptr)
    {
        LOG_DEBUG("%s: handle execute command, id %d", _serialInterface.getInfo(), _commandId);

        result = _executeHandler(this, _commandId);

        if (result == true)
        {
            // Send acknowledge response
            sendAck();
        }
        else
        {
            LOG_WARNING("%s: not supported command, id %d", _serialInterface.getInfo(), _commandId);
        }
    }

    return result;
}
