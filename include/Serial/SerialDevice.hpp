#pragma once

#include <stddef.h>
#include <string.h>

#include <elapsedMillis.h>

#include "SerialCommands.hpp"
#include "Interfaces/SerialInterface.hpp"

namespace Serials
{
/**
 * @brief Serial device class.
 * Implements finite state machine to receive and parseInput input data from host and reply to it
 */
class SerialDevice
{
    /**
     * @brief State of the receiving message
     */
    enum class State
    {
        Start,   // Waiting the start of the message
        Address, // Receiving slave address and separator
        Command, // Receiving command identifier and access code
        Data,    // Receiving data (for write commands only)
        End      // Waiting the end of the message
    };

    /**
     * @brief The read serial command handler function type
     */
    using ReadHandler = std::function<bool(SerialDevice *, CommandId, const char **)>;

    /**
     * @brief The write serial command handler function type
     */
    using WriteHandler = std::function<bool(SerialDevice *, CommandId, const char *)>;

    /**
     * @brief The execute serial command handler function type
     */
    using ExecuteHandler = std::function<bool(SerialDevice *, CommandId)>;

public:
    // Maximum length of command data field
    constexpr static size_t dataMaxLength = 100;
    // Invalid slave address
    constexpr static int invalidAddress = -1;
    // Address for broadcast messages
    constexpr static int broadcastAddress = 0;
    // Maximum valid address value
    constexpr static int maxAddress = 999;

    /**
     * @brief Status of input accumulation
     */
    enum class InputStatus
    {
        Failed,   // Input failed for some reason
        Continue, // Input is in progress
        Complete, // Input complete successfully
    };

    /**
     * @brief Construct a new Serial Device object
     *
     * @param serialInterface Reference to Serial interface object
     */
    explicit SerialDevice(SerialInterface &serialInterface);

    /**
     * @brief Initialize Serial Device object
     *
     * @param readHandler The read serial command handler
     * @param writeHandler The write serial command handler
     * @param executeHandler The execute serial command handler
     */
    void initialize(ReadHandler &&readHandler, WriteHandler &&writeHandler, ExecuteHandler &&executeHandler);

    /**
     * @brief Start serial device
     */
    void start();

    /**
     * @brief Stop serial device
     */
    void stop();

    /**
     * @brief Print formatted string to serial interface
     *
     * @param format Format string
     * @param ... Arguments
     * @return true if printing succeed, false otherwise
     */
    bool print(const char *format, ...);

    /**
     * @brief Write binary data to serial interface
     *
     * @param data Data buffer
     * @param size Size of buffer, bytes
     * @return true if writing succeed, false otherwise
     */
    bool write(const char *data, size_t size);

    /**
     * @brief Serial device input data process
     *
     * @return Status of message input
     */
    InputStatus process();

    /**
     * @brief Get current slave address of the device
     *
     * @return Slave address
     */
    int slaveAddress();

    /**
     * @brief Set the new slave address of the device
     *
     * @param address New address to set
     */
    void setSlaveAddress(int address);

private:
    /**
     * @brief Add new character to input string
     *
     * @param newChar New character to add
     * @param maxLength Maximum length of string
     * @return True if character successfully added to string, false otherwise
     */
    bool addToInput(char newChar, size_t maxLength);

    /**
     * @brief Set new state of finite machine and reset input string offset value
     *
     * @param newState New state to set
     */
    void setState(State newState);

    /**
     * @brief Recognise command identifier from input string
     *
     * @param accessMask Access bitmask of new command
     * @return True if command is recognised and exists in commands list, false otherwise
     */
    bool recogniseCommandId(uint8_t accessMask);

    /**
     * @brief Receive slave address from serial input
     *
     * @param inputChar New received character
     * @return Status of address input
     */
    InputStatus receiveAddress(char inputChar);

    /**
     * @brief Receive command identifier from serial input
     *
     * @param inputChar New received character
     * @return Status of command identifier input
     */
    InputStatus receiveCommand(char inputChar);

    /**
     * @brief Receive data from serial input
     *
     * @param inputChar New received character
     * @return Status of data input
     */
    InputStatus receiveData(char inputChar);

    /**
     * @brief Parse new input character received from serial interface
     *
     * @param inputChar New input character
     * @return Status of message input
     */
    InputStatus parseInput(char inputChar);

    /**
     * @brief Send not acknowledge response to the host
     */
    void sendNack();

    /**
     * @brief Send not acknowledge response to the host
     *
     * @param data Data string for response or null pointer (default) if nothing to send
     */
    void sendAck(const char *dataString = nullptr);

    /**
     * @brief Handle read command message. Pending response with read data + acknowledge
     *
     * @return True if handling succeed, false otherwise
     */
    bool handleReadCommand();

    /**
     * @brief Handle write command message. Pending response with acknowledge
     *
     * @return True if handling succeed, false otherwise
     */
    bool handleWriteCommand();

    /**
     * @brief Handle execute command message. Pending response with acknowledge
     *
     * @return True if handling succeed, false otherwise
     */
    bool handleExecuteCommand();

    bool _isActive = false; // true - device is active, false - inactive

    int _slaveAddress = invalidAddress; // Serial device slave address

    SerialInterface &_serialInterface; // Serial interface object

    State _state = State::Start;                    // State of receiving message
    CommandId _commandId = CommandId::SlaveAddress; // Identifier of received command
    bool _isBroadcastMessage = false;               // Flag of received broadcast message

    char _inputString[dataMaxLength + 1] = {0};  // Input string containing received data
    char _outputString[dataMaxLength + 1] = {0}; // Output string containing response data and ACK
    size_t _inputOffset = 0;                     // Offset in input string of the next character to add

    elapsedMillis _elapsed = 0; // Elapsed time from last character receiving from serial port

    ReadHandler _readHandler = nullptr;       // The read serial command handler
    WriteHandler _writeHandler = nullptr;     // The write serial command handler
    ExecuteHandler _executeHandler = nullptr; // The execute serial command handler
};
} // namespace Serials
