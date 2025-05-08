/**
 * @file main.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Main file with setup and loop entry points
 * @version 0.1
 * @date 2024-04-05
 *
 * @copyright Copyright (c) 2024
 *
 */

// Lib headers
#include <Log.hpp>
#include <Settings.hpp>
#include <SystemTime.hpp>

// Source headers
#include "Battery.hpp"
#include "Board.h"
#include "FileLoader.hpp"
#include "FileSD.hpp"
#include "FwVersion.hpp"
#include "Measurements/MeasureManager.h"
#include "Serial/SerialManager.hpp"

/**
 * @brief Register serial read command handlers
 */
void registerSerialReadHandlers()
{
    static char dataString[Serials::SerialDevice::dataMaxLength];

    LOG_TRACE("Register serial read common handlers");

    Serials::Manager::subscribeToRead(Serials::CommandId::Date,
                                      [](const char **responseString)
                                      {
                                          assert(sizeof(dataString) >= sizeof(SystemTime::DateString));

                                          SystemTime::getStringDate(dataString);

                                          *responseString = dataString;
                                      });

    Serials::Manager::subscribeToRead(Serials::CommandId::Time,
                                      [](const char **responseString)
                                      {
                                          assert(sizeof(dataString) >= sizeof(SystemTime::TimeString));

                                          SystemTime::getStringTime(dataString);

                                          *responseString = dataString;
                                      });

    Serials::Manager::subscribeToRead(Serials::CommandId::LogLevel,
                                      [](const char **responseString)
                                      {
                                          uint8_t logLevel = Log::getMaxLevel();
                                          snprintf(dataString, sizeof(dataString), "%u", logLevel);
                                          *responseString = dataString;
                                      });

    Serials::Manager::subscribeToRead(Serials::CommandId::FwVersion,
                                      [](const char **responseString)
                                      {
                                          const char *fwString = FwVersion::getVersionString();
                                          *responseString = fwString;
                                      });

    Serials::Manager::subscribeToRead(Serials::CommandId::BatteryStatus,
                                      [](const char **responseString)
                                      {
                                          const auto &batteryStatus = Battery::readStatus();

                                          snprintf(dataString, sizeof(dataString), "%umV %u%%",
                                                   batteryStatus.voltage, batteryStatus.level);
                                          *responseString = dataString;
                                      });
}

/**
 * @brief Register serial write command handlers
 */
void registerSerialWriteHandlers()
{
    LOG_TRACE("Register serial write common handlers");

    Serials::Manager::subscribeToWrite(Serials::CommandId::Date,
                                       [](const char *dataString)
                                       {
                                           SystemTime::setStringDate(dataString);
                                       });

    Serials::Manager::subscribeToWrite(Serials::CommandId::Time,
                                       [](const char *dataString)
                                       {
                                           SystemTime::setStringTime(dataString);
                                       });

    Serials::Manager::subscribeToWrite(Serials::CommandId::LogLevel,
                                       [](const char *dataString)
                                       {
                                           uint8_t logLevel = atoi(dataString);
                                           Log::setMaxLevel(logLevel);
                                       });
}

/**
 * @brief Setup preliminary stuff before starting the main loop
 */
void setup()
{
    // Setup the board first
    Board::setup();

    LOG_INFO("Application started, version %s", FwVersion::getVersionString());

    // Initialize settings first
    Settings::initialize();

    // Initialize the log module
    Log::initialize();

    // Initialize battery reading
    Battery::initialize();

    // Initialize serial manager
    Serials::Manager::initialize();
    // Register local serial handlers
    registerSerialReadHandlers();
    registerSerialWriteHandlers();

    // Initialize system time with RTC
    bool status = SystemTime::initialize(Wire);
    if (status == false)
    {
        LOG_ERROR("System time initialization failed");
    }

    // Start SD file system
    status = FileSD::startFileSystem(Board::SpiConfig::frequency);
    if (status == false)
    {
        LOG_ERROR("SD initialization failed");
    }

    status = Measurements::Manager::initialize();
    if (status == false)
    {
        LOG_ERROR("Measurements initialization failed");
    }

    FileLoader::initialize();

    LOG_INFO("Setup done");
}

/**
 * @brief The main loop function
 */
void loop()
{
    // Receive and handle serial commands from serial devices (if available)
    Serials::Manager::process();

    // Perform sensor input data processing (if needed)
    Measurements::Manager::process();
}
