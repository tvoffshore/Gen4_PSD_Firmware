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
#include "Board/Battery.hpp"
#include "Board/Board.hpp"
#include "Board/Power.hpp"
#include "FwVersion.hpp"
#include "Measurements/MeasureManager.h"
#include "Sd/File.hpp"
#include "Sd/FileLoader.hpp"
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

                                          SystemTime::DateString dateString;
                                          SystemTime::getStringDate(dateString);

                                          memcpy(dataString, dateString, sizeof(SystemTime::DateString));
                                          *responseString = dataString;
                                      });

    Serials::Manager::subscribeToRead(Serials::CommandId::Time,
                                      [](const char **responseString)
                                      {
                                          assert(sizeof(dataString) >= sizeof(SystemTime::TimeString));

                                          SystemTime::TimeString timeString;
                                          SystemTime::getStringTime(timeString);

                                          memcpy(dataString, timeString, sizeof(SystemTime::TimeString));
                                          *responseString = dataString;
                                      });

    Serials::Manager::subscribeToRead(Serials::CommandId::WakeUpSource,
                                      [](const char **responseString)
                                      {
                                          uint8_t wakeUpSources = Power::wakeUpSources();
                                          snprintf(dataString, sizeof(dataString), "%u", wakeUpSources);
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
                                           SystemTime::DateString dateString;
                                           memcpy(dateString, dataString, sizeof(SystemTime::DateString));
                                           SystemTime::setStringDate(dateString);
                                       });

    Serials::Manager::subscribeToWrite(Serials::CommandId::Time,
                                       [](const char *dataString)
                                       {
                                           SystemTime::TimeString timeString;
                                           memcpy(timeString, dataString, sizeof(SystemTime::TimeString));
                                           SystemTime::setStringTime(timeString);
                                       });

    Serials::Manager::subscribeToWrite(Serials::CommandId::WakeUpSource,
                                       [](const char *dataString)
                                       {
                                           uint8_t wakeUpSources = atoi(dataString);
                                           Power::setWakeUpSources(wakeUpSources);
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

    Power::WakeUpReason wakeUpReason = Power::getWakeUpReason();
    LOG_INFO("Wake up reason: %s", Power::wakeUpReasonToString(wakeUpReason));

    LOG_INFO("Application started, version %s", FwVersion::getVersionString());

    // Initialize settings
    Settings::initialize();

    // Initialize the log module
    Log::initialize();

    // Initialize Power module
    Power::initialize();
    // Set CPU frequency to the minimum possible
    Power::setCpuFrequency(Power::cpuFrequencyMinMHz);

    // Initialize battery reading
    Battery::initialize();
    auto batteryStatus = Battery::readStatus();
    LOG_INFO("Battery voltage %umV, level %u%%", batteryStatus.voltage, batteryStatus.level);

    // Initialize serial manager
    Serials::Manager::initialize();
    // Register local serial handlers
    registerSerialReadHandlers();
    registerSerialWriteHandlers();

    // Power up the board
    Board::powerUp();

    // Initialize system time with RTC
    bool status = SystemTime::initialize(Wire);
    if (status == false)
    {
        LOG_ERROR("System time initialization failed");
    }

    // Start SD file system
    status = SD::FS::start(Board::SpiConfig::pinCsSd, Board::SpiConfig::frequency);
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
