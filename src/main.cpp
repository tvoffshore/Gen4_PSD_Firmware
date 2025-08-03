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

#include <stdbool.h>
#include <stdint.h>

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

namespace
{
    // External application connection flag (true - connected)
    bool isAppConnected = false;
    // Sensors' measurements activity flag (true - active)
    bool isMeasurementsActive = false;

    // The last application keep alive command receive time, milliseconds
    int appKeepAliveRxTimeMs = 0;

    // Application keep alive timeout to decide it is no longer connected, milliseconds
    constexpr int appKeepAliveTimeoutMs = 5 * 1000;
} // namespace

/**
 * @brief Start sensors' measurements
 */
void startMeasurements()
{
    if (isMeasurementsActive == false)
    {
        LOG_INFO("Start sensors' measurements");

        bool result = Measurements::Manager::start();
        if (result == true)
        {
            isMeasurementsActive = true;
        }
        else
        {
            LOG_ERROR("Measurements start failed");
        }
    }
}

/**
 * @brief Stop sensors' measurements
 */
void stopMeasurements()
{
    if (isMeasurementsActive == true)
    {
        LOG_INFO("Stop sensors' measurements");

        bool result = Measurements::Manager::stop();
        if (result == true)
        {
            isMeasurementsActive = false;
        }
        else
        {
            LOG_ERROR("Measurements stop failed");
        }
    }
}

/**
 * @brief Handle application keep alive commands according to internal states
 */
void appKeepAliveHandler()
{
    if (isAppConnected == false)
    {
        isAppConnected = true;

        LOG_INFO("Application connected");

        // Stop sensors' measurements
        stopMeasurements();
    }

    // Update the last application keep alive command receive time
    appKeepAliveRxTimeMs = millis();
}

/**
 * @brief Perform application connected state checking
 */
void appKeepAliveProcess()
{
    int expiredMs = millis() - appKeepAliveRxTimeMs;
    if (expiredMs > appKeepAliveTimeoutMs)
    {
        isAppConnected = false;

        LOG_INFO("Application disconnected");

        // Start sensors' measurements
        startMeasurements();
    }
}

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
 * @brief Register to serial commands notifictions
 */
void registerSerialCommandNotifications()
{
    LOG_DEBUG("Register serial command common notifications");

    Serials::Manager::subscribeToNotify(Serials::CommandId::AppKeepAlive,
                                        [](Serials::CommandType commandType)
                                        {
                                            appKeepAliveHandler();
                                        });

    Serials::Manager::subscribeToNotify(Serials::CommandId::AppDownloadRecent,
                                        [](Serials::CommandType commandType)
                                        {
                                            appKeepAliveHandler();
                                        });

    Serials::Manager::subscribeToNotify(Serials::CommandId::AppDownloadHistory,
                                        [](Serials::CommandType commandType)
                                        {
                                            appKeepAliveHandler();
                                        });

    Serials::Manager::subscribeToNotify(Serials::CommandId::AppDownloadType,
                                        [](Serials::CommandType commandType)
                                        {
                                            appKeepAliveHandler();
                                        });

    Serials::Manager::subscribeToNotify(Serials::CommandId::AppDownloadId,
                                        [](Serials::CommandType commandType)
                                        {
                                            appKeepAliveHandler();
                                        });

    Serials::Manager::subscribeToNotify(Serials::CommandId::AppDownloadSize,
                                        [](Serials::CommandType commandType)
                                        {
                                            appKeepAliveHandler();
                                        });

    Serials::Manager::subscribeToNotify(Serials::CommandId::AppDownloadData,
                                        [](Serials::CommandType commandType)
                                        {
                                            appKeepAliveHandler();
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
    registerSerialCommandNotifications();

    // Power up the board
    Board::powerUp();

    // Initialize system time with RTC (after the board power-up and I2C pull-ups activation)
    SystemTime::initialize(Wire);

    // Start SD file system
    bool status = SD::FS::start(Board::SpiConfig::pinCsSd, Board::SpiConfig::frequency);
    if (status == false)
    {
        LOG_ERROR("SD initialization failed");
    }

    FileLoader::initialize();

    status = Measurements::Manager::initialize();
    if (status == true)
    {
        // Start sensors' measurements
        startMeasurements();
    }
    else
    {
        LOG_ERROR("Measurements initialization failed");
    }

    int coreID = xPortGetCoreID();
    LOG_DEBUG("Main task start on core #%d", coreID);
}

/**
 * @brief The main loop function
 */
void loop()
{
    // Receive and handle serial commands from serial devices (if available)
    Serials::Manager::process();

    if (isAppConnected == true)
    {
        // Perform application connected state checking
        appKeepAliveProcess();
    }

    if (isMeasurementsActive == true)
    {
        // Perform sensor input data processing (if needed)
        Measurements::Manager::process();
    }
}
