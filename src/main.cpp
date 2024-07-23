// System headers
#include <assert.h>
#include <atomic>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Arduino headers
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

// Lib headers
#include <Debug.hpp>
#include <Events.h>
#include <IIM42652.h>
#include <SdFat.h>
#include <SystemTime.hpp>

// Source headers
#include "InternalStorage.hpp"
#include "Power.h"
#include "Psd.h"
#include "Serial/SerialManager.hpp"

// Default time to take measurements, seconds
constexpr uint32_t measureIntervalDefault = 600;
// Default time between measurements, seconds
constexpr uint32_t pauseIntervalDefault = 300;
// Jitter time of measurements interval, seconds
constexpr uint32_t measureIntervalJitter = 5;

// Default sampling frequency, Hz
constexpr uint8_t sampleFrequencyDefault = 20;
// Minimum sampling frequency, Hz
constexpr uint8_t sampleFrequencyMin = 1;
// Maximum sampling frequency, Hz
constexpr uint8_t sampleFrequencyMax = 100;

// Default points to calculate PSD segment size, 2^x
constexpr uint8_t pointsPsdDefault = 8;
// Minimum points to calculate PSD segment size, 2^x
constexpr uint8_t pointsPsdMin = 2;
// Maximum points to calculate PSD segment size, 2^x
constexpr uint8_t pointsPsdMax = 10;

// Default points to store the PSD results
constexpr uint16_t pointsCutoffDefault = 128;
// Minimum points to store the PSD results
constexpr uint16_t pointsCutoffMin = 1;
// Maximum points to store the PSD results
constexpr uint16_t pointsCutoffMax = 1024;

// Milliseconds per second
constexpr size_t millisPerSecond = 1000;
// Settings identifier in internal storage
constexpr auto measureSettingsId = SettingsModules::Measurements;

namespace WirePin
{
    constexpr auto sda = GPIO_NUM_34; // I2C SDA pin
    constexpr auto scl = GPIO_NUM_33; // I2C SCL pin
} // namespace WirePin

namespace SpiPin
{
    constexpr auto sck = GPIO_NUM_1;   // SPI clock pin
    constexpr auto mosi = GPIO_NUM_47; // SPI MOSI pin
    constexpr auto miso = GPIO_NUM_48; // SPI MISO pin
} // namespace SpiPin

namespace SdPin
{
    constexpr auto cs = GPIO_NUM_26; // SD chip select pin
} // namespace SdPin

namespace EventBits
{
    constexpr EventBits_t startImu = BIT0;
    constexpr EventBits_t stopImu = BIT1;
    constexpr EventBits_t imuIdle = BIT2;
    constexpr EventBits_t imuRunning = BIT3;
    constexpr EventBits_t segment0Ready = BIT4;
    constexpr EventBits_t segment1Ready = BIT5;

    constexpr EventBits_t all = startImu | stopImu | imuIdle | imuRunning | segment0Ready | segment1Ready;
} // namespace EventBits

/**
 * @brief IMU sample structure
 */
struct ImuSample
{
    IIM42652_axis_t accel; // IMU accel axises
    IIM42652_axis_t gyro;  // IMU gyro axises
};

#pragma pack(push, 1)
/**
 * @brief Non volatile settings structure for measurements
 */
struct MeasureSettings
{
    uint32_t measureInterval; // Time for measuring, seconds
    uint32_t pauseInterval;   // Time between measurements, seconds
    uint16_t pointsCutoff;    // Points to store the PSD results
    uint8_t frequency;        // Sampling frequency, Hz
    uint8_t pointsPsd;        // Points to calculate PSD segment size, 2^x
};
#pragma pack(pop)

/**
 * Samples buffer structure
 */
struct Buffer
{
    int16_t accX[2 * PSD::samplesCountMax];
    int16_t accY[2 * PSD::samplesCountMax];
    int16_t accZ[2 * PSD::samplesCountMax];

    int16_t gyrX[2 * PSD::samplesCountMax];
    int16_t gyrY[2 * PSD::samplesCountMax];
    int16_t gyrZ[2 * PSD::samplesCountMax];
};

#if (LOG_LEVEL > LOG_LEVEL_NONE)
// Log level declaration
uint8_t LogsOutput::logLevelMax = LOG_LEVEL;
#endif // #if (LOG_LEVEL > LOG_LEVEL_NONE)

// Modules settings EEPROM addresses declaration
std::array<size_t, static_cast<size_t>(SettingsModules::Count)> InternalStorage::settingsAddressList;

// IMU driver object
IIM42652 imu;
// SD file system class
SdFs sd;
// Serial manager
Serials::SerialManager serialManager;

// RTOS event group object
RTOS::EventGroup eventGroup;

// Samples buffer
Buffer buffer = {0};

// Count of ready segments
size_t segmentCount;
// Size of segment, samples
size_t segmentSize;
// Interval between IMU samples, milliseconds
size_t imuIntervalMs;

// PSD for accelerometer axis X
PSD psdAccX;

// Measurements settings
MeasureSettings measureSettings = {
    .measureInterval = measureIntervalDefault,
    .pauseInterval = pauseIntervalDefault,
    .pointsCutoff = pointsCutoffDefault,
    .frequency = sampleFrequencyDefault,
    .pointsPsd = pointsPsdDefault,
};

// Functions prototypes
bool setupSD();
bool setupImu();
bool readImu(ImuSample &imuSample);
void RegisterSerialReadHandlers();
void RegisterSerialWriteHandlers();
void setupBoard();
void setupPSD(uint8_t sampleCount, uint8_t sampleFrequency);
void imuTask(void *pvParameters);

/**
 * @brief Convert seconds to milliseconds
 *
 * @param seconds Time in seconds
 * @return Milliseconds
 */
constexpr size_t secondsToMillis(size_t seconds)
{
    return seconds * 1000;
}

/**
 * @brief Setup SD file system class
 *
 * @return true if SD file system was started successfully, false otherwise
 */
bool setupSD()
{
    // Maximum SPI SCK frequency
    constexpr uint32_t maxFrequency = 20000000; // 20MHz
    // Bytes to megabytes ratio
    constexpr size_t sectorsToMbFactor = 2 * 1024;
    // SD card types
    const char *cardTypeNames[] = {"None", "MMC", "SD", "SDHC/SDXC", "Unknown"};

    // Begin SPI bus
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setFrequency(maxFrequency);
    SPI.begin(SpiPin::sck, SpiPin::miso, SpiPin::mosi);

    LOG_INFO("Start SD file system...");

    bool result = sd.begin(SdPin::cs, maxFrequency);
    if (result == true)
    {
        uint8_t cardType = sd.card()->type();
        if (cardType != 0)
        {
            uint32_t cardSizeMb = sd.card()->sectorCount() / sectorsToMbFactor;
            LOG_INFO("SD card initialized: type %s, size = %dMB", cardTypeNames[cardType], cardSizeMb);
        }
        else
        {
            LOG_ERROR("No SD card attached");
        }
    }
    else
    {
        LOG_ERROR("SD card initialization failed");
    }

    return result;
}

/**
 * @brief Setup IMU sensor
 *
 * @return true if operations succeed, false otherwise
 */
bool setupImu()
{
    // Initialize IMU sensor
    bool result = imu.begin(Wire, 0x68);
    if (result == true)
    {
        result = imu.accelerometer_enable();
        if (result == true)
        {
            result = imu.gyroscope_enable();
        }
    }
    else
    {
        LOG_ERROR("IMU initialization failed");
    }

    return result;
}

/**
 * @brief Read IMU data
 *
 * @param imuSample IMU sample data to read
 * @return true if reading succeed, false otherwise
 */
bool readImu(ImuSample &imuSample)
{
    bool result = imu.get_accel_data(&imuSample.accel);
    if (result == true)
    {
        result = imu.get_gyro_data(&imuSample.gyro);
    }

    return result;
}

/**
 * @brief Start IMU sampling
 */
void startImuTask()
{
    LOG_INFO("Start IMU task");

    // Start IMU sampling
    eventGroup.set(EventBits::startImu);

    // Wait IMU task is idle
    EventBits_t events = eventGroup.wait(EventBits::imuRunning);
    if (!(events & EventBits::imuRunning))
    {
        LOG_ERROR("IMU task start failed");
    }
}

/**
 * @brief Stop IMU sampling
 */
void stopImuTask()
{
    LOG_INFO("Stop IMU task");

    // Stop IMU sampling
    eventGroup.set(EventBits::stopImu);

    // Wait IMU task is idle
    EventBits_t events = eventGroup.wait(EventBits::imuIdle);
    if (!(events & EventBits::imuIdle))
    {
        LOG_ERROR("IMU task stop failed");
    }
}

/**
 * @brief Register serial read command handlers
 */
void RegisterSerialReadHandlers()
{
    static char dataString[Serials::SerialDevice::dataMaxLength];

    LOG_DEBUG("Register serial read handlers");

    serialManager.subscribeToRead(Serials::CommandId::Date,
                                  [](const char **responseString)
                                  {
                                      assert(sizeof(dataString) >= sizeof(SystemTime::DateString));

                                      SystemTime::getStringDate(dataString);

                                      *responseString = dataString;
                                  });

    serialManager.subscribeToRead(Serials::CommandId::Time,
                                  [](const char **responseString)
                                  {
                                      assert(sizeof(dataString) >= sizeof(SystemTime::TimeString));

                                      SystemTime::getStringTime(dataString);

                                      *responseString = dataString;
                                  });

    serialManager.subscribeToRead(Serials::CommandId::MeasureFrequency,
                                  [](const char **responseString)
                                  {
                                      snprintf(dataString, sizeof(dataString), "%u", measureSettings.frequency);

                                      *responseString = dataString;
                                  });

    serialManager.subscribeToRead(Serials::CommandId::MeasureInterval,
                                  [](const char **responseString)
                                  {
                                      snprintf(dataString, sizeof(dataString), "%u", measureSettings.measureInterval);

                                      *responseString = dataString;
                                  });

    serialManager.subscribeToRead(Serials::CommandId::PauseInterval,
                                  [](const char **responseString)
                                  {
                                      snprintf(dataString, sizeof(dataString), "%u", measureSettings.pauseInterval);

                                      *responseString = dataString;
                                  });

    serialManager.subscribeToRead(Serials::CommandId::PointsPsd,
                                  [](const char **responseString)
                                  {
                                      snprintf(dataString, sizeof(dataString), "%u", measureSettings.pointsPsd);

                                      *responseString = dataString;
                                  });

    serialManager.subscribeToRead(Serials::CommandId::PointsCutoff,
                                  [](const char **responseString)
                                  {
                                      snprintf(dataString, sizeof(dataString), "%u", measureSettings.pointsCutoff);

                                      *responseString = dataString;
                                  });
}

/**
 * @brief Register serial write command handlers
 */
void RegisterSerialWriteHandlers()
{
    LOG_DEBUG("Register serial write handlers");

    serialManager.subscribeToWrite(Serials::CommandId::Date,
                                   [](const char *dataString)
                                   {
                                       SystemTime::setStringDate(dataString);
                                   });

    serialManager.subscribeToWrite(Serials::CommandId::Time,
                                   [](const char *dataString)
                                   {
                                       SystemTime::setStringTime(dataString);
                                   });

    serialManager.subscribeToWrite(Serials::CommandId::MeasureFrequency,
                                   [](const char *dataString)
                                   {
                                       uint8_t value = atoi(dataString);

                                       if (value < sampleFrequencyMin)
                                       {
                                           value = sampleFrequencyMin;
                                       }
                                       else if (value > sampleFrequencyMax)
                                       {
                                           value = sampleFrequencyMax;
                                       }

                                       // Stop IMU sampling
                                       stopImuTask();

                                       // Update measure frequency setting
                                       measureSettings.frequency = value;
                                       InternalStorage::updateSettings(measureSettingsId, measureSettings);

                                       // Restart PSD measurements
                                       setupPSD(measureSettings.pointsPsd, measureSettings.frequency);

                                       // Start IMU sampling
                                       startImuTask();
                                   });

    serialManager.subscribeToWrite(Serials::CommandId::MeasureInterval,
                                   [](const char *dataString)
                                   {
                                       uint32_t value = atoi(dataString);

                                       // Update measure interval setting
                                       measureSettings.measureInterval = value;
                                       InternalStorage::updateSettings(measureSettingsId, measureSettings);
                                   });

    serialManager.subscribeToWrite(Serials::CommandId::PauseInterval,
                                   [](const char *dataString)
                                   {
                                       uint32_t value = atoi(dataString);

                                       // Update pause interval setting
                                       measureSettings.pauseInterval = value;
                                       InternalStorage::updateSettings(measureSettingsId, measureSettings);
                                   });

    serialManager.subscribeToWrite(Serials::CommandId::PointsPsd,
                                   [](const char *dataString)
                                   {
                                       uint8_t value = atoi(dataString);

                                       if (value < pointsPsdMin)
                                       {
                                           value = pointsPsdMin;
                                       }
                                       else if (value > pointsPsdMax)
                                       {
                                           value = pointsPsdMax;
                                       }

                                       // Stop IMU sampling
                                       stopImuTask();

                                       // Update points to calculate PSD segment size
                                       measureSettings.pointsPsd = value;
                                       InternalStorage::updateSettings(measureSettingsId, measureSettings);

                                       // Restart PSD measurements
                                       setupPSD(measureSettings.pointsPsd, measureSettings.frequency);

                                       // Start IMU sampling
                                       startImuTask();
                                   });

    serialManager.subscribeToWrite(Serials::CommandId::PointsCutoff,
                                   [](const char *dataString)
                                   {
                                       uint16_t value = atoi(dataString);

                                       if (value < pointsCutoffMin)
                                       {
                                           value = pointsCutoffMin;
                                       }
                                       else if (value > pointsCutoffMax)
                                       {
                                           value = pointsCutoffMax;
                                       }

                                       // Update measure frequency setting
                                       measureSettings.pointsCutoff = value;
                                       InternalStorage::updateSettings(measureSettingsId, measureSettings);
                                   });
}

/**
 * @brief Configure all interfaces and board-specific stuff
 */
void setupBoard()
{
    // Setup USB serial port for debug messages (115200 baudrate)
    Serial.begin(115200);

    // Setup I2C interface to communicate with IMU, RTC (100kHz frequency)
    Wire.begin(WirePin::sda, WirePin::scl, 100000);

    // Setup board power
    Power::setup();

    // Turn off build-in LED (LOW - off, HIGH - on)
    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, LOW);
}

/**
 * @brief Setup PSD measurements
 *
 * @param[in] pointsPsd Points to calculate PSD segment size, 2^x
 * @param[in] sampleFrequency Sampling frequency, Hz
 */
void setupPSD(uint8_t pointsPsd, uint8_t sampleFrequency)
{
    static const size_t pow2[] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192};

    assert(pointsPsd >= pointsPsdMin && pointsPsd <= pointsPsdMax);
    assert(sampleFrequency >= sampleFrequencyMin && sampleFrequency <= sampleFrequencyMax);

    segmentCount = 0;
    segmentSize = pow2[pointsPsd];
    imuIntervalMs = millisPerSecond / sampleFrequency;

    LOG_INFO("PSD setup: segment size %u samples, sample time %u ms", segmentSize, imuIntervalMs);

    psdAccX.setup(segmentSize, sampleFrequency);
}

/**
 * @brief Setup preliminary stuff before starting the main loop
 */
void setup()
{
    // Setup the board first
    setupBoard();

    // Initialize internal storage
    InternalStorage::initialize();

    // Read measurements settings
    InternalStorage::readSettings(measureSettingsId, measureSettings);

    // Initialize system time with RTC value
    bool status = SystemTime::initialize(Wire);
    if (status == false)
    {
        LOG_ERROR("System time initialization failed");
    }

    // Initialize serial manager
    status = serialManager.initialize();
    if (status == true)
    {
        RegisterSerialReadHandlers();
        RegisterSerialWriteHandlers();
    }
    else
    {
        LOG_ERROR("Serial manager initialization failed!");
    }

    // Start SD file system
    status = setupSD();
    if (status == false)
    {
        LOG_ERROR("SD initialization failed");
    }

    // Start accelerometer readings
    status = setupImu();
    if (status == true)
    {
        LOG_INFO("IMU initialized");

        xTaskCreatePinnedToCore(imuTask, "imuTask", 4096, NULL, 1, NULL, 0);

        // Wait IMU task is idle
        EventBits_t events = eventGroup.wait(EventBits::imuIdle);
        if (events & EventBits::imuIdle)
        {
            LOG_INFO("IMU task created");

            setupPSD(measureSettings.pointsPsd, measureSettings.frequency);

            // Start IMU sampling
            startImuTask();
        }
        else
        {
            LOG_ERROR("IMU task creation failed");
        }
    }
    else
    {
        LOG_ERROR("IMU initialization failed");
    }

    LOG_INFO("Setup done");
}

/**
 * @brief The main loop function
 */
void loop()
{
    // Receive and handle serial commands from serial devices (if available)
    serialManager.process();

    // Check if event occurs
    EventBits_t events = eventGroup.wait(EventBits::segment0Ready | EventBits::segment1Ready, 0);
    if (events != 0)
    {
        size_t segmentIndex = (events & EventBits::segment0Ready) ? 0 : 1;

        LOG_INFO("PSD segment %d is ready (index %d)", segmentCount, segmentIndex);
        segmentCount++;

        const int16_t *pSamples = &buffer.accX[segmentIndex * segmentSize];
        psdAccX.computeSegment(pSamples);

        size_t segmentTimeMs = segmentSize * imuIntervalMs;
        size_t measureTimeMs = segmentCount * segmentTimeMs;
        // Check if there is enough time to take the next segment
        if (measureTimeMs + segmentTimeMs > secondsToMillis(measureSettings.measureInterval + measureIntervalJitter))
        {
            LOG_DEBUG("Measure time %d ms + segment time %d ms > measure interval %u sec + interval jitter %d sec",
                      measureTimeMs, segmentTimeMs, measureSettings.measureInterval, measureIntervalJitter);

            // Stop IMU sampling
            stopImuTask();

            // If 𝑁 is even (segmentSize = 2^x), you have 𝑁/2+1 useful components
            // because the symmetric part of the FFT spectrum for real-valued signals
            // does not provide additional information beyond the Nyquist frequency
            size_t resultPoints = segmentSize / 2 + 1;
            if (resultPoints > measureSettings.pointsCutoff)
            {
                // Limit result points
                resultPoints = measureSettings.pointsCutoff;
            }

            double *result = psdAccX.getResult();

            double deltaFreq = static_cast<double>(measureSettings.frequency) / segmentSize;
            double freq = 0;
            for (size_t idx = 0; idx < resultPoints; idx++)
            {
                LOG_INFO("%lf: %lf", freq, result[idx]);
                freq += deltaFreq;
            }

            Power::deepSleep(measureSettings.pauseInterval);
        }
    }
}

/**
 * @brief IMU samples reading task function
 *
 * @param pvParameters Task parameters
 */
void imuTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    BaseType_t xWasDelayed;

    ImuSample imuSample = {0};
    ImuSample prevSample = imuSample;

    size_t segmentIndex = 0;
    size_t sampleIndex = 0;

    (void *)pvParameters; // unused

    while (1)
    {
        LOG_INFO("IMU task idle");

        // Report IMU is in IDLE state
        eventGroup.set(EventBits::imuIdle);

        // Wait for start event
        EventBits_t events = eventGroup.wait(EventBits::startImu);

        LOG_INFO("IMU task running");

        // Report IMU is in IDLE state
        eventGroup.set(EventBits::imuRunning);

        // Reset segment and sample index
        segmentIndex = 0;
        sampleIndex = 0;
        // Initialise the xLastWakeTime variable with the current time.
        xLastWakeTime = xTaskGetTickCount();

        while ((events & EventBits::stopImu) == 0)
        {
            // Perform action here. xWasDelayed value can be used to determine
            // whether a deadline was missed if the code here took too long.

            // Read new IMU sample
            bool status = readImu(imuSample);
            if (status == true)
            {
                LOG_TRACE("Acc X: %d, Acc Y: %d, Acc Z: %d, Gyr X: %d, Gyr Y: %d, Gyr Z: %d",
                          imuSample.accel.x, imuSample.accel.y, imuSample.accel.z, imuSample.gyro.x, imuSample.gyro.y, imuSample.gyro.z);
                prevSample = imuSample;
            }
            else
            {
                LOG_ERROR("IMU reading failed");
                // Duplicate previous sample
                imuSample = prevSample;
            }

            size_t offset = segmentIndex * segmentSize + sampleIndex;
            buffer.accX[offset] = imuSample.accel.x;
            buffer.accY[offset] = imuSample.accel.y;
            buffer.accZ[offset] = imuSample.accel.z;
            buffer.gyrX[offset] = imuSample.gyro.x;
            buffer.gyrY[offset] = imuSample.gyro.y;
            buffer.gyrZ[offset] = imuSample.gyro.z;

            sampleIndex++;
            if (sampleIndex >= segmentSize)
            {
                EventBits_t event = segmentIndex == 0 ? EventBits::segment0Ready : EventBits::segment1Ready;
                eventGroup.set(event);

                // Start filling the next segment
                segmentIndex = 1 - segmentIndex;
                sampleIndex = 0;
            }

            // Wait for the next cycle
            xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(imuIntervalMs));

            // Check if stop event occurs
            events = eventGroup.wait(EventBits::stopImu, 0);
        }
    }

    vTaskDelete(NULL);
}
