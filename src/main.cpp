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
#include "Psd.h"
#include "Serial/SerialManager.hpp"

// Default time to take measurements
constexpr uint16_t measureTimeDefault = 600;
// Default time between measurements
constexpr uint16_t pauseTimeDefault = 300;

// Default sampling frequency, Hz
constexpr uint8_t sampleFrequencyDefault = 20;
// Minimum sampling frequency, Hz
constexpr uint8_t sampleFrequencyMin = 1;
// Maximum sampling frequency, Hz
constexpr uint8_t sampleFrequencyMax = 100;

// Default points degree to calculate PSD segment size
constexpr uint8_t pointsDegreeDefault = 8;
// Minimum points degree
constexpr uint8_t pointsDegreeMin = 2;
// Maximum points degree
constexpr uint8_t pointsDegreeMax = 10;

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
    constexpr EventBits_t segment0Ready = BIT3;
    constexpr EventBits_t segment1Ready = BIT4;

    constexpr EventBits_t all = startImu | stopImu | imuIdle | segment0Ready | segment1Ready;
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
    uint16_t measureTime; // Time for measuring, seconds
    uint16_t pauseTime;   // Time between measurements, seconds
    uint8_t frequency;    // Sampling frequency, Hz
    uint8_t pointsDegree; // Degree of 2 to calculate segment samples count, 2^x
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

// Size of segment, samples
size_t segmentSize;
// Interval between IMU samples, milliseconds
size_t imuIntervalMs;

// PSD for accelerometer axis X
PSD psdAccX;

// Measurements settings
MeasureSettings measureSettings = {
    .measureTime = measureTimeDefault,
    .pauseTime = pauseTimeDefault,
    .frequency = sampleFrequencyDefault,
    .pointsDegree = pointsDegreeDefault,
};

// Functions prototypes
void moveLoraToSleep();
bool setupSD();
bool setupImu();
bool readImu(ImuSample &imuSample);
void RegisterSerialReadHandlers();
void RegisterSerialWriteHandlers();
void setupBoard();
void setupPSD(uint8_t sampleCount, uint8_t sampleFrequency);
void imuTask(void *pvParameters);

/**
 * @brief Move LoRa chip into sleep mode
 */
void moveLoraToSleep()
{
    // Initialize SPI bus and slave select pin to communicate with LoRa chip
    SPI.end();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setFrequency(2000000);
    SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI);
    pinMode(LoRa_NSS, OUTPUT);

    // Wake up LoRa chip
    digitalWrite(LoRa_NSS, LOW);
    SPI.transfer(0xC0);
    SPI.transfer(0);
    digitalWrite(LoRa_NSS, HIGH);

    // Wait until LoRa chip is ready
    size_t timeoutMs = 1000;
    pinMode(LoRa_BUSY, INPUT);
    while (digitalRead(LoRa_BUSY))
    {
        delay(1);
        timeoutMs--;
        if (timeoutMs == 0)
        {
            LOG_ERROR("Waiting LoRa ready failed");
            return;
        }
    }

    // Move LoRa chip into sleep mode
    digitalWrite(LoRa_NSS, LOW);
    SPI.transfer(0x84);
    SPI.transfer(0);
    digitalWrite(LoRa_NSS, HIGH);

    delay(2);

    // Deinitialize SPI bus and slave select pin
    SPI.end();
    pinMode(LoRa_NSS, INPUT);
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
        LOG_INFO("IMU initialized");

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
    // Start IMU sampling
    eventGroup.set(EventBits::startImu);
}

/**
 * @brief Stop IMU sampling
 */
void stopImuTask()
{
    // Stop IMU sampling
    eventGroup.set(EventBits::stopImu);

    // Wait IMU task is idle
    EventBits_t events = eventGroup.wait(EventBits::imuIdle);
    if (!(events & EventBits::imuIdle))
    {
        LOG_ERROR("IMU stop failed");
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

    serialManager.subscribeToRead(Serials::CommandId::PointsDegree,
                                  [](const char **responseString)
                                  {
                                      snprintf(dataString, sizeof(dataString), "%u", measureSettings.pointsDegree);

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
                                       setupPSD(measureSettings.pointsDegree, measureSettings.frequency);

                                       // Start IMU sampling
                                       startImuTask();
                                   });

    serialManager.subscribeToWrite(Serials::CommandId::PointsDegree,
                                   [](const char *dataString)
                                   {
                                       uint8_t value = atoi(dataString);

                                       if (value < pointsDegreeMin)
                                       {
                                           value = pointsDegreeMin;
                                       }
                                       else if (value > pointsDegreeMax)
                                       {
                                           value = pointsDegreeMax;
                                       }

                                       // Stop IMU sampling
                                       stopImuTask();

                                       // Update measure frequency setting
                                       measureSettings.pointsDegree = value;
                                       InternalStorage::updateSettings(measureSettingsId, measureSettings);

                                       // Restart PSD measurements
                                       setupPSD(measureSettings.pointsDegree, measureSettings.frequency);

                                       // Start IMU sampling
                                       startImuTask();
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

    // Set LoRa chip into sleep mode
    moveLoraToSleep();

    // Turn off build-in LED (LOW - off, HIGH - on)
    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, LOW);

    // Power up the board (LOW - power up, HIGH - power down)
    pinMode(Vext_CTRL, OUTPUT);
    digitalWrite(Vext_CTRL, LOW);
    delay(10);
    LOG_INFO("Board is powered up");
}

/**
 * @brief Setup PSD measurements
 *
 * @param pointsDegree
 * @param sampleFrequency
 */
void setupPSD(uint8_t pointsDegree, uint8_t sampleFrequency)
{
    static const size_t pow2[] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192};

    assert(pointsDegree >= pointsDegreeMin && pointsDegree <= pointsDegreeMax);
    assert(sampleFrequency >= sampleFrequencyMin && sampleFrequency <= sampleFrequencyMax);

    segmentSize = pow2[pointsDegree];
    imuIntervalMs = millisPerSecond / sampleFrequency;

    LOG_INFO("PSD setup: segment size %u samples, sample time %u ms", segmentSize, imuIntervalMs);

    psdAccX.setup(segmentSize, sampleFrequency);
}

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
            LOG_INFO("IMU task is ready");

            setupPSD(measureSettings.pointsDegree, measureSettings.frequency);

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

    // Initialize serial manager
    bool isInitialized = serialManager.initialize();
    if (isInitialized == true)
    {
        RegisterSerialReadHandlers();
        RegisterSerialWriteHandlers();
    }
    else
    {
        LOG_ERROR("Serial manager initialization failed!");
    }

    LOG_INFO("Setup done");
}

void loop()
{
    static size_t segmentCount = 0;

    // Receive and handle serial commands from serial devices (if available)
    serialManager.process();

    // Check if event occurs
    EventBits_t events = eventGroup.wait(EventBits::segment0Ready | EventBits::segment1Ready, 0);
    if (events != 0)
    {
        size_t segmentIndex = (events & EventBits::segment0Ready) ? 0 : 1;

        LOG_INFO("PSD segment %d is ready (index %d)", segmentCount, segmentIndex);

        const int16_t *pSamples = &buffer.accX[segmentIndex * segmentSize];
        psdAccX.computeSegment(pSamples);

        segmentCount++;
        if (segmentCount == 2)
        {
            segmentCount = 0;

            double *result = psdAccX.getResult();

            double deltaFreq = static_cast<double>(measureSettings.frequency) / segmentSize;
            double freq = 0;
            for (size_t idx = 0; idx < segmentSize / 2 + 1; idx++)
            {
                LOG_INFO("%lf: %lf", freq, result[idx]);
                freq += deltaFreq;
            }
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
        LOG_INFO("Stop IMU task");

        // Report IMU is in IDLE state
        eventGroup.set(EventBits::imuIdle);

        // Wait for start event
        EventBits_t events = eventGroup.wait(EventBits::startImu);

        LOG_INFO("Start IMU task");

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
