// System headers
#include <atomic>

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
#include "Serial/SerialManager.hpp"

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

namespace EventBits
{
    constexpr EventBits_t psdSegment25 = BIT0;
    constexpr EventBits_t psdSegment50 = BIT1;
    constexpr EventBits_t psdSegment75 = BIT2;
    constexpr EventBits_t psdSegmentReady = BIT3;

    constexpr EventBits_t all = psdSegment25 | psdSegment50 | psdSegment75 | psdSegmentReady;
} // namespace EventBits

namespace SdPin
{
    constexpr auto cs = GPIO_NUM_26; // SD chip select pin
} // namespace SdPin

/**
 * @brief IMU sample structure
 */
struct ImuSample
{
    IIM42652_axis_t accel; // IMU accel axises
    IIM42652_axis_t gyro;  // IMU gyro axises
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

// IMU data samples
const size_t segmentSize = 8192;
ImuSample imuSamples[2 * segmentSize] = {0};

std::atomic<uint8_t> imuReadyPercents;

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
 * @brief Start SD file system class
 *
 * @return true if SD file system was started successfully, false otherwise
 */
bool startSD()
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
 * @brief Initialize and start IMU sensor
 *
 * @return true if operations succeed, false otherwise
 */
bool startImu()
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
 * @brief IMU samples reading task function
 *
 * @param pvParameters Task parameters
 */
void imuTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz - 20ms period
    BaseType_t xWasDelayed;

    size_t sampleIndex = 0;
    ImuSample &prevSample = imuSamples[0];

    (void *)pvParameters; // unused

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    imuReadyPercents = 0;

    while (true)
    {
        // Perform action here. xWasDelayed value can be used to determine
        // whether a deadline was missed if the code here took too long.

        ImuSample &imuSample = imuSamples[sampleIndex];

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
            imuSample = prevSample;
        }

        sampleIndex++;
        if (sampleIndex >= segmentSize)
        {
            sampleIndex = 0;

            eventGroup.set(EventBits::psdSegmentReady);
        }

        // Wait for the next cycle.
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    vTaskDelete(NULL);
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
}

/**
 * @brief Register serial write command handlers
 */
void RegisterSerialWriteHandlers()
{
    LOG_DEBUG("Register serial write handlers");

    serialManager.subscribeToWrite(Serials::CommandId::Date,
                                   [](const char *dataString)
                                   { SystemTime::setStringDate(dataString); });

    serialManager.subscribeToWrite(Serials::CommandId::Time,
                                   [](const char *dataString)
                                   { SystemTime::setStringTime(dataString); });
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

void setup()
{
    // Setup the board first
    setupBoard();

    // Initialize internal storage
    InternalStorage::initialize();

    // Initialize system time with RTC value
    bool status = SystemTime::initialize(Wire);
    if (status == false)
    {
        LOG_ERROR("System time initialization failed");
    }

    // Start SD file system
    status = startSD();
    if (status == false)
    {
        LOG_ERROR("SD start failed");
    }

    // Start accelerometer readings
    status = startImu();
    if (status == true)
    {
        LOG_INFO("IMU started");

        xTaskCreatePinnedToCore(imuTask, "imuTask", 4096, NULL, 1, NULL, 0);
    }
    else
    {
        LOG_ERROR("IMU start failed");
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
    // Receive and handle serial commands from serial devices (if available)
    serialManager.process();

    // Check if event occurs
    bool isPsdSegmentReady = eventGroup.wait(EventBits::psdSegmentReady, 0);
    if (isPsdSegmentReady == true)
    {
        LOG_INFO("PSD segment is ready");
    }
}
