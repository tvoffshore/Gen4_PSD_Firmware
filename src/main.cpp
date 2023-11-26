#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <Debug.hpp>
#include <IIM42652.h>
#include <SdFat.h>
#include <SystemTime.hpp>

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

/**
 * @brief IMU data structure
 */
struct ImuData
{
    IIM42652_axis_t accel; // IMU accel axises
    IIM42652_axis_t gyro;  // IMU gyro axises
};

#if (LOG_LEVEL > LOG_LEVEL_NONE)
// Log level declaration
uint8_t LogsOutput::logLevelMax = LOG_LEVEL;
#endif // #if (LOG_LEVEL > LOG_LEVEL_NONE)

// IMU driver object
IIM42652 imu;

// SD file system class
SdFs sd;

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
 * @param pImuData Pointer to put IMU data
 * @return true if reading succeed, false otherwise
 */
bool readImu(ImuData *pImuData)
{
    bool result = imu.get_accel_data(&pImuData->accel);
    if (result == true)
    {
        result = imu.get_gyro_data(&pImuData->gyro);
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

    (void *)pvParameters; // unused

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        // Wait for the next cycle.
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Perform action here. xWasDelayed value can be used to determine
        // whether a deadline was missed if the code here took too long.

        // Read new IMU data
        ImuData imuData = {0};
        bool status = readImu(&imuData);
        if (status == true)
        {
            LOG_TRACE("Acc X: %d, Acc Y: %d, Acc Z: %d, Gyr X: %d, Gyr Y: %d, Gyr Z: %d",
                      imuData.accel.x, imuData.accel.y, imuData.accel.z, imuData.gyro.x, imuData.gyro.y, imuData.gyro.z);
        }
        else
        {
            LOG_ERROR("IMU reading failed");
        }
    }
}

void setup()
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

    LOG_INFO("Setup done");
}

void loop()
{
}