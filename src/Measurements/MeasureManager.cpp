/**
 * @file MeasureManager.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Sensor measurements manager implementaion
 * @version 0.1
 * @date 2024-07-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "Measurements/MeasureManager.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <Debug.hpp>
#include <Events.h>
#include <IIM42652.h>
#include <MadgwickAHRS.h>
#include <SdFat.h>
#include <SystemTime.hpp>
#include <Wire.h>

#include "Battery.hpp"
#include "Board.h"
#include "FileSD.hpp"
#include "FwVersion.hpp"
#include "InternalStorage.hpp"
#include "Measurements/Psd.h"
#include "Measurements/Statistic.h"
#include "Serial/SerialManager.hpp"

using namespace Measurements;

namespace
{
    // Default time to take measurements, seconds
    constexpr uint32_t measureIntervalDefault = 600;
    // Default time between measurements, seconds
    constexpr uint32_t pauseIntervalDefault = 300;
    // Jitter time of measurements interval, seconds
    constexpr uint32_t measureIntervalJitter = 5;

    // Default sampling frequency, Hz
    constexpr uint8_t sampleFrequencyDefault = 40;
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

    // State of statistic (1 enable, 0 disable)
    constexpr uint8_t statisticStateDefault = 1;

    // Settings identifier in internal storage
    constexpr auto settingsId = SettingsModules::Measurements;

    // Accelerometer range, G
    constexpr size_t accelRangeG = 2; // 2, 4, 8, 16
    // Gyroscope range, degrees per second
    constexpr size_t gyroRangeDps = 250; // 125, 250, 500, 1000, 2000

    // Milliseconds per second
    constexpr size_t millisPerSecond = 1000;

    // IMU axis raw value after the reset
    constexpr int16_t imuResetValue = -32768;
    // Delay between checking if the IMU axis values are valid, milliseconds
    constexpr uint32_t imuWaitValidDelayMs = 1;
    // Timeout of waiting for the valid IMU axis values, milliseconds
    constexpr uint32_t imuWaitValidTimeoutMs = 100;

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
     * @brief Non volatile settings structure
     */
    struct Settings
    {
        uint32_t measureInterval; // Time for measuring, seconds
        uint32_t pauseInterval;   // Time between measurements, seconds
        uint16_t pointsCutoff;    // Points to store the PSD results
        uint8_t frequency;        // Sampling frequency, Hz
        uint8_t pointsPsd;        // Points to calculate PSD segment size, 2^x
        uint8_t statisticState;   // State of statistic (1 enable, 0 disable)
    };
#pragma pack(pop)

    /**
     * Samples buffer structure
     */
    struct Buffer
    {
        int16_t accX[2 * Measurements::PSD::samplesCountMax];
        int16_t accY[2 * Measurements::PSD::samplesCountMax];
        int16_t accZ[2 * Measurements::PSD::samplesCountMax];

        int16_t gyrX[2 * Measurements::PSD::samplesCountMax];
        int16_t gyrY[2 * Measurements::PSD::samplesCountMax];
        int16_t gyrZ[2 * Measurements::PSD::samplesCountMax];

        float roll[2 * Measurements::PSD::samplesCountMax];
        float pitch[2 * Measurements::PSD::samplesCountMax];
    };

    /**
     * @brief Measurements context
     */
    struct Context
    {
        // Count of ready segments
        size_t segmentCount;
        // Size of segment, samples
        size_t segmentSize;
        // Time of segment accumulating, milliseconds
        size_t segmentTimeMs;
        // Interval between IMU samples, milliseconds
        size_t imuIntervalMs;
        // Start measurements date and time
        SystemTime::DateTime startDateTime;

        /**
         * @brief Setup new context
         *
         * @param[in] pointsPsd Points to calculate PSD segment size, 2^x
         * @param[in] sampleFrequency Sampling frequency, Hz
         */
        void setup(uint8_t pointsPsd, uint8_t sampleFrequency)
        {
            static const size_t pow2[] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192};

            assert(pointsPsd < sizeof(pow2) / sizeof(*pow2));

            // Reset count of ready segments
            segmentCount = 0;
            // Determine segment size
            segmentSize = pow2[pointsPsd];
            // Calculate interval between IMU samples
            imuIntervalMs = millisPerSecond / sampleFrequency;
            // Calculate time of segment accumulating
            segmentTimeMs = segmentSize * imuIntervalMs;

            // Obtain measurements start date and time
            SystemTime::getDateTime(startDateTime);
        }
    };

    // IMU driver object
    IIM42652 imu;
    // SD file system class
    SdFs sd;
    // Madgwick's IMU and AHRS filter
    Madgwick madgwickFilter;

    // RTOS event group object
    RTOS::EventGroup eventGroup;

    // Samples buffer
    Buffer buffer = {0};

    // Current measurements context
    Context context;

    // PSD measurements for accelerometer and gyroscope axises X/Y
    Measurements::PSD psdAccX;
    Measurements::PSD psdAccY;
    Measurements::PSD psdGyroX;
    Measurements::PSD psdGyroY;

    // Statistic for accelerometer axises X/Y/Z
    Measurements::Statistic<int16_t> statisticAccX;
    Measurements::Statistic<int16_t> statisticAccY;
    Measurements::Statistic<int16_t> statisticAccZ;
    // Statistic for gyroscope axises X/Y/Z
    Measurements::Statistic<int16_t> statisticGyroX;
    Measurements::Statistic<int16_t> statisticGyroY;
    Measurements::Statistic<int16_t> statisticGyroZ;
    // Statistic for angle axises Roll/Pitch
    Measurements::Statistic<float> statisticRoll;
    Measurements::Statistic<float> statisticPitch;

    // Measurements settings
    Settings settings = {
        .measureInterval = measureIntervalDefault,
        .pauseInterval = pauseIntervalDefault,
        .pointsCutoff = pointsCutoffDefault,
        .frequency = sampleFrequencyDefault,
        .pointsPsd = pointsPsdDefault,
        .statisticState = statisticStateDefault,
    };

    // Functions prototypes
    bool setupImu();
    bool readImu(ImuSample &imuSample);
    void startImuTask();
    void stopImuTask();
    void setupMeasurements(uint8_t sampleCount, uint8_t sampleFrequency);
    void performMeasurements(size_t index);
    void saveMeasurements();
    void fillBuffer(size_t offset, const ImuSample &imuSample);
    void imuTask(void *pvParameters);
    void registerSerialReadHandlers();
    void registerSerialWriteHandlers();

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
     * @brief Convert raw accelerometer value to m/s^2 units
     *
     * @param raw Raw value
     * @return Value in m/s^2 units
     */
    constexpr float rawAccelToMs2(int16_t raw)
    {
        return (float)raw * accelRangeG * 9.81 / 32768;
    }

    /**
     * @brief Convert raw accelerometer value to G units
     *
     * @param raw Raw value
     * @return Value in G units
     */
    constexpr float rawAccelToG(int16_t raw)
    {
        return (float)raw * accelRangeG / 32768;
    }

    /**
     * @brief Convert raw gyroscope value to RAD/s units
     *
     * @param raw Raw value
     * @return Value in RAD/s units
     */
    constexpr float rawGyroToRads(int16_t raw)
    {
        return (float)raw * gyroRangeDps * M_PI / 360 / 32768;
    }

    /**
     * @brief Convert raw gyroscope value to Deg/s units
     *
     * @param raw Raw value
     * @return Value in Deg/s units
     */
    constexpr float rawGyroToDegs(int16_t raw)
    {
        return (float)raw * gyroRangeDps / 32768;
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
            // Setup gyroscope FSR
            if (result == true)
            {
                IIM42652_GYRO_CONFIG0_FS_SEL_t gyroFsrDps;

                switch (gyroRangeDps)
                {
                case 125:
                    gyroFsrDps = IIM42652_GYRO_CONFIG0_FS_SEL_125dps;
                    break;
                case 250:
                    gyroFsrDps = IIM42652_GYRO_CONFIG0_FS_SEL_250dps;
                    break;
                case 500:
                    gyroFsrDps = IIM42652_GYRO_CONFIG0_FS_SEL_500dps;
                    break;
                case 1000:
                    gyroFsrDps = IIM42652_GYRO_CONFIG0_FS_SEL_1000dps;
                    break;
                case 2000:
                    gyroFsrDps = IIM42652_GYRO_CONFIG0_FS_SEL_2000dps;
                    break;
                default:
                    assert(0); // Invalid gyroRangeDps option
                    break;
                }

                result = imu.set_gyro_fsr(gyroFsrDps);
            }

            // Setup accelerometer FSR
            if (result == true)
            {
                IIM42652_ACCEL_CONFIG0_FS_SEL_t accelFsrG;

                switch (accelRangeG)
                {
                case 2:
                    accelFsrG = IIM42652_ACCEL_CONFIG0_FS_SEL_2g;
                    break;
                case 4:
                    accelFsrG = IIM42652_ACCEL_CONFIG0_FS_SEL_4g;
                    break;
                case 8:
                    accelFsrG = IIM42652_ACCEL_CONFIG0_FS_SEL_8g;
                    break;
                case 16:
                    accelFsrG = IIM42652_ACCEL_CONFIG0_FS_SEL_16g;
                    break;
                default:
                    assert(0); // Invalid accelRangeG option
                    break;
                }

                result = imu.set_accel_fsr(accelFsrG);
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
     * @brief Setup measurements
     *
     * @param[in] pointsPsd Points to calculate PSD segment size, 2^x
     * @param[in] sampleFrequency Sampling frequency, Hz
     */
    void setupMeasurements(uint8_t pointsPsd, uint8_t sampleFrequency)
    {
        assert(pointsPsd >= pointsPsdMin && pointsPsd <= pointsPsdMax);
        assert(sampleFrequency >= sampleFrequencyMin && sampleFrequency <= sampleFrequencyMax);

        context.setup(pointsPsd, sampleFrequency);

        LOG_INFO("PSD setup: segment size %d samples, sample time %d ms, segment time %d ms",
                 context.segmentSize, context.imuIntervalMs, context.segmentTimeMs);

        // Setup madgwick's IMU and AHRS filter
        madgwickFilter.begin(sampleFrequency);

        // Setup PSD measurements
        psdAccX.setup(context.segmentSize, sampleFrequency);
        psdAccY.setup(context.segmentSize, sampleFrequency);
        psdGyroX.setup(context.segmentSize, sampleFrequency);
        psdGyroY.setup(context.segmentSize, sampleFrequency);

        // Reset measurements statistic
        statisticAccX.reset();
        statisticAccY.reset();
        statisticAccZ.reset();
        statisticGyroX.reset();
        statisticGyroY.reset();
        statisticGyroZ.reset();
        statisticRoll.reset();
        statisticPitch.reset();
    }

    /**
     * @brief Perform required calculations on raw data
     *
     * @param[in] index Buffer index with new data
     */
    void performMeasurements(size_t index)
    {
        // Data offset in buffer
        const size_t offset = index * context.segmentSize;

        const int16_t *pSamplesAccX = &buffer.accX[offset];
        psdAccX.computeSegment(pSamplesAccX);
        statisticAccX.calculate(pSamplesAccX, context.segmentSize);

        const int16_t *pSamplesAccY = &buffer.accY[offset];
        psdAccY.computeSegment(pSamplesAccY);
        statisticAccY.calculate(pSamplesAccY, context.segmentSize);

        const int16_t *pSamplesAccZ = &buffer.accZ[offset];
        statisticAccZ.calculate(pSamplesAccZ, context.segmentSize);

        const int16_t *pSamplesGyroX = &buffer.gyrX[offset];
        psdGyroX.computeSegment(pSamplesGyroX);
        statisticGyroX.calculate(pSamplesGyroX, context.segmentSize);

        const int16_t *pSamplesGyroY = &buffer.gyrY[offset];
        psdGyroY.computeSegment(pSamplesGyroY);
        statisticGyroY.calculate(pSamplesGyroY, context.segmentSize);

        const int16_t *pSamplesGyroZ = &buffer.gyrZ[offset];
        statisticGyroZ.calculate(pSamplesGyroZ, context.segmentSize);

        const float *pSamplesRoll = &buffer.roll[offset];
        statisticRoll.calculate(pSamplesRoll, context.segmentSize);

        const float *pSamplesPitch = &buffer.pitch[offset];
        statisticPitch.calculate(pSamplesPitch, context.segmentSize);
    }

    /**
     * @brief Save measurements to the SD file
     */
    void saveMeasurements()
    {
        // If 𝑁 is even (segmentSize = 2^x), you have 𝑁/2+1 useful components
        // because the symmetric part of the FFT spectrum for real-valued signals
        // does not provide additional information beyond the Nyquist frequency
        size_t resultPoints = context.segmentSize / 2 + 1;
        if (resultPoints > settings.pointsCutoff)
        {
            // Limit result points
            resultPoints = settings.pointsCutoff;
        }

        Measurements::PsdBin coreBinAccX;
        Measurements::PsdBin coreBinAccY;
        Measurements::PsdBin coreBinGyroX;
        Measurements::PsdBin coreBinGyroY;

        const double *resultPsdAccX = psdAccX.getResult(&coreBinAccX);
        const double *resultPsdAccY = psdAccY.getResult(&coreBinAccY);
        const double *resultPsdGyroX = psdGyroX.getResult(&coreBinGyroX);
        const double *resultPsdGyroY = psdGyroY.getResult(&coreBinGyroY);

        Battery::Status batteryStatus = Battery::readStatus();

        SystemTime::TimestampString timestamp;
        SystemTime::getTimestamp(timestamp);

        FileSD _file;
        _file.create("PSD", timestamp);
        bool isOpen = _file.open();
        if (isOpen == true)
        {
            char string[100];

            snprintf(string, sizeof(string), "FW %s", FwVersion::getVersionString());
            _file.println(string);

            // File header
            float batteryVoltage = static_cast<float>(batteryStatus.voltage) / 1000;
            snprintf(string, sizeof(string), "BATT %.1fV", batteryVoltage);
            _file.println(string);
            snprintf(string, sizeof(string), "BATT %u%%", batteryStatus.level);
            _file.println(string);
            snprintf(string, sizeof(string), "START_TIME %u/%u/%u %u:%u:%u",
                     context.startDateTime.Day, context.startDateTime.Month, context.startDateTime.Year,
                     context.startDateTime.Hour, context.startDateTime.Minute, context.startDateTime.Second);
            _file.println(string);
            snprintf(string, sizeof(string), "Logging Rate,%u", settings.frequency);
            _file.println(string);
            _file.println(""); // End of header

            _file.println("Channel Name,ACC_X");
            _file.println("Channel Units,m/s^2");
            snprintf(string, sizeof(string), "Maximum,%G", rawAccelToMs2(statisticAccX.max()));
            _file.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", rawAccelToMs2(statisticAccX.min()));
            _file.println(string);
            snprintf(string, sizeof(string), "Mean,%G", rawAccelToMs2(statisticAccX.mean()));
            _file.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", rawAccelToMs2(statisticAccX.deviation()));
            _file.println(string);
            snprintf(string, sizeof(string), "Core Frequency (%dpt PSD),%G,%G", context.segmentSize, coreBinAccX.frequency, coreBinAccX.amplitude);
            _file.println(string);
            snprintf(string, sizeof(string), "PSD_%d_%d", resultPoints, context.segmentSize);
            _file.print(string);
            for (size_t idx = 0; idx < resultPoints; idx++)
            {
                snprintf(string, sizeof(string), ",%G", resultPsdAccX[idx]);
                _file.print(string);
            }
            _file.println(""); // End of PSD
            _file.println(""); // End of channel

            _file.println("Channel Name,ACC_Y");
            _file.println("Channel Units,m/s^2");
            snprintf(string, sizeof(string), "Maximum,%G", rawAccelToMs2(statisticAccY.max()));
            _file.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", rawAccelToMs2(statisticAccY.min()));
            _file.println(string);
            snprintf(string, sizeof(string), "Mean,%G", rawAccelToMs2(statisticAccY.mean()));
            _file.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", rawAccelToMs2(statisticAccY.deviation()));
            _file.println(string);
            snprintf(string, sizeof(string), "Core Frequency (%dpt PSD),%G,%G", context.segmentSize, coreBinAccY.frequency, coreBinAccY.amplitude);
            _file.println(string);
            snprintf(string, sizeof(string), "PSD_%d_%d", resultPoints, context.segmentSize);
            _file.print(string);
            for (size_t idx = 0; idx < resultPoints; idx++)
            {
                snprintf(string, sizeof(string), ",%G", resultPsdAccY[idx]);
                _file.print(string);
            }
            _file.println(""); // End of PSD
            _file.println(""); // End of channel

            _file.println("Channel Name,ACC_Z");
            _file.println("Channel Units,m/s^2");
            snprintf(string, sizeof(string), "Maximum,%G", rawAccelToMs2(statisticAccZ.max()));
            _file.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", rawAccelToMs2(statisticAccZ.min()));
            _file.println(string);
            snprintf(string, sizeof(string), "Mean,%G", rawAccelToMs2(statisticAccZ.mean()));
            _file.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", rawAccelToMs2(statisticAccZ.deviation()));
            _file.println(string);
            _file.println(""); // End of channel

            _file.println("Channel Name,GYRO_X");
            _file.println("Channel Units,rad/s");
            snprintf(string, sizeof(string), "Maximum,%G", rawGyroToRads(statisticGyroX.max()));
            _file.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", rawGyroToRads(statisticGyroX.min()));
            _file.println(string);
            snprintf(string, sizeof(string), "Mean,%G", rawGyroToRads(statisticGyroX.mean()));
            _file.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", rawGyroToRads(statisticGyroX.deviation()));
            _file.println(string);
            snprintf(string, sizeof(string), "Core Frequency (%dpt PSD),%G,%G", context.segmentSize, coreBinGyroX.frequency, coreBinGyroX.amplitude);
            _file.println(string);
            snprintf(string, sizeof(string), "PSD_%d_%d", resultPoints, context.segmentSize);
            _file.print(string);
            for (size_t idx = 0; idx < resultPoints; idx++)
            {
                snprintf(string, sizeof(string), ",%G", resultPsdGyroX[idx]);
                _file.print(string);
            }
            _file.println(""); // End of PSD
            _file.println(""); // End of channel

            _file.println("Channel Name,GYRO_Y");
            _file.println("Channel Units,rad/s");
            snprintf(string, sizeof(string), "Maximum,%G", rawGyroToRads(statisticGyroY.max()));
            _file.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", rawGyroToRads(statisticGyroY.min()));
            _file.println(string);
            snprintf(string, sizeof(string), "Mean,%G", rawGyroToRads(statisticGyroY.mean()));
            _file.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", rawGyroToRads(statisticGyroY.deviation()));
            _file.println(string);
            snprintf(string, sizeof(string), "Core Frequency (%dpt PSD),%G,%G", context.segmentSize, coreBinGyroY.frequency, coreBinGyroY.amplitude);
            _file.println(string);
            snprintf(string, sizeof(string), "PSD_%d_%d", resultPoints, context.segmentSize);
            _file.print(string);
            for (size_t idx = 0; idx < resultPoints; idx++)
            {
                snprintf(string, sizeof(string), ",%G", resultPsdGyroY[idx]);
                _file.print(string);
            }
            _file.println(""); // End of PSD
            _file.println(""); // End of channel

            _file.println("Channel Name,GYRO_Z");
            _file.println("Channel Units,rad/s");
            snprintf(string, sizeof(string), "Maximum,%G", rawGyroToRads(statisticGyroZ.max()));
            _file.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", rawGyroToRads(statisticGyroZ.min()));
            _file.println(string);
            snprintf(string, sizeof(string), "Mean,%G", rawGyroToRads(statisticGyroZ.mean()));
            _file.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", rawGyroToRads(statisticGyroZ.deviation()));
            _file.println(string);
            _file.println(""); // End of channel

            _file.println("Channel Name,ROLL");
            _file.println("Channel Units,deg");
            snprintf(string, sizeof(string), "Maximum,%G", statisticRoll.max());
            _file.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", statisticRoll.min());
            _file.println(string);
            snprintf(string, sizeof(string), "Mean,%G", statisticRoll.mean());
            _file.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", statisticRoll.deviation());
            _file.println(string);
            _file.println(""); // End of channel

            _file.println("Channel Name,PITCH");
            _file.println("Channel Units,deg");
            snprintf(string, sizeof(string), "Maximum,%G", statisticPitch.max());
            _file.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", statisticPitch.min());
            _file.println(string);
            snprintf(string, sizeof(string), "Mean,%G", statisticPitch.mean());
            _file.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", statisticPitch.deviation());
            _file.println(string);
            _file.println(""); // End of channel

            _file.close();
        }

        LOG_DEBUG("ACC_X: Max %d, Min %d, Mean %f, Standard Deviation %f, Core Frequency %lfHz - %lf",
                  statisticAccX.max(), statisticAccX.min(), statisticAccX.mean(), statisticAccX.deviation(),
                  coreBinAccX.frequency, coreBinAccX.amplitude);
        LOG_DEBUG("ACC_Y: Max %d, Min %d, Mean %f, Standard Deviation %f, Core Frequency %lfHz - %lf",
                  statisticAccY.max(), statisticAccY.min(), statisticAccY.mean(), statisticAccY.deviation(),
                  coreBinAccY.frequency, coreBinAccY.amplitude);
        LOG_DEBUG("ACC_Z: Max %d, Min %d, Mean %f, Standard Deviation %f",
                  statisticAccZ.max(), statisticAccZ.min(), statisticAccZ.mean(), statisticAccZ.deviation());

        LOG_DEBUG("GYRO_X: Max %d, Min %d, Mean %f, Standard Deviation %f, Core Frequency %lfHz - %lf",
                  statisticGyroX.max(), statisticGyroX.min(), statisticGyroX.mean(), statisticGyroX.deviation(),
                  coreBinGyroX.frequency, coreBinGyroX.amplitude);
        LOG_DEBUG("GYRO_Y: Max %d, Min %d, Mean %f, Standard Deviation %f, Core Frequency %lfHz - %lf",
                  statisticGyroY.max(), statisticGyroY.min(), statisticGyroY.mean(), statisticGyroY.deviation(),
                  coreBinGyroY.frequency, coreBinGyroY.amplitude);
        LOG_DEBUG("GYRO_Z: Max %d, Min %d, Mean %f, Standard Deviation %f",
                  statisticGyroZ.max(), statisticGyroZ.min(), statisticGyroZ.mean(), statisticGyroZ.deviation());

        LOG_DEBUG("ROLL: Max %f, Min %f, Mean %f, Standard Deviation %f",
                  statisticRoll.max(), statisticRoll.min(), statisticRoll.mean(), statisticRoll.deviation());
        LOG_DEBUG("PITCH: Max %f, Min %f, Mean %f, Standard Deviation %f",
                  statisticPitch.max(), statisticPitch.min(), statisticPitch.mean(), statisticPitch.deviation());
    }

    /**
     * @brief Fill buffer data with IMU sample
     *
     * @param[in] offset Data offset in the buffer
     * @param[in] imuSample IMU sample
     */
    void fillBuffer(size_t offset, const ImuSample &imuSample)
    {
        // Fill Accel/Gyro buffer data
        buffer.accX[offset] = imuSample.accel.x;
        buffer.accY[offset] = imuSample.accel.y;
        buffer.accZ[offset] = imuSample.accel.z;
        buffer.gyrX[offset] = imuSample.gyro.x;
        buffer.gyrY[offset] = imuSample.gyro.y;
        buffer.gyrZ[offset] = imuSample.gyro.z;

        // Convert to accel to G and gyro to DPS
        float accelGX = rawAccelToG(imuSample.accel.x);
        float accelGY = rawAccelToG(imuSample.accel.y);
        float accelGZ = rawAccelToG(imuSample.accel.z);
        float gyroDpsX = rawGyroToDegs(imuSample.gyro.x);
        float gyroDpsY = rawGyroToDegs(imuSample.gyro.y);
        float gyroDpsZ = rawGyroToDegs(imuSample.gyro.z);
        LOG_TRACE("Acc G X %.1f, Y %.1f, Z %.1f, Gyro DPS X %.1f, Y %.1f, Z %.1f",
                  accelGX, accelGY, accelGZ, gyroDpsX, gyroDpsY, gyroDpsZ);

        // Calculate/fill angle buffer data
        madgwickFilter.updateIMU(gyroDpsX, gyroDpsY, gyroDpsZ, accelGX, accelGY, accelGZ);
        buffer.roll[offset] = madgwickFilter.getRoll();
        buffer.pitch[offset] = madgwickFilter.getPitch();
        LOG_TRACE("Angle Roll %.1f, Pitch %.1f", buffer.roll[offset], buffer.pitch[offset]);
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

            // Enable sensors when task is started
            bool status = imu.accelerometer_enable();
            if (status == true)
            {
                status = imu.gyroscope_enable();
                if (status == true)
                {
                    status = readImu(prevSample);
                }
            }

            // Workaround to skip the first initial invalid samples
            uint32_t timeout = 0;
            while (status == true)
            {
                status = readImu(prevSample);
                if (status == true &&
                    prevSample.accel.x != imuResetValue &&
                    prevSample.accel.y != imuResetValue &&
                    prevSample.accel.z != imuResetValue &&
                    prevSample.gyro.x != imuResetValue &&
                    prevSample.gyro.y != imuResetValue &&
                    prevSample.gyro.z != imuResetValue)
                {
                    break;
                }
                
                if (timeout > imuWaitValidTimeoutMs)
                {
                    // IMU data is still invalid
                    status = false;
                    break;
                }

                delay(imuWaitValidDelayMs);
                timeout += imuWaitValidDelayMs;
            }

            if (status != true)
            {
                LOG_ERROR("IMU start failed");
                continue;
            }

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
                // Wait for the next cycle
                xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(context.imuIntervalMs));

                // Perform action here. xWasDelayed value can be used to determine
                // whether a deadline was missed if the code here took too long

                // Read new IMU sample
                status = readImu(imuSample);
                if (status == true)
                {
                    LOG_TRACE("Acc: X %d, Y %d, Z %d, Gyro: X %d, Y %d, Z %d",
                              imuSample.accel.x, imuSample.accel.y, imuSample.accel.z, imuSample.gyro.x, imuSample.gyro.y, imuSample.gyro.z);
                    prevSample = imuSample;
                }
                else
                {
                    LOG_ERROR("IMU reading failed");
                    // Duplicate previous sample
                    imuSample = prevSample;
                }

                size_t offset = segmentIndex * context.segmentSize + sampleIndex;

                // Fill buffer data with IMU sample
                fillBuffer(offset, imuSample);

                sampleIndex++;
                if (sampleIndex >= context.segmentSize)
                {
                    EventBits_t event = segmentIndex == 0 ? EventBits::segment0Ready : EventBits::segment1Ready;
                    eventGroup.set(event);

                    // Start filling the next segment
                    segmentIndex = 1 - segmentIndex;
                    sampleIndex = 0;
                }

                // Check if stop event occurs
                events = eventGroup.wait(EventBits::stopImu, 0);
            }

            // Disable sensors when task is stopped
            imu.accelerometer_disable();
            imu.gyroscope_disable();
        }

        vTaskDelete(NULL);
    }

    /**
     * @brief Register serial read command handlers
     */
    void registerSerialReadHandlers()
    {
        static char dataString[Serials::SerialDevice::dataMaxLength];

        LOG_TRACE("Register serial read measurement handlers");

        Serials::Manager::subscribeToRead(Serials::CommandId::MeasureFrequency,
                                          [](const char **responseString)
                                          {
                                              snprintf(dataString, sizeof(dataString), "%u", settings.frequency);

                                              *responseString = dataString;
                                          });

        Serials::Manager::subscribeToRead(Serials::CommandId::MeasureInterval,
                                          [](const char **responseString)
                                          {
                                              snprintf(dataString, sizeof(dataString), "%u", settings.measureInterval);

                                              *responseString = dataString;
                                          });

        Serials::Manager::subscribeToRead(Serials::CommandId::PauseInterval,
                                          [](const char **responseString)
                                          {
                                              snprintf(dataString, sizeof(dataString), "%u", settings.pauseInterval);

                                              *responseString = dataString;
                                          });

        Serials::Manager::subscribeToRead(Serials::CommandId::PointsPsd,
                                          [](const char **responseString)
                                          {
                                              snprintf(dataString, sizeof(dataString), "%u", settings.pointsPsd);

                                              *responseString = dataString;
                                          });

        Serials::Manager::subscribeToRead(Serials::CommandId::PointsCutoff,
                                          [](const char **responseString)
                                          {
                                              snprintf(dataString, sizeof(dataString), "%u", settings.pointsCutoff);

                                              *responseString = dataString;
                                          });

        Serials::Manager::subscribeToRead(Serials::CommandId::StatisticState,
                                          [](const char **responseString)
                                          {
                                              snprintf(dataString, sizeof(dataString), "%u", settings.statisticState);

                                              *responseString = dataString;
                                          });
    }

    /**
     * @brief Register serial write command handlers
     */
    void registerSerialWriteHandlers()
    {
        LOG_TRACE("Register serial write measurement handlers");

        Serials::Manager::subscribeToWrite(Serials::CommandId::MeasureFrequency,
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
                                               settings.frequency = value;
                                               InternalStorage::updateSettings(settingsId, settings);

                                               // Restart measurements
                                               setupMeasurements(settings.pointsPsd, settings.frequency);

                                               // Start IMU sampling
                                               startImuTask();
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::MeasureInterval,
                                           [](const char *dataString)
                                           {
                                               uint32_t value = atoi(dataString);

                                               // Update measure interval setting
                                               settings.measureInterval = value;
                                               InternalStorage::updateSettings(settingsId, settings);
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::PauseInterval,
                                           [](const char *dataString)
                                           {
                                               uint32_t value = atoi(dataString);

                                               // Update pause interval setting
                                               settings.pauseInterval = value;
                                               InternalStorage::updateSettings(settingsId, settings);
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::PointsPsd,
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
                                               settings.pointsPsd = value;
                                               InternalStorage::updateSettings(settingsId, settings);

                                               // Restart measurements
                                               setupMeasurements(settings.pointsPsd, settings.frequency);

                                               // Start IMU sampling
                                               startImuTask();
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::PointsCutoff,
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

                                               // Update points to store the PSD results setting
                                               settings.pointsCutoff = value;
                                               InternalStorage::updateSettings(settingsId, settings);
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::StatisticState,
                                           [](const char *dataString)
                                           {
                                               uint8_t value = atoi(dataString);

                                               // Update state of statistic setting
                                               settings.statisticState = value;
                                               InternalStorage::updateSettings(settingsId, settings);
                                           });
    }
} // namespace

/**
 * @brief Initialize sensor measurements
 *
 * @return true if initialization succeed, false otherwise
 */
bool Manager::initialize()
{
    // Read settings
    InternalStorage::readSettings(settingsId, settings);

    // Register local serial handlers
    registerSerialReadHandlers();
    registerSerialWriteHandlers();

    // Start accelerometer readings
    bool status = setupImu();
    if (status == true)
    {
        LOG_INFO("IMU initialized");

        xTaskCreatePinnedToCore(imuTask, "imuTask", 4096, NULL, 1, NULL, 0);

        // Wait IMU task is idle
        EventBits_t events = eventGroup.wait(EventBits::imuIdle);
        if (events & EventBits::imuIdle)
        {
            LOG_INFO("IMU task created");

            setupMeasurements(settings.pointsPsd, settings.frequency);

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

    return status;
}

/**
 * @brief Perform sensor input data processing
 */
void Manager::process()
{
    // Check if event occurs
    EventBits_t events = eventGroup.wait(EventBits::segment0Ready | EventBits::segment1Ready, 0);
    if (events != 0)
    {
        // Increment count of ready segments
        context.segmentCount++;
        size_t measureTimeMs = context.segmentCount * context.segmentTimeMs;

        float readyPercents = static_cast<float>(measureTimeMs) / secondsToMillis(settings.measureInterval) * 100;
        LOG_INFO("PSD segment %d is ready, %.1f%%", context.segmentCount, readyPercents);

        size_t segmentIndex = (events & EventBits::segment0Ready) ? 0 : 1;
        performMeasurements(segmentIndex);

        // Check if there is enough time to take the next segment
        if (measureTimeMs + context.segmentTimeMs > secondsToMillis(settings.measureInterval + measureIntervalJitter))
        {
            LOG_DEBUG("Measure time %d ms + segment time %d ms > measure interval %u sec + interval jitter %d sec",
                      measureTimeMs, context.segmentTimeMs, settings.measureInterval, measureIntervalJitter);

            context.segmentCount = 0;

            // Save measurements to the storage
            saveMeasurements();

            // Check if board should go to sleep during pause interval
            if (settings.pauseInterval > 0)
            {
                // Stop IMU sampling
                stopImuTask();

                FileSD::stopFileSystem();

                Board::deepSleep(settings.pauseInterval);
            }
            else
            {
                // Reset measurements statistic
                statisticAccX.reset();
                statisticAccY.reset();
                statisticAccZ.reset();
                statisticGyroX.reset();
                statisticGyroY.reset();
                statisticGyroZ.reset();
                statisticRoll.reset();
                statisticPitch.reset();
            }
        }
    }
}
