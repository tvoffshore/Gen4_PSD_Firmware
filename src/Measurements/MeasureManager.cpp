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

#include <Wire.h>

#include <Events.h>
#include <IIM42652.h>
#include <MadgwickAHRS.h>
#include <Log.hpp>
#include <SdFat.h>
#include <Settings.hpp>
#include <SystemTime.hpp>

#include "Analog/GainSelector.hpp"
#include "Analog/InputSelector.hpp"
#include "Analog/Sensor.hpp"
#include "Analog/VddController.hpp"
#include "Battery.hpp"
#include "Board.h"
#include "FwVersion.hpp"
#include "IoExpander/IoExpander.hpp"
#include "Measurements/Psd.h"
#include "Measurements/Statistic.h"
#include "Measurements/Types.hpp"
#include "Sd/File.hpp"
#include "Serial/SerialManager.hpp"

using namespace Measurements;

namespace
{
    // Analog sensor channel 1 input pin (ADC_1_CH3)
    constexpr auto pinAdc1Input = GPIO_NUM_4;
    // Analog sensor channel 1 enable pin
    constexpr auto pinAdc1Enable = GPIO_NUM_2;

    // Analog sensor channel 2 input pin (ADC_1_CH4)
    constexpr auto pinAdc2Input = GPIO_NUM_5;
    // Analog sensor channel 2 enable pin
    constexpr auto pinAdc2Enable = GPIO_NUM_3;

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
    constexpr auto settingsId = Settings::Id::Measurements;

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

    // CSV files
    const char *directoryCsv = "CSV";
    const char *fileExtensionCsv = "csv";
    // BIN files
    const char *directoryBin = "BIN";
    const char *fileExtensionBin = "bin";

    namespace EventBits
    {
        constexpr EventBits_t startTask = BIT0;
        constexpr EventBits_t stopTask = BIT1;
        constexpr EventBits_t taskIsIdle = BIT2;
        constexpr EventBits_t taskIsRunning = BIT3;
        constexpr EventBits_t segment0Ready = BIT4;
        constexpr EventBits_t segment1Ready = BIT5;

        constexpr EventBits_t all = startTask | stopTask | taskIsIdle | taskIsRunning | segment0Ready | segment1Ready;
    } // namespace EventBits

    /**
     * @brief Analog sensor sample structure
     */
    struct AdcSample
    {
        uint16_t chan1; // Analog channel 1 value
        uint16_t chan2; // Analog channel 2 value
    };

    /**
     * @brief IMU sensor sample structure
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
    struct MeasureSettings
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
        uint16_t adc1[2 * Measurements::samplesCountMax];
        uint16_t adc2[2 * Measurements::samplesCountMax];

        int16_t accX[2 * Measurements::samplesCountMax];
        int16_t accY[2 * Measurements::samplesCountMax];
        int16_t accZ[2 * Measurements::samplesCountMax];

        int16_t gyrX[2 * Measurements::samplesCountMax];
        int16_t gyrY[2 * Measurements::samplesCountMax];
        int16_t gyrZ[2 * Measurements::samplesCountMax];

        float roll[2 * Measurements::samplesCountMax];
        float pitch[2 * Measurements::samplesCountMax];
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
        // Interval between samples, milliseconds
        size_t sampleTimeMs;
        // Start measurements date and time
        SystemTime::DateTime startDateTime;
        // Start measurements epoch time
        time_t startEpochTime;

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
            // Calculate interval between samples
            sampleTimeMs = millisPerSecond / sampleFrequency;
            // Calculate time of segment accumulating
            segmentTimeMs = segmentSize * sampleTimeMs;

            // Obtain measurements start date and time
            startEpochTime = SystemTime::getDateTime(startDateTime);
        }
    };

    // Analog sensor channel 1
    Analog::Sensor adc1(pinAdc1Input, pinAdc1Enable);
    // Analog sensor channel 2
    Analog::Sensor adc2(pinAdc2Input, pinAdc2Enable);

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
    // Accelerometer resultant direction buffer
    float accelResult[Measurements::samplesCountMax];

    // Current measurements context
    Context context;

    // PSD measurements for ADC1 and ADC2
    Measurements::PSD<uint16_t> psdAdc1;
    Measurements::PSD<uint16_t> psdAdc2;
    // PSD measurements for accelerometer axises X/Y
    Measurements::PSD<int16_t> psdAccX;
    Measurements::PSD<int16_t> psdAccY;
    // PSD measurements for gyroscope axises X/Y
    Measurements::PSD<int16_t> psdGyroX;
    Measurements::PSD<int16_t> psdGyroY;
    // PSD measurements for accelerometer resultant direction
    Measurements::PSD<float> psdAccResult;

    // Statistic for ADC1 and ADC2
    Measurements::Statistic<uint16_t> statisticAdc1;
    Measurements::Statistic<uint16_t> statisticAdc2;
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
    // Statistic for accelerometer resultant direction
    Measurements::Statistic<float> statisticAccelResult;

    // Measurements settings
    MeasureSettings settings = {
        .measureInterval = measureIntervalDefault,
        .pauseInterval = pauseIntervalDefault,
        .pointsCutoff = pointsCutoffDefault,
        .frequency = sampleFrequencyDefault,
        .pointsPsd = pointsPsdDefault,
        .statisticState = statisticStateDefault,
    };

    // IMU sensor functions
    bool setupImu();
    bool readImu(ImuSample &imuSample);
    bool waitImuReady();

    // Main process functions
    void startSampling();
    void stopSampling();
    void setupMeasurements(uint8_t sampleCount, uint8_t sampleFrequency);
    void performCalculations(size_t index);
    void calculateAccelResult(const int16_t *pAccX, float meanAccX, const int16_t *pAccY, float meanAccY, size_t length);
    void saveMeasurements();
    void resetStatistics();

    // Sampling process functions
    bool enableSensors();
    void disableSensors();
    void fillBuffer(size_t offset, const AdcSample &adcSample, const ImuSample &imuSample);
    void samplingTask(void *pvParameters);

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
     * @brief Workaround to skip the first initial invalid samples
     *
     * @return true if IMU is ready, false otherwise
     */
    bool waitImuReady()
    {
        ImuSample imuSample = {0};
        bool result = false;
        uint32_t elapsedMs = 0;

        while (elapsedMs < imuWaitValidTimeoutMs)
        {
            bool status = readImu(imuSample);
            if (status == true &&
                imuSample.accel.x != imuResetValue &&
                imuSample.accel.y != imuResetValue &&
                imuSample.accel.z != imuResetValue &&
                imuSample.gyro.x != imuResetValue &&
                imuSample.gyro.y != imuResetValue &&
                imuSample.gyro.z != imuResetValue)
            {
                result = true;
                break;
            }

            delay(imuWaitValidDelayMs);
            elapsedMs += imuWaitValidDelayMs;
        }

        LOG_DEBUG("IMU %s ready in %u ms", result ? "is" : "isn't", elapsedMs);

        return result;
    }

    /**
     * @brief Start sensors sampling
     */
    void startSampling()
    {
        LOG_INFO("Start sampling");

        // Start sampling task
        eventGroup.set(EventBits::startTask);

        // Wait sampling task is running
        EventBits_t events = eventGroup.wait(EventBits::taskIsRunning);
        if (!(events & EventBits::taskIsRunning))
        {
            LOG_ERROR("Sampling task start failed");
        }
    }

    /**
     * @brief Stop sensors sampling
     */
    void stopSampling()
    {
        LOG_INFO("Stop sampling");

        // Stop sampling task
        eventGroup.set(EventBits::stopTask);

        // Wait sampling task is idle
        EventBits_t events = eventGroup.wait(EventBits::taskIsIdle);
        if (!(events & EventBits::taskIsIdle))
        {
            LOG_ERROR("Sampling task stop failed");
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
                 context.segmentSize, context.sampleTimeMs, context.segmentTimeMs);

        // Setup madgwick's IMU and AHRS filter
        madgwickFilter.begin(sampleFrequency);

        // Setup PSD measurements
        psdAdc1.setup(context.segmentSize, sampleFrequency);
        psdAdc2.setup(context.segmentSize, sampleFrequency);
        psdAccX.setup(context.segmentSize, sampleFrequency);
        psdAccY.setup(context.segmentSize, sampleFrequency);
        psdGyroX.setup(context.segmentSize, sampleFrequency);
        psdGyroY.setup(context.segmentSize, sampleFrequency);
        psdAccResult.setup(context.segmentSize, sampleFrequency);

        // Reset measurements statistic
        resetStatistics();
    }

    /**
     * @brief Perform required calculations on raw data
     *
     * @param[in] index Buffer index with new data
     */
    void performCalculations(size_t index)
    {
        // Data offset in buffer
        const size_t offset = index * context.segmentSize;

        const uint16_t *pSamplesAdc1 = &buffer.adc1[offset];
        psdAdc1.computeSegment(pSamplesAdc1);
        statisticAdc1.calculate(pSamplesAdc1, context.segmentSize);

        const uint16_t *pSamplesAdc2 = &buffer.adc2[offset];
        psdAdc2.computeSegment(pSamplesAdc2);
        statisticAdc2.calculate(pSamplesAdc2, context.segmentSize);

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

        calculateAccelResult(pSamplesAccX, statisticAccX.mean(),
                             pSamplesAccY, statisticAccY.mean(), context.segmentSize);
        psdAccResult.computeSegment(accelResult);
        statisticAccelResult.calculate(accelResult, context.segmentSize);
    }

    /**
     * @brief Calculate accelerometer resultant direction using Linear Least Square
     *
     * @param pAccX Pointer to accelerometer X axis data
     * @param pAccY Pointer to accelerometer Y axis data
     * @param length Number of data points
     */
    void calculateAccelResult(const int16_t *pAccX, float meanAccX, const int16_t *pAccY, float meanAccY, size_t length)
    {
        float sumX = 0;
        float sumY = 0;
        float sumX2 = 0;
        float sumXY = 0;

        for (size_t idx = 0; idx < length; idx++)
        {
            // Remove mean
            float x = rawAccelToMs2(pAccX[idx] - meanAccX);
            float y = rawAccelToMs2(pAccY[idx] - meanAccY);

            // Calculate sums
            sumX += x;
            sumY += y;
            sumX2 += x * x;
            sumXY += x * y;
        }

        float numerator = length * sumXY - sumX * sumY;
        float denominator = length * sumX2 - sumX * sumX;
        if (denominator != 0.0)
        {
            // Calculate slope
            float slope = numerator / denominator;

            // Calculate theta angle
            float theta = atan(slope);

            for (size_t idx = 0; idx < length; idx++)
            {
                float x = rawAccelToMs2(pAccX[idx] - meanAccX);
                float y = rawAccelToMs2(pAccY[idx] - meanAccY);

                accelResult[idx] = x * cos(theta) + y * sin(theta);
            }
        }
        else
        {
            for (size_t idx = 0; idx < length; idx++)
            {
                accelResult[idx] = 0.0;
            }
        }
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

        Measurements::PsdBin coreBinAdc1;
        Measurements::PsdBin coreBinAdc2;
        Measurements::PsdBin coreBinAccX;
        Measurements::PsdBin coreBinAccY;
        Measurements::PsdBin coreBinGyroX;
        Measurements::PsdBin coreBinGyroY;
        Measurements::PsdBin coreBinAccResult;

        const float *resultPsdAdc1 = psdAdc1.getResult(&coreBinAdc1);
        const float *resultPsdAdc2 = psdAdc2.getResult(&coreBinAdc2);
        const float *resultPsdAccX = psdAccX.getResult(&coreBinAccX);
        const float *resultPsdAccY = psdAccY.getResult(&coreBinAccY);
        const float *resultPsdGyroX = psdGyroX.getResult(&coreBinGyroX);
        const float *resultPsdGyroY = psdGyroY.getResult(&coreBinGyroY);
        const float *resultPsdAccResult = psdAccResult.getResult(&coreBinAccResult);

        SystemTime::DateTimeString dateTimeString;
        time_t epochTime = SystemTime::getTimestamp(dateTimeString);
        Battery::Status batteryStatus = Battery::readStatus();
        char directoryName[30];
        char fileName[30];
        SD::File sdFile;

        // CSV
        snprintf(directoryName, sizeof(directoryName), "%s/%.8s", directoryCsv, dateTimeString);
        bool isOpen = sdFile.create(directoryName, dateTimeString, fileExtensionCsv);
        if (isOpen == true)
        {
            char string[100];

            snprintf(string, sizeof(string), "FW %s", FwVersion::getVersionString());
            sdFile.println(string);

            // File header
            float batteryVoltage = static_cast<float>(batteryStatus.voltage) / 1000;
            snprintf(string, sizeof(string), "BATT %.1fV", batteryVoltage);
            sdFile.println(string);
            snprintf(string, sizeof(string), "BATT %u%%", batteryStatus.level);
            sdFile.println(string);
            snprintf(string, sizeof(string), "START_TIME %u/%u/%u %u:%u:%u",
                     context.startDateTime.Day, context.startDateTime.Month, context.startDateTime.Year,
                     context.startDateTime.Hour, context.startDateTime.Minute, context.startDateTime.Second);
            sdFile.println(string);
            snprintf(string, sizeof(string), "Logging Rate,%u", settings.frequency);
            sdFile.println(string);
            sdFile.println(""); // End of header

            sdFile.println("Channel Name,ADC_1");
            sdFile.println("Channel Units,raw_12bit");
            snprintf(string, sizeof(string), "Maximum,%G", statisticAdc1.max());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", statisticAdc1.min());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Mean,%G", statisticAdc1.mean());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", statisticAdc1.deviation());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Core Frequency (%dpt PSD),%G,%G", context.segmentSize, coreBinAdc1.frequency, coreBinAdc1.amplitude);
            sdFile.println(string);
            snprintf(string, sizeof(string), "PSD_%d_%d", resultPoints, context.segmentSize);
            sdFile.print(string);
            for (size_t idx = 0; idx < resultPoints; idx++)
            {
                snprintf(string, sizeof(string), ",%G", resultPsdAdc1[idx]);
                sdFile.print(string);
            }
            sdFile.println(""); // End of PSD
            sdFile.println(""); // End of channel

            sdFile.println("Channel Name,ADC_2");
            sdFile.println("Channel Units,raw_12bit");
            snprintf(string, sizeof(string), "Maximum,%G", statisticAdc2.max());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", statisticAdc2.min());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Mean,%G", statisticAdc2.mean());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", statisticAdc2.deviation());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Core Frequency (%dpt PSD),%G,%G", context.segmentSize, coreBinAdc2.frequency, coreBinAdc2.amplitude);
            sdFile.println(string);
            snprintf(string, sizeof(string), "PSD_%d_%d", resultPoints, context.segmentSize);
            sdFile.print(string);
            for (size_t idx = 0; idx < resultPoints; idx++)
            {
                snprintf(string, sizeof(string), ",%G", resultPsdAdc2[idx]);
                sdFile.print(string);
            }
            sdFile.println(""); // End of PSD
            sdFile.println(""); // End of channel

            sdFile.println("Channel Name,ACC_X");
            sdFile.println("Channel Units,m/s^2");
            snprintf(string, sizeof(string), "Maximum,%G", rawAccelToMs2(statisticAccX.max()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", rawAccelToMs2(statisticAccX.min()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Mean,%G", rawAccelToMs2(statisticAccX.mean()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", rawAccelToMs2(statisticAccX.deviation()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Core Frequency (%dpt PSD),%G,%G", context.segmentSize, coreBinAccX.frequency, coreBinAccX.amplitude);
            sdFile.println(string);
            snprintf(string, sizeof(string), "PSD_%d_%d", resultPoints, context.segmentSize);
            sdFile.print(string);
            for (size_t idx = 0; idx < resultPoints; idx++)
            {
                snprintf(string, sizeof(string), ",%G", resultPsdAccX[idx]);
                sdFile.print(string);
            }
            sdFile.println(""); // End of PSD
            sdFile.println(""); // End of channel

            sdFile.println("Channel Name,ACC_Y");
            sdFile.println("Channel Units,m/s^2");
            snprintf(string, sizeof(string), "Maximum,%G", rawAccelToMs2(statisticAccY.max()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", rawAccelToMs2(statisticAccY.min()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Mean,%G", rawAccelToMs2(statisticAccY.mean()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", rawAccelToMs2(statisticAccY.deviation()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Core Frequency (%dpt PSD),%G,%G", context.segmentSize, coreBinAccY.frequency, coreBinAccY.amplitude);
            sdFile.println(string);
            snprintf(string, sizeof(string), "PSD_%d_%d", resultPoints, context.segmentSize);
            sdFile.print(string);
            for (size_t idx = 0; idx < resultPoints; idx++)
            {
                snprintf(string, sizeof(string), ",%G", resultPsdAccY[idx]);
                sdFile.print(string);
            }
            sdFile.println(""); // End of PSD
            sdFile.println(""); // End of channel

            sdFile.println("Channel Name,ACC_Z");
            sdFile.println("Channel Units,m/s^2");
            snprintf(string, sizeof(string), "Maximum,%G", rawAccelToMs2(statisticAccZ.max()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", rawAccelToMs2(statisticAccZ.min()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Mean,%G", rawAccelToMs2(statisticAccZ.mean()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", rawAccelToMs2(statisticAccZ.deviation()));
            sdFile.println(string);
            sdFile.println(""); // End of channel

            sdFile.println("Channel Name,GYRO_X");
            sdFile.println("Channel Units,rad/s");
            snprintf(string, sizeof(string), "Maximum,%G", rawGyroToRads(statisticGyroX.max()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", rawGyroToRads(statisticGyroX.min()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Mean,%G", rawGyroToRads(statisticGyroX.mean()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", rawGyroToRads(statisticGyroX.deviation()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Core Frequency (%dpt PSD),%G,%G", context.segmentSize, coreBinGyroX.frequency, coreBinGyroX.amplitude);
            sdFile.println(string);
            snprintf(string, sizeof(string), "PSD_%d_%d", resultPoints, context.segmentSize);
            sdFile.print(string);
            for (size_t idx = 0; idx < resultPoints; idx++)
            {
                snprintf(string, sizeof(string), ",%G", resultPsdGyroX[idx]);
                sdFile.print(string);
            }
            sdFile.println(""); // End of PSD
            sdFile.println(""); // End of channel

            sdFile.println("Channel Name,GYRO_Y");
            sdFile.println("Channel Units,rad/s");
            snprintf(string, sizeof(string), "Maximum,%G", rawGyroToRads(statisticGyroY.max()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", rawGyroToRads(statisticGyroY.min()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Mean,%G", rawGyroToRads(statisticGyroY.mean()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", rawGyroToRads(statisticGyroY.deviation()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Core Frequency (%dpt PSD),%G,%G", context.segmentSize, coreBinGyroY.frequency, coreBinGyroY.amplitude);
            sdFile.println(string);
            snprintf(string, sizeof(string), "PSD_%d_%d", resultPoints, context.segmentSize);
            sdFile.print(string);
            for (size_t idx = 0; idx < resultPoints; idx++)
            {
                snprintf(string, sizeof(string), ",%G", resultPsdGyroY[idx]);
                sdFile.print(string);
            }
            sdFile.println(""); // End of PSD
            sdFile.println(""); // End of channel

            sdFile.println("Channel Name,GYRO_Z");
            sdFile.println("Channel Units,rad/s");
            snprintf(string, sizeof(string), "Maximum,%G", rawGyroToRads(statisticGyroZ.max()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", rawGyroToRads(statisticGyroZ.min()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Mean,%G", rawGyroToRads(statisticGyroZ.mean()));
            sdFile.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", rawGyroToRads(statisticGyroZ.deviation()));
            sdFile.println(string);
            sdFile.println(""); // End of channel

            sdFile.println("Channel Name,ROLL");
            sdFile.println("Channel Units,deg");
            snprintf(string, sizeof(string), "Maximum,%G", statisticRoll.max());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", statisticRoll.min());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Mean,%G", statisticRoll.mean());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", statisticRoll.deviation());
            sdFile.println(string);
            sdFile.println(""); // End of channel

            sdFile.println("Channel Name,PITCH");
            sdFile.println("Channel Units,deg");
            snprintf(string, sizeof(string), "Maximum,%G", statisticPitch.max());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", statisticPitch.min());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Mean,%G", statisticPitch.mean());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", statisticPitch.deviation());
            sdFile.println(string);
            sdFile.println(""); // End of channel

            sdFile.println("Channel Name,E_ACC_RES");
            sdFile.println("Channel Units,m/s^2");
            snprintf(string, sizeof(string), "Maximum,%G", statisticAccelResult.max());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Minimum,%G", statisticAccelResult.min());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Mean,%G", statisticAccelResult.mean());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Standard Deviation,%G", statisticAccelResult.deviation());
            sdFile.println(string);
            snprintf(string, sizeof(string), "Core Frequency (%dpt PSD),%G,%G", context.segmentSize, coreBinAccResult.frequency, coreBinAccResult.amplitude);
            sdFile.println(string);
            snprintf(string, sizeof(string), "PSD_%d_%d", resultPoints, context.segmentSize);
            sdFile.print(string);
            for (size_t idx = 0; idx < resultPoints; idx++)
            {
                snprintf(string, sizeof(string), ",%G", resultPsdAccResult[idx]);
                sdFile.print(string);
            }
            sdFile.println(""); // End of PSD
            sdFile.println(""); // End of channel

            sdFile.close();
        }

        // BIN/PSD/ADC1
        snprintf(directoryName, sizeof(directoryName), "%s/%s/%.8s", directoryBin,
                 getDirectory(SensorType::Adc1, DataType::Psd), dateTimeString);
        snprintf(fileName, sizeof(fileName), "%u", epochTime);
        isOpen = sdFile.create(directoryName, fileName, fileExtensionBin);
        if (isOpen == true)
        {
            sdFile.write(&context.startEpochTime, sizeof(context.startEpochTime));

            sdFile.write(&coreBinAdc1.frequency, sizeof(coreBinAdc1.frequency));
            sdFile.write(&coreBinAdc1.amplitude, sizeof(coreBinAdc1.amplitude));
            sdFile.write(resultPsdAdc1, resultPoints * sizeof(*resultPsdAdc1));

            sdFile.close();
        }

        // BIN/STAT/ADC1
        snprintf(directoryName, sizeof(directoryName), "%s/%s/%.8s", directoryBin,
                 getDirectory(SensorType::Adc1, DataType::Statistic), dateTimeString);
        snprintf(fileName, sizeof(fileName), "%u", epochTime);
        isOpen = sdFile.create(directoryName, fileName, fileExtensionBin);
        if (isOpen == true)
        {
            uint16_t max = statisticAdc1.max();
            uint16_t min = statisticAdc1.min();
            uint16_t mean = statisticAdc1.mean();
            uint16_t deviation = statisticAdc1.deviation();

            sdFile.write(&max, sizeof(max));
            sdFile.write(&min, sizeof(min));
            sdFile.write(&mean, sizeof(mean));
            sdFile.write(&deviation, sizeof(deviation));

            sdFile.close();
        }

        // BIN/PSD/ADC2
        snprintf(directoryName, sizeof(directoryName), "%s/%s/%.8s", directoryBin,
                 getDirectory(SensorType::Adc2, DataType::Psd), dateTimeString);
        snprintf(fileName, sizeof(fileName), "%u", epochTime);
        isOpen = sdFile.create(directoryName, fileName, fileExtensionBin);
        if (isOpen == true)
        {
            sdFile.write(&context.startEpochTime, sizeof(context.startEpochTime));

            sdFile.write(&coreBinAdc2.frequency, sizeof(coreBinAdc2.frequency));
            sdFile.write(&coreBinAdc2.amplitude, sizeof(coreBinAdc2.amplitude));
            sdFile.write(resultPsdAdc2, resultPoints * sizeof(*resultPsdAdc2));

            sdFile.close();
        }

        // BIN/STAT/ADC2
        snprintf(directoryName, sizeof(directoryName), "%s/%s/%.8s", directoryBin,
                 getDirectory(SensorType::Adc2, DataType::Statistic), dateTimeString);
        snprintf(fileName, sizeof(fileName), "%u", epochTime);
        isOpen = sdFile.create(directoryName, fileName, fileExtensionBin);
        if (isOpen == true)
        {
            uint16_t max = statisticAdc2.max();
            uint16_t min = statisticAdc2.min();
            uint16_t mean = statisticAdc2.mean();
            uint16_t deviation = statisticAdc2.deviation();

            sdFile.write(&max, sizeof(max));
            sdFile.write(&min, sizeof(min));
            sdFile.write(&mean, sizeof(mean));
            sdFile.write(&deviation, sizeof(deviation));

            sdFile.close();
        }

        // BIN/PSD/ACC
        snprintf(directoryName, sizeof(directoryName), "%s/%s/%.8s", directoryBin,
                 getDirectory(SensorType::Accel, DataType::Psd), dateTimeString);
        snprintf(fileName, sizeof(fileName), "%u", epochTime);
        isOpen = sdFile.create(directoryName, fileName, fileExtensionBin);
        if (isOpen == true)
        {
            sdFile.write(&context.startEpochTime, sizeof(context.startEpochTime));

            sdFile.write(&coreBinAccX.frequency, sizeof(coreBinAccX.frequency));
            sdFile.write(&coreBinAccX.amplitude, sizeof(coreBinAccX.amplitude));
            sdFile.write(resultPsdAccX, resultPoints * sizeof(*resultPsdAccX));

            sdFile.write(&coreBinAccY.frequency, sizeof(coreBinAccY.frequency));
            sdFile.write(&coreBinAccY.amplitude, sizeof(coreBinAccY.amplitude));
            sdFile.write(resultPsdAccY, resultPoints * sizeof(*resultPsdAccY));

            sdFile.close();
        }

        // BIN/STAT/ACC
        snprintf(directoryName, sizeof(directoryName), "%s/%s/%.8s", directoryBin,
                 getDirectory(SensorType::Accel, DataType::Statistic), dateTimeString);
        snprintf(fileName, sizeof(fileName), "%u", epochTime);
        isOpen = sdFile.create(directoryName, fileName, fileExtensionBin);
        if (isOpen == true)
        {
            uint16_t max = statisticAccX.max();
            uint16_t min = statisticAccX.min();
            uint16_t mean = statisticAccX.mean();
            uint16_t deviation = statisticAccX.deviation();

            sdFile.write(&max, sizeof(max));
            sdFile.write(&min, sizeof(min));
            sdFile.write(&mean, sizeof(mean));
            sdFile.write(&deviation, sizeof(deviation));

            max = statisticAccY.max();
            min = statisticAccY.min();
            mean = statisticAccY.mean();
            deviation = statisticAccY.deviation();

            sdFile.write(&max, sizeof(max));
            sdFile.write(&min, sizeof(min));
            sdFile.write(&mean, sizeof(mean));
            sdFile.write(&deviation, sizeof(deviation));

            max = statisticAccZ.max();
            min = statisticAccZ.min();
            mean = statisticAccZ.mean();
            deviation = statisticAccZ.deviation();

            sdFile.write(&max, sizeof(max));
            sdFile.write(&min, sizeof(min));
            sdFile.write(&mean, sizeof(mean));
            sdFile.write(&deviation, sizeof(deviation));

            sdFile.close();
        }

        // BIN/PSD/GYR
        snprintf(directoryName, sizeof(directoryName), "%s/%s/%.8s", directoryBin,
                 getDirectory(SensorType::Gyro, DataType::Psd), dateTimeString);
        snprintf(fileName, sizeof(fileName), "%u", epochTime);
        isOpen = sdFile.create(directoryName, fileName, fileExtensionBin);
        if (isOpen == true)
        {
            sdFile.write(&context.startEpochTime, sizeof(context.startEpochTime));

            sdFile.write(&coreBinGyroX.frequency, sizeof(coreBinGyroX.frequency));
            sdFile.write(&coreBinGyroX.amplitude, sizeof(coreBinGyroX.amplitude));
            sdFile.write(resultPsdGyroX, resultPoints * sizeof(*resultPsdGyroX));

            sdFile.write(&coreBinGyroY.frequency, sizeof(coreBinGyroY.frequency));
            sdFile.write(&coreBinGyroY.amplitude, sizeof(coreBinGyroY.amplitude));
            sdFile.write(resultPsdGyroY, resultPoints * sizeof(*resultPsdGyroY));

            sdFile.close();
        }

        // BIN/STAT/GYR
        snprintf(directoryName, sizeof(directoryName), "%s/%s/%.8s", directoryBin,
                 getDirectory(SensorType::Gyro, DataType::Statistic), dateTimeString);
        snprintf(fileName, sizeof(fileName), "%u", epochTime);
        isOpen = sdFile.create(directoryName, fileName, fileExtensionBin);
        if (isOpen == true)
        {
            uint16_t max = statisticGyroX.max();
            uint16_t min = statisticGyroX.min();
            uint16_t mean = statisticGyroX.mean();
            uint16_t deviation = statisticGyroX.deviation();

            sdFile.write(&max, sizeof(max));
            sdFile.write(&min, sizeof(min));
            sdFile.write(&mean, sizeof(mean));
            sdFile.write(&deviation, sizeof(deviation));

            max = statisticGyroY.max();
            min = statisticGyroY.min();
            mean = statisticGyroY.mean();
            deviation = statisticGyroY.deviation();

            sdFile.write(&max, sizeof(max));
            sdFile.write(&min, sizeof(min));
            sdFile.write(&mean, sizeof(mean));
            sdFile.write(&deviation, sizeof(deviation));

            max = statisticGyroZ.max();
            min = statisticGyroZ.min();
            mean = statisticGyroZ.mean();
            deviation = statisticGyroZ.deviation();

            sdFile.write(&max, sizeof(max));
            sdFile.write(&min, sizeof(min));
            sdFile.write(&mean, sizeof(mean));
            sdFile.write(&deviation, sizeof(deviation));

            sdFile.close();
        }

        // BIN/PSD/ACC_RES
        snprintf(directoryName, sizeof(directoryName), "%s/%s/%.8s", directoryBin,
                 getDirectory(SensorType::AccelResult, DataType::Psd), dateTimeString);
        snprintf(fileName, sizeof(fileName), "%u", epochTime);
        isOpen = sdFile.create(directoryName, fileName, fileExtensionBin);
        if (isOpen == true)
        {
            sdFile.write(&context.startEpochTime, sizeof(context.startEpochTime));

            sdFile.write(&coreBinAccResult.frequency, sizeof(coreBinAccResult.frequency));
            sdFile.write(&coreBinAccResult.amplitude, sizeof(coreBinAccResult.amplitude));
            sdFile.write(resultPsdAccResult, resultPoints * sizeof(*resultPsdAccResult));

            sdFile.close();
        }

        // BIN/STAT/ACC_RES
        snprintf(directoryName, sizeof(directoryName), "%s/%s/%.8s", directoryBin,
                 getDirectory(SensorType::AccelResult, DataType::Statistic), dateTimeString);
        snprintf(fileName, sizeof(fileName), "%u", epochTime);
        isOpen = sdFile.create(directoryName, fileName, fileExtensionBin);
        if (isOpen == true)
        {
            uint16_t max = statisticAccelResult.max();
            uint16_t min = statisticAccelResult.min();
            uint16_t mean = statisticAccelResult.mean();
            uint16_t deviation = statisticAccelResult.deviation();

            sdFile.write(&max, sizeof(max));
            sdFile.write(&min, sizeof(min));
            sdFile.write(&mean, sizeof(mean));
            sdFile.write(&deviation, sizeof(deviation));

            sdFile.close();
        }

        // BIN/STAT/ANG
        snprintf(directoryName, sizeof(directoryName), "%s/%s/%.8s", directoryBin,
                 getDirectory(SensorType::Angle, DataType::Statistic), dateTimeString);
        snprintf(fileName, sizeof(fileName), "%u", epochTime);
        isOpen = sdFile.create(directoryName, fileName, fileExtensionBin);
        if (isOpen == true)
        {
            uint16_t max = statisticRoll.max();
            uint16_t min = statisticRoll.min();
            uint16_t mean = statisticRoll.mean();
            uint16_t deviation = statisticRoll.deviation();

            sdFile.write(&max, sizeof(max));
            sdFile.write(&min, sizeof(min));
            sdFile.write(&mean, sizeof(mean));
            sdFile.write(&deviation, sizeof(deviation));

            max = statisticPitch.max();
            min = statisticPitch.min();
            mean = statisticPitch.mean();
            deviation = statisticPitch.deviation();

            sdFile.write(&max, sizeof(max));
            sdFile.write(&min, sizeof(min));
            sdFile.write(&mean, sizeof(mean));
            sdFile.write(&deviation, sizeof(deviation));

            sdFile.close();
        }

        LOG_DEBUG("ADC_1: Max %d, Min %d, Mean %f, Standard Deviation %f, Core Frequency %lfHz - %lf",
                  statisticAdc1.max(), statisticAdc1.min(), statisticAdc1.mean(), statisticAdc1.deviation(),
                  coreBinAdc1.frequency, coreBinAdc1.amplitude);
        LOG_DEBUG("ADC_2: Max %d, Min %d, Mean %f, Standard Deviation %f, Core Frequency %lfHz - %lf",
                  statisticAdc2.max(), statisticAdc2.min(), statisticAdc2.mean(), statisticAdc2.deviation(),
                  coreBinAdc2.frequency, coreBinAdc2.amplitude);

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

        LOG_DEBUG("ACC RESULT: Max %f, Min %f, Mean %f, Standard Deviation %f, Core Frequency %lfHz - %lf",
                  statisticAccelResult.max(), statisticAccelResult.min(), statisticAccelResult.mean(),
                  statisticAccelResult.deviation(), coreBinAccResult.frequency, coreBinAccResult.amplitude);
    }

    /**
     * @brief Reset measurements statistic
     */
    void resetStatistics()
    {
        statisticAdc1.reset();
        statisticAdc2.reset();
        statisticAccX.reset();
        statisticAccY.reset();
        statisticAccZ.reset();
        statisticGyroX.reset();
        statisticGyroY.reset();
        statisticGyroZ.reset();
        statisticRoll.reset();
        statisticPitch.reset();
        statisticAccelResult.reset();
    }

    /**
     * @brief Start sensors readings
     *
     * @return true if sensors are started successfully, false otherwsie
     */
    bool enableSensors()
    {
        bool status = false;

        adc1.enable();
        adc2.enable();

        status = Analog::VddController::applyVoltage();
        if (status != true)
        {
            LOG_ERROR("Sensor voltage apply fail!");
            return false;
        }

        status = imu.accelerometer_enable();
        if (status != true)
        {
            LOG_ERROR("Accelerometer enable failed");
            return false;
        }

        status = imu.gyroscope_enable();
        if (status != true)
        {
            LOG_ERROR("Gyroscope enable failed");
            return false;
        }

        status = waitImuReady();
        if (status != true)
        {
            LOG_ERROR("IMU wait ready failed");
            return false;
        }

        return status;
    }

    /**
     * @brief Start sensors readings
     */
    void disableSensors()
    {
        Analog::VddController::cutOffVoltage();
        adc1.disable();
        adc2.disable();
        imu.accelerometer_disable();
        imu.gyroscope_disable();
    }

    /**
     * @brief Fill buffer data with IMU sample
     *
     * @param[in] offset Data offset in the buffer
     * @param[in] adcSample Analog sensor sample
     * @param[in] imuSample IMU sensor sample
     */
    void fillBuffer(size_t offset, const AdcSample &adcSample, const ImuSample &imuSample)
    {
        // Fill ADC buffer data
        buffer.adc1[offset] = adcSample.chan1;
        buffer.adc2[offset] = adcSample.chan2;
        LOG_TRACE("Adc: chan1 %u, chan2 %u", adcSample.chan1, adcSample.chan2);

        // Fill Accel buffer data
        buffer.accX[offset] = imuSample.accel.x;
        buffer.accY[offset] = imuSample.accel.y;
        buffer.accZ[offset] = imuSample.accel.z;
        LOG_TRACE("Acc: X %d, Y %d, Z %d", imuSample.accel.x, imuSample.accel.y, imuSample.accel.z);

        // Fill Gyro buffer data
        buffer.gyrX[offset] = imuSample.gyro.x;
        buffer.gyrY[offset] = imuSample.gyro.y;
        buffer.gyrZ[offset] = imuSample.gyro.z;
        LOG_TRACE("Gyro: X %d, Y %d, Z %d", imuSample.gyro.x, imuSample.gyro.y, imuSample.gyro.z);

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
        LOG_TRACE("Angle: roll %.1f, pitch %.1f", buffer.roll[offset], buffer.pitch[offset]);
    }

    /**
     * @brief Sensors samples reading task function
     *
     * @param pvParameters Task parameters
     */
    void samplingTask(void *pvParameters)
    {
        TickType_t xLastWakeTime;
        BaseType_t xWasDelayed;

        AdcSample adcSample = {0};
        ImuSample imuSample = {0};

        size_t segmentIndex = 0;
        size_t sampleIndex = 0;

        (void *)pvParameters; // unused

        while (1)
        {
            LOG_INFO("Sampling task is IDLE");

            // Report sampling task is in IDLE state
            eventGroup.set(EventBits::taskIsIdle);

            // Wait for start event
            EventBits_t events = eventGroup.wait(EventBits::startTask);

            // Enable sensors when task is started
            bool status = enableSensors();
            if (status != true)
            {
                LOG_ERROR("Sensors start failed");
                // Disable sensors when start failed
                disableSensors();
                continue;
            }

            LOG_INFO("Sampling task ia RUNNING");

            // Report sampling task is in RUNNING state
            eventGroup.set(EventBits::taskIsRunning);

            // Reset segment and sample index
            segmentIndex = 0;
            sampleIndex = 0;
            // Initialise the xLastWakeTime variable with the current time.
            xLastWakeTime = xTaskGetTickCount();

            while ((events & EventBits::stopTask) == 0)
            {
                // Wait for the next cycle
                xWasDelayed = xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(context.sampleTimeMs));

                // Perform action here. xWasDelayed value can be used to determine
                // whether a deadline was missed if the code here took too long

                // Read new ADC samples
                adcSample.chan1 = adc1.read();
                adcSample.chan2 = adc2.read();

                // Read new IMU sample
                status = readImu(imuSample);
                if (status != true)
                {
                    LOG_WARNING("IMU reading failed");
                }

                size_t offset = segmentIndex * context.segmentSize + sampleIndex;

                // Fill buffer data with IMU sample
                fillBuffer(offset, adcSample, imuSample);

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
                events = eventGroup.wait(EventBits::stopTask, 0);
            }

            // Disable sensors when task is stopped
            disableSensors();
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

                                               // Stop sensors sampling
                                               stopSampling();

                                               // Update measure frequency setting
                                               settings.frequency = value;
                                               Settings::update(settingsId, settings);

                                               // Restart measurements
                                               setupMeasurements(settings.pointsPsd, settings.frequency);

                                               // Start sensors sampling
                                               startSampling();
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::MeasureInterval,
                                           [](const char *dataString)
                                           {
                                               uint32_t value = atoi(dataString);

                                               // Update measure interval setting
                                               settings.measureInterval = value;
                                               Settings::update(settingsId, settings);
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::PauseInterval,
                                           [](const char *dataString)
                                           {
                                               uint32_t value = atoi(dataString);

                                               // Update pause interval setting
                                               settings.pauseInterval = value;
                                               Settings::update(settingsId, settings);
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

                                               // Stop sensors sampling
                                               stopSampling();

                                               // Update points to calculate PSD segment size
                                               settings.pointsPsd = value;
                                               Settings::update(settingsId, settings);

                                               // Restart measurements
                                               setupMeasurements(settings.pointsPsd, settings.frequency);

                                               // Start sensors sampling
                                               startSampling();
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
                                               Settings::update(settingsId, settings);
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::StatisticState,
                                           [](const char *dataString)
                                           {
                                               uint8_t value = atoi(dataString);

                                               // Update state of statistic setting
                                               settings.statisticState = value;
                                               Settings::update(settingsId, settings);
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
    Settings::read(settingsId, settings);

    // Register local serial handlers
    registerSerialReadHandlers();
    registerSerialWriteHandlers();

    // Initialize sensors
    adc1.initialize();
    adc2.initialize();

    bool status = setupImu();
    if (status == true)
    {
        LOG_INFO("IMU initialized");
    }
    else
    {
        LOG_ERROR("IMU initialization failed");
    }

    status = IoExpander::initialize();
    if (status == false)
    {
        LOG_ERROR("IO expander initialization failed!");
    }

    status = Analog::GainSelector::initialize();
    if (status == false)
    {
        LOG_ERROR("Gain selector initialization failed!");
    }

    status = Analog::InputSelector::initialize();
    if (status == false)
    {
        LOG_ERROR("Input type selector initialization failed!");
    }

    status = Analog::VddController::initialize();
    if (status == false)
    {
        LOG_ERROR("Voltage controller initialization failed!");
    }

    if (status == true)
    {
        // Create sampling task
        xTaskCreatePinnedToCore(samplingTask, "samplingTask", 4096, NULL, 1, NULL, 0);

        // Wait sampling task is idle
        EventBits_t events = eventGroup.wait(EventBits::taskIsIdle);
        if (events & EventBits::taskIsIdle)
        {
            setupMeasurements(settings.pointsPsd, settings.frequency);

            // Start sensors sampling
            startSampling();
        }
        else
        {
            LOG_ERROR("Sampling task creation failed");
        }
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
        performCalculations(segmentIndex);

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
                // Stop sensors sampling
                stopSampling();

                SD::FS::stop();

                Board::deepSleep(settings.pauseInterval);
            }
            else
            {
                // Reset measurements statistic
                resetStatistics();
            }
        }
    }
}
