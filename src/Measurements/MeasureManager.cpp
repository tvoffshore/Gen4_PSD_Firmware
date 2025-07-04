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

#include <Events.h>
#include <MadgwickAHRS.h>
#include <Log.hpp>
#include <SdFat.h>
#include <Settings.hpp>
#include <SystemTime.hpp>

#include "Analog/GainSelector.hpp"
#include "Analog/InputSelector.hpp"
#include "Analog/Sensor.hpp"
#include "Analog/VddController.hpp"
#include "Board/Battery.hpp"
#include "Board/Power.hpp"
#include "FwVersion.hpp"
#include "Imu/Imu.hpp"
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

    namespace DataTypeMask
    {
        constexpr uint8_t Psd = (1 << static_cast<size_t>(DataType::Psd));             // PSD data type
        constexpr uint8_t Statistic = (1 << static_cast<size_t>(DataType::Statistic)); // Statistic data type
        constexpr uint8_t Raw = (1 << static_cast<size_t>(DataType::Raw));             // Raw data type
        constexpr uint8_t All = Raw | Statistic | Psd;
    } // namespace DataTypeMask

    namespace SensorMask
    {
        constexpr uint8_t Accel = (1 << 0);       // Accelerometer sensor
        constexpr uint8_t Gyro = (1 << 1);        // Gyroscope sensor
        constexpr uint8_t Angle = (1 << 2);       // Roll/Pitch angle sensor
        constexpr uint8_t Adc1 = (1 << 3);        // Analog 1 sensor
        constexpr uint8_t Adc2 = (1 << 4);        // Analog 2 sensor
        constexpr uint8_t AccelResult = (1 << 5); // Accelerometer sensor
        constexpr uint8_t All = AccelResult | Adc2 | Adc1 | Angle | Gyro | Accel;
    } // namespace SensorMask

    // Total time threshold to complete measurements, percents
    constexpr float completePcnt = 99.0;

    // Buffer contains 2 sets of samples (ping-pong)
    constexpr size_t bufferSize = 2 * Measurements::samplesCountMax;

    // Default PSD points count (PSD segment size = 2^x)
    constexpr uint8_t psdPointsDefault = 8;
    // Minimum PSD points
    constexpr uint8_t psdPointsMin = 1;
    // Maximum PSD points
    constexpr uint8_t psdPointsMax = 10;

    // Default PSD cutoff to store the results
    constexpr uint16_t psdCutoffDefault = 128;
    // Minimum PSD cutoff
    constexpr uint16_t psdCutoffMin = 1;
    // Maximum PSD cutoff
    constexpr uint16_t psdCutoffMax = 1024;

    // Default data type to calculate and store on SD
    constexpr uint8_t dataTypeMaskDefault = DataTypeMask::Psd | DataTypeMask::Statistic;
    // Default sensor type for measurements
    constexpr uint8_t sensorMaskDefault = SensorMask::All;

    // Default sampling frequency, Hz
    constexpr uint8_t sampleFrequencyDefault = 40;
    // Minimum sampling frequency, Hz
    constexpr uint8_t sampleFrequencyMin = 1;
    // Maximum sampling frequency, Hz
    constexpr uint8_t sampleFrequencyMax = 100;

    // Default time to take measurements, seconds
    constexpr uint32_t measureIntervalDefault = 600;
    // Minimum time to take measurements, seconds
    constexpr uint32_t measureIntervalMin = 1;
    // Default time between measurements, seconds
    constexpr uint32_t pauseIntervalDefault = 300;

    // Settings identifier in internal storage
    constexpr auto settingsId = Settings::Id::Measurements;

    // Milliseconds per second
    constexpr size_t millisPerSecond = 1000;

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

#pragma pack(push, 1)
    /**
     * @brief Non volatile settings structure
     */
    struct MeasureSettings
    {
        uint32_t measureInterval; // Time for measuring, seconds
        uint32_t pauseInterval;   // Time between measurements, seconds
        uint16_t psdCutoff;       // PSD cutoff to store the results
        uint8_t psdPoints;        // PSD points count (PSD segment size = 2^x)
        uint8_t sampleFrequency;  // Sampling frequency, Hz
        uint8_t sensorMask;       // Sensor type bitmask for measurements
        uint8_t dataTypeMask;     // Data type bitmask to calculate and store on SD
    };

    /**
     * @brief File header structure for measurements saving
     */
    struct FileHeader
    {
        uint32_t startEpochTime;
        uint32_t endEpochTime;
        uint16_t sampleTimeMs;
        uint8_t dataType;
        uint8_t sensorType;
    };

    /**
     * @brief Header structure for PSD measurements saving
     */
    struct PsdHeader
    {
        float coreFrequency;
        float coreAmplitude;
        uint32_t points;
    };

    /**
     * @brief Data structure for Statistic measurements saving
     */
    struct StatisticData
    {
        float max;
        float min;
        float mean;
        float deviation;
    };
#pragma pack(pop)

    /**
     * Samples buffer structure
     */
    struct Buffer
    {
        uint16_t adc1[bufferSize];
        uint16_t adc2[bufferSize];

        int16_t accX[bufferSize];
        int16_t accY[bufferSize];
        int16_t accZ[bufferSize];

        int16_t gyrX[bufferSize];
        int16_t gyrY[bufferSize];
        int16_t gyrZ[bufferSize];

        float roll[bufferSize];
        float pitch[bufferSize];
    };

    /**
     * @brief Measurements context
     */
    struct Context
    {
        size_t segmentCount;                // Count of ready segments
        size_t segmentSize;                 // Size of segment, samples
        size_t segmentTimeMs;               // Time of segment accumulating, milliseconds
        size_t sampleTimeMs;                // Interval between samples, milliseconds
        SystemTime::DateTime startDateTime; // Start measurements date and time
        time_t startEpochTime;              // Start measurements epoch time
        time_t endEpochTime;                // End of measurements epoch time
        MeasureSettings config;             // Current measurements configuration

        /**
         * @brief Setup new measurements' context
         *
         * @param[in] settings Measurements settings
         */
        void setup(const MeasureSettings &settings)
        {
            static const size_t pow2[] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192};

            assert(settings.psdPoints < sizeof(pow2) / sizeof(*pow2));

            config = settings;

            // Reset count of ready segments
            segmentCount = 0;
            // Determine segment size
            segmentSize = pow2[config.psdPoints];
            // Calculate interval between samples
            sampleTimeMs = millisPerSecond / config.sampleFrequency;
            // Calculate time of segment accumulating
            segmentTimeMs = segmentSize * sampleTimeMs;

            // Obtain measurements start date and time
            startEpochTime = SystemTime::getDateTime(startDateTime);
            endEpochTime = 0;

            LOG_INFO("Setup: sensors %u, data type %u, segment %d samples, sample time %d ms, segment time %d ms",
                     config.sensorMask, config.dataTypeMask, segmentSize, sampleTimeMs, segmentTimeMs);
        }
    };

    // Analog sensor channel 1
    Analog::Sensor adc1(pinAdc1Input, pinAdc1Enable);
    // Analog sensor channel 2
    Analog::Sensor adc2(pinAdc2Input, pinAdc2Enable);

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

    // File on SD to store raw measurements data
    SD::File sdRawFile;

    // Sensors' sampling active flag (true - active)
    bool isSamplingActive = false;

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
    Measurements::Statistic<float> statisticAccResult;

    // Measurements settings
    MeasureSettings settings = {
        .measureInterval = measureIntervalDefault,
        .pauseInterval = pauseIntervalDefault,
        .psdCutoff = psdCutoffDefault,
        .psdPoints = psdPointsDefault,
        .sampleFrequency = sampleFrequencyDefault,
        .sensorMask = sensorMaskDefault,
        .dataTypeMask = dataTypeMaskDefault,
    };

    // Main process functions
    bool startSampling();
    bool stopSampling();
    void setupMeasurements();
    void finishMeasurements();
    void restartMeasurements();
    void processMeasurements(size_t index);
    void calculateAccelResult(const int16_t *pAccX, const int16_t *pAccY, size_t length);
    void saveMeasurements();
    bool savePsdFile(SensorType sensorType, const PsdBin &coreBin, const float *psd, size_t points);
    bool saveStatisticFile(SensorType sensorType, const StatisticData &data);

    // Sampling process functions
    bool enableSensors();
    void disableSensors();
    bool readSensors(size_t offset);
    void samplingTask(void *pvParameters);

    // Serial handlers
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

    inline bool isAccelSet()
    {
        return (context.config.sensorMask & SensorMask::Accel) ||
               (context.config.sensorMask & SensorMask::Angle) ||
               (context.config.sensorMask & SensorMask::AccelResult);
    }

    inline bool isGyroSet()
    {
        return (context.config.sensorMask & SensorMask::Gyro) ||
               (context.config.sensorMask & SensorMask::Angle);
    }

    inline bool isAdcSet()
    {
        return (context.config.sensorMask & SensorMask::Adc1) ||
               (context.config.sensorMask & SensorMask::Adc2);
    }

    /**
     * @brief Set PSD points setting
     *
     * @param psdPoints New PSD points
     * @return true if new setting was set, false otherwise
     */
    bool setPsdPoints(uint8_t psdPoints)
    {
        bool result = false;

        if (psdPoints < psdPointsMin)
        {
            psdPoints = psdPointsMin;
        }
        else if (psdPoints > psdPointsMax)
        {
            psdPoints = psdPointsMax;
        }

        if (settings.psdPoints != psdPoints)
        {
            LOG_DEBUG("Update PSD points: %u -> %u", settings.psdPoints, psdPoints);
            // Update setting
            settings.psdPoints = psdPoints;
            Settings::update(settingsId, settings);
            result = true;
        }

        return result;
    }

    /**
     * @brief Set PSD cutoff setting
     *
     * @param psdCutoff New PSD cutoff
     * @return true if new setting was set, false otherwise
     */
    bool setPsdCutoff(uint16_t psdCutoff)
    {
        bool result = false;

        if (psdCutoff < psdCutoffMin)
        {
            psdCutoff = psdCutoffMin;
        }
        else if (psdCutoff > psdCutoffMax)
        {
            psdCutoff = psdCutoffMax;
        }

        if (settings.psdCutoff != psdCutoff)
        {
            LOG_DEBUG("Update PSD cutoff: %u -> %u", settings.psdCutoff, psdCutoff);
            // Update setting
            settings.psdCutoff = psdCutoff;
            Settings::update(settingsId, settings);
            result = true;
        }

        return result;
    }

    /**
     * @brief Set data type mask setting
     *
     * @param dataTypeMask New data type mask
     * @return true if new setting was set, false otherwise
     */
    bool setDataTypeMask(uint8_t dataTypeMask)
    {
        bool result = false;

        if (dataTypeMask == 0 || dataTypeMask > DataTypeMask::All)
        {
            dataTypeMask = DataTypeMask::All;
        }

        if (settings.dataTypeMask != dataTypeMask)
        {
            LOG_DEBUG("Update data type: %u -> %u", settings.dataTypeMask, dataTypeMask);
            // Update setting
            settings.dataTypeMask = dataTypeMask;
            Settings::update(settingsId, settings);
            result = true;
        }

        return result;
    }

    /**
     * @brief Set sensor mask setting
     *
     * @param sensorMask New sensor mask
     * @return true if new setting was set, false otherwise
     */
    bool setSensorMask(uint8_t sensorMask)
    {
        bool result = false;

        if (sensorMask == 0 || sensorMask > SensorMask::All)
        {
            sensorMask = SensorMask::All;
        }

        if (settings.sensorMask != sensorMask)
        {
            LOG_DEBUG("Update sensor type: %u -> %u", settings.sensorMask, sensorMask);
            // Update setting
            settings.sensorMask = sensorMask;
            Settings::update(settingsId, settings);
            result = true;
        }

        return result;
    }

    /**
     * @brief Set sampling frequency setting
     *
     * @param sampleFrequency New sampling frequency
     * @return true if new setting was set, false otherwise
     */
    bool setSampleFrequency(uint8_t sampleFrequency)
    {
        bool result = false;

        if (sampleFrequency < sampleFrequencyMin)
        {
            sampleFrequency = sampleFrequencyMin;
        }
        else if (sampleFrequency > sampleFrequencyMax)
        {
            sampleFrequency = sampleFrequencyMax;
        }

        if (settings.sampleFrequency != sampleFrequency)
        {
            LOG_DEBUG("Update sampling frequency: %u -> %u", settings.sampleFrequency, sampleFrequency);
            // Update setting
            settings.sampleFrequency = sampleFrequency;
            Settings::update(settingsId, settings);
            result = true;
        }

        return result;
    }

    /**
     * @brief Set measurements interval setting
     *
     * @param measureInterval New measurements interval
     * @return true if new setting was set, false otherwise
     */
    bool setMeasureInterval(uint32_t measureInterval)
    {
        bool result = false;

        if (measureInterval < measureIntervalMin)
        {
            measureInterval = measureIntervalMin;
        }

        if (settings.measureInterval != measureInterval)
        {
            LOG_DEBUG("Update measurements interval: %u -> %u", settings.measureInterval, measureInterval);
            // Update setting
            settings.measureInterval = measureInterval;
            Settings::update(settingsId, settings);
            result = true;
        }

        return result;
    }

    /**
     * @brief Set pause interval setting
     *
     * @param pauseInterval New pause interval
     * @return true if new setting was set, false otherwise
     */
    bool setPauseInterval(uint32_t pauseInterval)
    {
        bool result = false;

        if (settings.pauseInterval != pauseInterval)
        {
            LOG_DEBUG("Update pause interval: %u -> %u", settings.pauseInterval, pauseInterval);
            // Update setting
            settings.pauseInterval = pauseInterval;
            Settings::update(settingsId, settings);
            result = true;
        }

        return result;
    }

    /**
     * @brief Start sensors sampling
     */
    bool startSampling()
    {
        if (isSamplingActive == false)
        {
            LOG_INFO("Start sampling");

            // Start sampling task
            eventGroup.set(EventBits::startTask);

            // Wait sampling task is running
            EventBits_t events = eventGroup.wait(EventBits::taskIsRunning);
            if (events & EventBits::taskIsRunning)
            {
                isSamplingActive = true;
            }
            else
            {
                LOG_ERROR("Sampling start failed");
            }
        }

        return (isSamplingActive == true);
    }

    /**
     * @brief Stop sensors sampling
     */
    bool stopSampling()
    {
        if (isSamplingActive == true)
        {
            LOG_INFO("Stop sampling");

            // Stop sampling task
            eventGroup.set(EventBits::stopTask);

            // Wait sampling task is idle
            EventBits_t events = eventGroup.wait(EventBits::taskIsIdle);
            if (events & EventBits::taskIsIdle)
            {
                isSamplingActive = false;
            }
            else
            {
                LOG_ERROR("Sampling stop failed");
            }
        }

        return (isSamplingActive == false);
    }

    /**
     * @brief Setup the measurements process
     */
    void setupMeasurements()
    {
        context.setup(settings);

        if (context.config.dataTypeMask & DataTypeMask::Psd)
        {
            // Setup PSD measurements
            if (context.config.sensorMask & SensorMask::Adc1)
            {
                psdAdc1.setup(context.segmentSize, context.config.sampleFrequency);
            }
            if (context.config.sensorMask & SensorMask::Adc2)
            {
                psdAdc2.setup(context.segmentSize, context.config.sampleFrequency);
            }
            if (context.config.sensorMask & SensorMask::Accel)
            {
                psdAccX.setup(context.segmentSize, context.config.sampleFrequency);
                psdAccY.setup(context.segmentSize, context.config.sampleFrequency);
            }
            if (context.config.sensorMask & SensorMask::Gyro)
            {
                psdGyroX.setup(context.segmentSize, context.config.sampleFrequency);
                psdGyroY.setup(context.segmentSize, context.config.sampleFrequency);
            }
            if (context.config.sensorMask & SensorMask::AccelResult)
            {
                psdAccResult.setup(context.segmentSize, context.config.sampleFrequency);
            }
        }

        if (context.config.dataTypeMask & DataTypeMask::Statistic)
        {
            // Reset measurements statistic
            if (context.config.sensorMask & SensorMask::Adc1)
            {
                statisticAdc1.reset();
            }
            if (context.config.sensorMask & SensorMask::Adc2)
            {
                statisticAdc2.reset();
            }
            if (context.config.sensorMask & SensorMask::Accel)
            {
                statisticAccX.reset();
                statisticAccY.reset();
                statisticAccZ.reset();
            }
            if (context.config.sensorMask & SensorMask::Gyro)
            {
                statisticGyroX.reset();
                statisticGyroY.reset();
                statisticGyroZ.reset();
            }
            if (context.config.sensorMask & SensorMask::Angle)
            {
                statisticRoll.reset();
                statisticPitch.reset();
            }
            if (context.config.sensorMask & SensorMask::AccelResult)
            {
                statisticAccResult.reset();
            }
        }

        if (context.config.dataTypeMask & DataTypeMask::Raw)
        {
            char directoryName[30];

            SystemTime::DateTimeString fileTimestamp;
            SystemTime::epochToTimestamp(context.startEpochTime, fileTimestamp);

            // CSV/RAW
            snprintf(directoryName, sizeof(directoryName), "%s/RAW/%.8s", directoryCsv, fileTimestamp);
            bool isOpen = sdRawFile.create(directoryName, fileTimestamp, fileExtensionCsv);
            if (isOpen == true)
            {
                char string[200] = {0};
                int stringOffset = 0;
                size_t stringRemain = sizeof(string);

                int written = snprintf(&string[stringOffset], stringRemain, "Timestamp,");
                if (written > 0)
                {
                    stringOffset += written;
                    stringRemain -= written;
                }

                if (context.config.sensorMask & SensorMask::Adc1)
                {
                    written = snprintf(&string[stringOffset], stringRemain, "Adc1,");
                    if (written > 0)
                    {
                        stringOffset += written;
                        stringRemain -= written;
                    }
                }
                if (context.config.sensorMask & SensorMask::Adc2)
                {
                    written = snprintf(&string[stringOffset], stringRemain, "Adc2,");
                    if (written > 0)
                    {
                        stringOffset += written;
                        stringRemain -= written;
                    }
                }
                if (context.config.sensorMask & SensorMask::Accel)
                {
                    written = snprintf(&string[stringOffset], stringRemain, "AccelX,AccelY,AccelZ,");
                    if (written > 0)
                    {
                        stringOffset += written;
                        stringRemain -= written;
                    }
                }
                if (context.config.sensorMask & SensorMask::Gyro)
                {
                    written = snprintf(&string[stringOffset], stringRemain, "GyroX,GyroY,GyroZ,");
                    if (written > 0)
                    {
                        stringOffset += written;
                        stringRemain -= written;
                    }
                }
                if (context.config.sensorMask & SensorMask::Angle)
                {
                    written = snprintf(&string[stringOffset], stringRemain, "Roll,Pitch,");
                    if (written > 0)
                    {
                        stringOffset += written;
                        stringRemain -= written;
                    }
                }
                if (context.config.sensorMask & SensorMask::AccelResult)
                {
                    written = snprintf(&string[stringOffset], stringRemain, "AccelResult,");
                    if (written > 0)
                    {
                        stringOffset += written;
                        stringRemain -= written;
                    }
                }

                sdRawFile.println(string);
                sdRawFile.close();
            }
        }
    }

    /**
     * @brief Finish the measurements process
     */
    void finishMeasurements()
    {
        // Check if at least one segment is ready
        if (context.segmentCount > 0)
        {
            size_t durationMs = context.segmentCount * context.segmentTimeMs;
            context.endEpochTime = context.startEpochTime + durationMs / millisPerSecond;

            // Save measurements to the storage
            saveMeasurements();
        }
    }

    /**
     * @brief Restart measurements
     */
    void restartMeasurements()
    {
        if (isSamplingActive == true)
        {
            // Stop current sensor measurements
            Manager::stop();

            // Start next sensor measurements
            Manager::start();
        }
    }

    /**
     * @brief Process new measurements
     *
     * @param[in] index Segment index with new data
     */
    void processMeasurements(size_t index)
    {
        // Data offset in buffer
        const size_t offset = index * context.segmentSize;
        if (offset >= bufferSize || offset + context.segmentSize > bufferSize)
        {
            LOG_ERROR("Invalid buffer offset %u", offset);
            return;
        }

        // Set CPU frequency to the maximum possible
        Power::setCpuFrequency(Power::cpuFrequencyMaxMHz);

        const uint16_t *pSamplesAdc1 = &buffer.adc1[offset];
        const uint16_t *pSamplesAdc2 = &buffer.adc2[offset];
        const int16_t *pSamplesAccX = &buffer.accX[offset];
        const int16_t *pSamplesAccY = &buffer.accY[offset];
        const int16_t *pSamplesAccZ = &buffer.accZ[offset];
        const int16_t *pSamplesGyroX = &buffer.gyrX[offset];
        const int16_t *pSamplesGyroY = &buffer.gyrY[offset];
        const int16_t *pSamplesGyroZ = &buffer.gyrZ[offset];
        const float *pSamplesRoll = &buffer.roll[offset];
        const float *pSamplesPitch = &buffer.pitch[offset];

        if (context.config.sensorMask & SensorMask::AccelResult)
        {
            // Calculate Accel Result samples first before PSD and Statistic
            calculateAccelResult(pSamplesAccX, pSamplesAccY, context.segmentSize);
        }

        if (context.config.dataTypeMask & DataTypeMask::Psd)
        {
            // Perform PSD calculations
            if (context.config.sensorMask & SensorMask::Adc1)
            {
                psdAdc1.computeSegment(pSamplesAdc1);
            }
            if (context.config.sensorMask & SensorMask::Adc2)
            {
                psdAdc2.computeSegment(pSamplesAdc2);
            }
            if (context.config.sensorMask & SensorMask::Accel)
            {
                psdAccX.computeSegment(pSamplesAccX);
                psdAccY.computeSegment(pSamplesAccY);
            }
            if (context.config.sensorMask & SensorMask::Gyro)
            {
                psdGyroX.computeSegment(pSamplesGyroX);
                psdGyroY.computeSegment(pSamplesGyroY);
            }
            if (context.config.sensorMask & SensorMask::AccelResult)
            {
                psdAccResult.computeSegment(accelResult);
            }
        }

        if (context.config.dataTypeMask & DataTypeMask::Statistic)
        {
            // Perform Statistic calculations
            if (context.config.sensorMask & SensorMask::Adc1)
            {
                statisticAdc1.calculate(pSamplesAdc1, context.segmentSize);
            }
            if (context.config.sensorMask & SensorMask::Adc2)
            {
                statisticAdc2.calculate(pSamplesAdc2, context.segmentSize);
            }
            if (context.config.sensorMask & SensorMask::Accel)
            {
                statisticAccX.calculate(pSamplesAccX, context.segmentSize);
                statisticAccY.calculate(pSamplesAccY, context.segmentSize);
                statisticAccZ.calculate(pSamplesAccZ, context.segmentSize);
            }
            if (context.config.sensorMask & SensorMask::Gyro)
            {
                statisticGyroX.calculate(pSamplesGyroX, context.segmentSize);
                statisticGyroY.calculate(pSamplesGyroY, context.segmentSize);
                statisticGyroZ.calculate(pSamplesGyroZ, context.segmentSize);
            }
            if (context.config.sensorMask & SensorMask::Angle)
            {
                statisticRoll.calculate(pSamplesRoll, context.segmentSize);
                statisticPitch.calculate(pSamplesPitch, context.segmentSize);
            }
            if (context.config.sensorMask & SensorMask::AccelResult)
            {
                statisticAccResult.calculate(accelResult, context.segmentSize);
            }
        }

        if (context.config.dataTypeMask & DataTypeMask::Raw)
        {
            // Save RAW data to SD file
            bool isOpen = sdRawFile.open(nullptr, O_WRONLY | O_APPEND);
            if (isOpen == true)
            {
                char string[200] = {0};
                size_t timeOffsetMs = context.segmentCount * context.segmentTimeMs;

                for (size_t idx = 0; idx < context.segmentSize; idx++)
                {
                    int stringOffset = 0;
                    size_t stringRemain = sizeof(string);

                    SystemTime::DateTimeString timestamp;
                    time_t epochTime = context.startEpochTime + timeOffsetMs / millisPerSecond;
                    SystemTime::epochToTimestamp(epochTime, timestamp);
                    int written = snprintf(&string[stringOffset], stringRemain, "%s", timestamp);
                    if (written > 0)
                    {
                        stringOffset += written;
                        stringRemain -= written;
                    }

                    size_t timeMs = timeOffsetMs % millisPerSecond;
                    written = snprintf(&string[stringOffset], stringRemain, ".%03d,", timeMs);
                    if (written > 0)
                    {
                        stringOffset += written;
                        stringRemain -= written;
                    }

                    if (context.config.sensorMask & SensorMask::Adc1)
                    {
                        written = snprintf(&string[stringOffset], stringRemain, "%u,", pSamplesAdc1[idx]);
                        if (written > 0)
                        {
                            stringOffset += written;
                            stringRemain -= written;
                        }
                    }
                    if (context.config.sensorMask & SensorMask::Adc2)
                    {
                        written = snprintf(&string[stringOffset], stringRemain, "%u,", pSamplesAdc2[idx]);
                        if (written > 0)
                        {
                            stringOffset += written;
                            stringRemain -= written;
                        }
                    }
                    if (context.config.sensorMask & SensorMask::Accel)
                    {
                        written = snprintf(&string[stringOffset], stringRemain, "%d,", pSamplesAccX[idx]);
                        if (written > 0)
                        {
                            stringOffset += written;
                            stringRemain -= written;
                        }
                        written = snprintf(&string[stringOffset], stringRemain, "%d,", pSamplesAccY[idx]);
                        if (written > 0)
                        {
                            stringOffset += written;
                            stringRemain -= written;
                        }
                        written = snprintf(&string[stringOffset], stringRemain, "%d,", pSamplesAccZ[idx]);
                        if (written > 0)
                        {
                            stringOffset += written;
                            stringRemain -= written;
                        }
                    }
                    if (context.config.sensorMask & SensorMask::Gyro)
                    {
                        written = snprintf(&string[stringOffset], stringRemain, "%d,", pSamplesGyroX[idx]);
                        if (written > 0)
                        {
                            stringOffset += written;
                            stringRemain -= written;
                        }
                        written = snprintf(&string[stringOffset], stringRemain, "%d,", pSamplesGyroY[idx]);
                        if (written > 0)
                        {
                            stringOffset += written;
                            stringRemain -= written;
                        }
                        written = snprintf(&string[stringOffset], stringRemain, "%d,", pSamplesGyroZ[idx]);
                        if (written > 0)
                        {
                            stringOffset += written;
                            stringRemain -= written;
                        }
                    }
                    if (context.config.sensorMask & SensorMask::Angle)
                    {
                        written = snprintf(&string[stringOffset], stringRemain, "%.1f,", pSamplesRoll[idx]);
                        if (written > 0)
                        {
                            stringOffset += written;
                            stringRemain -= written;
                        }
                        written = snprintf(&string[stringOffset], stringRemain, "%.1f,", pSamplesPitch[idx]);
                        if (written > 0)
                        {
                            stringOffset += written;
                            stringRemain -= written;
                        }
                    }
                    if (context.config.sensorMask & SensorMask::AccelResult)
                    {
                        written = snprintf(&string[stringOffset], stringRemain, "%.2f,", accelResult[idx]);
                        if (written > 0)
                        {
                            stringOffset += written;
                            stringRemain -= written;
                        }
                    }

                    sdRawFile.println(string);
                    timeOffsetMs += context.sampleTimeMs;
                }

                sdRawFile.close();
            }
        }

        // Set CPU frequency to the minimum possible
        Power::setCpuFrequency(Power::cpuFrequencyMinMHz);
    }

    /**
     * @brief Calculate accelerometer resultant direction using Linear Least Square
     *
     * @param pAccX Pointer to accelerometer X axis data
     * @param pAccY Pointer to accelerometer Y axis data
     * @param length Number of data points
     */
    void calculateAccelResult(const int16_t *pAccX, const int16_t *pAccY, size_t length)
    {
        float sumX = 0;
        float sumY = 0;
        float sumX2 = 0;
        float sumXY = 0;
        float meanAccX = 0;
        float meanAccY = 0;

        for (size_t idx = 0; idx < length; idx++)
        {
            meanAccX += static_cast<float>(pAccX[idx]);
            meanAccY += static_cast<float>(pAccY[idx]);
        }

        // Calculate average
        meanAccX /= length;
        meanAccY /= length;

        for (size_t idx = 0; idx < length; idx++)
        {
            // Remove average
            float x = Imu::accelToMs2(pAccX[idx] - meanAccX);
            float y = Imu::accelToMs2(pAccY[idx] - meanAccY);

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
                float x = Imu::accelToMs2(pAccX[idx] - meanAccX);
                float y = Imu::accelToMs2(pAccY[idx] - meanAccY);

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
        PsdBin coreBinAdc1;
        PsdBin coreBinAdc2;
        PsdBin coreBinAccX;
        PsdBin coreBinAccY;
        PsdBin coreBinGyroX;
        PsdBin coreBinGyroY;
        PsdBin coreBinAccResult;

        const float *resultPsdAdc1 = nullptr;
        const float *resultPsdAdc2 = nullptr;
        const float *resultPsdAccX = nullptr;
        const float *resultPsdAccY = nullptr;
        const float *resultPsdGyroX = nullptr;
        const float *resultPsdGyroY = nullptr;
        const float *resultPsdAccResult = nullptr;

        // If 𝑁 is even (segmentSize = 2^N), you have 𝑁/2+1 useful components
        // because the symmetric part of the FFT spectrum for real-valued signals
        // does not provide additional information beyond the Nyquist frequency
        size_t resultPoints = context.segmentSize / 2 + 1;
        if (resultPoints > context.config.psdCutoff)
        {
            // Limit result points
            resultPoints = context.config.psdCutoff;
        }

        // Get current battery level
        Battery::Status batteryStatus = Battery::readStatus();

        // Set CPU frequency to the maximum possible
        Power::setCpuFrequency(Power::cpuFrequencyMaxMHz);

        if (context.config.dataTypeMask & DataTypeMask::Psd)
        {
            if (context.config.sensorMask & SensorMask::Adc1)
            {
                resultPsdAdc1 = psdAdc1.getResult(&coreBinAdc1);
                // BIN/PSD/ADC1
                savePsdFile(SensorType::Adc1, coreBinAdc1, resultPsdAdc1, resultPoints);
            }

            if (context.config.sensorMask & SensorMask::Adc2)
            {
                resultPsdAdc2 = psdAdc2.getResult(&coreBinAdc2);
                // BIN/PSD/ADC2
                savePsdFile(SensorType::Adc2, coreBinAdc2, resultPsdAdc2, resultPoints);
            }

            if (context.config.sensorMask & SensorMask::Accel)
            {
                resultPsdAccX = psdAccX.getResult(&coreBinAccX);
                // BIN/PSD/ACC_X
                savePsdFile(SensorType::AccelX, coreBinAccX, resultPsdAccX, resultPoints);

                resultPsdAccY = psdAccY.getResult(&coreBinAccY);
                // BIN/PSD/ACC_Y
                savePsdFile(SensorType::AccelY, coreBinAccY, resultPsdAccY, resultPoints);
            }

            if (context.config.sensorMask & SensorMask::Gyro)
            {
                resultPsdGyroX = psdGyroX.getResult(&coreBinGyroX);
                // BIN/PSD/GYR_X
                savePsdFile(SensorType::GyroX, coreBinGyroX, resultPsdGyroX, resultPoints);

                resultPsdGyroY = psdGyroY.getResult(&coreBinGyroY);
                // BIN/PSD/GYR_Y
                savePsdFile(SensorType::GyroY, coreBinGyroY, resultPsdGyroY, resultPoints);
            }

            if (context.config.sensorMask & SensorMask::AccelResult)
            {
                resultPsdAccResult = psdAccResult.getResult(&coreBinAccResult);
                // BIN/PSD/ACC_RES
                savePsdFile(SensorType::AccelResult, coreBinAccResult, resultPsdAccResult, resultPoints);
            }
        }

        if (context.config.dataTypeMask & DataTypeMask::Statistic)
        {
            if (context.config.sensorMask & SensorMask::Adc1)
            {
                StatisticData data = {
                    .max = static_cast<float>(statisticAdc1.max()),
                    .min = static_cast<float>(statisticAdc1.min()),
                    .mean = statisticAdc1.mean(),
                    .deviation = statisticAdc1.deviation(),
                };
                // BIN/STAT/ADC1
                saveStatisticFile(SensorType::Adc1, data);
            }

            if (context.config.sensorMask & SensorMask::Adc2)
            {
                StatisticData data = {
                    .max = static_cast<float>(statisticAdc2.max()),
                    .min = static_cast<float>(statisticAdc2.min()),
                    .mean = statisticAdc2.mean(),
                    .deviation = statisticAdc2.deviation(),
                };
                // BIN/STAT/ADC2
                saveStatisticFile(SensorType::Adc2, data);
            }

            if (context.config.sensorMask & SensorMask::Accel)
            {
                StatisticData dataX = {
                    .max = static_cast<float>(statisticAccX.max()),
                    .min = static_cast<float>(statisticAccX.min()),
                    .mean = statisticAccX.mean(),
                    .deviation = statisticAccX.deviation(),
                };
                // BIN/STAT/ACC_X
                saveStatisticFile(SensorType::AccelX, dataX);

                StatisticData dataY = {
                    .max = static_cast<float>(statisticAccY.max()),
                    .min = static_cast<float>(statisticAccY.min()),
                    .mean = statisticAccY.mean(),
                    .deviation = statisticAccY.deviation(),
                };
                // BIN/STAT/ACC_Y
                saveStatisticFile(SensorType::AccelY, dataY);

                StatisticData dataZ = {
                    .max = static_cast<float>(statisticAccZ.max()),
                    .min = static_cast<float>(statisticAccZ.min()),
                    .mean = statisticAccZ.mean(),
                    .deviation = statisticAccZ.deviation(),
                };
                // BIN/STAT/ACC_Z
                saveStatisticFile(SensorType::AccelZ, dataZ);
            }

            if (context.config.sensorMask & SensorMask::Gyro)
            {
                StatisticData dataX = {
                    .max = static_cast<float>(statisticGyroX.max()),
                    .min = static_cast<float>(statisticGyroX.min()),
                    .mean = statisticGyroX.mean(),
                    .deviation = statisticGyroX.deviation(),
                };
                // BIN/STAT/GYR_X
                saveStatisticFile(SensorType::GyroX, dataX);

                StatisticData dataY = {
                    .max = static_cast<float>(statisticGyroY.max()),
                    .min = static_cast<float>(statisticGyroY.min()),
                    .mean = statisticGyroY.mean(),
                    .deviation = statisticGyroY.deviation(),
                };
                // BIN/STAT/GYR_Y
                saveStatisticFile(SensorType::GyroY, dataX);

                StatisticData dataZ = {
                    .max = static_cast<float>(statisticGyroZ.max()),
                    .min = static_cast<float>(statisticGyroZ.min()),
                    .mean = statisticGyroZ.mean(),
                    .deviation = statisticGyroZ.deviation(),
                };
                // BIN/STAT/GYR_Z
                saveStatisticFile(SensorType::GyroZ, dataZ);
            }

            if (context.config.sensorMask & SensorMask::Angle)
            {
                StatisticData dataRoll = {
                    .max = statisticRoll.max(),
                    .min = statisticRoll.min(),
                    .mean = statisticRoll.mean(),
                    .deviation = statisticRoll.deviation(),
                };
                // BIN/STAT/ROLL
                saveStatisticFile(SensorType::Roll, dataRoll);

                StatisticData dataPitch = {
                    .max = statisticPitch.max(),
                    .min = statisticPitch.min(),
                    .mean = statisticPitch.mean(),
                    .deviation = statisticPitch.deviation(),
                };
                // BIN/STAT/PITCH
                saveStatisticFile(SensorType::Pitch, dataPitch);
            }

            if (context.config.sensorMask & SensorMask::AccelResult)
            {
                StatisticData data = {
                    .max = statisticAccResult.max(),
                    .min = statisticAccResult.min(),
                    .mean = statisticAccResult.mean(),
                    .deviation = statisticAccResult.deviation(),
                };
                // BIN/STAT/ACC_RES
                saveStatisticFile(SensorType::AccelResult, data);
            }
        }

        // CSV/PSD_STAT
        char directory[30];
        SystemTime::DateTimeString fileTimestamp;
        SystemTime::epochToTimestamp(context.startEpochTime, fileTimestamp);
        snprintf(directory, sizeof(directory), "%s/PSD_STAT/%.8s", directoryCsv, fileTimestamp);

        SD::File sdFile;
        bool isOpen = sdFile.create(directory, fileTimestamp, fileExtensionCsv);
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
            snprintf(string, sizeof(string), "Logging Rate,%u", context.config.sampleFrequency);
            sdFile.println(string);
            sdFile.println(""); // End of header

            if (context.config.sensorMask & SensorMask::Adc1)
            {
                sdFile.println("Channel Name,ADC_1");
                sdFile.println("Channel Units,raw_12bit");
                if (context.config.dataTypeMask & DataTypeMask::Statistic)
                {
                    snprintf(string, sizeof(string), "Maximum,%G", statisticAdc1.max());
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Minimum,%G", statisticAdc1.min());
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Mean,%G", statisticAdc1.mean());
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Standard Deviation,%G", statisticAdc1.deviation());
                    sdFile.println(string);
                }
                if (context.config.dataTypeMask & DataTypeMask::Psd)
                {
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
                }
                sdFile.println(""); // End of channel
            }

            if (context.config.sensorMask & SensorMask::Adc2)
            {
                sdFile.println("Channel Name,ADC_2");
                sdFile.println("Channel Units,raw_12bit");
                if (context.config.dataTypeMask & DataTypeMask::Statistic)
                {
                    snprintf(string, sizeof(string), "Maximum,%G", statisticAdc2.max());
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Minimum,%G", statisticAdc2.min());
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Mean,%G", statisticAdc2.mean());
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Standard Deviation,%G", statisticAdc2.deviation());
                    sdFile.println(string);
                }
                if (context.config.dataTypeMask & DataTypeMask::Psd)
                {
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
                }
                sdFile.println(""); // End of channel
            }

            if (context.config.sensorMask & SensorMask::Accel)
            {
                sdFile.println("Channel Name,ACC_X");
                sdFile.println("Channel Units,m/s^2");
                if (context.config.dataTypeMask & DataTypeMask::Statistic)
                {
                    snprintf(string, sizeof(string), "Maximum,%G", Imu::accelToMs2(statisticAccX.max()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Minimum,%G", Imu::accelToMs2(statisticAccX.min()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Mean,%G", Imu::accelToMs2(statisticAccX.mean()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Standard Deviation,%G", Imu::accelToMs2(statisticAccX.deviation()));
                    sdFile.println(string);
                }
                if (context.config.dataTypeMask & DataTypeMask::Psd)
                {
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
                }
                sdFile.println(""); // End of channel

                sdFile.println("Channel Name,ACC_Y");
                sdFile.println("Channel Units,m/s^2");
                if (context.config.dataTypeMask & DataTypeMask::Statistic)
                {
                    snprintf(string, sizeof(string), "Maximum,%G", Imu::accelToMs2(statisticAccY.max()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Minimum,%G", Imu::accelToMs2(statisticAccY.min()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Mean,%G", Imu::accelToMs2(statisticAccY.mean()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Standard Deviation,%G", Imu::accelToMs2(statisticAccY.deviation()));
                    sdFile.println(string);
                }
                if (context.config.dataTypeMask & DataTypeMask::Psd)
                {
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
                }
                sdFile.println(""); // End of channel

                sdFile.println("Channel Name,ACC_Z");
                sdFile.println("Channel Units,m/s^2");
                if (context.config.dataTypeMask & DataTypeMask::Statistic)
                {
                    snprintf(string, sizeof(string), "Maximum,%G", Imu::accelToMs2(statisticAccZ.max()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Minimum,%G", Imu::accelToMs2(statisticAccZ.min()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Mean,%G", Imu::accelToMs2(statisticAccZ.mean()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Standard Deviation,%G", Imu::accelToMs2(statisticAccZ.deviation()));
                    sdFile.println(string);
                }
                sdFile.println(""); // End of channel
            }

            if (context.config.sensorMask & SensorMask::Gyro)
            {
                sdFile.println("Channel Name,GYRO_X");
                sdFile.println("Channel Units,rad/s");
                if (context.config.dataTypeMask & DataTypeMask::Statistic)
                {
                    snprintf(string, sizeof(string), "Maximum,%G", Imu::gyroToRads(statisticGyroX.max()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Minimum,%G", Imu::gyroToRads(statisticGyroX.min()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Mean,%G", Imu::gyroToRads(statisticGyroX.mean()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Standard Deviation,%G", Imu::gyroToRads(statisticGyroX.deviation()));
                    sdFile.println(string);
                }
                if (context.config.dataTypeMask & DataTypeMask::Psd)
                {
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
                }
                sdFile.println(""); // End of channel

                sdFile.println("Channel Name,GYRO_Y");
                sdFile.println("Channel Units,rad/s");
                if (context.config.dataTypeMask & DataTypeMask::Statistic)
                {
                    snprintf(string, sizeof(string), "Maximum,%G", Imu::gyroToRads(statisticGyroY.max()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Minimum,%G", Imu::gyroToRads(statisticGyroY.min()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Mean,%G", Imu::gyroToRads(statisticGyroY.mean()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Standard Deviation,%G", Imu::gyroToRads(statisticGyroY.deviation()));
                    sdFile.println(string);
                }
                if (context.config.dataTypeMask & DataTypeMask::Psd)
                {
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
                }
                sdFile.println(""); // End of channel

                sdFile.println("Channel Name,GYRO_Z");
                sdFile.println("Channel Units,rad/s");
                if (context.config.dataTypeMask & DataTypeMask::Statistic)
                {
                    snprintf(string, sizeof(string), "Maximum,%G", Imu::gyroToRads(statisticGyroZ.max()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Minimum,%G", Imu::gyroToRads(statisticGyroZ.min()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Mean,%G", Imu::gyroToRads(statisticGyroZ.mean()));
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Standard Deviation,%G", Imu::gyroToRads(statisticGyroZ.deviation()));
                    sdFile.println(string);
                }
                sdFile.println(""); // End of channel
            }

            if (context.config.sensorMask & SensorMask::Angle)
            {
                sdFile.println("Channel Name,ROLL");
                sdFile.println("Channel Units,deg");
                if (context.config.dataTypeMask & DataTypeMask::Statistic)
                {
                    snprintf(string, sizeof(string), "Maximum,%G", statisticRoll.max());
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Minimum,%G", statisticRoll.min());
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Mean,%G", statisticRoll.mean());
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Standard Deviation,%G", statisticRoll.deviation());
                    sdFile.println(string);
                }
                sdFile.println(""); // End of channel

                sdFile.println("Channel Name,PITCH");
                sdFile.println("Channel Units,deg");
                if (context.config.dataTypeMask & DataTypeMask::Statistic)
                {
                    snprintf(string, sizeof(string), "Maximum,%G", statisticPitch.max());
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Minimum,%G", statisticPitch.min());
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Mean,%G", statisticPitch.mean());
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Standard Deviation,%G", statisticPitch.deviation());
                    sdFile.println(string);
                }
                sdFile.println(""); // End of channel
            }

            if (context.config.sensorMask & SensorMask::AccelResult)
            {
                sdFile.println("Channel Name,E_ACC_RES");
                sdFile.println("Channel Units,m/s^2");
                if (context.config.dataTypeMask & DataTypeMask::Statistic)
                {
                    snprintf(string, sizeof(string), "Maximum,%G", statisticAccResult.max());
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Minimum,%G", statisticAccResult.min());
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Mean,%G", statisticAccResult.mean());
                    sdFile.println(string);
                    snprintf(string, sizeof(string), "Standard Deviation,%G", statisticAccResult.deviation());
                    sdFile.println(string);
                }
                if (context.config.dataTypeMask & DataTypeMask::Psd)
                {
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
                }
                sdFile.println(""); // End of channel
            }

            sdFile.close();
        }

        // Set CPU frequency to the minimum possible
        Power::setCpuFrequency(Power::cpuFrequencyMinMHz);
    }

    /**
     * @brief Save PSD measurements to file on SD
     *
     * @param sensorType Type of sensor
     * @param coreBin PSD core bin info
     * @param psd PSD points
     * @param points Number of PSD points
     * @return true if PSD was saved successfully, false otherwise
     */
    bool savePsdFile(SensorType sensorType, const PsdBin &coreBin, const float *psd, size_t points)
    {
        assert(psd);
        assert(points > 0);

        const char *sensorName = getSensorName(sensorType);
        const char *dataName = getDataName(DataType::Psd);
        LOG_DEBUG("Save %s %s: core bin %lfHz - %lf, points %d",
                  sensorName, dataName, coreBin.frequency, coreBin.amplitude, points);

        char fileName[30];
        char directory[30];
        SystemTime::DateTimeString startTimestamp;
        SystemTime::epochToTimestamp(context.startEpochTime, startTimestamp);
        snprintf(directory, sizeof(directory), "%s/%s/%s/%.8s", directoryBin, dataName, sensorName, startTimestamp);
        snprintf(fileName, sizeof(fileName), "%u", context.startEpochTime);

        SD::File sdFile;
        bool result = sdFile.create(directory, fileName, fileExtensionBin);
        if (result == true)
        {
            FileHeader fileHeader = {
                .startEpochTime = static_cast<uint32_t>(context.startEpochTime),
                .endEpochTime = static_cast<uint32_t>(context.endEpochTime),
                .sampleTimeMs = static_cast<uint16_t>(context.sampleTimeMs),
                .dataType = static_cast<uint8_t>(DataType::Psd),
                .sensorType = static_cast<uint8_t>(sensorType),
            };

            // Save file header
            result = sdFile.write(&fileHeader, sizeof(fileHeader));
            if (result == true)
            {
                PsdHeader psdHeader = {
                    .coreFrequency = coreBin.frequency,
                    .coreAmplitude = coreBin.amplitude,
                    .points = points,
                };

                // Save PSD header
                result = sdFile.write(&psdHeader, sizeof(psdHeader));
                if (result == true)
                {
                    // Save PSD points
                    result = sdFile.write(psd, points * sizeof(*psd));
                }
            }

            sdFile.close();
        }

        return result;
    }

    /**
     * @brief Save Statistic to file on SD
     *
     * @param sensorType Type of sensor
     * @param data Statistic data
     * @return true if Statistic was saved successfully, false otherwise
     */
    bool saveStatisticFile(SensorType sensorType, const StatisticData &data)
    {
        const char *sensorName = getSensorName(sensorType);
        const char *dataName = getDataName(DataType::Statistic);
        LOG_DEBUG("Save %s %s: Max %f, Min %f, Mean %f, Standard Deviation %f",
                  sensorName, dataName, data.max, data.min, data.mean, data.deviation);

        char fileName[30];
        char directory[30];
        SystemTime::DateTimeString startTimestamp;
        SystemTime::epochToTimestamp(context.startEpochTime, startTimestamp);
        snprintf(directory, sizeof(directory), "%s/%s/%s/%.8s", directoryBin, dataName, sensorName, startTimestamp);
        snprintf(fileName, sizeof(fileName), "%u", context.startEpochTime);

        SD::File sdFile;
        bool result = sdFile.create(directory, fileName, fileExtensionBin);
        if (result == true)
        {
            FileHeader fileHeader = {
                .startEpochTime = static_cast<uint32_t>(context.startEpochTime),
                .endEpochTime = static_cast<uint32_t>(context.endEpochTime),
                .sampleTimeMs = static_cast<uint16_t>(context.sampleTimeMs),
                .dataType = static_cast<uint8_t>(DataType::Statistic),
                .sensorType = static_cast<uint8_t>(sensorType),
            };

            // Save file header
            result = sdFile.write(&fileHeader, sizeof(fileHeader));
            if (result == true)
            {
                // Save Statistic data
                result = sdFile.write(&data, sizeof(data));
            }

            sdFile.close();
        }

        return result;
    }

    /**
     * @brief Start sensors readings
     *
     * @return true if sensors are started successfully, false otherwsie
     */
    bool enableSensors()
    {
        bool result = false;

        if (isAdcSet())
        {
            result = Analog::VddController::applyVoltage();
            if (result != true)
            {
                LOG_ERROR("Sensor voltage apply fail!");
                return false;
            }

            if (context.config.sensorMask & SensorMask::Adc1)
            {
                adc1.enable();
            }

            if (context.config.sensorMask & SensorMask::Adc2)
            {
                adc2.enable();
            }
        }

        if (isAccelSet())
        {
            result = Imu::start(Imu::Module::Accel);
            if (result != true)
            {
                LOG_ERROR("Accelerometer start failed");
                return false;
            }
            LOG_INFO("Accelerometer started");
        }

        if (isGyroSet())
        {
            result = Imu::start(Imu::Module::Gyro);
            if (result != true)
            {
                LOG_ERROR("Gyroscope start failed");
                return false;
            }
            LOG_INFO("Gyroscope started");
        }

        if (context.config.sensorMask & SensorMask::Angle)
        {
            // Setup madgwick's IMU and AHRS filter
            madgwickFilter.begin(context.config.sampleFrequency);
        }

        return result;
    }

    /**
     * @brief Start sensors readings
     */
    void disableSensors()
    {
        Analog::VddController::cutOffVoltage();
        adc1.disable();
        adc2.disable();
        Imu::stop(Imu::Module::Accel);
        Imu::stop(Imu::Module::Gyro);
    }

    /**
     * @brief Read sensors data to specified offset in the buffer
     *
     * @param offset Data offset in the buffer
     * @return true if reading succeed, false otherwise
     */
    bool readSensors(size_t offset)
    {
        if (offset >= bufferSize)
        {
            LOG_ERROR("Invalid buffer offset %u", offset);
            return false;
        }

        float accelGX = 0;
        float accelGY = 0;
        float accelGZ = 0;
        float gyroDpsX = 0;
        float gyroDpsY = 0;
        float gyroDpsZ = 0;

        if (context.config.sensorMask & SensorMask::Adc1)
        {
            // Read new analog 1 sample
            buffer.adc1[offset] = adc1.read();
        }

        if (context.config.sensorMask & SensorMask::Adc2)
        {
            // Read new analog 2 sample
            buffer.adc2[offset] = adc2.read();
        }

        if (isAccelSet())
        {
            Imu::Data imuData;
            bool result = Imu::read(Imu::Module::Accel, imuData);
            if (result == false)
            {
                LOG_ERROR("Accelerometer read failed");
                return false;
            }
            buffer.accX[offset] = imuData.x;
            buffer.accY[offset] = imuData.y;
            buffer.accZ[offset] = imuData.z;

            // Convert to accel to G
            accelGX = Imu::accelToG(imuData.x);
            accelGY = Imu::accelToG(imuData.y);
            accelGZ = Imu::accelToG(imuData.z);
        }

        if (isGyroSet())
        {
            Imu::Data imuData;
            bool result = Imu::read(Imu::Module::Gyro, imuData);
            if (result == false)
            {
                LOG_ERROR("Gyroscope read failed");
                return false;
            }
            buffer.gyrX[offset] = imuData.x;
            buffer.gyrY[offset] = imuData.y;
            buffer.gyrZ[offset] = imuData.z;

            // Convert to gyro to DPS
            gyroDpsX = Imu::gyroToDegs(imuData.x);
            gyroDpsY = Imu::gyroToDegs(imuData.y);
            gyroDpsZ = Imu::gyroToDegs(imuData.z);
        }

        if (context.config.sensorMask & SensorMask::Angle)
        {
            // Calculate/fill angle buffer data
            madgwickFilter.updateIMU(gyroDpsX, gyroDpsY, gyroDpsZ, accelGX, accelGY, accelGZ);
            buffer.roll[offset] = madgwickFilter.getRoll();
            buffer.pitch[offset] = madgwickFilter.getPitch();
        }

        return true;
    }

    /**
     * @brief Sensors samples reading task function
     *
     * @param pvParameters Task parameters
     */
    void samplingTask(void *pvParameters)
    {
        size_t segmentIndex = 0;
        size_t sampleIndex = 0;
        TickType_t xLastWakeTime;

        (void *)pvParameters; // unused

        int coreID = xPortGetCoreID();
        LOG_DEBUG("Samling task start on core #%d", coreID);

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

            LOG_INFO("Sampling task is RUNNING");

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
                xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(context.sampleTimeMs));

                size_t offset = segmentIndex * context.segmentSize + sampleIndex;

                // Read sensors data to specified offset in the buffer
                status = readSensors(offset);
                if (status != true)
                {
                    LOG_WARNING("Sensors read failed");
                }

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

        Serials::Manager::subscribeToRead(Serials::CommandId::PsdPoints,
                                          [](const char **responseString)
                                          {
                                              snprintf(dataString, sizeof(dataString), "%u", settings.psdPoints);
                                              *responseString = dataString;
                                          });

        Serials::Manager::subscribeToRead(Serials::CommandId::PsdCutoff,
                                          [](const char **responseString)
                                          {
                                              snprintf(dataString, sizeof(dataString), "%u", settings.psdCutoff);
                                              *responseString = dataString;
                                          });

        Serials::Manager::subscribeToRead(Serials::CommandId::DataTypeControl,
                                          [](const char **responseString)
                                          {
                                              snprintf(dataString, sizeof(dataString), "%u", settings.dataTypeMask);
                                              *responseString = dataString;
                                          });

        Serials::Manager::subscribeToRead(Serials::CommandId::MeasureControl,
                                          [](const char **responseString)
                                          {
                                              snprintf(dataString, sizeof(dataString), "%u", settings.sensorMask);
                                              *responseString = dataString;
                                          });

        Serials::Manager::subscribeToRead(Serials::CommandId::MeasureFrequency,
                                          [](const char **responseString)
                                          {
                                              snprintf(dataString, sizeof(dataString), "%u", settings.sampleFrequency);
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

        Serials::Manager::subscribeToRead(Serials::CommandId::AccelRange,
                                          [](const char **responseString)
                                          {
                                              uint8_t value = Imu::range(Imu::Module::Accel);
                                              snprintf(dataString, sizeof(dataString), "%u", value);
                                              *responseString = dataString;
                                          });

        Serials::Manager::subscribeToRead(Serials::CommandId::GyroRange,
                                          [](const char **responseString)
                                          {
                                              uint8_t value = Imu::range(Imu::Module::Gyro);
                                              snprintf(dataString, sizeof(dataString), "%u", value);
                                              *responseString = dataString;
                                          });

        Serials::Manager::subscribeToRead(Serials::CommandId::AdcVoltage,
                                          [](const char **responseString)
                                          {
                                              float value = Analog::VddController::targetVoltage();
                                              snprintf(dataString, sizeof(dataString), "%.1f", value);
                                              *responseString = dataString;
                                          });

        Serials::Manager::subscribeToRead(Serials::CommandId::GainSelect,
                                          [](const char **responseString)
                                          {
                                              uint8_t value = static_cast<uint8_t>(Analog::GainSelector::gain());
                                              snprintf(dataString, sizeof(dataString), "%u", value);
                                              *responseString = dataString;
                                          });

        Serials::Manager::subscribeToRead(Serials::CommandId::InputTypeSelect,
                                          [](const char **responseString)
                                          {
                                              uint8_t value = static_cast<uint8_t>(Analog::InputSelector::inputType());
                                              snprintf(dataString, sizeof(dataString), "%u", value);
                                              *responseString = dataString;
                                          });
    }

    /**
     * @brief Register serial write command handlers
     */
    void registerSerialWriteHandlers()
    {
        LOG_TRACE("Register serial write measurement handlers");

        Serials::Manager::subscribeToWrite(Serials::CommandId::PsdPoints,
                                           [](const char *dataString)
                                           {
                                               uint8_t value = atoi(dataString);
                                               bool result = setPsdPoints(value);
                                               if (result == true)
                                               {
                                                   // Restart measurements if PSD points was changed
                                                   restartMeasurements();
                                               }
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::PsdCutoff,
                                           [](const char *dataString)
                                           {
                                               uint16_t value = atoi(dataString);
                                               setPsdCutoff(value);
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::DataTypeControl,
                                           [](const char *dataString)
                                           {
                                               uint8_t value = atoi(dataString);
                                               bool result = setDataTypeMask(value);
                                               if (result == true)
                                               {
                                                   // Restart measurements if data type was changed
                                                   restartMeasurements();
                                               }
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::MeasureControl,
                                           [](const char *dataString)
                                           {
                                               uint8_t value = atoi(dataString);
                                               bool result = setSensorMask(value);
                                               if (result == true)
                                               {
                                                   // Restart measurements if sensor type was changed
                                                   restartMeasurements();
                                               }
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::MeasureFrequency,
                                           [](const char *dataString)
                                           {
                                               uint8_t value = atoi(dataString);
                                               bool result = setSampleFrequency(value);
                                               if (result == true)
                                               {
                                                   // Restart measurements if sampling frequency was changed
                                                   restartMeasurements();
                                               }
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::MeasureInterval,
                                           [](const char *dataString)
                                           {
                                               uint32_t value = atoi(dataString);
                                               bool result = setMeasureInterval(value);
                                               if (result == true)
                                               {
                                                   // Change current context right away
                                                   context.config.measureInterval = settings.measureInterval;
                                               }
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::PauseInterval,
                                           [](const char *dataString)
                                           {
                                               uint32_t value = atoi(dataString);
                                               bool result = setPauseInterval(value);
                                               if (result == true)
                                               {
                                                   // Change current context right away
                                                   context.config.pauseInterval = settings.pauseInterval;
                                               }
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::AccelRange,
                                           [](const char *dataString)
                                           {
                                               uint8_t value = atoi(dataString);
                                               bool result = Imu::setRange(Imu::Module::Accel, value);
                                               if (result == true && isAccelSet())
                                               {
                                                   // Restart measurements if accelerometer range was changed
                                                   restartMeasurements();
                                               }
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::GyroRange,
                                           [](const char *dataString)
                                           {
                                               uint8_t value = atoi(dataString);
                                               bool result = Imu::setRange(Imu::Module::Gyro, value);
                                               if (result == true && isGyroSet())
                                               {
                                                   // Restart measurements if gyroscope range was changed
                                                   restartMeasurements();
                                               }
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::AdcVoltage,
                                           [](const char *dataString)
                                           {
                                               float value = atof(dataString);
                                               Analog::VddController::setTargetVoltage(value);
                                               if (isAdcSet())
                                               {
                                                   // Restart measurements if ADC target voltage was changed
                                                   restartMeasurements();
                                               }
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::GainSelect,
                                           [](const char *dataString)
                                           {
                                               uint8_t value = atoi(dataString);
                                               Analog::GainSelector::setGain(static_cast<Analog::Gain>(value));
                                               if (context.config.sensorMask & SensorMask::Adc1)
                                               {
                                                   // Restart measurements if ADC1 gain was changed
                                                   restartMeasurements();
                                               }
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::InputTypeSelect,
                                           [](const char *dataString)
                                           {
                                               uint8_t value = atoi(dataString);
                                               bool result = Analog::InputSelector::setInputType(static_cast<Analog::InputType>(value));
                                               if (result == true && (context.config.sensorMask & SensorMask::Adc2))
                                               {
                                                   // Restart measurements if ADC2 input type was changed
                                                   restartMeasurements();
                                               }
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

    bool status = Imu::initialize();
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
        if (!(events & EventBits::taskIsIdle))
        {
            LOG_ERROR("Sampling task creation failed");
            status = false;
        }
    }

    return status;
}

/**
 * @brief Start sensor measurements
 *
 * @return true if processing is started, false otherwise
 */
bool Manager::start()
{
    // Setup the measurements process
    setupMeasurements();

    // Start sensors sampling
    bool result = startSampling();

    return result;
}

/**
 * @brief Stop sensor measurements
 *
 * @return true if processing is stopped, false otherwise
 */
bool Manager::stop()
{
    // Stop sensors sampling
    bool result = stopSampling();

    // Finish the measurements process
    finishMeasurements();

    return result;
}

/**
 * @brief Perform sensor measurements
 */
void Manager::process()
{
    // Check if event occurs
    EventBits_t events = eventGroup.wait(EventBits::segment0Ready | EventBits::segment1Ready, 0);
    if (events != 0)
    {
        size_t segmentIndex = (events & EventBits::segment0Ready) ? 0 : 1;
        processMeasurements(segmentIndex);

        // Increment count of ready segments
        context.segmentCount++;
        size_t measureTimeMs = context.segmentCount * context.segmentTimeMs;

        float readyPcnt = static_cast<float>(measureTimeMs) / secondsToMillis(context.config.measureInterval) * 100;
        LOG_INFO("Segment %d is ready, complete %.1f%%", context.segmentCount, readyPcnt);

        if (readyPcnt >= completePcnt)
        {
            // Stop current sensor measurements
            Manager::stop();

            // Check if board should go to sleep during pause interval
            if (context.config.pauseInterval > 0)
            {
                SD::FS::stop();

                Power::sleep(context.config.pauseInterval);
            }
            else
            {
                // Start next sensor measurements
                Manager::start();
            }
        }
    }
}
