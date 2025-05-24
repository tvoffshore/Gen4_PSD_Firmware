/**
 * @file Imu.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief IMU module implementation
 * @version 0.1
 * @date 2025-05-24
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "Imu/Imu.hpp"

#include <stdbool.h>
#include <stdint.h>

#include <Wire.h>

#include <IIM42652.h>
#include <Log.hpp>
#include <Settings.hpp>

using namespace Imu;

namespace
{
    // I2C transport
    constexpr auto &wire = Wire;
    // I2C address (AP_AD0 is logic low)
    constexpr int address = 0x68;

    // Accelerometer ranges, G
    constexpr size_t accelRangesG[] = {2, 4, 8, 16};
    static_assert(sizeof(accelRangesG) / sizeof(*accelRangesG) > AccelRange::G16);

    // Gyroscope ranges, deg/sec
    constexpr size_t gyroRangesDps[] = {125, 250, 500, 1000, 2000};
    static_assert(sizeof(gyroRangesDps) / sizeof(*gyroRangesDps) > GyroRange::DPS2000);

    // Accelerometer full-scale settings
    constexpr IIM42652_ACCEL_CONFIG0_FS_SEL_t accelFullScaleList[] = {
        IIM42652_ACCEL_CONFIG0_FS_SEL_2g,  // Option #0: +/- 2g
        IIM42652_ACCEL_CONFIG0_FS_SEL_4g,  // Option #1: +/- 4g
        IIM42652_ACCEL_CONFIG0_FS_SEL_8g,  // Option #2: +/- 8g
        IIM42652_ACCEL_CONFIG0_FS_SEL_16g, // Option #3: +/- 16g
    };
    static_assert(sizeof(accelFullScaleList) / sizeof(*accelFullScaleList) ==
                  sizeof(accelRangesG) / sizeof(*accelRangesG));

    // Gyroscope full-scale settings
    constexpr IIM42652_GYRO_CONFIG0_FS_SEL_t gyroFullScaleList[] = {
        IIM42652_GYRO_CONFIG0_FS_SEL_125dps,  // Option #0: +/- 125 degrees/second
        IIM42652_GYRO_CONFIG0_FS_SEL_250dps,  // Option #1: +/- 250 degrees/second
        IIM42652_GYRO_CONFIG0_FS_SEL_500dps,  // Option #2: +/- 500 degrees/second
        IIM42652_GYRO_CONFIG0_FS_SEL_1000dps, // Option #3: +/- 1000 degrees/second
        IIM42652_GYRO_CONFIG0_FS_SEL_2000dps, // Option #4: +/- 2000 degrees/second
    };
    static_assert(sizeof(gyroFullScaleList) / sizeof(*gyroFullScaleList) ==
                  sizeof(gyroRangesDps) / sizeof(*gyroRangesDps));

    // IMU axis maximum value
    constexpr size_t axisMaxValue = 32768;
    // IMU axis value after the reset
    constexpr Axis axisResetValue = -32768;
    // Delay between checking if the IMU axis values are valid, milliseconds
    constexpr uint32_t waitValidDelayMs = 1;
    // Timeout of waiting for the valid IMU axis values, milliseconds
    constexpr uint32_t waitValidTimeoutMs = 100;

    // Accelerometer default range index
    constexpr uint8_t accelRangeDefault = AccelRange::G2;
    // Gyroscope default range index
    constexpr uint8_t gyroRangeDefault = GyroRange::DPS500;

    // Settings identifier in internal storage
    constexpr auto settingsId = Settings::Id::ImuSensor;

#pragma pack(push, 1)
    /**
     * @brief Non volatile settings structure for IMU
     */
    struct ImuSettings
    {
        uint8_t accelRange; // Accelerometer range
        uint8_t gyroRange;  // Gyroscope range
    };
#pragma pack(pop)

    // Settings for input selector
    ImuSettings settings = {
        .accelRange = accelRangeDefault,
        .gyroRange = gyroRangeDefault,
    };

    // IIM42652 initialized flag
    bool isInitialized = false;
    // IIM42652 driver object
    IIM42652 iim42652;
} // namespace

/**
 * @brief Initialize IMU sensor
 *
 * @return True if sensor has been successfully initialized, false otherwise
 */
bool Imu::initialize()
{
    if (isInitialized == false)
    {
        Settings::read(settingsId, settings);

        isInitialized = iim42652.begin(wire, address);
        if (isInitialized == true)
        {
            isInitialized = setRange(Module::Accel, settings.accelRange);
            if (isInitialized == true)
            {
                isInitialized = setRange(Module::Gyro, settings.gyroRange);
            }
        }
    }
    else
    {
        LOG_WARNING("Sensor was already initialized");
    }

    return isInitialized;
}

/**
 * @brief Switch IMU module into active mode
 *
 * @param module IMU module
 * @return True if start succeed, false otherwise
 */
bool Imu::start(Module module)
{
    bool result = false;

    if (isInitialized == true)
    {
        LOG_DEBUG("Activate %s", module == Module::Accel ? "ACCEL" : "GYRO");

        if (module == Module::Accel)
        {
            result = iim42652.accelerometer_enable();
        }
        else
        {
            result = iim42652.gyroscope_enable();
        }

        if (result == true)
        {
            result = false;

            uint32_t elapsedMs = 0;
            while (elapsedMs < waitValidTimeoutMs)
            {
                Data data;
                bool status = read(module, data);
                if (status == true &&
                    data.x != axisResetValue && data.y != axisResetValue && data.z != axisResetValue)
                {
                    result = true;
                    break;
                }

                delay(waitValidDelayMs);
                elapsedMs += waitValidDelayMs;
            }

            LOG_DEBUG("%s %s ready in %u ms", module == Module::Accel ? "ACCEL" : "GYRO",
                      result ? "is" : "isn't", elapsedMs);
        }
    }
    else
    {
        LOG_WARNING("Sensor isn't initialized, can't be started");
    }

    return result;
}

/**
 * @brief Switch IMU module into suspended mode
 *
 * @param module IMU module
 * @return True if stop succeed, false otherwise
 */
bool Imu::stop(Module module)
{
    bool result = false;

    if (isInitialized == true)
    {
        LOG_INFO("Suspend %s", module == Module::Accel ? "ACCEL" : "GYRO");

        if (module == Module::Accel)
        {
            result = iim42652.accelerometer_disable();
        }
        else
        {
            result = iim42652.gyroscope_disable();
        }
    }
    else
    {
        LOG_WARNING("Sensor isn't initialized, can't be stopped");
    }

    return result;
}

/**
 * @brief Read data from IMU module
 *
 * @param module IMU module
 * @param data IMU data
 * @return True if data has been successfully read, false otherwise
 */
bool Imu::read(Module module, Data &data)
{
    bool result = false;

    if (isInitialized == true)
    {
        IIM42652_axis_t readData;

        if (module == Module::Accel)
        {
            result = iim42652.get_accel_data(&readData);
        }
        else
        {
            result = iim42652.get_gyro_data(&readData);
        }

        if (result == true)
        {
            data.x = readData.x;
            data.y = readData.y;
            data.z = readData.z;
        }
    }

    return result;
}

/**
 * @brief Set the IMU module range
 *
 * @param module IMU module
 * @param range New modlue range to set
 * @return True if range has been set successfully, false otherwise
 */
bool Imu::setRange(Module module, uint8_t range)
{
    bool result = false;

    if (module == Module::Accel)
    {
        if (range > AccelRange::G16)
        {
            range = AccelRange::G16;
        }

        if (settings.accelRange != range)
        {
            settings.accelRange = range;
            Settings::update(settingsId, settings);
        }

        IIM42652_ACCEL_CONFIG0_FS_SEL_t accelFullScale = accelFullScaleList[range];
        result = iim42652.set_accel_fsr(accelFullScale);
    }
    else
    {
        if (range > GyroRange::DPS2000)
        {
            range = GyroRange::DPS2000;
        }

        if (settings.gyroRange != range)
        {
            settings.gyroRange = range;
            Settings::update(settingsId, settings);
        }

        IIM42652_GYRO_CONFIG0_FS_SEL_t gyroFullScale = gyroFullScaleList[range];
        result = iim42652.set_gyro_fsr(gyroFullScale);
    }

    return result;
}

/**
 * @brief Get IMU module range
 *
 * @param module IMU module
 * @return Current module range
 */
uint8_t Imu::range(Module module)
{
    uint8_t range = module == Module::Accel ? settings.accelRange : settings.gyroRange;

    return range;
}

/**
 * @brief Convert accelerometer axis value to m/s^2 units
 *
 * @param value Axis value
 * @return Value in m/s^2 units
 */
float Imu::accelToMs2(Axis value)
{
    assert(settings.accelRange < sizeof(accelRangesG) / sizeof(*accelRangesG));

    size_t accelRangeG = accelRangesG[settings.accelRange];
    float units = (float)value * accelRangeG * 9.81 / axisMaxValue;

    return units;
}

/**
 * @brief Convert accelerometer axis value to G units
 *
 * @param value Axis value
 * @return Value in G units
 */
float Imu::accelToG(Axis value)
{
    assert(settings.accelRange < sizeof(accelRangesG) / sizeof(*accelRangesG));

    size_t accelRangeG = accelRangesG[settings.accelRange];
    float units = (float)value * accelRangeG / axisMaxValue;

    return units;
}

/**
 * @brief Convert gyroscope axis value to RAD/s units
 *
 * @param value Axis value
 * @return Value in RAD/s units
 */
float Imu::gyroToRads(Axis value)
{
    assert(settings.gyroRange < sizeof(gyroRangesDps) / sizeof(*gyroRangesDps));

    size_t gyroRangeDps = gyroRangesDps[settings.gyroRange];
    float units = (float)value * gyroRangeDps * M_PI / 360 / axisMaxValue;

    return units;
}

/**
 * @brief Convert gyroscope axis value to Deg/s units
 *
 * @param value Axis value
 * @return Value in Deg/s units
 */
float Imu::gyroToDegs(Axis value)
{
    assert(settings.gyroRange < sizeof(gyroRangesDps) / sizeof(*gyroRangesDps));

    size_t gyroRangeDps = gyroRangesDps[settings.gyroRange];
    float units = (float)value * gyroRangeDps / axisMaxValue;

    return units;
}
