/**
 * @file Imu.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief IMU module API
 * @version 0.1
 * @date 2025-05-24
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

namespace Imu
{
    /**
     * @brief IMU device modules
     */
    enum class Module
    {
        Accel, // Accelerometer
        Gyro   // Gyroscope
    };

    // IMU axis value type
    typedef int16_t Axis;

    /**
     * @brief Raw IMU sensor data
     */
    struct Data
    {
        Axis x; // X axis data
        Axis y; // Y axis data
        Axis z; // Z axis data
    };
    static_assert(sizeof(Data) == sizeof(Axis) * 3, "Invalid Data structure packing");

    namespace AccelRange
    {
        constexpr uint8_t G2 = 0;  // +/- 2g
        constexpr uint8_t G4 = 1;  // +/- 4g
        constexpr uint8_t G8 = 2;  // +/- 8g
        constexpr uint8_t G16 = 3; // +/- 16g
    } // namespace AccelRange

    namespace GyroRange
    {
        constexpr uint8_t DPS125 = 0;  // +/- 125deg/sec
        constexpr uint8_t DPS250 = 1;  // +/- 250deg/sec
        constexpr uint8_t DPS500 = 2;  // +/- 500deg/sec
        constexpr uint8_t DPS1000 = 3; // +/- 1000deg/sec
        constexpr uint8_t DPS2000 = 4; // +/- 2000deg/sec
    } // namespace GyroRange

    /**
     * @brief Initialize IMU sensor
     *
     * @return True if sensor has been successfully initialized, false otherwise
     */
    bool initialize();

    /**
     * @brief Switch IMU module into active mode
     *
     * @param module IMU module
     * @return True if start succeed, false otherwise
     */
    bool start(Module module);

    /**
     * @brief Switch IMU module into suspended mode
     *
     * @param module IMU module
     * @return True if stop succeed, false otherwise
     */
    bool stop(Module module);

    /**
     * @brief Read data from IMU module
     *
     * @param module IMU module
     * @param data IMU data
     * @return True if data has been successfully read, false otherwise
     */
    bool read(Module module, Data &data);

    /**
     * @brief Set IMU module range
     *
     * @param module IMU module
     * @param range New modlue range to set
     * @return True if range has been set successfully, false otherwise
     */
    bool setRange(Module module, uint8_t range);

    /**
     * @brief Return IMU module range
     *
     * @param module IMU module
     * @return Current module range
     */
    uint8_t getRange(Module module);

    /**
     * @brief Return the accelerometer scale from raw value to G units
     *
     * @return The factor value
     */
    float getAccelScaleG();

    /**
     * @brief Return the accelerometer scale from raw value to meters/second^2 units
     *
     * @return The factor value
     */
    float getAccelScaleMs2();

    /**
     * @brief Return the gyroscope scale from raw value to degrees/second units
     *
     * @return The factor value
     */
    float getGyroScaleDPS();

    /**
     * @brief Return the gyroscope scale from raw value to radians/second units
     *
     * @return The factor value
     */
    float getGyroScaleRPS();
} // namespace Imu
