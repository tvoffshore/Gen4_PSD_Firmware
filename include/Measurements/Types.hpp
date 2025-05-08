/**
 * @file Types.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Measurement types API
 * @version 0.1
 * @date 2025-05-08
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

namespace Measurements
{
    /**
     * @brief Sensor type identifiers
     */
    enum class SensorType : int
    {
        Adc1,
        Adc2,
        Accel,
        AccelResult,
        Gyro,
        Angle,

        Count
    };

    /**
     * @brief Data type identifiers
     */
    enum class DataType : int
    {
        Psd,
        Statistic,
        Raw,

        Count
    };

    /**
     * @brief Return the directory name for specified sensor and data
     *
     * @param sensor Sensor type
     * @param data Data type
     * @return Directory name
     */
    const char *getDirectory(SensorType sensor, DataType data);
} // namespace Measurements
