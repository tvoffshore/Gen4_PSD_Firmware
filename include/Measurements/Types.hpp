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
    enum class SensorType
    {
        AccelX,
        AccelY,
        AccelZ,
        AccelResult,
        GyroX,
        GyroY,
        GyroZ,
        Roll,
        Pitch,
        Adc1,
        Adc2,

        Count
    };

    /**
     * @brief Data type identifiers
     */
    enum class DataType
    {
        Psd,
        Statistic,
        Raw,

        Count
    };

    /**
     * @brief Return the sensor name for specified sensor type
     *
     * @param sensor Sensor type
     * @return Sensor name
     */
    const char *getSensorName(SensorType sensor);

    /**
     * @brief Return the data name for specified data type
     *
     * @param data Data type
     * @return Data name
     */
    const char *getDataName(DataType data);
} // namespace Measurements
