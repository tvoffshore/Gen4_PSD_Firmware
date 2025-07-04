/**
 * @file Types.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Measurement types implementation
 * @version 0.1
 * @date 2025-05-08
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "Measurements/Types.hpp"

#include <assert.h>

using namespace Measurements;

namespace
{
    // Sensor names list
    const char *sensorNames[] = {
        "ACC_X",
        "ACC_Y",
        "ACC_Z",
        "ACC_RES",
        "GYR_X",
        "GYR_Y",
        "GYR_Z",
        "ROLL",
        "PITCH",
        "ADC1",
        "ADC2",
    };
    static_assert(sizeof(sensorNames) / sizeof(*sensorNames) == static_cast<int>(SensorType::Count),
                  "Sensor names list doesn't match to sensor type count!");

    // Data names list
    const char *dataNames[] = {
        "PSD",
        "STAT",
        "RAW",
    };
    static_assert(sizeof(dataNames) / sizeof(*dataNames) == static_cast<int>(DataType::Count),
                  "Data names list doesn't match to data type count!");
} // namespace

/**
 * @brief Return the sensor name for specified sensor type
 *
 * @param sensor Sensor type
 * @return Sensor name
 */
const char *Measurements::getSensorName(SensorType sensor)
{
    assert(sensor < SensorType::Count);

    const char *sensorName = sensorNames[static_cast<int>(sensor)];
    return sensorName;
}

/**
 * @brief Return the data name for specified data type
 *
 * @param data Data type
 * @return Data name
 */
const char *Measurements::getDataName(DataType data)
{
    assert(data < DataType::Count);

    const char *dataName = dataNames[static_cast<int>(data)];
    return dataName;
}
