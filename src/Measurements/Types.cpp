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
    // Directory names for different sensor and data types
    const char *directories[static_cast<int>(SensorType::Count)][static_cast<int>(DataType::Count)] = {
        {"PSD/ACC", "STAT/ACC", "RAW/ACC"},
        {"PSD/GYR", "STAT/GYR", "RAW/GYR"},
        {"PSD/ANG", "STAT/ANG", "RAW/ANG"},
        {"PSD/ADC1", "STAT/ADC1", "RAW/ADC1"},
        {"PSD/ADC2", "STAT/ADC2", "RAW/ADC2"},
        {"PSD/ACC_RES", "STAT/ACC_RES", "RAW/ACC_RES"},
    };
} // namespace

/**
 * @brief Return the directory name for specified sensor and data
 *
 * @param sensor Sensor type
 * @param data Data type
 * @return Directory name
 */
const char *Measurements::getDirectory(SensorType sensor, DataType data)
{
    assert(sensor < SensorType::Count);
    assert(data < DataType::Count);

    const char *name = directories[static_cast<int>(sensor)][static_cast<int>(data)];
    return name;
}
