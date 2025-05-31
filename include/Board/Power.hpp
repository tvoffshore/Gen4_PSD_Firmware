/**
 * @file Power.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Power controller API
 * @version 0.1
 * @date 2025-05-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <stdint.h>

namespace Power
{
    // Minimum CPU frequency
    constexpr uint32_t cpuFrequencyMinMHz = 10;
    // Maximum CPU frequency
    constexpr uint32_t cpuFrequencyMaxMHz = 240;

    /**
     * @brief Wake up reasons
     */
    enum class WakeUpReason
    {
        Light,    // Wake up by Light signal
        Serial,   // Wake up by Serial command
        Timer,    // Wake up by Timer timeout
        NotSleep, // Wake was not caused by exit from sleep
        Unknow,   // Other reasons
        Count     // Total count of wake up reasons
    };

    /**
     * @brief Sleep mode identifiers
     */
    enum class SleepMode
    {
        LightSleep, // 0: Light sleep
        DeepSleep,  // 1: Deep sleep
        Count       // Total count of sleep modes
    };

    /**
     * @brief Get wake up reason string
     *
     * @param mode Wake up reason
     * @return Reason string
     */
    const char *wakeUpReasonToString(WakeUpReason reason);

    /**
     * @brief Get sleep mode string
     *
     * @param mode Sleep mode
     * @return Mode string
     */
    const char *sleepModeToString(SleepMode mode);

    /**
     * @brief Initialize Power module
     */
    void initialize();

    /**
     * @brief Get current sources to wake up from sleep, bitmask
     *
     * @return Current wake up sources
     */
    uint8_t wakeUpSources();

    /**
     * @brief Set new sources to wake up from sleep, bitmask
     *
     * @param sources New wake up sources
     */
    void setWakeUpSources(uint8_t wakeUpSources);

    /**
     * @brief Set CPU frequency
     *
     * @param frequencyMHz New frequency (10MHz min, 240MHz max)
     */
    void setCpuFrequency(uint32_t frequencyMHz);

    /**
     * @brief Goes into sleep mode and wait wake up events
     *
     * @param sleepDuration Time to sleep (ignored if Timer wake source isn't set), seconds
     * @param sleepMode Sleep mode to move into
     * @return Reason of waking up
     */
    WakeUpReason sleep(uint32_t sleepDuration, SleepMode sleepMode = SleepMode::DeepSleep);

    /**
     * @brief Return the reason by which board has been awakened from sleep
     *
     * @return Wake up reason
     */
    WakeUpReason getWakeUpReason();
} // namespace Power
