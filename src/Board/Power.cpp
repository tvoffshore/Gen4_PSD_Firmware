/**
 * @file Power.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-05-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "Board/Power.hpp"

#include <stddef.h>
#include <stdint.h>

#include <esp_sleep.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>

#include <Log.hpp>
#include <Settings.hpp>

#include "Board/Board.hpp"
#include "Serial/Interfaces/Max3221.hpp"

using namespace Power;

namespace
{
    // Photo diode pin (ADC_1_CH6)
    constexpr auto pinPhotoDiode = GPIO_NUM_7;

    /**
     * @brief Wake up sources
     */
    namespace WakeUpSource
    {
        constexpr uint8_t Light = (1 << 0);             // 1: Wake up on light sensor
        constexpr uint8_t Serial = (1 << 1);            // 2: Wake up on read measurements serial commands received
        constexpr uint8_t Timer = (1 << 2);             // 4: Wake up timer with specified frequency
        constexpr uint8_t All = Light | Serial | Timer; // 7: All wake up sources
    }; // namespace WakeUpSource

    /**
     * @brief Convert seconds to microseconds
     *
     * @param seconds Time in seconds
     * @return Microseconds
     */
    constexpr uint64_t secondsToMicros(uint32_t seconds)
    {
        return static_cast<uint64_t>(seconds) * 1000 * 1000;
    }

    // Default sources to wake up from sleep, bitmask
    constexpr uint8_t wakeUpSourcesDefault = WakeUpSource::All;
    // Settings identifier in internal storage
    constexpr auto settingsId = Settings::Id::PowerModule;

#pragma pack(push, 1)
    /**
     * @brief Non volatile settings structure for log module
     */
    struct PowerSettings
    {
        uint8_t wakeUpSources; // Sources to wake up from sleep, bitmask
    };
#pragma pack(pop)

    // Settings for log module
    PowerSettings settings = {.wakeUpSources = wakeUpSourcesDefault};
} // namespace

/**
 * @brief Get wake up reason string
 *
 * @param mode Wake up reason
 * @return Reason string
 */
const char *Power::wakeUpReasonToString(WakeUpReason reason)
{
    // Wake up reason strings
    const char *wakeUpReasonString[static_cast<size_t>(WakeUpReason::Count)] = {
        "LIGHT",
        "SERIAL",
        "TIMER",
        "NOT SLEEP",
        "UNKNOWN",
    };

    const char *string = wakeUpReasonString[static_cast<size_t>(reason)];
    return string;
}

/**
 * @brief Get sleep mode string
 *
 * @param mode Sleep mode
 * @return Mode string
 */
const char *Power::sleepModeToString(SleepMode mode)
{
    // Sleep mode strings
    const char *sleepModeString[static_cast<size_t>(SleepMode::Count)] = {
        "LIGHT",
        "DEEP",
    };

    const char *string = sleepModeString[static_cast<size_t>(mode)];
    return string;
}

/**
 * @brief Initialize Power module
 */
void Power::initialize()
{
    Settings::read(settingsId, settings);
}

/**
 * @brief Get current sources to wake up from sleep, bitmask
 *
 * @return Current wake up sources
 */
uint8_t Power::wakeUpSources()
{
    return settings.wakeUpSources;
}

/**
 * @brief Set new sources to wake up from sleep, bitmask
 *
 * @param wakeUpSources New wake up sources
 */
void Power::setWakeUpSources(uint8_t wakeUpSources)
{
    if (settings.wakeUpSources != wakeUpSources)
    {
        LOG_INFO("Set new wake up sources: %u -> %u", settings.wakeUpSources, wakeUpSources);
        settings.wakeUpSources = wakeUpSources;
        Settings::update(settingsId, settings);
    }
}

/**
 * @brief Set CPU frequency
 *
 * @param frequencyMHz New frequency (10MHz min, 240MHz max)
 */
void Power::setCpuFrequency(uint32_t frequencyMHz)
{
    if (frequencyMHz < cpuFrequencyMinMHz)
    {
        frequencyMHz = cpuFrequencyMinMHz;
    }
    if (frequencyMHz > cpuFrequencyMaxMHz)
    {
        frequencyMHz = cpuFrequencyMaxMHz;
    }

    setCpuFrequencyMhz(frequencyMHz);
    LOG_DEBUG("CPU frequency: %u MHz", getCpuFrequencyMhz());
}

/**
 * @brief Goes into sleep mode and wait wake up events
 *
 * @param sleepDuration Time to sleep (ignored if Timer wake source isn't set), seconds
 * @param sleepMode Sleep mode to move into
 * @return Reason of waking up
 */
WakeUpReason Power::sleep(uint32_t sleepDuration, SleepMode sleepMode)
{
    LOG_INFO("Entering into %s sleep mode", sleepModeToString(sleepMode));

    if (settings.wakeUpSources & WakeUpSource::Light)
    {
        esp_sleep_enable_ext1_wakeup(1 << pinPhotoDiode, ESP_EXT1_WAKEUP_ANY_HIGH);
    }

    if (settings.wakeUpSources & WakeUpSource::Serial)
    {
        esp_sleep_enable_ext0_wakeup(Serials::Max3221::pinRx, LOW);
    }

    if ((settings.wakeUpSources & WakeUpSource::Timer) && sleepDuration > 0)
    {
        LOG_INFO("Wake up in %u sec", sleepDuration);

        esp_sleep_enable_timer_wakeup(secondsToMicros(sleepDuration));
    }

    switch (sleepMode)
    {
    case SleepMode::LightSleep:
        esp_light_sleep_start();
        break;
    case SleepMode::DeepSleep:
        // Power down the board
        Board::powerDown();
        // Hold gpio pins
        Board::holdPins();

        esp_deep_sleep_start();
        // Never reachable
        break;
    default:
        break;
    }

    WakeUpReason wakeUpReason = getWakeUpReason();
    LOG_INFO("Wake up reason: %s", wakeUpReasonToString(wakeUpReason));

    if (settings.wakeUpSources & WakeUpSource::Light)
    {
        // Release the Light pin
        rtc_gpio_deinit(pinPhotoDiode);
    }

    if (settings.wakeUpSources & WakeUpSource::Serial)
    {
        // Release the Rx pin
        rtc_gpio_deinit(Serials::Max3221::pinRx);
    }

    // Disable all wakeup sources
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    return wakeUpReason;
}

/**
 * @brief Return the reason by which board has been awakened from sleep
 *
 * @return Wake up reason
 */
WakeUpReason Power::getWakeUpReason()
{
    auto wakeupCause = esp_sleep_get_wakeup_cause();

    WakeUpReason wakeUpReason;
    switch (wakeupCause)
    {
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        wakeUpReason = WakeUpReason::NotSleep;
        break;

    case ESP_SLEEP_WAKEUP_EXT0:
        wakeUpReason = WakeUpReason::Serial;
        break;

    case ESP_SLEEP_WAKEUP_EXT1:
        wakeUpReason = WakeUpReason::Light;
        break;

    case ESP_SLEEP_WAKEUP_TIMER:
        wakeUpReason = WakeUpReason::Timer;
        break;

    default:
        wakeUpReason = WakeUpReason::Unknow;
        break;
    }

    return wakeUpReason;
}
