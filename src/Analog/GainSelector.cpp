/**
 * @file GainSelector.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Analog channel 1 gain selector implementation
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "Analog/GainSelector.hpp"

#include <stdint.h>

#include <Arduino.h>

#include <Log.hpp>
#include <Settings.hpp>

using namespace Analog;

namespace
{
    // Gain representation in a human-readable format
    const char *gainString[] = {"100", "150", "200", "250"};
    static_assert(sizeof(gainString) / sizeof(*gainString) == static_cast<size_t>(Gain::Count));

    // Analog switch logic control input A0
    constexpr auto pinA0 = GPIO_NUM_38;
    // Analog switch logic control input A1
    constexpr auto pinA1 = GPIO_NUM_39;

    // Default gain
    constexpr auto gainDefault = Gain::Gain100;
    // Settings identifier in internal storage
    constexpr auto settingsId = Settings::Id::GainSelector;

#pragma pack(push, 1)
    /**
     * @brief Non volatile settings structure for gain selector
     */
    struct GainSettings
    {
        Gain gain; // The selected gain
    };
#pragma pack(pop)

    // Settings for gain selector
    GainSettings settings = {.gain = gainDefault};

    // Module initialization flag
    bool isInitialized = false;
} // namespace

/**
 * @brief Initialize module
 * Initialize output pins and set gain from the settings
 *
 * @return true if initialization succeeds, false otherwise
 */
bool GainSelector::initialize()
{
    LOG_DEBUG("Initialize gain selector...");

    Settings::read(settingsId, settings);

    // Initialize output control pins
    pinMode(pinA0, OUTPUT);
    pinMode(pinA1, OUTPUT);

    isInitialized = true;

    setGain(settings.gain);

    LOG_INFO("Gain selector initialized, gain %s", gainString[static_cast<size_t>(settings.gain)]);

    return true;
}

/**
 * @brief Get the current gain
 *
 * @return The current gain option
 */
Gain GainSelector::gain()
{
    return settings.gain;
}

/**
 * @brief Set new gain
 *
 * @param[in] selection New gain option
 */
void GainSelector::setGain(Gain gain)
{
    if (isInitialized == false)
    {
        LOG_ERROR("GainSelector isn't initialized!");
        return;
    }

    if (gain >= Gain::Count)
    {
        LOG_WARNING("Unknown gain option %d, force set to default %d", gain, gainDefault);
        gain = gainDefault;
    }

    LOG_DEBUG("Setup gain %s", gainString[static_cast<size_t>(gain)]);

    switch (gain)
    {
    case Gain::Gain100:
        digitalWrite(pinA0, LOW);
        digitalWrite(pinA1, LOW);
        break;
    case Gain::Gain150:
        digitalWrite(pinA1, LOW);
        digitalWrite(pinA0, HIGH);
        break;
    case Gain::Gain200:
        digitalWrite(pinA0, LOW);
        digitalWrite(pinA1, HIGH);
        break;
    case Gain::Gain250:
        digitalWrite(pinA0, HIGH);
        digitalWrite(pinA1, HIGH);
        break;
    }

    if (settings.gain != gain)
    {
        settings.gain = gain;
        Settings::update(settingsId, settings);
    }
}
