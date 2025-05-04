/**
 * @file Pot.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Digital POT implementation
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "Pot/Pot.hpp"

#include <stdbool.h>
#include <stdint.h>

#include <Debug.hpp>

#include "Pot/Mcp45hv.hpp"

using namespace POT;

namespace
{
    // Total resistance steps count
    constexpr uint8_t wiperCodeMax = 255; // 8-bit
    // Maximum resistance
    constexpr uint32_t resistanceMaxOhm = 100000; // 100kOhm
    // Resistance of one step
    constexpr uint32_t resistanceStepOhm = resistanceMaxOhm / wiperCodeMax; // 392.16

    /**
     * @brief Convert resistance to wiper code
     *
     * @param resistanceOhm Resistance, Ohm
     * @return Wiper code
     */
    constexpr uint8_t convertToWiperCode(uint32_t resistanceOhm)
    {
        return resistanceOhm * wiperCodeMax / resistanceMaxOhm;
    }

    /**
     * @brief Convert wiper code to resistance
     *
     * @param wiperCode Wiper code
     * @return Resistance, Ohm
     */
    constexpr uint32_t convertToResistanceOhm(uint8_t wiperCode)
    {
        return wiperCode * resistanceMaxOhm / wiperCodeMax;
    }
} // namespace

/**
 * @brief Initialize POT device
 * @note Setup W and B terminals and maximum resistance
 *
 * @return true if initialization succeed, false otherwise
 */
bool POT::initialize()
{
    LOG_DEBUG("Initialize POT device...");

    // Set maximum resistance
    bool result = setResistance(resistanceMaxOhm);
    if (result == false)
    {
        LOG_ERROR("Set maximum resistace fail!");
        return false;
    }

    uint32_t resistance;
    result = getResistance(resistance);
    if (result == false || resistance != resistanceMaxOhm)
    {
        LOG_ERROR("Get maximum resistace fail!");
        return false;
    }

    // Connect W and B terminals
    result = Mcp45hv::configure(POT::TerminalConfig::ConnectWB);
    if (result == false)
    {
        LOG_ERROR("Terminal configuration fail!");
        return false;
    }

    if (result == true)
    {
        LOG_INFO("POT device initialized");
    }

    return result;
}

/**
 * @brief Set specified resistance to Ohm value
 *
 * @param[in] resistanceOhm Resistance to set
 * @return true if setting succeed, false otherwise
 */
bool POT::setResistance(uint32_t resistanceOhm)
{
    if (resistanceOhm > resistanceMaxOhm)
    {
        resistanceOhm = resistanceMaxOhm;
    }

    uint8_t wiperCode = convertToWiperCode(resistanceOhm);

    LOG_TRACE("Set resistance %dOhm, wiper code 0x%02X", resistanceOhm, wiperCode);

    return Mcp45hv::setWiper(wiperCode);
}

/**
 * @brief Get the current resistance value
 *
 * @param[out] resistanceOhm Reference to the current resistance value
 * @return true if getting succeed, false otherwise
 */
bool POT::getResistance(uint32_t &resistanceOhm)
{
    uint8_t wiperCode;

    bool result = Mcp45hv::getWiper(wiperCode);
    if (result == true)
    {
        resistanceOhm = convertToResistanceOhm(wiperCode);

        LOG_TRACE("Get wiper code 0x%02X, resistance %dOhm", wiperCode, resistanceOhm);
    }

    return result;
}

/**
 * @brief Increase resistance value by one step
 *
 * @return true if increasing succeed, false otherwise
 */
bool POT::increaseResistance()
{
    LOG_TRACE("Increase resistance by one step");

    return Mcp45hv::incrementWiper();
}

/**
 * @brief Decrease resistance value by one step
 *
 * @return true if decreasing succeed, false otherwise
 */
bool POT::decreaseResistance()
{
    LOG_TRACE("Decrease resistance by one step");

    return Mcp45hv::decrementWiper();
}
