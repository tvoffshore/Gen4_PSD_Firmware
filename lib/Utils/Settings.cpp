/**
 * @file Settings.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Settings module implementation
 * @version 0.1
 * @date 2025-05-07
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "Settings.hpp"

#include <array>

#include <EEPROM.h>

#include <Log.hpp>

using namespace Settings;

namespace
{
    // Modules settings EEPROM addresses
    std::array<size_t, static_cast<size_t>(Settings::Id::Count)> settingsAddressList;
}; // namespace

/**
 * @brief Initialize internal storage
 */
void Settings::initialize()
{
    size_t offset = 0;
    settingsAddressList[0] = offset;
    LOG_TRACE("Module 0 address %d, size %d", offset, settingsSizeList[0]);

    for (size_t i = 1; i < static_cast<size_t>(Settings::Id::Count); i++)
    {
        offset += settingsSizeList[i - 1];
        settingsAddressList[i] = offset;
        LOG_TRACE("Module %d address %d, size %d", i, offset, settingsSizeList[i]);
    }

    offset += settingsSizeList[static_cast<size_t>(Settings::Id::Count) - 1];

    bool result = EEPROM.begin(offset);
    if (result == true)
    {
        LOG_INFO("Settings initialized, total size: %d bytes", offset);
    }
    else
    {
        LOG_ERROR("Settings initialization failed");
    }
}

/**
 * @brief Return adderss of the module settings
 *
 * @param id Module identifier
 * @return Module's address
 */
size_t Settings::getModuleAddress(Id id)
{
    size_t address = settingsAddressList[static_cast<size_t>(id)];

    return address;
}
