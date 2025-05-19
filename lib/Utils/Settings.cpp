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
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <EEPROM.h>
#include <FastCRC.h>

#include <Log.hpp>

using namespace Settings;

namespace
{
    // Maximum size of the settings raw data
    constexpr size_t settingsSizeMax = 100;
    // Offset of settings raw data in the EEPROM
    constexpr size_t settingsOffset = 0;

    // Modules settings EEPROM addresses
    std::array<size_t, static_cast<size_t>(Settings::Id::Count)> settingsAddressList;
    // Settings raw data size
    size_t settingsSize = 0;
    // Settings raw data buffer
    uint8_t settingsData[settingsSizeMax] = {0};
}; // namespace

/**
 * @brief Initialize settings storage
 */
void Settings::initialize()
{
    // Reset settings size first
    settingsSize = 0;

    for (size_t idx = 0; idx < static_cast<size_t>(Settings::Id::Count); idx++)
    {
        settingsAddressList[idx] = settingsOffset + settingsSize;
        LOG_DEBUG("Module %d address %d, size %d", idx, settingsAddressList[idx], settingsSizeList[idx]);
        settingsSize += settingsSizeList[idx];
    }

    bool result = EEPROM.begin(settingsOffset + settingsSize + sizeof(uint32_t));
    if (result == true)
    {
        LOG_INFO("EEPROM initialized, size %d bytes", settingsSize);

        assert(settingsSize > 0 && settingsSize <= settingsSizeMax);

        // Read raw settings data and CRC32 value from the storage
        size_t readSize = EEPROM.readBytes(settingsOffset, settingsData, settingsSize);
        if (readSize == settingsSize)
        {
            FastCRC32 fastCRC32;
            uint32_t calcCrc = fastCRC32.crc32(settingsData, readSize);
            uint32_t readCrc = EEPROM.readULong(settingsOffset + settingsSize);
            if (readCrc == calcCrc)
            {
                LOG_INFO("Storage is valid");
            }
            else
            {
                LOG_WARNING("Storage isn't valid");
                resetStorage();
            }
        }
        else
        {
            LOG_ERROR("Storage read failed");
        }
    }
    else
    {
        LOG_ERROR("EEPROM initialize failed");
    }
}

/**
 * @brief Reset settings storage
 */
void Settings::resetStorage()
{
    assert(settingsSize > 0 && settingsSize <= settingsSizeMax);

    LOG_INFO("Reset storage, size %d", settingsSize);

    // Write zero data to the storage
    memset(settingsData, 0, settingsSize);
    EEPROM.writeBytes(settingsOffset, settingsData, settingsSize);

    updateStorage();
}

/**
 * @brief Update settings storage with new data
 */
void Settings::updateStorage()
{
    assert(settingsSize > 0 && settingsSize <= settingsSizeMax);

    // Read raw settings data from the storage
    size_t readSize = EEPROM.readBytes(settingsOffset, settingsData, settingsSize);
    if (readSize == settingsSize)
    {
        FastCRC32 fastCRC32;
        uint32_t crc32 = fastCRC32.crc32(settingsData, settingsSize);
        EEPROM.writeULong(settingsSize, crc32);

        EEPROM.commit();
    }
    else
    {
        LOG_ERROR("Storage read failed");
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
