#pragma once

#include <array>
#include <assert.h>
#include <Debug.hpp>
#include <EEPROM.h>
#include <FastCRC.h>

/**
 * @brief Settings modules identifiers
 */
enum class SettingsModules
{
    SerialManager, // SerialManager settings id
    Measurements,  // Measurements setting id

    Count // Total count of settings modules
};

namespace
{
    // Modules settings sizes
    constexpr size_t settingsSizeList[] = {
        6,  // SerialManager (int + uint8_t + CRC8) = 6
        11, // Measurements (uint32_t * 2 + uint8_t * 2 + CRC8) = 11
    };
    static_assert(sizeof(settingsSizeList) / sizeof(*settingsSizeList) == static_cast<size_t>(SettingsModules::Count),
                  "Settings size list doesn't match to modules count!");
}; // namespace

/**
 * @brief Internal storage of non volatile data
 * Settings saved along with corresponding CRC8 value right after data
 */
struct InternalStorage
{
    /**
     * @brief Initialize internal storage
     */
    static void initialize()
    {
        LOG_INFO("Initialize internal storage...");

        size_t offset = 0;
        settingsAddressList[0] = offset;
        LOG_TRACE("Module 0 address %d, size %d", offset, settingsSizeList[0]);

        for (size_t i = 1; i < static_cast<size_t>(SettingsModules::Count); i++)
        {
            offset += settingsSizeList[i - 1];
            settingsAddressList[i] = offset;
            LOG_TRACE("Module %d address %d, size %d", i, offset, settingsSizeList[i]);
        }

        offset += settingsSizeList[static_cast<size_t>(SettingsModules::Count) - 1];

        bool result = EEPROM.begin(offset);
        if (result == true)
        {
            LOG_INFO("Internal storage initialized, total settings size: %d bytes", offset);
        }
        else
        {
            LOG_ERROR("Internal storage initialization failed");
        }
    }

    /**
     * @brief Read settings from internal storage and check it with CRC8
     * If stored data isn't valid write current data to the storage
     *
     * @tparam Type Type of data to read
     * @param id Identifiers of settings to read
     * @param data Data to read
     * @param forceUpdate True - force update with current settings, false - read and check first
     */
    template <typename Type>
    static void readSettings(SettingsModules id, Type &data, bool forceUpdate = false)
    {
        if (forceUpdate == false)
        {
            assert(id < SettingsModules::Count);
            assert(sizeof(Type) < settingsSizeList[static_cast<size_t>(id)]);

            // Determine start address of data in EEPROM
            size_t address = settingsAddressList[static_cast<size_t>(id)];
            uint8_t storedCrc8;
            Type storedData;

            // Get data and CRC8 value from storage
            EEPROM.get(address, storedData);
            EEPROM.get(address + sizeof(Type), storedCrc8);

            LOG_TRACE("Module %d address %d EEPROM get %d bytes:", id, address, sizeof(Type));
            for (size_t i = 0; i < sizeof(Type); i++)
            {
                LOG_TRACE("[%d]: %d", i, ((uint8_t *)&storedData)[i]);
            }

            // Check data and return it if valid, otherwise write current data to storage
            if (calculateCrc8(storedData) == storedCrc8)
            {
                data = storedData;
            }
            else
            {
                LOG_ERROR("Module %d data isn't valid", id);
                forceUpdate = true;
            }
        }

        if (forceUpdate == true)
        {
            LOG_INFO("Module %d force update", id);
            // Force update current settings data in internal storage
            updateSettings(id, data);
        }
    }

    /**
     * @brief Update settings and CRC8 value in internal storage
     *
     * @tparam Type Type of data to update
     * @param id Identifiers of settings to update
     * @param data Data to update
     */
    template <typename Type>
    static void updateSettings(SettingsModules id, const Type &data)
    {
        assert(id < SettingsModules::Count);

        // Determine start address of data in EEPROM
        size_t address = settingsAddressList[static_cast<size_t>(id)];
        // Calculate CRC8 value for data to write
        uint8_t crc8 = calculateCrc8(data);

        LOG_TRACE("Module %d address %d EEPROM put %d bytes:", id, address, sizeof(Type));
        for (size_t i = 0; i < sizeof(Type); i++)
        {
            LOG_TRACE("[%d]: %d", i, ((uint8_t *)&data)[i]);
        }

        // Update data and CRC8 value in storage
        EEPROM.put(address, data);
        EEPROM.put(address + sizeof(Type), crc8);

        EEPROM.commit();
    }

private:
    /**
     * @brief Caclulate CRC8 value for specified data
     *
     * @tparam Type Type of data
     * @param data Data to calculation
     * @return CRC8 value
     */
    template <typename Type>
    static uint8_t calculateCrc8(const Type &data)
    {
        FastCRC8 crc8;
        const uint8_t *rawData = static_cast<const uint8_t *>(static_cast<const void *>(&data));

        return crc8.smbus(rawData, sizeof(Type));
    }

    // Modules settings EEPROM addresses
    static std::array<size_t, static_cast<size_t>(SettingsModules::Count)> settingsAddressList;
};
