/**
 * @file Settings.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Settings module API
 * @version 0.1
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <assert.h>
#include <stddef.h>

#include <EEPROM.h>
#include <FastCRC.h>
#include <Log.hpp>

namespace Settings
{
    /**
     * @brief Settings modules identifiers
     */
    enum class Id
    {
        SerialManager, // SerialManager settings id
        Measurements,  // Measurements setting id
        GainSelector,  // GainSelector settings id
        InputSelector, // InputSelector settings id
        VddController, // VddController settings id
        LogModule,     // LogModule settings id

        Count // Total count of settings modules
    };

    // Modules settings sizes. Settings saved along with corresponding CRC8 value right after data
    constexpr size_t settingsSizeList[] = {
        6,  // SerialManager (int + uint8_t + CRC8) = 6
        14, // Measurements (uint32_t * 2 + uint16_t + uint8_t * 3 + CRC8) = 14
        2,  // GainSelector (uint8_t + CRC8) = 2
        2,  // InputSelector (uint8_t + CRC8) = 2
        5,  // VddController (float + CRC8) = 5
        2,  // LogModule (uint8_t + CRC8) = 2
    };
    static_assert(sizeof(settingsSizeList) / sizeof(*settingsSizeList) == static_cast<size_t>(Settings::Id::Count),
                  "Settings size list doesn't match to modules count!");

    /**
     * @brief Initialize settings storage
     */
    void initialize();

    /**
     * @brief Reset settings storage
     */
    void resetStorage();

    /**
     * @brief Update settings storage with new data
     */
    void updateStorage();

    /**
     * @brief Return adderss of the module settings
     *
     * @param id Module identifier
     * @return Module's address
     */
    size_t getModuleAddress(Id id);

    /**
     * @brief Caclulate CRC8 value for specified data
     *
     * @tparam Type Type of data
     * @param data Data to calculation
     * @return CRC8 value
     */
    template <typename Type>
    uint8_t calculateCrc8(const Type &data)
    {
        FastCRC8 fastCRC8;
        const uint8_t *rawData = static_cast<const uint8_t *>(static_cast<const void *>(&data));

        uint8_t crc8 = fastCRC8.smbus(rawData, sizeof(Type));
        return ~crc8;
    }

    /**
     * @brief Update settings and CRC8 value in internal storage
     *
     * @tparam Type Type of data to update
     * @param id Identifiers of settings to update
     * @param data Data to update
     */
    template <typename Type>
    void update(Settings::Id id, const Type &data)
    {
        assert(id < Settings::Id::Count);

        // Determine start address of data in EEPROM
        size_t address = getModuleAddress(id);
        // Calculate CRC8 value for data to write
        uint8_t crc8 = calculateCrc8(data);

        // Update data and CRC8 value in storage
        EEPROM.put(address, data);
        EEPROM.put(address + sizeof(Type), crc8);

        EEPROM.commit();

        updateStorage();
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
    void read(Settings::Id id, Type &data, bool forceUpdate = false)
    {
        if (forceUpdate == false)
        {
            assert(id < Settings::Id::Count);
            assert(sizeof(Type) < settingsSizeList[static_cast<size_t>(id)]);

            // Determine start address of data in EEPROM
            size_t address = getModuleAddress(id);
            uint8_t storedCrc8;
            Type storedData;

            // Get data and CRC8 value from storage
            EEPROM.get(address, storedData);
            EEPROM.get(address + sizeof(Type), storedCrc8);

            // Check data and return it if valid, otherwise write current data to storage
            uint8_t crc8 = calculateCrc8(storedData);
            if (crc8 == storedCrc8)
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
            update(id, data);
        }
    }
} // namespace Settings
