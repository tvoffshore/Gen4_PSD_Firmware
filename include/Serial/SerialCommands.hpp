/**
 * @file SerialCommands.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Serial commands list
 * @version 0.1
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <assert.h>
#include <stdint.h>

namespace Serials
{
    namespace AccessMask
    {
        constexpr uint8_t none = 0;           // No access code
        constexpr uint8_t read = (1 << 0);    // Read access bitmask
        constexpr uint8_t write = (1 << 1);   // Write access bitmask
        constexpr uint8_t execute = (1 << 2); // Execute access bitmask

    } // namespace AccessMask

    /**
     * @brief Serial command identifiers
     */
    enum class CommandId
    {
        SlaveAddress,     // 0: Set/Get serial device slave address
        Date,             // 1: Set/Get current date
        Time,             // 2: Set/Get current time
        PsdPoints,        // 3: Set/Get the PSD points count (PSD segment size = 2^x)
        PsdCutoff,        // 4: Set/Get PSD cutoff to store the results
        DataTypeControl,  // 5: Set/Get bitmask for data type to calculate and store on SD
        MeasureControl,   // 6: Set/Get bitmask for sensor measurements capabilities
        MeasureFrequency, // 7: Set/Get the frequency of measurements
        MeasureInterval,  // 8: Set/Get the interval of measurements
        PauseInterval,    // 9: Set/Get the interval of pause between measurements
        AccelRange,       // 10: Set/Get ACCEL sensor range
        GyroRange,        // 11: Set/Get GYRO sensor range
        AdcVoltage,       // 12: Set/Get desired voltage for ADC1/ADC2 sensors
        GainSelect,       // 13: Set/Get gain selection for ADC1 sensor
        InputTypeSelect,  // 14: Set/Get input type selection for ADC2 sensor
        SerialSelect,     // 15: Set/Get serial interface selection
        BacklightState,   // 16: Set/Get display backlight state
        BacklightLevel,   // 17: Set/Get display backlight level
        DisplayOnSource,  // 18: Set/Get the sources that activate the display after sleep
        WakeUpSource,     // 19: Set/Get the sources to wake up the board from sleep
        LogLevel,         // 20: Set/Get log messages level
        BatteryStatus,    // 21: Get battery status
        FwVersion,        // 22: Get FW version information

        // There are application commands below, hidden for user
        AppKeepAlive,       // 23: External application keep alive command
        AppDownloadRecent,  // 24: Set download range starting from now
        AppDownloadHistory, // 25: Set download range starting from timestamp
        AppDownloadType,    // 26: Set download data type
        AppDownloadSize,    // 27: Get download data size in bytes
        AppDownloadData,    // 28: Get current download data packet
        AppDownloadNext,    // 29: Switch to the next download data packet

        Commands // Total number of serial commands
    };

    /**
     * @brief Command structure
     */
    struct Command
    {
        CommandId id;       // Command identifier
        const char *string; // Command string
        uint8_t accessMask; // Access bitmask
    };

    // List of commands
    constexpr Command commandsList[] = {
        {
            .id = CommandId::SlaveAddress,
            .string = "ADDR",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::Date,
            .string = "DATE",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::Time,
            .string = "TIME",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::PsdPoints,
            .string = "PSDP",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::PsdCutoff,
            .string = "PSDC",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::DataTypeControl,
            .string = "DTYP",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::MeasureControl,
            .string = "MCTR",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::MeasureFrequency,
            .string = "MFRQ",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::MeasureInterval,
            .string = "MINT",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::PauseInterval,
            .string = "PINT",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::AccelRange,
            .string = "ACRG",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::GyroRange,
            .string = "GYRG",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::AdcVoltage,
            .string = "VSNS",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::GainSelect,
            .string = "GNSL",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::InputTypeSelect,
            .string = "ITSL",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::SerialSelect,
            .string = "SERS",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::BacklightState,
            .string = "BLST",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::BacklightLevel,
            .string = "BLLV",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::DisplayOnSource,
            .string = "DWSR",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::WakeUpSource,
            .string = "WSRC",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::LogLevel,
            .string = "LOGL",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::BatteryStatus,
            .string = "BATT",
            .accessMask = AccessMask::read,
        },
        {
            .id = CommandId::FwVersion,
            .string = "FVER",
            .accessMask = AccessMask::read,
        },
        {
            .id = CommandId::AppKeepAlive,
            .string = "KPLV",
            .accessMask = AccessMask::execute,
        },
        {
            .id = CommandId::AppDownloadRecent,
            .string = "DWNR",
            .accessMask = AccessMask::write,
        },
        {
            .id = CommandId::AppDownloadHistory,
            .string = "DWNH",
            .accessMask = AccessMask::write,
        },
        {
            .id = CommandId::AppDownloadType,
            .string = "DWNT",
            .accessMask = AccessMask::write,
        },
        {
            .id = CommandId::AppDownloadSize,
            .string = "DWNS",
            .accessMask = AccessMask::read,
        },
        {
            .id = CommandId::AppDownloadData,
            .string = "DWND",
            .accessMask = AccessMask::read,
        },
        {
            .id = CommandId::AppDownloadNext,
            .string = "DWNN",
            .accessMask = AccessMask::execute,
        },
    };
    static_assert(sizeof(commandsList) / sizeof(*commandsList) == static_cast<size_t>(CommandId::Commands),
                  "Commands list doesn't match to commands count!");
} // namespace Serials
