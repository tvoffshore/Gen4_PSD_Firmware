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
        SerialSelect,     // 3: Set/Get serial interface selection
        MeasureFrequency, // 4: Set/Get the frequency of measurements
        MeasureInterval,  // 5: Set/Get the interval of measurements
        PauseInterval,    // 6: Set/Get the interval of pause between measurements
        PointsPsd,        // 7: Set/Get the points to calculate PSD segment size, 2^x
        PointsCutoff,     // 8: Set/Get the points to store the PSD results
        StatisticState,   // 9: Set/Get the state of statistic (1 enable, 0 disable)
        LogLevel,         // 10: Set/Get serial debug log level
        FwVersion,        // 11: Get FW version information
        BatteryStatus,    // 12: Get battery status
        DownloadRecent,   // 13: Set download range starting from now
        DownloadHistory,  // 14: Set download range starting from timestamp
        DownloadType,     // 15: Set download data type
        DownloadSize,     // 16: Get download data size in bytes
        DownloadChunk,    // 17: Get next data chunk

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
            .id = CommandId::SerialSelect,
            .string = "SERS",
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
            .id = CommandId::PointsPsd,
            .string = "PPSD",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::PointsCutoff,
            .string = "PCUT",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::StatisticState,
            .string = "STAT",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::LogLevel,
            .string = "LOGL",
            .accessMask = AccessMask::read | AccessMask::write,
        },
        {
            .id = CommandId::FwVersion,
            .string = "FVER",
            .accessMask = AccessMask::read,
        },
        {
            .id = CommandId::BatteryStatus,
            .string = "BATT",
            .accessMask = AccessMask::read,
        },
        {
            .id = CommandId::DownloadRecent,
            .string = "DWNR",
            .accessMask = AccessMask::write,
        },
        {
            .id = CommandId::DownloadHistory,
            .string = "DWNH",
            .accessMask = AccessMask::write,
        },
        {
            .id = CommandId::DownloadType,
            .string = "DWNT",
            .accessMask = AccessMask::write,
        },
        {
            .id = CommandId::DownloadSize,
            .string = "DWNS",
            .accessMask = AccessMask::read,
        },
        {
            .id = CommandId::DownloadChunk,
            .string = "DWNC",
            .accessMask = AccessMask::read,
        },
    };
    static_assert(sizeof(commandsList) / sizeof(*commandsList) == static_cast<size_t>(CommandId::Commands),
                  "Commands list doesn't match to commands count!");
} // namespace Serials
