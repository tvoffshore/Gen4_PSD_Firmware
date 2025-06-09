/**
 * @file FwVersion.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief Firmware version API
 * @version 0.1
 * @date 2024-07-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "FwVersion.hpp"

#include <cstdio>
#include <stddef.h>

namespace
{
    /* Major firmware version value */
    constexpr unsigned int FW_VERSION_MAJOR = 0;
    /* Minor firmware version value */
    constexpr unsigned int FW_VERSION_MINOR = 3;

    // MAJOR(2) + .(1) + MINOR(2) < 6
    constexpr size_t fwStringLength = 6;
} // namespace

const char *FwVersion::getVersionString()
{
    static char fwString[fwStringLength];

    snprintf(fwString, sizeof(fwString), "%u.%u", FW_VERSION_MAJOR, FW_VERSION_MINOR);

    return fwString;
}
