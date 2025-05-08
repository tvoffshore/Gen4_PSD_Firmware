/**
 * @file FileLoader.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-05-08
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "FileLoader.hpp"

#include <Log.hpp>
#include <SystemTime.hpp>

#include "FileSD.hpp"
#include "Serial/SerialManager.hpp"

using namespace FileLoader;

namespace
{
    /**
     * @brief Register serial read command handlers
     */
    void registerSerialReadHandlers()
    {
        static char dataString[Serials::SerialDevice::dataMaxLength];

        LOG_TRACE("Register serial read common handlers");

        Serials::Manager::subscribeToRead(Serials::CommandId::DownloadSize,
                                          [](const char **responseString)
                                          {
                                              assert(sizeof(dataString) >= sizeof(SystemTime::DateString));

                                              SystemTime::getStringDate(dataString);

                                              *responseString = dataString;
                                          });
    }

    /**
     * @brief Register serial write command handlers
     */
    void registerSerialWriteHandlers()
    {
        LOG_TRACE("Register serial write common handlers");

        Serials::Manager::subscribeToWrite(Serials::CommandId::DownloadRecent,
                                           [](const char *dataString)
                                           {
                                               SystemTime::setStringDate(dataString);
                                           });
    }
} // namespace

/**
 * @brief Initialize file loader module
 */
void FileLoader::initialize()
{
    // Register local serial handlers
    registerSerialReadHandlers();
    registerSerialWriteHandlers();
}
