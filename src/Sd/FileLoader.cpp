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

#include "Sd/FileLoader.hpp"

#include <iostream>
#include <sstream>
#include <stdint.h>
#include <string.h>
#include <vector>

#include <FastCRC.h>
#include <Log.hpp>
#include <SystemTime.hpp>

#include "Measurements/Types.hpp"
#include "Sd/File.hpp"
#include "Serial/SerialManager.hpp"

using namespace FileLoader;

namespace
{
    struct FileInfo
    {
        char path[48];
        int size;
        time_t time;
    };

#pragma pack(push, 1)
    struct BinHeader
    {
        const uint32_t magic;
        uint16_t crc16;
        uint16_t length;
    };

    struct BinPacket
    {
        BinHeader header;
        char buffer[5120];
    };
#pragma pack(pop)

    constexpr char binMagic = 0xFE;

    Measurements::SensorType sensorType = Measurements::SensorType::Count;
    Measurements::DataType dataType = Measurements::DataType::Count;
    SystemTime::DateTimeString startTimeString;
    time_t startTime = 0;
    int startFileId = 0;
    int endFileId = 0;
    std::vector<FileInfo> downloadList;
    BinPacket binPacket = {.header = {.magic = 0xFEDCBA98}};

    int getDownloadSize()
    {
        int downloadSize = 0;
        downloadList.clear();

        if (sensorType < Measurements::SensorType::Count &&
            dataType < Measurements::DataType::Count &&
            startTime > 0 && endFileId >= startFileId)
        {
            char directory[30];
            const char *sensorDataDir = Measurements::getDirectory(sensorType, dataType);

            SystemTime::epochToTimestamp(startTime, startTimeString);
            snprintf(directory, sizeof(directory), "BIN/%s/%.8s", sensorDataDir, startTimeString);

            LOG_DEBUG("List \"%s\"", directory);

            std::vector<FileInfo> fileList;
            const char *content = SD::FS::list(directory, LS_SIZE);
            if (content && strlen(content) > 0)
            {
                std::istringstream stream(content);
                std::string line;
                int fileId = 0;

                while (std::getline(stream, line))
                {
                    FileInfo file;
                    int recognized = sscanf(line.c_str(), "%d %ld.bin", &file.size, &file.time);
                    if (recognized == 2)
                    {
                        if (file.time < startTime)
                        {
                            snprintf(file.path, sizeof(file.path), "%s/%ld.bin", directory, file.time);
                            fileList.push_back(file);
                        }
                    }
                }
            }
            else
            {
                LOG_WARNING("Directory \"%s\" is empty", directory);
            }

            int fileId = 0;
            for (auto it = fileList.rbegin(); it != fileList.rend(); it++)
            {
                if (fileId >= startFileId)
                {
                    LOG_DEBUG("File \"%s\", size %d, time %ld", it->path, it->size, it->time);
                    downloadList.push_back(*it);
                    downloadSize += it->size;
                }

                fileId++;
                if (fileId > endFileId)
                {
                    break;
                }
            }
        }
        else
        {
            LOG_ERROR("Wrong load params: sensor %d, data %d, start time %ld, file id %d-%d",
                      sensorType, dataType, startTime, startFileId, endFileId);
        }

        LOG_DEBUG("Files to download %d, total size %d", downloadList.size(), downloadSize);

        return downloadSize;
    }

    void downloadData()
    {
        SD::File sdFile;

        for (auto it = downloadList.rbegin(); it != downloadList.rend(); ++it)
        {
            bool result = sdFile.open(it->path);
            if (result == true)
            {
                int fileSize = it->size;
                if (fileSize == sdFile.size() && fileSize <= sizeof(binPacket.buffer))
                {
                    result = sdFile.read(binPacket.buffer, fileSize);
                    if (result == true)
                    {
                        Serials::SerialDevice *serialDevice = Serials::Manager::getCommandSourceDevice();
                        if (serialDevice != nullptr)
                        {
                            FastCRC16 crc16;
                            binPacket.header.crc16 = crc16.modbus((uint8_t *)binPacket.buffer, fileSize);
                            binPacket.header.length = fileSize;

                            const char *binData = (char *)&binPacket;
                            size_t binSize = sizeof(BinHeader) + fileSize;
                            result = serialDevice->write(binData, binSize);
                            if (result != true)
                            {
                                LOG_WARNING("Write bin data to serial failed, size %d", binSize);
                            }
                        }
                        else
                        {
                            LOG_WARNING("Source serial device failed");
                        }
                    }
                    else
                    {
                        LOG_WARNING("File \"%s\" failed to read, size %d", it->path, fileSize);
                    }
                }
                else
                {
                    LOG_WARNING("File size %d error, buffer size %d", fileSize, sizeof(binPacket.buffer));
                }

                sdFile.close();
            }
            else
            {
                LOG_WARNING("File \"%s\" failed to open", it->path);
            }
        }
    }

    /**
     * @brief Register serial read command handlers
     */
    void registerSerialReadHandlers()
    {
        static char dataString[Serials::SerialDevice::dataMaxLength];

        LOG_TRACE("Register serial read common handlers");

        Serials::Manager::subscribeToRead(Serials::CommandId::AppDownloadSize,
                                          [](const char **responseString)
                                          {
                                              int size = getDownloadSize();
                                              snprintf(dataString, sizeof(dataString), "%d", size);

                                              *responseString = dataString;
                                          });

        Serials::Manager::subscribeToRead(Serials::CommandId::AppDownloadData,
                                          [](const char **responseString)
                                          {
                                              downloadData();
                                              snprintf(dataString, sizeof(dataString), "Sent");

                                              *responseString = dataString;
                                          });
    }

    /**
     * @brief Register serial write command handlers
     */
    void registerSerialWriteHandlers()
    {
        LOG_TRACE("Register serial write common handlers");

        Serials::Manager::subscribeToWrite(Serials::CommandId::AppDownloadRecent,
                                           [](const char *dataString)
                                           {
                                               // "START,END"
                                               int recognized = sscanf(dataString, "%d,%d", &startFileId, &endFileId);
                                               if (recognized == 2)
                                               {
                                                   startTime = SystemTime::getTimestamp(startTimeString);
                                                   LOG_DEBUG("Start time %ld, start file ID %d, end file ID %d", startTime, startFileId, endFileId);
                                               }
                                               else
                                               {
                                                   LOG_WARNING("Parse Download recent command filed");
                                               }
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::AppDownloadHistory,
                                           [](const char *dataString)
                                           {
                                               // "TIMESTAMP,START,END"
                                               int recognized = sscanf(dataString, "%ld,%d,%d", &startTime, &startFileId, &endFileId);
                                               if (recognized == 3)
                                               {
                                                   LOG_DEBUG("Start time %ld, start file ID %d, end file ID %d", startTime, startFileId, endFileId);
                                               }
                                               else
                                               {
                                                   LOG_WARNING("Parse Download history command filed");
                                               }
                                           });

        Serials::Manager::subscribeToWrite(Serials::CommandId::AppDownloadType,
                                           [](const char *dataString)
                                           {
                                               // "SENSOR,DATA"
                                               int recognized = sscanf(dataString, "%d,%d", &sensorType, &dataType);
                                               if (recognized == 2)
                                               {
                                                   LOG_DEBUG("Sensor type %d, data type %d", sensorType, dataType);
                                               }
                                               else
                                               {
                                                   LOG_WARNING("Parse Download type command filed");
                                               }
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
