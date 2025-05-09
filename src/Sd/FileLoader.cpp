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
        char buffer[2048];
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
    char downloadBuffer[2048];

    int getDownloadSize()
    {
        std::vector<FileInfo> fileList;
        int size = 0;
        downloadList.clear();

        if (sensorType < Measurements::SensorType::Count &&
            dataType < Measurements::DataType::Count &&
            startTime > 0 && endFileId >= startFileId)
        {
            SystemTime::epochToTimestamp(startTime, startTimeString);
            char directory[30];
            snprintf(directory, sizeof(directory), "BIN/%s/%.8s",
                     Measurements::getDirectory(sensorType, dataType), startTimeString);

            LOG_DEBUG("List \"%s\"", directory);

            const char *content = SD::FS::list(directory, LS_SIZE);
            if (content && strlen(content) > 0)
            {
                std::istringstream stream(content);
                std::string line;
                int fileId = 0;

                Log::write(content);

                while (std::getline(stream, line))
                {
                    FileInfo file;
                    int recognized = sscanf(line.c_str(), "%d %ld.bin", &file.size, &file.time);
                    if (recognized == 2)
                    {
                        if (file.time <= startTime)
                        {
                            snprintf(file.path, sizeof(file.path), "%s/%ld.bin", directory, file.time);
                            fileList.push_back(file);
                        }
                    }
                }
            }

            int fileId = 0;
            for (auto it = fileList.rbegin(); it != fileList.rend(); it++)
            {
                if (fileId > endFileId)
                {
                    break;
                }

                if (fileId >= startFileId)
                {
                    LOG_DEBUG("File \"%s\", size %d, time %ld", it->path, it->size, it->time);
                    downloadList.push_back(*it);
                    size += it->size;
                }
                fileId++;
            }
        }

        return size;
    }

    void downloadData()
    {
        SD::File sdFile;

        for (auto it = downloadList.rbegin(); it != downloadList.rend(); ++it)
        {
            bool result = sdFile.open(it->path);
            if (result == true)
            {
                if (it->size == sdFile.size() && it->size <= sizeof(binPacket.buffer))
                {
                    int size = it->size;
                    result = sdFile.read(binPacket.buffer, size);
                    if (result == true)
                    {
                        Serials::SerialDevice *device = Serials::Manager::getCommandSourceDevice();
                        if (device != nullptr)
                        {
                            FastCRC16 crc16;
                            binPacket.header.crc16 = crc16.modbus((uint8_t *)binPacket.buffer, size);
                            binPacket.header.length = size;

                            const char *binData = (char *)&binPacket;
                            size_t binSize = sizeof(BinHeader) + size;
                            device->write(binData, binSize);
                        }
                    }
                }

                sdFile.close();
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

        Serials::Manager::subscribeToRead(Serials::CommandId::DownloadSize,
                                          [](const char **responseString)
                                          {
                                              int size = getDownloadSize();
                                              snprintf(dataString, sizeof(dataString), "%d", size);

                                              *responseString = dataString;
                                          });

        Serials::Manager::subscribeToRead(Serials::CommandId::DownloadData,
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

        Serials::Manager::subscribeToWrite(Serials::CommandId::DownloadRecent,
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

        Serials::Manager::subscribeToWrite(Serials::CommandId::DownloadHistory,
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

        Serials::Manager::subscribeToWrite(Serials::CommandId::DownloadType,
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
