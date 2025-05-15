/**
 * @file File.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief File system implementation
 * @version 0.1
 * @date 2024-06-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "Sd/File.hpp"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include <Log.hpp>
#include <SystemTime.hpp>

/**
 * @brief Print-compatible string buffer
 *
 */
class StringBuffer : public Print
{
public:
    size_t write(uint8_t c) override
    {
        string += (char)c;
        return 1;
    }

    const char *getString() const
    {
        return string.c_str();
    }

    void clear()
    {
        string.clear();
    }

private:
    String string;
};

namespace
{
    // Delimiter of directory and filename
    const char *directoryDelimiter = "/";

    // Bytes to megabytes ratio
    constexpr size_t sectorsToMbFactor = 2 * 1024;
    // SD card types
    const char *cardTypeNames[] = {"None", "MMC", "SD", "SDHC/SDXC", "Unknown"};

    // SD file system class
    SdFs sd;

    // String buffer to list command
    StringBuffer listBuffer;
} // namespace

/**
 * @brief Start SD file system
 *
 * @param[in] csPin SD card chip select pin
 * @param[in] frequency Maximum SCK frequency
 * @return true if SD file system was started successfully, false otherwise
 */
bool SD::FS::start(uint8_t csPin, uint32_t frequency)
{
    LOG_DEBUG("Start SD file system...");

    bool result = sd.begin(csPin, frequency);
    if (result == true)
    {
        result = isAttached();
        if (result == true)
        {
            uint8_t cardType = sd.card()->type();
            uint32_t cardSizeMb = sd.card()->sectorCount() / sectorsToMbFactor;
            LOG_INFO("SD card initialized: type %s, size = %dMB", cardTypeNames[cardType], cardSizeMb);
        }
        else
        {
            LOG_ERROR("No SD card attached");
        }
    }
    else
    {
        LOG_ERROR("SD card initialization fail!");
    }

    return result;
}

/**
 * @brief Stop SD file system
 */
void SD::FS::stop()
{
    LOG_INFO("Stop SD file system");

    sd.end();
}

/**
 * @brief Check if SD card is attached
 *
 * @return true if SD card is attached, false otherwise
 */
bool SD::FS::isAttached()
{
    uint8_t cardType = sd.card()->type();

    return (cardType != 0);
}

/**
 * @brief Get SD file system object
 *
 * @return Reference to SD file system object
 */
SdFs &SD::FS::sdFs()
{
    return sd;
}

/**
 * @brief List SD file system directory content
 *
 * @param directory Directory name
 * @param flags Flags for list command (LS_DATE, LS_SIZE, LS_R)
 * @return String with directory content
 */
const char *SD::FS::list(const char *directory, uint8_t flags)
{
    assert(directory);

    listBuffer.clear();
    sd.ls(&listBuffer, directory, flags);

    const char *string = listBuffer.getString();
    return string;
}

/**
 * @brief Create new SD file
 *
 * @param directory Directory name
 * @param fileName File name
 * @param extension File extension
 * @return True if file has been created successfully, false otherwise
 */
bool SD::File::create(const char *directory, const char *fileName, const char *extension)
{
    assert(directory);
    assert(fileName);
    assert(extension);

    if (SD::FS::isAttached() == false)
    {
        LOG_ERROR("No SD, file create failed");
        return false;
    }

    if (_file)
    {
        LOG_DEBUG("Previous file \"%s\" is still opened - close", _path);

        bool result = close();
        if (result == false)
        {
            LOG_ERROR("Previous file \"%s\" can't be closed!", _path);
            return false;
        }
    }

    // Check directory and create it if needs
    bool isDirectory = sd.exists(directory);
    if (isDirectory == false)
    {
        LOG_INFO("Directory \"%s\" isn't exist - create", directory);

        isDirectory = sd.mkdir(directory);
    }

    if (isDirectory == true)
    {
        // Create path to new file
        snprintf(_path, sizeof(_path), "%s%s%s.%s", directory, directoryDelimiter, fileName, extension);

        // Try to open the file with append (if exists) or O_CREAT (if doesn't exist) access
        oflag_t oFlags = O_WRONLY | (sd.exists(_path) ? O_APPEND : O_CREAT);
        _file = sd.open(_path, oFlags);
        if (_file)
        {
            // File was successfully created/opened, add timestamp
            timestamp((oFlags & O_CREAT) ? T_CREATE | T_ACCESS : T_ACCESS);
        }

        LOG_INFO("File \"%s\" %s %s on SD", _path, _file ? "is" : "isn't", (oFlags & O_CREAT) ? "created" : "opened");
    }
    else
    {
        LOG_ERROR("Directory \"%s\" isn't created or file system isn't mounted", directory);
    }

    return _file;
}

/**
 * @brief Open existing file
 *
 * @param path File path
 * @param flags Flags to open
 * @return True if the file successfully opened, false otherwise
 */
bool SD::File::open(const char *path, oflag_t flags)
{
    if (SD::FS::isAttached() == false)
    {
        LOG_ERROR("No SD, file open failed");
        return false;
    }

    if (_file)
    {
        LOG_DEBUG("Previous file \"%s\" is still opened - close", _path);

        bool result = close();
        if (result == false)
        {
            LOG_ERROR("Previous file \"%s\" can't be closed!", _path);
            return false;
        }
    }

    if (path != nullptr)
    {
        strncpy(_path, path, sizeof(_path));
    }

    if (sd.exists(_path))
    {
        // Try to open file
        _file = sd.open(_path, flags);
        if (_file)
        {
            // File was successfully opened, add timestamp
            timestamp(T_ACCESS);
        }

        LOG_INFO("File \"%s\" %s opened", _path, _file ? "is" : "isn't");
    }
    else
    {
        LOG_ERROR("File \"%s\" isn't exist to open", _path);
    }

    return _file;
}

/**
 * @brief Close the file
 */
bool SD::File::close()
{
    if (_file)
    {
        _file.close();
    }

    LOG_INFO("File \"%s\" %s closed", _path, !_file ? "is" : "isn't");

    return !_file;
}

/**
 * @brief Print data string to the file
 *
 * @param string Data string to print
 * @return True if data has been added to the file, false otherwise
 */
bool SD::File::print(const char *string)
{
    assert(string);

    bool result = false;

    if (_file)
    {
        // Print sample data to file
        size_t printedLength = _file.print(string);
        // Printed length should be equal to the string length
        result = (printedLength == strlen(string));

        if (printedLength > 0)
        {
            // Data was written to the file, add timestamp
            timestamp(T_WRITE);
        }
    }

    LOG_TRACE("String \"%s\" %s printed to file \"%s\"", string, result ? "is" : "isn't", _path);

    return result;
}

/**
 * @brief Print data string to the file with the end of line characters
 *
 * @param string Data string to print
 * @return True if data has been added to file, false otherwise
 */
bool SD::File::println(const char *string)
{
    assert(string);

    bool result = false;

    if (_file)
    {
        // Print string to file
        size_t printedLength = _file.println(string);
        // Printed length should be equal to the string length + "\r\n"
        result = (printedLength == (strlen(string) + 2));

        if (printedLength > 0)
        {
            // Data was written to the file, add timestamp
            timestamp(T_WRITE);
        }
    }

    LOG_TRACE("String \"%s\" %s printed to file \"%s\"", string, result ? "is" : "isn't", _path);

    return result;
}

/**
 * @brief Write buffer to the file
 *
 * @param buffer Pointer to the buffer data
 * @param size Number of bytes to write
 * @return True if buffer has been written to the file, false otherwise
 */
bool SD::File::write(const void *buffer, size_t size)
{
    assert(buffer);
    assert(size > 0);

    bool result = false;

    if (_file)
    {
        // Write buffer to the file
        size_t writtenBytes = _file.write(buffer, size);
        // Written bytes should be equal to the buffer size
        result = (writtenBytes == size);

        if (writtenBytes > 0)
        {
            // Data was written to the file, add timestamp
            timestamp(T_WRITE);
        }
    }

    LOG_TRACE("Buffer size %d %s written to file \"%s\"", size, result ? "is" : "isn't", _path);

    return result;
}

/**
 * @brief Read file to the buffer
 *
 * @param buffer Pointer to the buffer data
 * @param size Number of bytes to read
 * @return True if file has been read to the buffer, false otherwise
 */
bool SD::File::read(char *buffer, size_t size)
{
    assert(buffer);
    assert(size > 0);

    bool result = false;

    if (_file)
    {
        // Read file to the buffer
        size_t readBytes = _file.readBytes(buffer, size);
        // Read bytes should be equal to the buffer size
        result = (readBytes == size);
    }

    LOG_TRACE("File \"%s\" %s read to buffer size %d", _path, result ? "is" : "isn't", size);

    return result;
}

/**
 * @brief Set a file's operation timestamp in its directory entry
 *
 * @param flags Type of operation to be timestamped (T_ACCESS, T_CREATE, T_WRITE)
 * @return true if timestamp was added, false otherwise
 */
bool SD::File::timestamp(uint8_t flags)
{
    bool result = false;

    if (_file)
    {
        SystemTime::DateTime dateTime;
        SystemTime::getDateTime(dateTime);

        result = _file.timestamp(flags, dateTime.Year, dateTime.Month, dateTime.Day,
                                 dateTime.Hour, dateTime.Minute, dateTime.Second);
    }

    return result;
}

/**
 * @brief Get file size
 *
 * @return File size, bytes
 */
size_t SD::File::size()
{
    size_t fileSize = 0;

    if (_file)
    {
        fileSize = _file.size();
    }

    return fileSize;
}
