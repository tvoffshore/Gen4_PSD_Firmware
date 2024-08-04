/**
 * @file FileSD.cpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief SD File system implementation
 * @version 0.1
 * @date 2024-06-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "FileSD.hpp"

#include <assert.h>
#include <string.h>

#include <Debug.hpp>

namespace
{
    // SD chip select pin
    constexpr auto pinCS = GPIO_NUM_26;

    // Delimiter of directory and filename
    const char *directoryDelimiter = "/";
    // Logs files extension
    const char *fileExtension = "csv";

    // Bytes to megabytes ratio
    constexpr size_t sectorsToMbFactor = 2 * 1024;
    // SD card types
    const char *cardTypeNames[] = {"None", "MMC", "SD", "SDHC/SDXC", "Unknown"};

    // SD file system class
    SdFs sd;
} // namespace

/**
 * @brief Start SD file system class
 *
 * @param[in] frequency Maximum SCK frequency
 * @return true if SD file system was started successfully, false otherwise
 */
bool FileSD::startFileSystem(uint32_t frequency)
{
    LOG_INFO("Start SD file system...");

    bool result = sd.begin(pinCS, frequency);
    if (result == true)
    {
        result = isCardAttached();
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
 * @brief Start SD file system class
 */
void FileSD::stopFileSystem()
{
    LOG_INFO("Stop SD file system");

    sd.end();
}

/**
 * @brief Check if SD card is atteched
 *
 * @return true if SD card is attached, false otherwise
 */
bool FileSD::isCardAttached()
{
    uint8_t cardType = sd.card()->type();

    return (cardType != 0);
}

/**
 * @brief Get SD file system
 *
 * @return SD file system
 */
SdFs &FileSD::sdFs()
{
    return sd;
}

/**
 * @brief Create new SD file
 *
 * @param directory Directory name
 * @param fileName File name
 * @return True if file has been created successfully, false otherwise
 */
bool FileSD::create(const char *directory, const char *fileName)
{
    assert(directory);
    assert(fileName);

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

    if (isDirectory)
    {
        // Create path to new file
        snprintf(_path, sizeof(_path), "%s%s%s.%s", directory, directoryDelimiter, fileName, fileExtension);

        // Try to open the file with append (if exists) or O_CREAT (if doesn't exist) access
        oflag_t oFlags = O_WRONLY | (sd.exists(_path) ? O_APPEND : O_CREAT);
        _file = sd.open(_path, oFlags);

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
 * @return True if the file successfully opened, false otherwise
 */
bool FileSD::open()
{
    if (_file == false)
    {
        if (sd.exists(_path))
        {
            // Try to open file with append access (add new data at the end of the file)
            _file = sd.open(_path, O_WRONLY | O_APPEND);

            LOG_INFO("File \"%s\" %s opened", _path, _file ? "is" : "isn't");
        }
        else
        {
            LOG_ERROR("File \"%s\" isn't exist to open", _path);
        }
    }

    return _file;
}

/**
 * @brief Close the file
 */
bool FileSD::close()
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
bool FileSD::print(const char *string)
{
    assert(string);

    bool result = false;

    if (_file)
    {
        // Print sample data to file
        size_t printLen = _file.print(string);
        // Print length should be equal to string length
        result = (printLen == strlen(string));
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
bool FileSD::println(const char *string)
{
    assert(string);

    bool result = false;

    if (_file)
    {
        // Print sample data to file
        size_t printLen = _file.println(string);
        // Print length should be equal to string length + "\r\n"
        result = (printLen == (strlen(string) + 2));
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
bool FileSD::write(const void *buffer, size_t size)
{
    assert(buffer);
    assert(size > 0);

    bool result = false;

    if (_file)
    {
        // Print sample data to file
        size_t writeSize = _file.write(buffer, size);

        result = (writeSize == size);
    }

    LOG_TRACE("Buffer size %d %s written to file \"%s\"", size, result ? "is" : "isn't", _path);

    return result;
}

/**
 * @brief Get file size
 *
 * @return File size, bytes
 */
size_t FileSD::size()
{
    size_t fileSize = 0;

    if (_file)
    {
        fileSize = _file.size();
    }

    return fileSize;
}
