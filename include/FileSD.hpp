/**
 * @file FileSD.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief SD File system API
 * @version 0.1
 * @date 2024-06-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <stddef.h>

#include <SdFat.h>

/**
 * @brief SD card file object
 * Located in the specified directory on external storage
 */
class FileSD
{
    constexpr static size_t pathMaxLength = 100; ///< Maximum length of path to file

public:
    /**
     * @brief Start SD file system class
     *
     * @param[in] frequency Maximum SCK frequency
     * @return true if SD file system was started successfully, false otherwise
     */
    static bool startFileSystem(uint32_t frequency);

    /**
     * @brief Start SD file system class
     */
    static void stopFileSystem();

    /**
     * @brief Check if SD card is atteched
     *
     * @return true if SD card is attached, false otherwise
     */
    static bool isCardAttached();

    /**
     * @brief Get SD file system
     *
     * @return SD file system
     */
    static SdFs &sdFs();

    /**
     * @brief List SD file system directory content
     *
     * @param directory Directory name
     * @param flags Flags for list command (LS_DATE, LS_SIZE, LS_R)
     * @return String with directory content
     */
    static const char *ls(const char *directory, uint8_t flags);

    /**
     * @brief Create new SD file
     *
     * @param directory Directory name
     * @param fileName File name
     * @param extension File extension
     * @return True if file has been created successfully, false otherwise
     */
    bool create(const char *directory, const char *fileName, const char *extension);

    /**
     * @brief Open existing file
     *
     * @return True if the file is successfully opened, false otherwise
     */
    bool open();

    /**
     * @brief Close the file
     *
     * @return True if the file is successfully closed, false otherwise
     */
    bool close();

    /**
     * @brief Print data string to the file
     *
     * @param string Data string to print
     * @return True if data has been added to the file, false otherwise
     */
    bool print(const char *string);

    /**
     * @brief Print data string to the file with the end of line characters
     *
     * @param string Data string to print
     * @return True if data has been added to the file, false otherwise
     */
    bool println(const char *string);

    /**
     * @brief Write buffer to the file
     *
     * @param buffer Pointer to the buffer data
     * @param size Number of bytes to write
     * @return True if buffer has been written to the file, false otherwise
     */
    bool write(const void *buffer, size_t size);

    /**
     * @brief Set a file's opration timestamp in its directory entry
     *
     * @param flags Type of operation to be timestamped (T_ACCESS, T_CREATE, T_WRITE)
     * @return true if timestamp was added, false otherwise
     */
    bool timestamp(uint8_t flags);

    /**
     * @brief Get file size
     *
     * @return File size, bytes
     */
    size_t size();

private:
    char _path[pathMaxLength + 1] = {0}; // Path to file in external storage
    FsFile _file;                        // Opened file handler
};