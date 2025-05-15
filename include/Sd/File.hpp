/**
 * @file File.hpp
 * @author Mikhail Kalina (apollo.mk58@gmail.com)
 * @brief File system API
 * @version 0.1
 * @date 2024-06-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <stddef.h>

#include <SdFat.h>

namespace SD
{
    namespace FS
    {
        /**
         * @brief Start SD file system class
         *
         * @param[in] frequency Maximum SCK frequency
         * @return true if SD file system was started successfully, false otherwise
         */
        bool start(uint32_t frequency);

        /**
         * @brief Start SD file system class
         */
        void stop();

        /**
         * @brief Check if SD card is atteched
         *
         * @return true if SD card is attached, false otherwise
         */
        bool isAttached();

        /**
         * @brief Get SD file system
         *
         * @return SD file system
         */
        SdFs &sdFs();

        /**
         * @brief List SD file system directory content
         *
         * @param directory Directory name
         * @param flags Flags for list command (LS_DATE, LS_SIZE, LS_R)
         * @return String with directory content
         */
        const char *list(const char *directory, uint8_t flags);
    } // namespace FS

    /**
     * @brief SD card file object
     * Located in the specified directory on external storage
     */
    class File
    {
        constexpr static size_t pathMaxLength = 100; ///< Maximum length of path to file

    public:
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
         * @param path File path
         * @param flags Flags to open
         * @return True if the file is successfully opened, false otherwise
         */
        bool open(const char *path, oflag_t flags = O_RDONLY);

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
         * @brief Read file to the buffer
         *
         * @param buffer Pointer to the buffer data
         * @param size Number of bytes to read
         * @return True if file has been read to the buffer, false otherwise
         */
        bool read(char *buffer, size_t size);

        /**
         * @brief Set a file's operation timestamp in its directory entry
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
} // namespace SD
