#pragma once

#include <cstdint>
#include <string>

#include "depthai/utility/JoiningThread.hpp"

// Lock specific headers
#ifdef _WIN32
    #include <winsock2.h>
#else
    #include <fcntl.h>
    #include <unistd.h>
#endif

namespace dai {
namespace platform {

/**
 * @brief Check if a path exists
 * @param path Path to check
 * @param directory If true, check if path is a directory
 * @return True if path exists, false otherwise
 */
bool checkPathExists(const std::string& path, bool directory = false);

/**
 * @brief Get the temporary path
 * @return Temporary path
 */
std::string getTempPath();

/**
 * @brief Join two paths, OS-agnostic. For instance, if the paths are "/home/user" and "test.txt", the joined path is "/home/user/test.txt".
 * @param path1 First path
 * @param path2 Second path
 * @return Joined path
 */
std::string joinPaths(const std::string& path1, const std::string& path2);

/**
 * @brief Check if a path has write permissions
 * @param path Path to check
 * @return True if path has write permissions, false otherwise
 */
bool checkWritePermissions(const std::string& path);

/**
 * @brief Get the directory from a path, OS-agnostic. For instance, if the path is "/home/user/test.txt", the directory is "/home/user".
 * @param path Path to get the directory from
 * @return directory path
 */
std::string getDirFromPath(const std::string& path);

uint32_t getIPv4AddressAsBinary(std::string address);
std::string getIPv4AddressAsString(std::uint32_t binary);

// TODO change this to std::thread
void setThreadName(JoiningThread& thread, const std::string& name);

class FSLock {
   public:
    explicit FSLock(const std::string& fname);
    virtual ~FSLock();

    /**
     * @brief Lock the file produced by getLockPath()
     */
    void lock();

    /**
     * @brief Unlock the file produced by getLockPath()
     */
    void unlock();

    /**
     * @brief Check if the lock is held
     * @return True if the lock is held, false otherwise
     */
    bool holding() const;

    /**
     * @brief Get the path to the lock file
     * @param path Path to the file
     * @return Path to the lock file
     */
    virtual std::string getLockPath(const std::string& path) = 0;

    FSLock(const FSLock&) = delete;
    FSLock& operator=(const FSLock&) = delete;

   protected:
    #ifdef _WIN32
    HANDLE handle = INVALID_HANDLE_VALUE;
    #else
    int fd = -1;
    #endif

    std::string filename;
    std::string lockPath;
    bool isLocked;
};

class FileLock : public FSLock {
   public:
    explicit FileLock(const std::string& path, bool createIfNotExists = false);
    std::string getLockPath(const std::string& path) override;
    using FSLock::lock;

    /**
     * @brief Lock the file given by path
     * @param path Path to the file
     * @param createIfNotExists If true, do not throw an error if the file does not exist, create it instead
     * @return FSLock instance
     */
    static std::unique_ptr<FileLock> lock(const std::string& path, bool createIfNotExists = false);
};

class FolderLock : public FSLock {
   public:
    explicit FolderLock(const std::string& path);
    std::string getLockPath(const std::string& path) override;
    using FSLock::lock;

    /**
     * @brief Lock the folder given by path
     * @param path Path to the folder
     * @return FSLock instance
     */
    static std::unique_ptr<FolderLock> lock(const std::string& path);
};
}  // namespace platform
}  // namespace dai
