#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

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

uint32_t getIPv4AddressAsBinary(std::string address);
std::string getIPv4AddressAsString(std::uint32_t binary);
std::string getLocalIpAddress();

/**
 * @brief Get the temporary path
 * @return Temporary path
 */
std::filesystem::path getTempPath();

/**
 * @brief Check if a path exists
 * @param path Path to check
 * @param directory If true, check if path is a directory
 * @return True if path exists, false otherwise
 */
bool checkPathExists(const std::filesystem::path& path, bool directory = false);

/**
 * @brief Check if a path has write permissions
 * @param path Path to check
 * @return True if path has write permissions, false otherwise
 */
bool checkWritePermissions(const std::filesystem::path& path);

/**
 * @brief Check if a path has read permissions
 * @param path Path to check
 * @return True if path has read permissions, false otherwise
 */
bool checkReadPermissions(const std::filesystem::path& path);

/**
 * @brief Join two paths, OS-agnostic. This is a wrapper around std::filesystem::path::operator/
 * @param path1 First path
 * @param path2 Second path
 * @return Joined path
 */
std::filesystem::path joinPaths(const std::filesystem::path& path1, const std::filesystem::path& path2);

/**
 * @brief Get the directory from a path, OS-agnostic. For instance, if the path is "/home/user/test.txt", the directory is "/home/user".
 * @param path Path to get the directory from
 * @return directory path
 */
std::filesystem::path getDirFromPath(const std::filesystem::path& path);

// TODO change this to std::thread
void setThreadName(JoiningThread& thread, const std::string& name);

/**
 * @brief Filesystem process-level lock with thread safety. This is both a cross-process and cross-thread synchronization primitive.
 * Uses both file-based locking for process synchronization and mutex-based locking for thread synchronization.
 */
class FSLock {
   public:
    explicit FSLock(const std::filesystem::path& fname);
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
    virtual std::filesystem::path getLockPath(const std::filesystem::path& path) = 0;

    FSLock(const FSLock&) = delete;
    FSLock& operator=(const FSLock&) = delete;

   protected:
#ifdef _WIN32
    HANDLE handle = INVALID_HANDLE_VALUE;
#else
    int fd = -1;
#endif

    std::filesystem::path filename;
    std::filesystem::path lockPath;
    bool isLocked;

    // Thread synchronization
    static std::mutex& getThreadLock(const std::filesystem::path& key) {
        static std::mutex map_mutex;
        std::lock_guard<std::mutex> map_lock(map_mutex);  // prevents race condition when accessing thread_locks

        static std::unordered_map<std::string, std::mutex> thread_locks;
        return thread_locks[key.string()];
    }
    std::mutex& threadLock;
};

/**
 * @brief Filesystem cross-process and cross-thread lock for files.
 */
class FileLock : public FSLock {
   public:
    explicit FileLock(const std::filesystem::path& path, bool createIfNotExists = false);
    std::filesystem::path getLockPath(const std::filesystem::path& path) override;
    using FSLock::lock;

    /**
     * @brief Lock the file given by path
     * @param path Path to the file
     * @param createIfNotExists If true, do not throw an error if the file does not exist, create it instead
     * @return FSLock instance
     */
    static std::unique_ptr<FileLock> lock(const std::filesystem::path& path, bool createIfNotExists = false);
};

/**
 * @brief Filesystem process-level lock for folders. This is both a cross-process synchronization primitive and thread-safe.
 * Uses both file-based locking for process synchronization and mutex-based locking for thread synchronization.
 */
class FolderLock : public FSLock {
   public:
    explicit FolderLock(const std::filesystem::path& path);
    std::filesystem::path getLockPath(const std::filesystem::path& path) override;
    using FSLock::lock;

    /**
     * @brief Lock the folder given by path
     * @param path Path to the folder
     * @return FSLock instance
     */
    static std::unique_ptr<FolderLock> lock(const std::filesystem::path& path);
};
}  // namespace platform
}  // namespace dai
