#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "depthai/utility/JoiningThread.hpp"

#ifdef _WIN32
    #include <winsock2.h>
#else
    #include <fcntl.h>
    #include <unistd.h>
#endif

namespace dai {
namespace platform {

uint32_t getIPv4AddressAsBinary(const std::string& address);
std::string getIPv4AddressAsString(std::uint32_t binary);

bool isIPv4Address(const std::string& address);  // Strict, as inet_pton: no leading zeros, whitespace or hostnames

std::string getLocalIpAddress();

std::string getOSPlatform();
std::string getOSVersion();
std::filesystem::path getDaiCacheDir();

std::filesystem::path getTempPath();

bool checkPathExists(const std::filesystem::path& path, bool directory = false);

bool checkWritePermissions(const std::filesystem::path& path);

bool checkReadPermissions(const std::filesystem::path& path);

std::filesystem::path joinPaths(const std::filesystem::path& path1, const std::filesystem::path& path2);

std::filesystem::path getDirFromPath(const std::filesystem::path& path);

std::vector<std::string> getFilenamesInDirectory(const std::filesystem::path& path);

std::string pathToUtf8String(const std::filesystem::path& path);  // path::string() is not UTF-8 on Windows (active code page)

std::string lowercase(std::string value);

// TODO change this to std::thread
void setThreadName(JoiningThread& thread, const std::string& name);

class FSLock {
   public:
    explicit FSLock(const std::filesystem::path& fname);
    virtual ~FSLock();

    void lock();

    void unlock();

    bool holding() const;

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

    static std::mutex& getThreadLock(const std::filesystem::path& key) {
        static std::mutex map_mutex;
        std::lock_guard<std::mutex> map_lock(map_mutex);  // prevents race condition when accessing thread_locks

        static std::unordered_map<std::string, std::mutex> thread_locks;
        return thread_locks[key.string()];
    }
    std::mutex& threadLock;
};

class FileLock : public FSLock {
   public:
    explicit FileLock(const std::filesystem::path& path, bool createIfNotExists = false);
    std::filesystem::path getLockPath(const std::filesystem::path& path) override;
    using FSLock::lock;

    static std::unique_ptr<FileLock> lock(const std::filesystem::path& path, bool createIfNotExists = false);  // Throws if missing, unless createIfNotExists
};

class FolderLock : public FSLock {
   public:
    explicit FolderLock(const std::filesystem::path& path);
    std::filesystem::path getLockPath(const std::filesystem::path& path) override;
    using FSLock::lock;

    static std::unique_ptr<FolderLock> lock(const std::filesystem::path& path);
};
}  // namespace platform
}  // namespace dai
