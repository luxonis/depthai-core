#include "Platform.hpp"

#include <filesystem>
#include <memory>

// Platform specific
#if defined(_WIN32) || defined(__USE_W32_SOCKETS)
    #include <ws2tcpip.h>
    #ifdef _MSC_VER
        #pragma comment(lib, "Ws2_32.lib")
    #endif
#else
    #if defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__) || defined(__bsdi__) || defined(__DragonFly__)
        #include <netinet/in.h>
    #endif
    #include <arpa/inet.h>
#endif

#ifdef __linux__
    #include <pthread.h>
#endif

#if defined(_WIN32) || defined(__USE_W32_SOCKETS)
    #include <windows.h>
#endif

#ifndef _WIN32
    #include <sys/stat.h>
    #include <unistd.h>
#endif

namespace dai {
namespace platform {

uint32_t getIPv4AddressAsBinary(std::string address) {
    uint32_t binary = 0;
    if(address == "") {
        // inet_addr returns 0xFFFFFFFF if addr is ""
        return 0;
    }

#if defined(_WIN32) || defined(__USE_W32_SOCKETS)
    #if(_WIN32_WINNT <= 0x0501)
    binary = inet_addr(address.c_str());  // for XP
    #else
    inet_pton(AF_INET, address.c_str(), &binary);  // for Vista or higher
    #endif
#else
    inet_pton(AF_INET, address.c_str(), &binary);
#endif

    return binary;
}

std::string getIPv4AddressAsString(std::uint32_t binary) {
    char address[INET_ADDRSTRLEN] = {0};

#if defined(_WIN32) || defined(__USE_W32_SOCKETS)
    InetNtopA(AF_INET, &binary, address, sizeof(address));
#else
    inet_ntop(AF_INET, &binary, address, sizeof(address));
#endif

    return {address};
}

void setThreadName(JoiningThread& thread, const std::string& name) {
#ifdef __linux__
    auto handle = thread.native_handle();
    pthread_setname_np(handle, name.c_str());
#endif
    return;
}

std::string getTempPath() {
    return std::filesystem::temp_directory_path().string();
}

bool checkPathExists(const std::string& path, bool directory) {
    std::error_code ec;
    if(!std::filesystem::exists(path, ec)) {
        return false;  // Path does not exist
    }
    if(directory) {
        return std::filesystem::is_directory(path, ec);  // Check if path is directory
    }
    return true;  // Path exists and directory check not required
}

bool checkWritePermissions(const std::string& path) {
    std::filesystem::path fsPath(path);
    std::error_code ec;

    if(!checkPathExists(path, true)) {
        return false;
    }

    auto perms = std::filesystem::status(fsPath, ec).permissions();
    if(ec) {
        return false;
    }

    bool hasWritePermissions = (perms & std::filesystem::perms::owner_write) != std::filesystem::perms::none;
    return hasWritePermissions;
}

std::string joinPaths(const std::string& p1, const std::string& p2) {
    std::filesystem::path path1(p1);
    std::filesystem::path path2(p2);
    return (path1 / path2).string();
}

std::string getDirFromPath(const std::string& path) {
    std::filesystem::path fsPath = std::filesystem::absolute(path);
    if(std::filesystem::is_directory(fsPath)) {
        return fsPath.string();
    }
    return fsPath.parent_path().string();
}

FSLock::FSLock(const std::string& fname) : filename(fname), isLocked(false) {}

FSLock::~FSLock() {
    if(holding()) {
        unlock();
    }

#ifdef _WIN32
    if(handle != INVALID_HANDLE_VALUE) {
        CloseHandle(handle);
    }
#else
    if(fd != -1) {
        close(fd);
    }
#endif
}

void FSLock::lock() {
    lockPath = getLockPath(filename);

#ifdef _WIN32
    handle = CreateFileA(filename.c_str(), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if(handle == INVALID_HANDLE_VALUE) {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    OVERLAPPED overlapped = {0};
    if(!LockFileEx(handle, LOCKFILE_EXCLUSIVE_LOCK, 0, MAXDWORD, MAXDWORD, &overlapped)) {
        throw std::runtime_error("Failed to acquire lock on file: " + lockPath);
    }

#else
    fd = open(lockPath.c_str(), O_RDWR | O_CREAT, 0666);
    if(fd == -1) {
        throw std::runtime_error("Failed to open file: " + lockPath);
    }

    struct flock fl {};
    fl.l_type = F_WRLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len = 0;
    if(fcntl(fd, F_SETLKW, &fl) == -1) {
        throw std::runtime_error("Failed to acquire lock on file: " + lockPath);
    }
#endif

    isLocked = true;
}

void FSLock::unlock() {
#ifdef _WIN32
    OVERLAPPED overlapped = {0};
    if(!UnlockFileEx(handle, 0, MAXDWORD, MAXDWORD, &overlapped)) {
        throw std::runtime_error("Failed to release lock on file: " + lockPath);
    }
#else
    struct flock fl {};
    fl.l_type = F_UNLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len = 0;
    if(fcntl(fd, F_SETLK, &fl) == -1) {
        throw std::runtime_error("Failed to release lock on file: " + lockPath);
    }
#endif

    isLocked = false;
}

bool FSLock::holding() const {
    return isLocked;
}

FileLock::FileLock(const std::string& path, bool createIfNotExists) : FSLock(path) {
    if(!createIfNotExists && !std::filesystem::exists(path)) {
        throw std::runtime_error("File does not exist: " + path);
    }
}

std::string FileLock::getLockPath(const std::string& path) {
    return path;
}

FolderLock::FolderLock(const std::string& path) : FSLock(path) {
    if(!std::filesystem::exists(path)) {
        throw std::runtime_error("Folder does not exist: " + path);
    }
    if(!std::filesystem::is_directory(path)) {
        throw std::runtime_error("Path is not a folder: " + path);
    }
}

std::string FolderLock::getLockPath(const std::string& path) {
    return joinPaths(path, ".folder_lock");
}

std::unique_ptr<FileLock> FileLock::lock(const std::string& path, bool createIfNotExists) {
    auto fileLock = std::make_unique<FileLock>(path, createIfNotExists);
    fileLock->lock();
    return fileLock;
}

std::unique_ptr<FolderLock> FolderLock::lock(const std::string& path) {
    auto folderLock = std::make_unique<FolderLock>(path);
    folderLock->lock();
    return folderLock;
}

}  // namespace platform
}  // namespace dai
