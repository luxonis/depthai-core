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
    #if (_WIN32_WINNT <= 0x0501)
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

std::filesystem::path getTempPath() {
    std::string tmpPath;
#if defined(_WIN32) || defined(__USE_W32_SOCKETS)
    char tmpPathBuffer[MAX_PATH];
    GetTempPathA(MAX_PATH, tmpPathBuffer);
    tmpPath = tmpPathBuffer;
#else
    char tmpTemplate[] = "/tmp/depthai_XXXXXX";
    char* tmpName = mkdtemp(tmpTemplate);
    if(tmpName == nullptr) {
        tmpPath = "/tmp";
    } else {
        tmpPath = tmpName;
        tmpPath += '/';
    }
#endif
    return std::filesystem::path(tmpPath);
}

bool checkPathExists(const std::filesystem::path& path, bool directory) {
    if(directory) {
        return std::filesystem::exists(path) && std::filesystem::is_directory(path);
    } else {
        return std::filesystem::exists(path);
    }
}

bool checkWritePermissions(const std::filesystem::path& path) {
#if defined(_WIN32) || defined(__USE_W32_SOCKETS)
    DWORD ftyp = GetFileAttributesA(path.string().c_str());
    if(ftyp == INVALID_FILE_ATTRIBUTES) {
        return false;  // Path does not exist
    } else if(ftyp & FILE_ATTRIBUTE_READONLY) {
        return false;  // Path is read-only
    } else {
        return true;  // Path is writable
    }
#else
    struct stat info;
    if(stat(path.string().c_str(), &info) != 0) {
        return false;  // Path does not exist
    } else if(info.st_mode & S_IWUSR) {
        return true;  // Path is writable
    } else {
        return false;  // Path is read-only
    }
#endif
}


FSLock::FSLock(const std::filesystem::path& fname) : filename(fname), isLocked(false), threadLock(getThreadLock(fname)) {}

FSLock::~FSLock() {
    if(holding()) {
        unlock();
    }
}

void FSLock::lock() {
    // First acquire the thread lock
    threadLock.lock();

    lockPath = getLockPath(filename);

#ifdef _WIN32
    handle = CreateFileW(lockPath.c_str(), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if(handle == INVALID_HANDLE_VALUE) {
        threadLock.unlock();  // Release thread lock if file lock fails
        throw std::runtime_error("Failed to open file: " + lockPath.string());
    }

    OVERLAPPED overlapped = {0};
    if(!LockFileEx(handle, LOCKFILE_EXCLUSIVE_LOCK, 0, MAXDWORD, MAXDWORD, &overlapped)) {
        CloseHandle(handle);
        handle = INVALID_HANDLE_VALUE;
        threadLock.unlock();  // Release thread lock if file lock fails
        throw std::runtime_error("Failed to acquire lock on file: " + lockPath.string());
    }

#else
    fd = open(lockPath.c_str(), O_RDWR | O_CREAT, 0666);
    if(fd == -1) {
        threadLock.unlock();  // Release thread lock if file lock fails
        throw std::runtime_error("Failed to open file: " + lockPath.string());
    }

    struct flock fl{};
    fl.l_type = F_WRLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len = 0;
    if(fcntl(fd, F_SETLKW, &fl) == -1) {
        close(fd);
        fd = -1;
        threadLock.unlock();  // Release thread lock if file lock fails
        throw std::runtime_error("Failed to acquire lock on file: " + lockPath.string());
    }
#endif

    isLocked = true;
}

void FSLock::unlock() {
#ifdef _WIN32
    OVERLAPPED overlapped = {0};
    if(!UnlockFileEx(handle, 0, MAXDWORD, MAXDWORD, &overlapped)) {
        throw std::runtime_error("Failed to release lock on file: " + lockPath.string());
    }
    CloseHandle(handle);
    handle = INVALID_HANDLE_VALUE;
#else
    struct flock fl{};
    fl.l_type = F_UNLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len = 0;
    if(fcntl(fd, F_SETLK, &fl) == -1) {
        throw std::runtime_error("Failed to release lock on file: " + lockPath.string());
    }
    close(fd);
    fd = -1;
#endif

    isLocked = false;
    threadLock.unlock();  // Release the thread lock after file lock is released
}

bool FSLock::holding() const {
    return isLocked;
}

FileLock::FileLock(const std::filesystem::path& path, bool createIfNotExists) : FSLock(path) {
    if(!createIfNotExists && !std::filesystem::exists(path)) {
        throw std::runtime_error("File does not exist: " + path.string());
    }
}

std::filesystem::path FileLock::getLockPath(const std::filesystem::path& path) {
    return path;
}

FolderLock::FolderLock(const std::filesystem::path& path) : FSLock(path) {
    if(!std::filesystem::exists(path)) {
        throw std::runtime_error("Folder does not exist: " + path.string());
    }
    if(!std::filesystem::is_directory(path)) {
        throw std::runtime_error("Path is not a folder: " + path.string());
    }
}

std::filesystem::path FolderLock::getLockPath(const std::filesystem::path& path) {
    return joinPaths(path, ".folder_lock");
}

std::unique_ptr<FileLock> FileLock::lock(const std::filesystem::path& path, bool createIfNotExists) {
    auto fileLock = std::make_unique<FileLock>(path, createIfNotExists);
    fileLock->lock();
    return fileLock;
}

std::unique_ptr<FolderLock> FolderLock::lock(const std::filesystem::path& path) {
    auto folderLock = std::make_unique<FolderLock>(path);
    folderLock->lock();
    return folderLock;
}

std::filesystem::path joinPaths(const std::filesystem::path& p1, const std::filesystem::path& p2) {
    return p1 / p2;
}

std::filesystem::path getDirFromPath(const std::filesystem::path& path) {
    return std::filesystem::path(std::filesystem::absolute(path).parent_path());
}

}  // namespace platform
}  // namespace dai
