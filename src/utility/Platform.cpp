#include "Platform.hpp"

#include <filesystem>
#include <memory>

// Platform specific
#if defined(_WIN32) || defined(__USE_W32_SOCKETS)
    #include <iphlpapi.h>
    #include <ws2tcpip.h>
    #ifdef _MSC_VER
        #pragma comment(lib, "Ws2_32.lib")
        #pragma comment(lib, "IPHLPAPI.lib")
    #endif
#else
    #if defined(__FreeBSD__) || defined(__NetBSD__) || defined(__OpenBSD__) || defined(__bsdi__) || defined(__DragonFly__)
        #include <netinet/in.h>
    #endif
    #include <arpa/inet.h>
    #include <ifaddrs.h>

    #include <cstring>
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

std::string getLocalIpAddress() {
#if defined(_WIN32) || defined(__USE_W32_SOCKETS)
    std::string result = "127.0.0.1";

    ULONG flags = GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER;
    ULONG outBufLen = 0;

    // First call to get required buffer size
    GetAdaptersAddresses(AF_INET, flags, nullptr, nullptr, &outBufLen);
    if(outBufLen == 0) {
        return result;
    }

    std::unique_ptr<BYTE[]> buffer(new BYTE[outBufLen]);
    PIP_ADAPTER_ADDRESSES pAddresses = reinterpret_cast<PIP_ADAPTER_ADDRESSES>(buffer.get());

    if(GetAdaptersAddresses(AF_INET, flags, nullptr, pAddresses, &outBufLen) != NO_ERROR) {
        return result;
    }

    for(auto* pCurrAddresses = pAddresses; pCurrAddresses; pCurrAddresses = pCurrAddresses->Next) {
        // Skip adapters that are not up
        if(pCurrAddresses->OperStatus != IfOperStatusUp) {
            continue;
        }

        // Skip loopback adapters
        if(pCurrAddresses->IfType == IF_TYPE_SOFTWARE_LOOPBACK) {
            continue;
        }

        for(auto* pUnicast = pCurrAddresses->FirstUnicastAddress; pUnicast; pUnicast = pUnicast->Next) {
            auto* addr = pUnicast->Address.lpSockaddr;
            if(addr->sa_family == AF_INET) {
                char ipStr[INET_ADDRSTRLEN] = {0};
                auto* sa_in = reinterpret_cast<sockaddr_in*>(addr);
                InetNtopA(AF_INET, &(sa_in->sin_addr), ipStr, sizeof(ipStr));
                std::string ip(ipStr);
                if(ip != "127.0.0.1") {
                    return ip;
                }
            }
        }
    }

    return result;
#else
    ifaddrs* ifaddr = nullptr;
    if(getifaddrs(&ifaddr) == -1) {
        return "127.0.0.1";
    }

    std::string result = "127.0.0.1";
    for(auto* ifa = ifaddr; ifa; ifa = ifa->ifa_next) {
        if(!ifa->ifa_addr || ifa->ifa_addr->sa_family != AF_INET) {
            continue;
        }

        auto* sin = reinterpret_cast<sockaddr_in*>(ifa->ifa_addr);
        char ipStr[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET, &(sin->sin_addr), ipStr, sizeof(ipStr));
        std::string ip(ipStr);

        if(ip != "127.0.0.1" && std::strncmp(ifa->ifa_name, "lo", 2) != 0) {
            result = ip;
            break;
        }
    }

    freeifaddrs(ifaddr);
    return result;
#endif
}

void setThreadName(JoiningThread& thread, const std::string& name) {
#ifdef __linux__
    auto handle = thread.native_handle();
    pthread_setname_np(handle, name.c_str());
#else
    (void)thread;  // unused
    (void)name;    // unused
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
    // On Windows, using _waccess() checks for existence for directories, not read and write permissions.
    // We check for write permission by creating a dummy file and deleting it.
    if(std::filesystem::is_directory(path)) {
        std::string uniqueName = std::to_string(std::time(nullptr)) + "_random_name";
        std::filesystem::path probe = path / uniqueName;

        HANDLE h = CreateFileW(probe.c_str(),
                               GENERIC_WRITE,
                               0,
                               nullptr,
                               CREATE_NEW,  // will fail if it already exists
                               FILE_ATTRIBUTE_TEMPORARY | FILE_FLAG_DELETE_ON_CLOSE,
                               nullptr);

        // Check if the file was created
        if(h == INVALID_HANDLE_VALUE) {
            return false;
        }

        // Close handle and delete the file
        CloseHandle(h);
        DeleteFileW(probe.c_str());

        // All checks passed, folder has write permissions
        return true;
    }
    // 2 = write permission
    return (_waccess(path.c_str(), 2) == 0);
#else
    return (access(path.c_str(), W_OK) == 0);
#endif
}

bool checkReadPermissions(const std::filesystem::path& path) {
#if defined(_WIN32) || defined(__USE_W32_SOCKETS)
    if(std::filesystem::is_directory(path)) {
        // Try to open the directory for listing
        HANDLE h = CreateFileW(path.c_str(),
                               FILE_LIST_DIRECTORY,
                               FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                               nullptr,
                               OPEN_EXISTING,
                               FILE_FLAG_BACKUP_SEMANTICS,
                               nullptr);

        // Check for valid handle, otherwise, we don't have read permissions
        if(h == INVALID_HANDLE_VALUE) {
            return false;
        }

        // properly close everything
        CloseHandle(h);
        return true;
    }

    // 4 = read permission
    return (_waccess(path.c_str(), 4) == 0);
#else
    return (access(path.c_str(), R_OK) == 0);
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
