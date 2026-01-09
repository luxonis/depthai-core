#pragma once

// std
#include <cstdint>
#include <cstring>
#include <stdexcept>
#if defined(__unix__) && !defined(__APPLE__)
    #include <fcntl.h>
    #include <sys/mman.h>
    #include <sys/stat.h>
    #include <sys/un.h>
    #include <unistd.h>
#endif

// project
#include "depthai/utility/Memory.hpp"
#include "depthai/utility/MemoryWrappers.hpp"

namespace dai {

// memory as interface
class SharedMemory : public Memory {
   protected:
    long fd = -1;
    void* mapping;
    void mapFd() {
        if(fd < 0) {
            /* Error handling here */
        }
#if defined(__unix__) && !defined(__APPLE__)
        mapping = mmap(NULL, getSize(), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if(mapping == NULL) {
            /* Error handling here */
        }
#endif
    }
    void unmapFd() {
        if(mapping == NULL) {
            return;
        }
#if defined(__unix__) && !defined(__APPLE__)
        munmap(mapping, getSize());
#endif
    }

   public:
    SharedMemory() = default;

    SharedMemory(long argFd) : fd(argFd) {
        mapFd();
    }

    SharedMemory(long argFd, std::size_t size) : fd(argFd) {
        setSize(size);
        mapFd();
    }

    SharedMemory(const char* name) {
#if defined(__unix__) && !defined(__APPLE__)
        fd = memfd_create(name, 0);
#else
        (void)name;
        fd = -1;
#endif
        mapFd();
    }

    SharedMemory(const char* name, std::size_t size) {
#if defined(__unix__) && !defined(__APPLE__)
        fd = memfd_create(name, 0);
#else
        (void)name;
        fd = -1;
#endif

        setSize(size);
        mapFd();
    }

    ~SharedMemory() override;

    SharedMemory& operator=(long argFd) {
        unmapFd();
        fd = argFd;
        mapFd();

        return *this;
    }

    span<std::uint8_t> getData() override {
        if(mapping == NULL) {
            mapFd();
        }

        return {(uint8_t*)mapping, getSize()};
    }
    span<const std::uint8_t> getData() const override {
        return {(const uint8_t*)mapping, getSize()};
    }
    std::size_t getMaxSize() const override {
#if defined(__unix__) && !defined(__APPLE__)
        struct stat fileStats;
        fstat(fd, &fileStats);

        return fileStats.st_size;
#else
        return 0;
#endif
    }
    std::size_t getOffset() const override {
#if defined(__unix__) && !defined(__APPLE__)
        return ftell(fdopen(fd, "r"));
#else
        return 0;
#endif
    }
    void setSize(std::size_t size) override {
        if(mapping != NULL) {
            unmapFd();
        }

#if defined(__unix__) && !defined(__APPLE__)
        if(ftruncate(fd, size) < 0) {
            throw std::runtime_error("Failed to set shared memory size");
        }
#else
        (void)size;  // size is not used
#endif

        mapFd();
    }

    std::size_t getSize() const {
        return getMaxSize();
    }

    virtual long getFd() const {
        return fd;
    }
};

}  // namespace dai
