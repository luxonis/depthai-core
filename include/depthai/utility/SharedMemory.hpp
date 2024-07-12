#pragma once

// std
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#if defined(__unix__) && !defined(__APPLE__)
    #include <fcntl.h>
    #include <sys/mman.h>
    #include <sys/stat.h>
    #include <sys/un.h>
    #include <unistd.h>
#endif

// project
#include "depthai/utility/Memory.hpp"

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
    SharedMemory() {
        kind = MemoryKinds::MEMORY_KIND_SHARED;
        fd = -1;
    }

    SharedMemory(long argFd) : fd(argFd) {
        kind = MemoryKinds::MEMORY_KIND_SHARED;
        mapFd();
    }

    SharedMemory(long argFd, std::size_t size) : fd(argFd) {
        kind = MemoryKinds::MEMORY_KIND_SHARED;
        setSize(size);
        mapFd();
    }

    SharedMemory(const char* name) {
        kind = MemoryKinds::MEMORY_KIND_SHARED;
#if defined(__unix__) && !defined(__APPLE__)
        fd = memfd_create(name, 0);
#else
	(void)name;
	fd = -1;
#endif
        mapFd();
    }

    SharedMemory(const char* name, std::size_t size) {
        kind = MemoryKinds::MEMORY_KIND_SHARED;
#if defined(__unix__) && !defined(__APPLE__)
        fd = memfd_create(name, 0);
#else
	(void)name;
	fd = -1;
#endif

        setSize(size);
        mapFd();
    }

    ~SharedMemory() {
        unmapFd();
        close(fd);
    }

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
        ftruncate(fd, size);
#endif
        mapFd();
    }

    std::size_t getSize() const {
        return getMaxSize();
    }

    long getFd() const {
        return fd;
    }
};

}  // namespace dai
