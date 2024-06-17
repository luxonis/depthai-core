#pragma once

// std
#include <cstdint>
#include <cstring>
#include <functional>
#include <fcntl.h>
#include <iostream>
#include <sys/mman.h>
#include <sys/un.h>
#include <unistd.h>

// project
#include "depthai/utility/Memory.hpp"

namespace dai {

// memory as interface
class SharedMemory : public Memory {
   private:
    long fd;
    void *mapping;
   public:
    SharedMemory() = default;
    SharedMemory(long argFd) : fd(argFd) {
	mapping = mmap(NULL, getSize(), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    }

    ~SharedMemory() {
        munmap(fd, getSize());
    }

    SharedMemory& operator=(long argFd) {
        munmap(fd, getSize());
	fd = argFd;
	mapping = mmap(NULL, getSize(), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

        return *this;
    }

    span<std::uint8_t> getData() override {
        return {(uint8_t*)mapping, getSize()};
    }
    span<const std::uint8_t> getData() const override {
        return {(const uint8_t*)mapping, getSize()};
    }
    std::size_t getMaxSize() const override {
	struct stat fileStats;
	fstat(fd, &fileStats);

        return fileStats.st_size;
    }
    std::size_t getOffset() const override {
        return ftell(fdopen(fd, "r"));
    }
    void setSize(std::size_t size) override {
	ftruncate(fd, size);
    }

    std::size_t getSize() const override {
	return getMaxSize();
    }
};

}  // namespace dai
