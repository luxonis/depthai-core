#pragma once

// std
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#ifdef __unix__
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
	mapping = mmap(NULL, getSize(), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (mapping == NULL) {
	    /* Error handling here */
	}
    }
    void unmapFd() {
	if (mapping == NULL) {
	    return;
	}

	munmap(mapping, getSize());
    }
   public:
    SharedMemory(long argFd) : fd(argFd) {
	mapFd();
    }

    SharedMemory(long argFd, std::size_t size) : fd(argFd) {
	setSize(size);
	mapFd();
    }

    /* TODO: memfd_create() those?
    SharedMemory() {
	mapFd();
    }

    SharedMemory(std::size_t size) {
	setSize(size);
	mapFd();
    }
    */

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
	if (mapping == NULL) {
	    mapFd();
	}

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
	if (mapping != NULL) {
	    unmapFd();
	}

	ftruncate(fd, size);
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
