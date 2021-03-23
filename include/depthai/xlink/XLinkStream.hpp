#pragma once

// Std
#include <atomic>
#include <chrono>
#include <cstdint>
#include <list>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

// libraries
#include <XLink/XLinkPublicDefines.h>

// project
#include "depthai/xlink/XLinkConnection.hpp"

namespace dai {

class XLinkStream {
    // static
    constexpr static int STREAM_OPEN_RETRIES = 5;
    constexpr static std::chrono::milliseconds WAIT_FOR_STREAM_RETRY{50};
    static std::mutex xlinkStreamOperationMutex;

    std::string streamName;
    streamId_t streamId{INVALID_STREAM_ID};

   public:
    XLinkStream(const XLinkConnection& conn, const std::string& name, std::size_t maxWriteSize);
    XLinkStream(const XLinkStream&) = delete;
    XLinkStream(XLinkStream&& stream);
    ~XLinkStream();

    // Blocking
    void write(const void* data, std::size_t size);
    void write(const std::uint8_t* data, std::size_t size);
    void write(const std::vector<std::uint8_t>& data);
    std::vector<std::uint8_t> read();
    void read(std::vector<std::uint8_t>& data);
    // split write helper
    void writeSplit(const void* data, std::size_t size, std::size_t split);
    void writeSplit(const std::vector<uint8_t>& data, std::size_t split);
    // USE ONLY WHEN COPYING DATA AT LATER STAGES
    streamPacketDesc_t* readRaw();

    // Timeout
    bool write(const void* data, std::size_t size, std::chrono::milliseconds timeout);
    bool write(const std::uint8_t* data, std::size_t size, std::chrono::milliseconds timeout);
    bool write(const std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout);
    bool read(std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout);
    bool readRaw(streamPacketDesc_t*& pPacket, std::chrono::milliseconds timeout);

    // USE ONLY WHEN COPYING DATA AT LATER STAGES
    void readRawRelease();

    streamId_t getStreamId() const;
};

}  // namespace dai
