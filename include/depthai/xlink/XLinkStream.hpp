#pragma once

// Std
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// libraries
#include <XLink/XLinkPublicDefines.h>
#include <XLink/XLinkTime.h>

// project
#include "depthai/utility/Memory.hpp"
#include "depthai/utility/span.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

namespace dai {

class StreamPacketDesc : public streamPacketDesc_t {
   public:
    /**
     * Construct an empty stream packet descriptor.
     */
    StreamPacketDesc() noexcept : streamPacketDesc_t{nullptr, 0, -1, {}, {}} {};
    /** Deleted copy constructor. */
    StreamPacketDesc(const StreamPacketDesc&) = delete;
    /**
     * Move-construct a stream packet descriptor.
     */
    StreamPacketDesc(StreamPacketDesc&& other) noexcept;
    StreamPacketDesc& operator=(const StreamPacketDesc&) = delete;
    /**
     * Move-assign a stream packet descriptor.
     */
    StreamPacketDesc& operator=(StreamPacketDesc&& other) noexcept;
    ~StreamPacketDesc() noexcept;
};

class StreamPacketMemory : public StreamPacketDesc, public Memory {
    size_t size;

   public:
    StreamPacketMemory() = default;
    /**
     * Construct memory wrapper from a moved packet descriptor.
     */
    StreamPacketMemory(StreamPacketDesc&& d) : StreamPacketDesc(std::move(d)) {
        size = length;
    }
    /**
     * Assign memory wrapper from a moved packet descriptor.
     */
    StreamPacketMemory& operator=(StreamPacketDesc&& d) {
        StreamPacketDesc::operator=(std::move(d));
        size = length;
        return *this;
    }
    span<std::uint8_t> getData() override;
    span<const std::uint8_t> getData() const override;
    std::size_t getMaxSize() const override;
    std::size_t getOffset() const override;
    void setSize(size_t size) override;
};

class XLinkStream {
    // static
    constexpr static int STREAM_OPEN_RETRIES = 5;
    constexpr static std::chrono::milliseconds WAIT_FOR_STREAM_RETRY{50};

    std::shared_ptr<XLinkConnection> connection;
    std::string streamName;
    streamId_t streamId{INVALID_STREAM_ID};

   public:
    /**
     * Construct an XLink stream with a maximum write size.
     */
    XLinkStream(const std::shared_ptr<XLinkConnection> conn, const std::string& name, std::size_t maxWriteSize);
    /** Deleted copy constructor. */
    XLinkStream(const XLinkStream&) = delete;
    /**
     * Move-construct an XLink stream.
     */
    XLinkStream(XLinkStream&& stream);
    XLinkStream& operator=(const XLinkStream&) = delete;
    /**
     * Move-assign an XLink stream.
     */
    XLinkStream& operator=(XLinkStream&& stream);
    ~XLinkStream();

    // Blocking
    /**
     * Write two buffers to the stream (blocking).
     */
    void write(span<const uint8_t> data, span<const uint8_t> data2);
    /**
     * Write a buffer to the stream (blocking).
     */
    void write(span<const uint8_t> data);
    /**
     * Write data from a file descriptor (blocking).
     */
    void write(long fd);
    /**
     * Write a file descriptor and an extra buffer (blocking).
     */
    void write(long fd, span<const uint8_t> data);
    /**
     * Write raw data to the stream (blocking).
     */
    void write(const void* data, std::size_t size);
    /**
     * Read data from the stream (blocking).
     */
    std::vector<std::uint8_t> read();
    /**
     * Read data from the stream with timeout (blocking).
     */
    std::vector<std::uint8_t> read(std::chrono::milliseconds timeout);
    /**
     * Read data and capture receive timestamp (blocking).
     */
    std::vector<std::uint8_t> read(XLinkTimespec& timestampReceived);
    /**
     * Read data into a provided buffer (blocking).
     */
    void read(std::vector<std::uint8_t>& data);
    /**
     * Read data into a buffer and return file descriptor (blocking).
     */
    void read(std::vector<std::uint8_t>& data, long& fd);
    /**
     * Read data into a buffer and capture receive timestamp (blocking).
     */
    void read(std::vector<std::uint8_t>& data, XLinkTimespec& timestampReceived);
    /**
     * Read data into a buffer and return file descriptor and timestamp (blocking).
     */
    void read(std::vector<std::uint8_t>& data, long& fd, XLinkTimespec& timestampReceived);
    // split write helper
    /**
     * Write data in chunks of a given split size.
     */
    void writeSplit(const void* data, std::size_t size, std::size_t split);
    /**
     * Write vector data in chunks of a given split size.
     */
    void writeSplit(const std::vector<uint8_t>& data, std::size_t split);
    /**
     * Read a packet into a movable descriptor.
     */
    StreamPacketDesc readMove();

    // Timeout
    /**
     * Write raw data with a timeout.
     */
    bool write(const void* data, std::size_t size, std::chrono::milliseconds timeout);
    /**
     * Write raw data with a timeout.
     */
    bool write(const std::uint8_t* data, std::size_t size, std::chrono::milliseconds timeout);
    /**
     * Write vector data with a timeout.
     */
    bool write(const std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout);
    /**
     * Read data with a timeout into a buffer.
     */
    bool read(std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout);
    /**
     * Read a packet with a timeout into a descriptor.
     */
    bool readMove(StreamPacketDesc& packet, const std::chrono::milliseconds timeout);
    // TODO optional<StreamPacketDesc> readMove(timeout) -or- tuple<bool, StreamPacketDesc> readMove(timeout)

    // deprecated use readMove() instead; readRaw leads to memory violations and/or memory leaks
    [[deprecated("use readMove()")]] streamPacketDesc_t* readRaw();
    // deprecated use readMove(packet, timeout) instead; readRaw leads to memory violations and/or memory leaks
    [[deprecated("use readMove(packet, timeout)")]] bool readRaw(streamPacketDesc_t*& pPacket, std::chrono::milliseconds timeout);
    // deprecated; unsafe leads to memory violations and/or memory leaks
    [[deprecated]] void readRawRelease();

    /**
     * Return the stream id.
     */
    streamId_t getStreamId() const;
    /**
     * Return the stream name.
     */
    std::string getStreamName() const;
};

struct XLinkError : public std::runtime_error {
    const XLinkError_t status = X_LINK_ERROR;
    const std::string streamName;

    using std::runtime_error::runtime_error;
    ~XLinkError() override;

    /**
     * Construct an XLink error with status and stream name.
     */
    XLinkError(XLinkError_t statusID, std::string stream, const std::string& message)
        : runtime_error(message), status(statusID), streamName(std::move(stream)) {}
};
struct XLinkReadError : public XLinkError {
    using XLinkError = XLinkError;
    ~XLinkReadError() override;
    /**
     * Construct a read error from status and stream name.
     */
    XLinkReadError(XLinkError_t status, const std::string& stream);
};
struct XLinkWriteError : public XLinkError {
    using XLinkError = XLinkError;
    ~XLinkWriteError() override;
    /**
     * Construct a write error from status and stream name.
     */
    XLinkWriteError(XLinkError_t status, const std::string& stream);
};

}  // namespace dai
