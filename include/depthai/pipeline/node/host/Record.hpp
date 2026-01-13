#pragma once

#include <depthai/pipeline/ThreadedNode.hpp>

// shared
#include <depthai/properties/internal/XLinkOutProperties.hpp>

// project
#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/utility/RecordReplay.hpp>

#include "depthai/pipeline/ThreadedHostNode.hpp"

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    #error Record node needs OpenCV support
#endif

namespace dai {
namespace node {

using XLinkOutProperties = ::dai::internal::XLinkOutProperties;

/**
 * @brief RecordVideo node, used to record a video source stream to a file
 */
class RecordVideo : public NodeCRTP<ThreadedHostNode, RecordVideo> {
   public:
    using CompressionLevel = dai::RecordConfig::CompressionLevel;

    constexpr static const char* NAME = "RecordVideo";

    /**
     * Input for ImgFrame or EncodedFrame messages to be recorded
     *
     * Default queue is blocking with size 15
     */
    Input input{
        *this,
        {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, 15, {{{DatatypeEnum::ImgFrame, false}, {DatatypeEnum::EncodedFrame, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    void run() override;

    /**
     * Get metadata output file path.
     */
    std::filesystem::path getRecordMetadataFile() const;
    /**
     * Get video output file path.
     */
    std::filesystem::path getRecordVideoFile() const;
    /**
     * Get current compression level.
     */
    CompressionLevel getCompressionLevel() const;

    /**
     * Set metadata output file path.
     */
    RecordVideo& setRecordMetadataFile(const std::filesystem::path& recordFile);
    /**
     * Set video output file path.
     */
    RecordVideo& setRecordVideoFile(const std::filesystem::path& recordFile);
    /**
     * Set compression level.
     */
    RecordVideo& setCompressionLevel(CompressionLevel compressionLevel);
    /**
     * Set target frames per second.
     */
    RecordVideo& setFps(unsigned int fps);

   private:
    std::filesystem::path recordMetadataFile;
    std::filesystem::path recordVideoFile;
    CompressionLevel compressionLevel = CompressionLevel::DEFAULT;
    std::optional<unsigned int> fps;
    unsigned int fpsInitLength = 10;
};

/**
 * @brief RecordMetadataOnly node, used to record a source stream to a file
 */
class RecordMetadataOnly : public NodeCRTP<ThreadedHostNode, RecordMetadataOnly> {
   public:
    using CompressionLevel = dai::RecordConfig::CompressionLevel;

    constexpr static const char* NAME = "RecordMetadataOnly";

    /**
     * Input IMU messages to be recorded (will support other types in the future)
     *
     * Default queue is blocking with size 8
     */
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, 15, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    void run() override;

    /**
     * Get record output file path.
     */
    std::filesystem::path getRecordFile() const;
    /**
     * Get current compression level.
     */
    CompressionLevel getCompressionLevel() const;

    /**
     * Set record output file path.
     */
    RecordMetadataOnly& setRecordFile(const std::filesystem::path& recordFile);
    /**
     * Set compression level.
     */
    RecordMetadataOnly& setCompressionLevel(CompressionLevel compressionLevel);

   private:
    std::filesystem::path recordFile;
    CompressionLevel compressionLevel = CompressionLevel::DEFAULT;
};

}  // namespace node
}  // namespace dai
