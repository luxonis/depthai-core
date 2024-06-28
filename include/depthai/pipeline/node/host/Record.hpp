#pragma once

#include <depthai/pipeline/ThreadedNode.hpp>

// shared
#include <depthai/properties/XLinkOutProperties.hpp>

// project
#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/utility/RecordReplay.hpp>

#include "depthai/pipeline/ThreadedHostNode.hpp"

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    #error Record node needs OpenCV support
#endif

namespace dai {
namespace node {

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
    Input input{*this, {"input", DEFAULT_GROUP, DEFAULT_BLOCKING, 15, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    void run() override;

    std::filesystem::path getRecordMetadataFile() const;
    std::filesystem::path getRecordVideoFile() const;
    CompressionLevel getCompressionLevel() const;

    RecordVideo& setRecordMetadataFile(const std::filesystem::path& recordFile);
    RecordVideo& setRecordVideoFile(const std::filesystem::path& recordFile);
    RecordVideo& setCompressionLevel(CompressionLevel compressionLevel);

   private:
    std::filesystem::path recordMetadataFile;
    std::filesystem::path recordVideoFile;
    unsigned int fpsInitLength = 10;
    CompressionLevel compressionLevel = CompressionLevel::DEFAULT;
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
    Input input{*this, {.name = "input", .queueSize = 15, .types = {{DatatypeEnum::Buffer, true}}}};

    void run() override;

    std::filesystem::path getRecordFile() const;
    CompressionLevel getCompressionLevel() const;

    RecordMetadataOnly& setRecordFile(const std::filesystem::path& recordFile);
    RecordMetadataOnly& setCompressionLevel(CompressionLevel compressionLevel);

   private:
    std::filesystem::path recordFile;
    CompressionLevel compressionLevel = CompressionLevel::DEFAULT;
};

}  // namespace node
}  // namespace dai
