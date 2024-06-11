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
 * @brief RecordVideo node, used to record a source stream to a file
 */
class RecordVideo : public NodeCRTP<ThreadedHostNode, RecordVideo> {
   public:
    using CompressionLevel = dai::utility::ByteRecorder::CompressionLevel;

    constexpr static const char* NAME = "RecordVideo";

    /**
     * Input for any type of messages to be transferred over XLink stream
     *
     * Default queue is blocking with size 8
     */
    Input input{*this, {.name = "input", .queueSize = 15, .types = {{DatatypeEnum::Buffer, true}}}};

    void run() override;

    std::string getRecordMetadataFile() const;
    std::string getRecordVideoFile() const;
    CompressionLevel getCompressionLevel() const;

    RecordVideo& setRecordMetadataFile(const std::string& recordFile);
    RecordVideo& setRecordVideoFile(const std::string& recordFile);
    RecordVideo& setCompressionLevel(CompressionLevel compressionLevel);

   private:
    std::string recordMetadataFile;
    std::string recordVideoFile;
    unsigned int fpsInitLength = 10;
    CompressionLevel compressionLevel = CompressionLevel::DEFAULT;
};

/**
 * @brief RecordMessage node, used to record a source stream to a file
 */
class RecordMessage : public NodeCRTP<ThreadedHostNode, RecordMessage> {
   public:
    using CompressionLevel = dai::utility::ByteRecorder::CompressionLevel;

    constexpr static const char* NAME = "RecordMessage";

    /**
     * Input for any type of messages to be transferred over XLink stream
     *
     * Default queue is blocking with size 8
     */
    Input input{*this, {.name = "input", .queueSize = 15, .types = {{DatatypeEnum::Buffer, true}}}};

    void run() override;

    std::string getRecordFile() const;
    CompressionLevel getCompressionLevel() const;

    RecordMessage& setRecordFile(const std::string& recordFile);
    RecordMessage& setCompressionLevel(CompressionLevel compressionLevel);

   private:
    std::string recordFile;
    CompressionLevel compressionLevel = CompressionLevel::DEFAULT;
};

}  // namespace node
}  // namespace dai
