#pragma once

#include <depthai/pipeline/ThreadedNode.hpp>

// shared
#include <depthai/properties/XLinkOutProperties.hpp>

// project
#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/utility/RecordReplay.hpp>

#include "depthai/pipeline/HostNode.hpp"

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    #error Record node needs OpenCV support
#endif

namespace dai {
namespace node {

/**
 * @brief Record node, used to record a source stream to a file
 */
class Record : public NodeCRTP<HostNode, Record> {
   public:
    using CompressionLevel = dai::utility::ByteRecorder::CompressionLevel;

    constexpr static const char* NAME = "Record";

    /**
     * Input for any type of messages to be transferred over XLink stream
     *
     * Default queue is blocking with size 8
     */
    Input input{*this, {.name = "input", .queueSize = 15, .types = {{DatatypeEnum::Buffer, true}}}};

    void run() override;

    Record& setRecordFile(const std::string& recordFile);

    Record& setCompressionLevel(CompressionLevel compressionLevel);

   private:
    std::string recordFile;
    unsigned int fpsInitLength = 10;
    CompressionLevel compressionLevel = CompressionLevel::DEFAULT;
};

}  // namespace node
}  // namespace dai
