#pragma once

#include <depthai/pipeline/ThreadedNode.hpp>
#include <memory>

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
 * @brief Record node, used to record a source stream to a file
 */
class Record : public NodeCRTP<ThreadedHostNode, Record> {
   public:
    using RecordCompressionLevel = dai::utility::ByteRecorder::CompressionLevel;

    constexpr static const char* NAME = "Record";

    /**
     * Input for any type of messages to be transferred over XLink stream
     *
     * Default queue is blocking with size 8
     */
    Input input{*this, {"in", DEFAULT_GROUP, DEFAULT_BLOCKING, 15, {{{DatatypeEnum::Buffer, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    void run() override;

    Record& setRecordFile(const std::string& recordFile);

    Record& setCompressionLevel(RecordCompressionLevel compressionLevel);

   private:
    std::string recordFile;
    unsigned int fpsInitLength = 10;
    RecordCompressionLevel compressionLevel = RecordCompressionLevel::DEFAULT;
};

}  // namespace node
}  // namespace dai
