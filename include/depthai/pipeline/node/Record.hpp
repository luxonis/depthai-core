#pragma once

#include <depthai/pipeline/ThreadedNode.hpp>
#include <memory>

// shared
#include <depthai/properties/XLinkOutProperties.hpp>

// project
#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/utility/RecordReplay.hpp>

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
#error Record node needs OpenCV support
#endif

namespace dai {
namespace node {

/**
 * @brief Record node, used to record a source stream to a file
 */
class Record : public NodeCRTP<ThreadedNode, Record> {
   public:
    using RecordCompressionLevel = dai::utility::ByteRecorder::CompressionLevel;

    constexpr static const char* NAME = "Record";
    void build();

    /**
     * Input for any type of messages to be transferred over XLink stream
     *
     * Default queue is blocking with size 8
     */
    Input in{true, *this, "in", Input::Type::SReceiver, true, 15, true, {{DatatypeEnum::Buffer, true}}};

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
