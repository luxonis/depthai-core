#pragma once

#include <memory>

#include <depthai/pipeline/ThreadedNode.hpp>

// shared
#include <depthai/properties/XLinkOutProperties.hpp>

// project
#include <depthai/pipeline/datatype/Buffer.hpp>

#include <depthai/utility/RecordReplay.hpp>

namespace dai {
namespace node {

/**
 * @brief Record node, used to record a source stream to a file
 */
class Record : public NodeCRTP<ThreadedNode, Record> {
private:
    std::shared_ptr<utility::VideoRecorder> videoRecorder;
    std::string recordFile;
   public:
    constexpr static const char* NAME = "Record";
    void build();

    /**
     * Input for any type of messages to be transferred over XLink stream
     *
     * Default queue is blocking with size 8
     */
    Input input{true, *this, "in", Input::Type::SReceiver, true, 8, true, {{DatatypeEnum::Buffer, true}}};

    void start() override;
    void run() override;
    void stop() override;

    Record& setRecordFile(const std::string& recordFile);
};

}  // namespace node
}  // namespace dai
