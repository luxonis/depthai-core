#include "depthai/pipeline/datatype/RecordControl.hpp"

namespace dai {

RecordControl::~RecordControl() = default;

std::shared_ptr<RecordControl> RecordControl::start() {
    auto message = std::make_shared<RecordControl>();
    message->command = Command::START;
    message->maxFrames = -1;
    message->maxDurationMs = -1;
    message->maxSizeKb = -1;
    return message;
}

std::shared_ptr<RecordControl> RecordControl::start(std::int32_t maxFrames, std::int32_t maxDurationMs) {
    auto message = start();
    message->maxFrames = maxFrames;
    message->maxDurationMs = maxDurationMs;
    return message;
}

std::shared_ptr<RecordControl> RecordControl::start(std::int32_t maxFrames, std::int32_t maxDurationMs, std::int32_t maxSizeKb) {
    auto message = start(maxFrames, maxDurationMs);
    message->maxSizeKb = maxSizeKb;
    return message;
}

std::shared_ptr<RecordControl> RecordControl::stop() {
    auto message = std::make_shared<RecordControl>();
    message->command = Command::STOP;
    message->maxFrames = -1;
    message->maxDurationMs = -1;
    message->maxSizeKb = -1;
    return message;
}

}  // namespace dai
