#include "depthai/pipeline/datatype/MessageGroup.hpp"

#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/datatype/RawMessageGroup.hpp"

namespace dai {

std::shared_ptr<RawBuffer> MessageGroup::serialize() const {
    return raw;
}

MessageGroup::MessageGroup() : Buffer(std::make_shared<RawMessageGroup>()), grp(*dynamic_cast<RawMessageGroup*>(raw.get())) {}
MessageGroup::MessageGroup(std::shared_ptr<RawMessageGroup> ptr) : Buffer(std::move(ptr)), grp(*dynamic_cast<RawMessageGroup*>(raw.get())) {}

Buffer MessageGroup::operator[](const std::string& name) {
    return Buffer(grp.group.at(name).buffer);
}

// setters
MessageGroup& MessageGroup::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<MessageGroup&>(Buffer::setTimestamp(tp));
}
MessageGroup& MessageGroup::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<MessageGroup&>(Buffer::setTimestampDevice(tp));
}
MessageGroup& MessageGroup::setSequenceNum(int64_t sequenceNum) {
    return static_cast<MessageGroup&>(Buffer::setSequenceNum(sequenceNum));
}

}  // namespace dai
