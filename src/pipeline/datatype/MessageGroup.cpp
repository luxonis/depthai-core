#include "depthai/pipeline/datatype/MessageGroup.hpp"

#include <chrono>
#include <memory>

#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"

namespace dai {

std::shared_ptr<RawBuffer> MessageGroup::serialize() const {
    return raw;
}

MessageGroup::MessageGroup() : Buffer(std::make_shared<RawMessageGroup>()), rawGrp(*dynamic_cast<RawMessageGroup*>(raw.get())) {}
MessageGroup::MessageGroup(std::shared_ptr<RawMessageGroup> ptr) : Buffer(std::move(ptr)), rawGrp(*dynamic_cast<RawMessageGroup*>(raw.get())) {}

std::shared_ptr<ADatatype> MessageGroup::operator[](const std::string& name) {
    return group.at(name);
}
void MessageGroup::add(const std::string& name, const std::shared_ptr<ADatatype>& value) {
    group[name] = value;
    rawGrp.group[name] = {value->getRaw(), 0};
}

std::unordered_map<std::string, std::shared_ptr<ADatatype>>::iterator MessageGroup::begin() {
    return group.begin();
}
std::unordered_map<std::string, std::shared_ptr<ADatatype>>::iterator MessageGroup::end() {
    return group.end();
}

int64_t MessageGroup::getIntervalNs() const {
    if(!rawGrp.group.empty()) {
        auto oldest = std::dynamic_pointer_cast<Buffer>(group.begin()->second)->getTimestampDevice();
        auto latest = oldest;
        for(const auto& entry : group) {
            auto ts = std::dynamic_pointer_cast<Buffer>(entry.second)->getTimestampDevice();
            if(ts < oldest) oldest = ts;
            if(ts > latest) latest = ts;
        }
        return std::chrono::duration_cast<std::chrono::nanoseconds>(latest - oldest).count();
    }
    return {};
}

int64_t MessageGroup::getNumMessages() const {
    return rawGrp.group.size();
}

std::vector<std::string> MessageGroup::getMessageNames() const {
    std::vector<std::string> names;
    names.reserve(group.size());
    for(const auto& entry : group) {
        names.push_back(entry.first);
    }
    return names;
}

bool MessageGroup::isSynced(int64_t thresholdNs) const {
    return getIntervalNs() <= thresholdNs;
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
