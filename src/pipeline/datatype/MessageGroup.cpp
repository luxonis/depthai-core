#include "depthai/pipeline/datatype/MessageGroup.hpp"

#include <chrono>
#include <memory>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"

namespace dai {

MessageGroup::~MessageGroup() = default;

void MessageGroup::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::MessageGroup;
}

std::shared_ptr<ADatatype> MessageGroup::operator[](const std::string& name) {
    return group.at(name);
}
void MessageGroup::add(const std::string& name, const std::shared_ptr<ADatatype>& value) {
    group[name] = value;
}

std::map<std::string, std::shared_ptr<ADatatype>>::iterator MessageGroup::begin() {
    return group.begin();
}
std::map<std::string, std::shared_ptr<ADatatype>>::iterator MessageGroup::end() {
    return group.end();
}

int64_t MessageGroup::getIntervalNs() const {
    // Measure with the timestamp source the group was synced with; entries that are
    // not Buffers or lack the requested timestamp are skipped
    using Source = SyncProperties::TimestampSource;
    bool first = true;
    std::chrono::nanoseconds oldest{};
    std::chrono::nanoseconds latest{};
    auto consider = [&](std::chrono::nanoseconds ts) {
        if(first) {
            oldest = latest = ts;
            first = false;
            return;
        }
        if(ts < oldest) oldest = ts;
        if(ts > latest) latest = ts;
    };
    for(const auto& entry : group) {
        auto buffer = std::dynamic_pointer_cast<Buffer>(entry.second);
        if(buffer == nullptr) continue;
        switch(timestampSource) {
            case Source::HOST:
                consider(std::chrono::duration_cast<std::chrono::nanoseconds>(buffer->getTimestamp().time_since_epoch()));
                break;
            case Source::SYSTEM: {
                auto ts = buffer->getTimestampSystem();
                if(ts.has_value()) consider(std::chrono::duration_cast<std::chrono::nanoseconds>(ts->time_since_epoch()));
                break;
            }
            case Source::DEFAULT:
            case Source::DEVICE:
                consider(std::chrono::duration_cast<std::chrono::nanoseconds>(buffer->getTimestampDevice().time_since_epoch()));
                break;
        }
    }
    if(first) return {};
    return (latest - oldest).count();
}

int64_t MessageGroup::getNumMessages() const {
    return group.size();
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

}  // namespace dai
