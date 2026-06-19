#include "depthai/pipeline/datatype/MessageGroup.hpp"

#include <chrono>
#include <memory>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {
namespace {

std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getGroupTimestampDevice(
    const std::shared_ptr<ADatatype>& value) {
    auto buffer = std::dynamic_pointer_cast<Buffer>(value);
    if(auto imgFrame = std::dynamic_pointer_cast<ImgFrame>(buffer)) {
        return imgFrame->getTimestampDevice(CameraExposureOffset::END);
    }

    return buffer->getTimestampDevice();
}

}  // namespace

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
    if(!group.empty()) {
        auto oldest = getGroupTimestampDevice(group.begin()->second);
        auto latest = oldest;
        for(const auto& entry : group) {
            auto ts = getGroupTimestampDevice(entry.second);
            if(ts < oldest) oldest = ts;
            if(ts > latest) latest = ts;
        }
        return std::chrono::duration_cast<std::chrono::nanoseconds>(latest - oldest).count();
    }
    return {};
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
