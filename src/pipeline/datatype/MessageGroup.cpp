#include "depthai/pipeline/datatype/MessageGroup.hpp"

#include <chrono>
#include <memory>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"

namespace dai {

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
