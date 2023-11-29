#pragma once

#include <chrono>
#include <memory>
#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawMessageGroup.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * MessageGroup message. Carries multiple messages in one.
 */
class MessageGroup : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawMessageGroup& rawGrp;
    std::unordered_map<std::string, std::shared_ptr<ADatatype>> group;

   public:
    /// Construct MessageGroup message
    MessageGroup();
    explicit MessageGroup(std::shared_ptr<RawMessageGroup> ptr);
    virtual ~MessageGroup() = default;

    /// Group
    std::shared_ptr<ADatatype> operator[](const std::string& name);
    template <typename T>
    std::shared_ptr<T> get(const std::string& name) {
        return std::dynamic_pointer_cast<T>(group[name]);
    }
    void add(const std::string& name, const std::shared_ptr<ADatatype>& value);
    template <typename T>
    void add(const std::string& name, const T& value) {
        static_assert(std::is_base_of<ADatatype, T>::value, "T must derive from ADatatype");
        group[name] = std::make_shared<T>(value);
        rawGrp.group[name] = {value.getRaw(), 0};
    }

    // Iterators
    std::unordered_map<std::string, std::shared_ptr<ADatatype>>::iterator begin();
    std::unordered_map<std::string, std::shared_ptr<ADatatype>>::iterator end();

    /**
     * True if all messages in the group are in the interval
     * @param thresholdNs Maximal interval between messages
     */
    bool isSynced(int64_t thresholdNs) const;

    /**
     * Retrieves interval between the first and the last message in the group.
     */
    int64_t getIntervalNs() const;

    int64_t getNumMessages() const;

    /**
     * Gets the names of messages in the group
     */
    std::vector<std::string> getMessageNames() const;

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    MessageGroup& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    MessageGroup& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Retrieves image sequence number
     */
    MessageGroup& setSequenceNum(int64_t sequenceNum);
};

}  // namespace dai
