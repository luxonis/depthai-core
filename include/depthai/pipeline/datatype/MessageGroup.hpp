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

    /**
     * True if sync was successful.
     */
    bool syncSuccessful() const;

    /**
     * Retrieves interval between the first and the last message in the group.
     */
    int64_t getIntervalNs() const;

    int64_t getNumMessages() const;

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

    MessageGroup& setSuccess(bool success);
};

}  // namespace dai
