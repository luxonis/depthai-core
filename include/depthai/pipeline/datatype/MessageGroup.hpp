#pragma once

#include <chrono>
#include <memory>
#include <unordered_map>
#include <vector>

#include "../../utility/TypeToEnum.hpp"
#include "depthai-shared/datatype/RawMessageGroup.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
namespace dai {

/**
 * MessageGroup message. Carries multiple messages in one.
 */
class MessageGroup : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawMessageGroup& grp;

   public:
    /// Construct IMUData message
    MessageGroup();
    explicit MessageGroup(std::shared_ptr<RawMessageGroup> ptr);
    virtual ~MessageGroup() = default;

    /// Group
    std::shared_ptr<ADatatype> operator[](const std::string& name);
    template <typename T>
    T get(const std::string& name) {
        return T(grp.group.at(name).buffer);
    }
    template <typename T>
    void add(const std::string& name, const std::shared_ptr<T>& value) {
        grp.group[name] = RawGroupMessage{value->getRaw(), 0};
    }

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
};

}  // namespace dai
