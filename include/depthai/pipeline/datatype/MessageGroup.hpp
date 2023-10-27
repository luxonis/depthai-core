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
 * IMUData message. Carries normalized detection results
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
    Buffer operator[](const std::string& name);
    template <typename T>
    T get(const std::string& name) {
        return T(grp.group.at(name).buffer);
    }
    template <typename T>
    void add(const std::string& name, const std::shared_ptr<T>& value) {
        // TODO(asahtik): How to get the correct type when using pybind11?
        DatatypeEnum type = rawToType<T>();
        grp.group[name] = RawGroupMessage{type, value->getRaw()};
    }

    /**
     * True if sync was successful.
     */
    bool syncSuccessful() const;

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
