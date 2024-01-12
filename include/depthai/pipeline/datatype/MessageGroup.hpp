#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "depthai/common/ADatatypeSharedPtrSerialization.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/Serialization.hpp"
namespace dai {
/**
 * MessageGroup message. Carries multiple messages in one.
 */
class MessageGroup : public Buffer {
   public:
    std::map<std::string, std::shared_ptr<ADatatype>> group;

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
    }

    // Iterators
    std::map<std::string, std::shared_ptr<ADatatype>>::iterator begin();
    std::map<std::string, std::shared_ptr<ADatatype>>::iterator end();

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

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::MessageGroup;
    };
    DEPTHAI_SERIALIZE(MessageGroup, group, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum);
};

}  // namespace dai