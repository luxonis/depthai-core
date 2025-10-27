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

    virtual ~MessageGroup();

    /// Group
    std::shared_ptr<ADatatype> operator[](const std::string& name);
    template <typename T>
    std::shared_ptr<T> get(const std::string& name) {
        return std::dynamic_pointer_cast<T>(group[name]);
    }

    std::shared_ptr<ADatatype> get(const std::string& name) {
        return group[name];
    }

    // TODO(Morato) - this API is dangerous, when T is a base reference to a derived class
    // template <typename T>
    // void add(const std::string& name, const T& value) {
    //     static_assert(std::is_base_of<ADatatype, T>::value, "T must derive from ADatatype");
    //     group[name] = std::make_shared<T>(value);
    // }
    void add(const std::string& name, const std::shared_ptr<ADatatype>& value);

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

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
    DEPTHAI_SERIALIZE(MessageGroup, group, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum);
};

}  // namespace dai