#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "depthai/common/ADatatypeSharedPtrSerialization.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/properties/SyncProperties.hpp"
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
     * Retrieves interval between the first and the last message in the group,
     * measured with the timestamp source that produced the group.
     */
    int64_t getIntervalNs() const;

    /**
     * Get the timestamp source this group was synced with.
     */
    SyncProperties::TimestampSource getTimestampSource() const {
        return timestampSource;
    }

    /**
     * Set the timestamp source this group was synced with. Used by the host Sync node;
     * groups received from a device keep the DEVICE default.
     */
    void setTimestampSource(SyncProperties::TimestampSource source) {
        timestampSource = source;
    }

    int64_t getNumMessages() const;

    /**
     * Gets the names of messages in the group
     */
    std::vector<std::string> getMessageNames() const;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::MessageGroup;
    }
    DEPTHAI_SERIALIZE(MessageGroup, group, Buffer::ts, Buffer::tsDevice, Buffer::tsSystem, Buffer::sequenceNum);

   private:
    // Deliberately NOT serialized (wire format is shared with device firmware):
    // device-produced groups are synced with DEVICE timestamps, host Sync overwrites
    SyncProperties::TimestampSource timestampSource = SyncProperties::TimestampSource::DEVICE;
};

}  // namespace dai