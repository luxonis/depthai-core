#pragma once

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/util/time_util.h>

#include <chrono>
#include <memory>
#include <queue>
#include <vector>
#include "depthai/schemas/common.pb.h"

namespace dai {

namespace utility {

// Writes the FileDescriptor of this descriptor and all transitive dependencies
// to a string, for use as a channel schema.
static std::string serializeFdSet(const google::protobuf::Descriptor* toplevelDescriptor) {
    google::protobuf::FileDescriptorSet fdSet;
    std::queue<const google::protobuf::FileDescriptor*> toAdd;
    toAdd.push(toplevelDescriptor->file());
    std::unordered_set<std::string> seenDependencies;
    while(!toAdd.empty()) {
        const google::protobuf::FileDescriptor* next = toAdd.front();
        toAdd.pop();
        next->CopyTo(fdSet.add_file());
        for(int i = 0; i < next->dependency_count(); ++i) {
            const auto& dep = next->dependency(i);
            if(seenDependencies.find(dep->name()) == seenDependencies.end()) {
                seenDependencies.insert(dep->name());
                toAdd.push(dep);
            }
        }
    }
    return fdSet.SerializeAsString();
}

class ProtoSerializable {
   public:
    struct schemaPair {
        std::string schemaName;
        std::string schema;
    };

    virtual ~ProtoSerializable() = default;

    /**
     * @brief Serialize the protobuf message of this object
     * @return serialized protobuf message
     */
    virtual std::vector<std::uint8_t> serializeProto() const final {
        auto protoMessage = getProtoMessage();
        std::size_t nbytes = protoMessage->ByteSizeLong();
        std::vector<std::uint8_t> buffer(nbytes);

        // The test is necessary becaue v.data could be NULL if nbytes is 0
        if(nbytes > 0) {
            protoMessage->SerializeToArray(buffer.data(), nbytes);
        }

        return buffer;
    }

    /**
     * @brief Serialize the schema of this object
     * @return schemaPair
     */
    virtual schemaPair serializeSchema() const {
        auto protoMessage = getProtoMessage();
        const auto* descriptor = protoMessage->GetDescriptor();
        if(descriptor == nullptr) {
            throw std::runtime_error("Failed to get protobuf descriptor");
        }
        schemaPair returnPair;
        returnPair.schemaName = descriptor->full_name();
        returnPair.schema = serializeFdSet(descriptor);
        return returnPair;
    }

   protected:
    /**
     * @brief Generate the corresponding protobuf message from this object
     * @return std::unique_ptr<google::protobuf::Message>
     */
    virtual std::unique_ptr<google::protobuf::Message> getProtoMessage() const = 0;
};

class ProtoDeserializable {
   public:
    virtual ~ProtoDeserializable() = default;

   protected:
    virtual void setProtoMessage(const std::unique_ptr<google::protobuf::Message>) = 0;
};

inline std::chrono::time_point<std::chrono::steady_clock> fromProtoTimestamp(const dai::proto::common::Timestamp& ts) {
    using namespace std::chrono;
    return time_point<steady_clock>(seconds(ts.sec()) + nanoseconds(ts.nsec()));
}

}  // namespace utility

}  // namespace dai
