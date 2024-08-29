#pragma once

#include <memory>
#include <vector>
#include <google/protobuf/message.h>

namespace dai {

namespace utility {

class ProtoSerializable {
public:
    virtual ~ProtoSerializable() = default;

    // This function is final to prevent overriding
    virtual std::vector<std::uint8_t> serializeProto() const final {
        auto protoMessage = getProtoMessage();
        std::size_t nbytes = protoMessage->ByteSizeLong();
        std::vector<std::uint8_t> buffer(nbytes);

        // The test is necessary becaue v.data could be NULL if nbytes is 0
        if (nbytes > 0) {
            protoMessage->SerializeToArray(buffer.data(), nbytes);
        }

        return buffer;
    }

protected:
    virtual std::unique_ptr<google::protobuf::Message> getProtoMessage() const = 0;
};

}  // namespace utility

}  // namespace dai