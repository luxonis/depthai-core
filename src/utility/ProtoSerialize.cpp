#include "ProtoSerialize.hpp"

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/util/time_util.h>

#include <queue>

namespace dai {
namespace utility {
// // Writes the FileDescriptor of this descriptor and all transitive dependencies
// // to a string, for use as a channel schema.
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

std::vector<std::uint8_t> serializeProto(std::unique_ptr<google::protobuf::Message> protoMessage) {
    std::size_t nbytes = protoMessage->ByteSizeLong();
    std::vector<std::uint8_t> buffer(nbytes);

    // The test is necessary becaue v.data could be NULL if nbytes is 0
    if(nbytes > 0) {
        protoMessage->SerializeToArray(buffer.data(), nbytes);
    }

    return buffer;
}

ProtoSerializable::SchemaPair serializeSchema(std::unique_ptr<google::protobuf::Message> protoMessage) {
    const auto* descriptor = protoMessage->GetDescriptor();
    if(descriptor == nullptr) {
        throw std::runtime_error("Failed to get protobuf descriptor");
    }
    ProtoSerializable::SchemaPair returnPair;
    returnPair.schemaName = descriptor->full_name();
    returnPair.schema = serializeFdSet(descriptor);
    return returnPair;
}

void serializeImgTransormation(proto::common::ImgTransformation* imgTransformation, const ImgTransformation& transformation) {
    const auto [width, height] = transformation.getSize();
    const auto [srcWidth, srcHeight] = transformation.getSourceSize();
    imgTransformation->set_width(width);
    imgTransformation->set_height(height);
    imgTransformation->set_srcwidth(srcWidth);
    imgTransformation->set_srcheight(srcHeight);

    proto::common::TransformationMatrix* transformationMatrix = imgTransformation->mutable_transformationmatrix();
    for(const auto& array : transformation.getMatrix()) {
        proto::common::FloatArray* floatArray = transformationMatrix->add_arrays();
        for(const auto& value : array) {
            floatArray->add_values(value);
        }
    }
    proto::common::TransformationMatrix* sourceIntrinsicMatrix = imgTransformation->mutable_sourceintrinsicmatrix();
    for(const auto& array : transformation.getSourceIntrinsicMatrix()) {
        proto::common::FloatArray* floatArray = sourceIntrinsicMatrix->add_arrays();
        for(const auto& value : array) {
            floatArray->add_values(value);
        }
    }

    imgTransformation->set_distortionmodel(static_cast<proto::common::CameraModel>(transformation.getDistortionModel()));
    proto::common::FloatArray* distortionCoefficients = imgTransformation->mutable_distortioncoefficients();
    for(const auto& value : transformation.getDistortionCoefficients()) {
        distortionCoefficients->add_values(value);
    }
}

};  // namespace utility
};  // namespace dai
