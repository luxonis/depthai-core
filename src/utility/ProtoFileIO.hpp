#pragma once

#include <vector>

#include "ProtoSerialize.hpp"

namespace dai {
namespace utility {

namespace detail {

template <typename T>
struct ProtoMessageType;

template <>
struct ProtoMessageType<ImgFrame> {
    using type = proto::img_frame::ImgFrame;
};

template <>
struct ProtoMessageType<EncodedFrame> {
    using type = proto::encoded_frame::EncodedFrame;
};

template <>
struct ProtoMessageType<IMUData> {
    using type = proto::imu_data::IMUData;
};

template <>
struct ProtoMessageType<PointCloudData> {
    using type = proto::point_cloud_data::PointCloudData;
};

template <>
struct ProtoMessageType<RGBDData> {
    using type = proto::rgbd_data::RGBDData;
};

}  // namespace detail

template <typename T>
void loadProtoMessageFromBytes(T& message, const std::vector<uint8_t>& bytes, bool metadataOnly = false) {
    using ProtoType = typename detail::ProtoMessageType<T>::type;

    ProtoType protoMessage;
    if(!protoMessage.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()))) {
        throw std::runtime_error("Failed to parse protobuf message");
    }

    utility::setProtoMessage(message, &protoMessage, metadataOnly);
}

}  // namespace utility
}  // namespace dai
