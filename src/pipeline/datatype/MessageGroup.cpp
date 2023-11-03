#include "depthai/pipeline/datatype/MessageGroup.hpp"

#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"

namespace dai {

std::shared_ptr<RawBuffer> MessageGroup::serialize() const {
    return raw;
}

MessageGroup::MessageGroup() : Buffer(std::make_shared<RawMessageGroup>()), grp(*dynamic_cast<RawMessageGroup*>(raw.get())) {}
MessageGroup::MessageGroup(std::shared_ptr<RawMessageGroup> ptr) : Buffer(std::move(ptr)), grp(*dynamic_cast<RawMessageGroup*>(raw.get())) {}

std::shared_ptr<ADatatype> MessageGroup::operator[](const std::string& name) {
    std::shared_ptr<RawBuffer> ptr = grp.group.at(name).buffer;
    switch(ptr->getType()) {
        case DatatypeEnum::Buffer:
            return std::make_shared<Buffer>(std::dynamic_pointer_cast<RawBuffer>(ptr));
        case DatatypeEnum::ImgFrame:
            return std::make_shared<ImgFrame>(std::dynamic_pointer_cast<RawImgFrame>(ptr));
        case DatatypeEnum::EncodedFrame:
            return std::make_shared<EncodedFrame>(std::dynamic_pointer_cast<RawEncodedFrame>(ptr));
        case DatatypeEnum::NNData:
            return std::make_shared<NNData>(std::dynamic_pointer_cast<RawNNData>(ptr));
        case DatatypeEnum::ImageManipConfig:
            return std::make_shared<ImageManipConfig>(std::dynamic_pointer_cast<RawImageManipConfig>(ptr));
        case DatatypeEnum::CameraControl:
            return std::make_shared<CameraControl>(std::dynamic_pointer_cast<RawCameraControl>(ptr));
        case DatatypeEnum::ImgDetections:
            return std::make_shared<ImgDetections>(std::dynamic_pointer_cast<RawImgDetections>(ptr));
        case DatatypeEnum::SpatialImgDetections:
            return std::make_shared<SpatialImgDetections>(std::dynamic_pointer_cast<RawSpatialImgDetections>(ptr));
        case DatatypeEnum::SystemInformation:
            return std::make_shared<SystemInformation>(std::dynamic_pointer_cast<RawSystemInformation>(ptr));
        case DatatypeEnum::SpatialLocationCalculatorConfig:
            return std::make_shared<SpatialLocationCalculatorConfig>(std::dynamic_pointer_cast<RawSpatialLocationCalculatorConfig>(ptr));
        case DatatypeEnum::SpatialLocationCalculatorData:
            return std::make_shared<SpatialLocationCalculatorData>(std::dynamic_pointer_cast<RawSpatialLocations>(ptr));
        case DatatypeEnum::EdgeDetectorConfig:
            return std::make_shared<EdgeDetectorConfig>(std::dynamic_pointer_cast<RawEdgeDetectorConfig>(ptr));
        case DatatypeEnum::AprilTagConfig:
            return std::make_shared<AprilTagConfig>(std::dynamic_pointer_cast<RawAprilTagConfig>(ptr));
        case DatatypeEnum::AprilTags:
            return std::make_shared<AprilTags>(std::dynamic_pointer_cast<RawAprilTags>(ptr));
        case DatatypeEnum::Tracklets:
            return std::make_shared<Tracklets>(std::dynamic_pointer_cast<RawTracklets>(ptr));
        case DatatypeEnum::IMUData:
            return std::make_shared<IMUData>(std::dynamic_pointer_cast<RawIMUData>(ptr));
        case DatatypeEnum::StereoDepthConfig:
            return std::make_shared<StereoDepthConfig>(std::dynamic_pointer_cast<RawStereoDepthConfig>(ptr));
        case DatatypeEnum::FeatureTrackerConfig:
            return std::make_shared<FeatureTrackerConfig>(std::dynamic_pointer_cast<RawFeatureTrackerConfig>(ptr));
        case DatatypeEnum::ToFConfig:
            return std::make_shared<ToFConfig>(std::dynamic_pointer_cast<RawToFConfig>(ptr));
        case DatatypeEnum::TrackedFeatures:
            return std::make_shared<TrackedFeatures>(std::dynamic_pointer_cast<RawTrackedFeatures>(ptr));
        case DatatypeEnum::MessageGroup:
            return {};
    }
    return {};
}

bool MessageGroup::syncSuccessful() const {
    return grp.success;
}

int64_t MessageGroup::getIntervalNs() const {
    if(!grp.group.empty()) {
        auto first = grp.group.begin()->second.buffer->tsDevice;
        int64_t oldest = first.sec * (int64_t)1e9 + first.nsec;
        int64_t latest = oldest;
        for(auto& entry : grp.group) {
            auto& ts = entry.second.buffer->tsDevice;
            int64_t tsNs = ts.sec * (int64_t)1e9 + ts.nsec;
            if(tsNs < oldest) oldest = tsNs;
            if(tsNs > latest) latest = tsNs;
        }
        return latest - oldest;
    }
    return {};
}

int64_t MessageGroup::getNumMessages() const {
    return grp.group.size();
}

// setters
MessageGroup& MessageGroup::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<MessageGroup&>(Buffer::setTimestamp(tp));
}
MessageGroup& MessageGroup::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<MessageGroup&>(Buffer::setTimestampDevice(tp));
}
MessageGroup& MessageGroup::setSequenceNum(int64_t sequenceNum) {
    return static_cast<MessageGroup&>(Buffer::setSequenceNum(sequenceNum));
}

}  // namespace dai
