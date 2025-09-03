#include "depthai/pipeline/datatype/Keypoints.hpp"

#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "utility/ProtoSerialize.hpp"
#endif

namespace dai {

void Keypoints::setKeypoints(const std::vector<Keypoint>& keypoints) {
    keypointsList.setKeypoints(keypoints);
}

void Keypoints::setKeypoints(const std::vector<Point3f>& keypoints) {
    keypointsList.setKeypoints(keypoints);
}

void Keypoints::setKeypoints(const std::vector<Point2f>& keypoints) {
    keypointsList.setKeypoints(keypoints);
}

void Keypoints::setKeypoints(const std::vector<Keypoint>& keypoints, const std::vector<std::array<uint32_t, 2>>& edges) {
    keypointsList.setKeypoints(keypoints, edges);
}

void Keypoints::setEdges(const std::vector<std::array<uint32_t, 2>>& edges) {
    keypointsList.setEdges(edges);
}

std::vector<Keypoint> Keypoints::getKeypoints() const {
    return keypointsList.getKeypoints();
}

std::vector<std::array<uint32_t, 2>> Keypoints::getEdges() const {
    return keypointsList.getEdges();
}

std::vector<Point3f> Keypoints::getCoordinates3f() const {
    return keypointsList.getCoordinates3f();
}

std::vector<Point2f> Keypoints::getCoordinates2f() const {
    return keypointsList.getCoordinates2f();
}

std::vector<std::string> Keypoints::getLabels() const {
    return keypointsList.getLabels();
}

#ifdef DEPTHAI_ENABLE_PROTOBUF
ProtoSerializable::SchemaPair Keypoints::serializeSchema() const {
    return utility::serializeSchema(utility::getProtoMessage(this));
}

std::vector<std::uint8_t> Keypoints::serializeProto(bool) const {
    return utility::serializeProto(utility::getProtoMessage(this));
}
#endif

}  // namespace dai