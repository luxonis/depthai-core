#include "depthai/pipeline/datatype/AprilTagData.hpp"

namespace dai {

std::shared_ptr<RawBuffer> AprilTagData::serialize() const {
    return raw;
}

AprilTagData::AprilTagData() : Buffer(std::make_shared<RawAprilTags>()), rawdata(*dynamic_cast<RawAprilTags*>(raw.get())), aprilTags(rawdata.aprilTags) {}
AprilTagData::AprilTagData(std::shared_ptr<RawAprilTags> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawAprilTags*>(raw.get())), aprilTags(rawdata.aprilTags) {}

std::vector<AprilTags>& AprilTagData::getAprilTag() const {
    return rawdata.aprilTags;
}

}  // namespace dai