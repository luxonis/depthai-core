#include "depthai/pipeline/datatype/AprilTags.hpp"

namespace dai {

std::shared_ptr<RawBuffer> AprilTags::serialize() const {
    return raw;
}

AprilTags::AprilTags() : Buffer(std::make_shared<RawAprilTags>()), rawdata(*dynamic_cast<RawAprilTags*>(raw.get())), aprilTags(rawdata.aprilTags) {}
AprilTags::AprilTags(std::shared_ptr<RawAprilTags> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawAprilTags*>(raw.get())), aprilTags(rawdata.aprilTags) {}

}  // namespace dai