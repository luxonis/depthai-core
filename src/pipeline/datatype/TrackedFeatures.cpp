#include "depthai/pipeline/datatype/TrackedFeatures.hpp"

namespace dai {

std::shared_ptr<RawBuffer> TrackedFeatures::serialize() const {
    return raw;
}

TrackedFeatures::TrackedFeatures()
    : Buffer(std::make_shared<RawTrackedFeatures>()), rawdata(*dynamic_cast<RawTrackedFeatures*>(raw.get())), trackedFeatures(rawdata.trackedFeatures) {}
TrackedFeatures::TrackedFeatures(std::shared_ptr<RawTrackedFeatures> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawTrackedFeatures*>(raw.get())), trackedFeatures(rawdata.trackedFeatures) {}

}  // namespace dai
