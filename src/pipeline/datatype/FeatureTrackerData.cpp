#include "depthai/pipeline/datatype/FeatureTrackerData.hpp"

namespace dai {

std::shared_ptr<RawBuffer> FeatureTrackerData::serialize() const {
    return raw;
}

FeatureTrackerData::FeatureTrackerData()
    : Buffer(std::make_shared<RawTrackedFeatures>()), rawdata(*dynamic_cast<RawTrackedFeatures*>(raw.get())), trackedFeatures(rawdata.trackedFeatures) {}
FeatureTrackerData::FeatureTrackerData(std::shared_ptr<RawTrackedFeatures> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawTrackedFeatures*>(raw.get())), trackedFeatures(rawdata.trackedFeatures) {}

}  // namespace dai
