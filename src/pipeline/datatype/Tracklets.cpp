#include "depthai/pipeline/datatype/Tracklets.hpp"

namespace dai {

std::shared_ptr<RawBuffer> Tracklets::serialize() const {
    return raw;
}

Tracklets::Tracklets() : Buffer(std::make_shared<RawTracklets>()), rawdata(*dynamic_cast<RawTracklets*>(raw.get())), tracklets(rawdata.tracklets) {}
Tracklets::Tracklets(std::shared_ptr<RawTracklets> ptr)
    : Buffer(std::move(ptr)), rawdata(*dynamic_cast<RawTracklets*>(raw.get())), tracklets(rawdata.tracklets) {}

}  // namespace dai
