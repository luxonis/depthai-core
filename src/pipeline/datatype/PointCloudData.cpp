#include "depthai/pipeline/datatype/PointCloudData.hpp"

#include <iostream>

namespace dai {

std::shared_ptr<RawBuffer> PointCloudData::serialize() const {
    return raw;
}

PointCloudData::PointCloudData() : Buffer(std::make_shared<RawPointCloudData>()), pcl(*dynamic_cast<RawPointCloudData*>(raw.get())) {
    // set timestamp to now
    setTimestamp(std::chrono::steady_clock::now());
}
PointCloudData::PointCloudData(std::shared_ptr<RawPointCloudData> ptr) : Buffer(std::move(ptr)), pcl(*dynamic_cast<RawPointCloudData*>(raw.get())) {}

std::vector<Point3f>& PointCloudData::getPoints() {
    if(points.empty() && !pcl.data.empty()) {
        auto* dataPtr = (Point3f*)pcl.data.data();
        points.insert(points.end(), dataPtr, dataPtr + pcl.data.size() / sizeof(Point3f));
        assert(isSparse() || points.size() == pcl.width * pcl.height);
        assert(!isSparse() || points.size() <= pcl.width * pcl.height);
    }
    return points;
}

unsigned int PointCloudData::getInstanceNum() const {
    return pcl.instanceNum;
}
unsigned int PointCloudData::getWidth() const {
    return pcl.width;
}
unsigned int PointCloudData::getHeight() const {
    return pcl.height;
}
float PointCloudData::getMinX() const {
    return pcl.minx;
}
float PointCloudData::getMinY() const {
    return pcl.miny;
}
float PointCloudData::getMinZ() const {
    return pcl.minz;
}
float PointCloudData::getMaxX() const {
    return pcl.maxx;
}
float PointCloudData::getMaxY() const {
    return pcl.maxy;
}
float PointCloudData::getMaxZ() const {
    return pcl.maxz;
}
bool PointCloudData::isSparse() const {
    return pcl.sparse;
}

// setters
PointCloudData& PointCloudData::setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<PointCloudData&>(Buffer::setTimestamp(tp));
}
PointCloudData& PointCloudData::setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> tp) {
    // Set timestamp from timepoint
    return static_cast<PointCloudData&>(Buffer::setTimestampDevice(tp));
}
PointCloudData& PointCloudData::setInstanceNum(unsigned int instanceNum) {
    pcl.instanceNum = instanceNum;
    return *this;
}
PointCloudData& PointCloudData::setSequenceNum(int64_t sequenceNum) {
    return static_cast<PointCloudData&>(Buffer::setSequenceNum(sequenceNum));
}
PointCloudData& PointCloudData::setWidth(unsigned int width) {
    pcl.width = width;
    return *this;
}
PointCloudData& PointCloudData::setHeight(unsigned int height) {
    pcl.height = height;
    return *this;
}
PointCloudData& PointCloudData::setSize(unsigned int width, unsigned int height) {
    setWidth(width);
    setHeight(height);
    return *this;
}
PointCloudData& PointCloudData::setSize(std::tuple<unsigned int, unsigned int> size) {
    setSize(std::get<0>(size), std::get<1>(size));
    return *this;
}
PointCloudData& PointCloudData::setMinX(float val) {
    pcl.minx = val;
    return *this;
}
PointCloudData& PointCloudData::setMinY(float val) {
    pcl.miny = val;
    return *this;
}
PointCloudData& PointCloudData::setMinZ(float val) {
    pcl.minz = val;
    return *this;
}
PointCloudData& PointCloudData::setMaxX(float val) {
    pcl.maxx = val;
    return *this;
}
PointCloudData& PointCloudData::setMaxY(float val) {
    pcl.maxy = val;
    return *this;
}
PointCloudData& PointCloudData::setMaxZ(float val) {
    pcl.maxz = val;
    return *this;
}

static_assert(sizeof(Point3f) == 12, "Point3f size must be 12 bytes");

}  // namespace dai
