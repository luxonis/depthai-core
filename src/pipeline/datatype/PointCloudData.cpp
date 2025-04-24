#include "depthai/pipeline/datatype/PointCloudData.hpp"

#include "depthai/common/Point3f.hpp"
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/PointCloudData.pb.h"
    #include "utility/ProtoSerialize.hpp"
#endif
namespace dai {

std::vector<Point3f> PointCloudData::getPoints() {
    if(isColor()) {
        span<const Point3fRGBA> pointData(reinterpret_cast<Point3fRGBA*>(data->getData().data()), data->getData().size() / sizeof(Point3fRGBA));
        std::vector<Point3fRGBA> points(pointData.begin(), pointData.end());
        std::vector<Point3f> points3f;
        for(const auto& p : points) {
            points3f.push_back({p.x, p.y, p.z});
        }
        return points3f;
    }
    span<const Point3f> pointData(reinterpret_cast<Point3f*>(data->getData().data()), data->getData().size() / sizeof(Point3f));
    std::vector<Point3f> points(pointData.begin(), pointData.end());
    assert(isSparse() || points.size() == width * height);
    assert(!isSparse() || points.size() <= width * height);

    return points;
}

std::vector<Point3fRGBA> PointCloudData::getPointsRGB() {
    if(!isColor()) {
        throw std::runtime_error("PointCloudData does not contain color data");
    }
    span<const Point3fRGBA> pointData(reinterpret_cast<Point3fRGBA*>(data->getData().data()), data->getData().size() / sizeof(Point3fRGBA));
    std::vector<Point3fRGBA> points(pointData.begin(), pointData.end());
    assert(isSparse() || points.size() == width * height);
    assert(!isSparse() || points.size() <= width * height);

    return points;
}

void PointCloudData::setPoints(const std::vector<Point3f>& points) {
    auto size = points.size();
    std::vector<uint8_t> data(size * sizeof(Point3f));
    std::memcpy(data.data(), points.data(), size * sizeof(Point3f));
    setData(std::move(data));
    setColor(false);
}

void PointCloudData::setPointsRGB(const std::vector<Point3fRGBA>& points) {
    auto size = points.size();
    std::vector<uint8_t> data(size * sizeof(Point3fRGBA));
    std::memcpy(data.data(), points.data(), size * sizeof(Point3fRGBA));
    setData(std::move(data));
    setColor(true);
}

unsigned int PointCloudData::getInstanceNum() const {
    return instanceNum;
}
unsigned int PointCloudData::getWidth() const {
    return width;
}
unsigned int PointCloudData::getHeight() const {
    return height;
}
float PointCloudData::getMinX() const {
    return minx;
}
float PointCloudData::getMinY() const {
    return miny;
}
float PointCloudData::getMinZ() const {
    return minz;
}
float PointCloudData::getMaxX() const {
    return maxx;
}
float PointCloudData::getMaxY() const {
    return maxy;
}
float PointCloudData::getMaxZ() const {
    return maxz;
}
bool PointCloudData::isSparse() const {
    return sparse;
}

bool PointCloudData::isColor() const {
    return color;
}

PointCloudData& PointCloudData::setInstanceNum(unsigned int instanceNum) {
    this->instanceNum = instanceNum;
    return *this;
}

PointCloudData& PointCloudData::setWidth(unsigned int width) {
    this->width = width;
    return *this;
}
PointCloudData& PointCloudData::setHeight(unsigned int height) {
    this->height = height;
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
    this->minx = val;
    return *this;
}
PointCloudData& PointCloudData::setMinY(float val) {
    this->miny = val;
    return *this;
}
PointCloudData& PointCloudData::setMinZ(float val) {
    this->minz = val;
    return *this;
}
PointCloudData& PointCloudData::setMaxX(float val) {
    this->maxx = val;
    return *this;
}
PointCloudData& PointCloudData::setMaxY(float val) {
    this->maxy = val;
    return *this;
}
PointCloudData& PointCloudData::setMaxZ(float val) {
    this->maxz = val;
    return *this;
}
PointCloudData& PointCloudData::setSparse(bool val) {
    sparse = val;
    return *this;
}

PointCloudData& PointCloudData::setColor(bool val) {
    color = val;
    return *this;
}

#ifdef DEPTHAI_ENABLE_PROTOBUF
std::vector<std::uint8_t> PointCloudData::serializeProto(bool metadataOnly) const {
    return utility::serializeProto(utility::getProtoMessage(this, metadataOnly));
}

ProtoSerializable::SchemaPair PointCloudData::serializeSchema() const {
    return utility::serializeSchema(utility::getProtoMessage(this));
}

#endif
static_assert(sizeof(Point3f) == 12, "Point3f size must be 12 bytes");
static_assert(sizeof(Point3fRGBA) == 16, "Point3fRGBA size must be 16 bytes");
}  // namespace dai
