#include "depthai/pipeline/datatype/PointCloudData.hpp"

#include <algorithm>

#include "depthai/common/Point3f.hpp"
#ifdef DEPTHAI_ENABLE_PROTOBUF
    #include "depthai/schemas/PointCloudData.pb.h"
    #include "utility/ProtoSerialize.hpp"
#endif
namespace dai {

PointCloudData::~PointCloudData() = default;

void PointCloudData::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::PointCloudData;
}

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
    // Organized: points.size() == width * height
    // Sparse: points.size() <= width (height == 1)
    assert(isOrganized() || points.size() <= width);
    assert(!isOrganized() || points.size() == width * height);

    return points;
}

std::vector<Point3fRGBA> PointCloudData::getPointsRGB() {
    if(!isColor()) {
        throw std::runtime_error("PointCloudData does not contain color data");
    }
    span<const Point3fRGBA> pointData(reinterpret_cast<Point3fRGBA*>(data->getData().data()), data->getData().size() / sizeof(Point3fRGBA));
    std::vector<Point3fRGBA> points(pointData.begin(), pointData.end());
    // Organized: points.size() == width * height
    // Sparse: points.size() <= width (height == 1)
    assert(isOrganized() || points.size() <= width);
    assert(!isOrganized() || points.size() == width * height);

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
    return !isOrganized();
}

bool PointCloudData::isOrganized() const {
    return height > 1;
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
PointCloudData& PointCloudData::setSparse(bool /*val*/) {
    // Deprecated no-op - organization is now determined by width/height (isOrganized = height > 1)
    return *this;
}

PointCloudData& PointCloudData::updateBoundingBox() {
    const auto* rawData = data->getData().data();
    const auto rawSize = data->getData().size();

    bool foundValid = false;
    float mnX = 0.f, mnY = 0.f, mnZ = 0.f;
    float mxX = 0.f, mxY = 0.f, mxZ = 0.f;

    auto update = [&](float x, float y, float z) {
        if(z > 0.0f) {
            if(!foundValid) {
                mnX = mxX = x;
                mnY = mxY = y;
                mnZ = mxZ = z;
                foundValid = true;
            } else {
                mnX = std::min(mnX, x);
                mnY = std::min(mnY, y);
                mnZ = std::min(mnZ, z);
                mxX = std::max(mxX, x);
                mxY = std::max(mxY, y);
                mxZ = std::max(mxZ, z);
            }
        }
    };

    if(isColor()) {
        const auto count = rawSize / sizeof(Point3fRGBA);
        const auto* pts = reinterpret_cast<const Point3fRGBA*>(rawData);
        for(std::size_t i = 0; i < count; ++i) {
            update(pts[i].x, pts[i].y, pts[i].z);
        }
    } else {
        const auto count = rawSize / sizeof(Point3f);
        const auto* pts = reinterpret_cast<const Point3f*>(rawData);
        for(std::size_t i = 0; i < count; ++i) {
            update(pts[i].x, pts[i].y, pts[i].z);
        }
    }

    minx = mnX; miny = mnY; minz = mnZ;
    maxx = mxX; maxy = mxY; maxz = mxZ;
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
