#include "depthai/pipeline/datatype/Odometry.hpp"

namespace dai {
namespace {
struct OdometryFrameMetadata {
    std::uint32_t version = 1;
    Extrinsics extrinsics;
    NOP_STRUCTURE(OdometryFrameMetadata, version, extrinsics);
};
}  // namespace

namespace utility {
template <>
bool serialize<SerializationType::LIBNOP, Odometry>(const Odometry& obj, std::vector<std::uint8_t>& data) {
    nop::Serializer<VectorWriter> serializer{std::move(data)};
    auto status = serializer.Write(obj);
    if(status && obj.trackedFrameExtrinsics) {
        status = serializer.Write(OdometryFrameMetadata{1, *obj.trackedFrameExtrinsics});
    }
    if(!status) {
#ifdef DEPTHAI_EXCEPTIONS
        throw std::runtime_error(status.GetErrorMessage());
#else
        return false;
#endif
    }
    data = std::move(serializer.writer().take());
    return true;
}

template <>
bool deserialize<SerializationType::LIBNOP, Odometry>(const std::uint8_t* data, std::size_t size, Odometry& obj) {
    obj.trackedFrameExtrinsics.reset();
    nop::Deserializer<nop::BufferReader> deserializer{data, size};
    auto status = deserializer.Read(&obj);
    if(status && deserializer.reader().remaining() > 0) {
        OdometryFrameMetadata extension;
        status = deserializer.Read(&extension);
        if(status && (extension.version != 1 || deserializer.reader().remaining() != 0)) {
#ifdef DEPTHAI_EXCEPTIONS
            throw std::runtime_error("Odometry: unsupported or malformed frame metadata extension");
#else
            return false;
#endif
        }
        if(status) obj.trackedFrameExtrinsics = extension.extrinsics;
    }
    if(!status) {
#ifdef DEPTHAI_EXCEPTIONS
        throw std::runtime_error(status.GetErrorMessage());
#else
        return false;
#endif
    }
    return true;
}
}  // namespace utility

Odometry::Odometry() : TransformData(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0) {}

Odometry::Odometry(const Transform& transform, const Point3d& velocity) : TransformData(transform), velocity(velocity) {}

Odometry::Odometry(double x, double y, double z, double qx, double qy, double qz, double qw, const Point3d& velocity)
    : TransformData(x, y, z, qx, qy, qz, qw), velocity(velocity) {}

Odometry::~Odometry() = default;

void Odometry::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::Odometry;
}

}  // namespace dai
