#include "depthai/pipeline/datatype/Odometry.hpp"

namespace dai {

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
