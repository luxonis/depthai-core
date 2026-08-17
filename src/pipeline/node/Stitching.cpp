#include "depthai/pipeline/node/host/Stitching.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "StitchingPlatform.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {

namespace {

struct Vector3d {
    double x;
    double y;
    double z;
};

Vector3d cross(const Vector3d& lhs, const Vector3d& rhs) {
    return {lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x};
}

double norm(const Vector3d& vector) {
    return std::sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

Vector3d normalized(const Vector3d& vector) {
    const auto length = norm(vector);
    return {vector.x / length, vector.y / length, vector.z / length};
}

}  // namespace

StitchingProperties::VirtualCamera StitchingProperties::VirtualCamera::lookAt(
    const Point3f& position, const Point3f& target, const Point3f& up, float hFovDegrees, uint32_t width, uint32_t height, LengthUnit unit) {
    DAI_CHECK_V(width > 0 && height > 0, "The view must not be empty, got {}x{} pixels", width, height);
    DAI_CHECK_V(hFovDegrees > 0.0f && hFovDegrees < 180.0f, "The horizontal field of view must be within (0, 180) degrees, got {}", hFovDegrees);

    const Vector3d towardsTarget{target.x - position.x, target.y - position.y, target.z - position.z};
    DAI_CHECK_V(norm(towardsTarget) > 1e-6, "The camera cannot look at its own position");
    const auto forward = normalized(towardsTarget);
    const Vector3d upVector{up.x, up.y, up.z};
    const auto unnormalizedRight = cross(forward, upVector);
    DAI_CHECK_V(norm(unnormalizedRight) > 1e-6, "The up direction must not be parallel to the optical axis");
    const auto right = normalized(unnormalizedRight);
    const auto down = cross(forward, right);

    VirtualCamera camera;
    camera.width = width;
    camera.height = height;
    camera.unit = unit;
    constexpr double PI = 3.14159265358979323846;
    const auto focal = static_cast<float>(0.5 * width / std::tan(0.5 * static_cast<double>(hFovDegrees) * PI / 180.0));
    camera.intrinsics = {{{focal, 0.0f, 0.5f * (width - 1)}, {0.0f, focal, 0.5f * (height - 1)}, {0.0f, 0.0f, 1.0f}}};
    camera.pose = {{{static_cast<float>(right.x), static_cast<float>(down.x), static_cast<float>(forward.x), position.x},
                    {static_cast<float>(right.y), static_cast<float>(down.y), static_cast<float>(forward.y), position.y},
                    {static_cast<float>(right.z), static_cast<float>(down.z), static_cast<float>(forward.z), position.z},
                    {0.0f, 0.0f, 0.0f, 1.0f}}};
    return camera;
}

namespace node {

Stitching::Stitching() = default;

Stitching::~Stitching() = default;

Stitching::Properties& Stitching::getProperties() {
    if(device && !runOnHostVar && !stitching::isDevicePlatformSupported(device->getPlatform())) {
        throw std::runtime_error("Stitching node is only supported on RVC4 devices. Use setRunOnHost(true) instead.");
    }
    return properties;
}

void Stitching::buildInternal() {
    sync->out.link(inSync);
    sync->setRunOnHost(true);
}

std::shared_ptr<Stitching> Stitching::build(size_t numInputs) {
    DAI_CHECK_V(inputNames.empty(), "Stitching node was already built");
    DAI_CHECK_V(numInputs >= 2, "Stitching node needs at least two inputs, got {}", numInputs);

    for(size_t i = 0; i < numInputs; ++i) {
        auto name = fmt::format("input{}", i);
        auto& input = inputs[name];
        input.setBlocking(false);
        input.setMaxSize(4);
        inputNames.push_back(std::move(name));
    }

    return std::static_pointer_cast<Stitching>(shared_from_this());
}

std::shared_ptr<Stitching> Stitching::build(const std::vector<Node::Output*>& sources) {
    build(sources.size());

    for(size_t i = 0; i < sources.size(); ++i) {
        DAI_CHECK_V(sources[i] != nullptr, "Stitching source {} is null", i);
        sources[i]->link(inputs[inputNames[i]]);
    }

    return std::static_pointer_cast<Stitching>(shared_from_this());
}

size_t Stitching::getNumInputs() const {
    return inputNames.size();
}

void Stitching::setSyncThreshold(std::chrono::nanoseconds syncThreshold) {
    sync->setSyncThreshold(syncThreshold);
}

void Stitching::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool Stitching::runOnHost() const {
    return runOnHostVar;
}

void Stitching::setMode(Mode mode) {
    properties.mode = mode;
    invalidateHostState();
}

Stitching::Mode Stitching::getMode() const {
    return properties.mode;
}

void Stitching::setPlane(const Plane& plane) {
    DAI_CHECK_V(plane.normal.x != 0.0f || plane.normal.y != 0.0f || plane.normal.z != 0.0f, "The plane normal must not be a zero vector");
    properties.plane = plane;
    invalidateHostState();
}

void Stitching::setPlane(const Point3f& point, const Point3f& normal, LengthUnit unit) {
    setPlane(Plane{point, normal, unit});
}

std::optional<Stitching::Plane> Stitching::getPlane() const {
    return properties.plane;
}

void Stitching::setView(const VirtualCamera& view) {
    DAI_CHECK_V(view.width > 0 && view.height > 0, "The view must not be empty, got {}x{} pixels", view.width, view.height);
    DAI_CHECK_V(view.intrinsics[0][0] > 0.0f && view.intrinsics[1][1] > 0.0f, "The view needs a positive focal length");
    properties.view = view;
    invalidateHostState();
}

void Stitching::setViewAuto() {
    properties.view.reset();
    invalidateHostState();
}

std::optional<Stitching::VirtualCamera> Stitching::getView() const {
    return properties.view;
}

void Stitching::setMaxViewSize(uint32_t width, uint32_t height) {
    DAI_CHECK_V(width > 0 && height > 0, "The maximum view size must not be empty, got {}x{} pixels", width, height);
    properties.maxViewWidth = width;
    properties.maxViewHeight = height;
    invalidateHostState();
}

void Stitching::setMaxRange(float range, LengthUnit unit) {
    DAI_CHECK_V(range > 0.0f, "The maximum range must be positive, got {}", range);
    properties.maxRange = range * getDistanceUnitScale(LengthUnit::CENTIMETER, unit);
    invalidateHostState();
}

float Stitching::getMaxRange(LengthUnit unit) const {
    return properties.maxRange * getDistanceUnitScale(unit, LengthUnit::CENTIMETER);
}

void Stitching::setMinIncidenceAngle(float degrees) {
    DAI_CHECK_V(degrees >= 0.0f && degrees < 90.0f, "The minimum incidence angle must be within [0, 90) degrees, got {}", degrees);
    properties.minIncidenceAngle = degrees;
    invalidateHostState();
}

float Stitching::getMinIncidenceAngle() const {
    return properties.minIncidenceAngle;
}

void Stitching::setCameraModel(CameraModel model) {
    properties.cameraModel = model;
    invalidateHostState();
}

Stitching::CameraModel Stitching::getCameraModel() const {
    return properties.cameraModel;
}

void Stitching::setContinuous(bool continuous) {
    properties.continuous = continuous;
    invalidateHostState();
}

bool Stitching::getContinuous() const {
    return properties.continuous;
}

void Stitching::setEstimationFrames(uint32_t frames) {
    DAI_CHECK_V(frames >= 1, "Stitching needs at least one estimation frame");
    properties.estimationFrames = frames;
    invalidateHostState();
}

uint32_t Stitching::getEstimationFrames() const {
    return properties.estimationFrames;
}

void Stitching::setMaxPanoramaSize(uint32_t width, uint32_t height) {
    DAI_CHECK_V(width > 0 && height > 0, "The maximum panorama size must not be empty, got {}x{} pixels", width, height);
    properties.maxPanoramaWidth = width;
    properties.maxPanoramaHeight = height;
}

void Stitching::resetTransform() {
    invalidateHostState();
}

void Stitching::setPanoConfidenceThreshold(double threshold) {
    properties.panoConfidenceThreshold = threshold;
    invalidateHostState();
}

double Stitching::getPanoConfidenceThreshold() const {
    return properties.panoConfidenceThreshold;
}

void Stitching::setSeamFinder(SeamFinder finder) {
    properties.seamFinder = finder;
    invalidateHostState();
}

Stitching::SeamFinder Stitching::getSeamFinder() const {
    return properties.seamFinder;
}

}  // namespace node
}  // namespace dai
