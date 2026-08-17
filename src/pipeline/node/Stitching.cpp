#include "depthai/pipeline/node/host/Stitching.hpp"

#include <stdexcept>
#include <utility>

#include "StitchingPlatform.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {

namespace node {

Stitching::Stitching() = default;

Stitching::Stitching(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, Stitching, StitchingProperties>(std::move(props)) {
    initializeInputNames(properties.numInputs);
}

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

    initializeInputNames(numInputs);
    properties.numInputs = static_cast<std::uint32_t>(numInputs);
    for(const auto& name : inputNames) {
        auto& input = inputs[name];
        input.setBlocking(false);
        input.setMaxSize(4);
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

void Stitching::initializeInputNames(size_t numInputs) {
    inputNames.reserve(numInputs);
    for(size_t i = 0; i < numInputs; ++i) {
        inputNames.push_back(fmt::format("input{}", i));
    }
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
