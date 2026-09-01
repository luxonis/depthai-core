#include "depthai/pipeline/node/host/MultiDeviceCalibration.hpp"

#include <DynamicCalibration.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/core.hpp>
#endif

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationControl.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "pipeline/node/DynamicCalibrationUtils.hpp"
#include "pipeline/node/MultiDeviceScaleEstimation.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

namespace {

bool isConcreteSocket(CameraBoardSocket socket) {
    return socket != CameraBoardSocket::AUTO && static_cast<int32_t>(socket) >= static_cast<int32_t>(CameraBoardSocket::CAM_A)
           && static_cast<int32_t>(socket) <= static_cast<int32_t>(CameraBoardSocket::CBA);
}

struct Endpoint {
    std::string deviceId;
    CameraBoardSocket socket = CameraBoardSocket::AUTO;

    bool operator<(const Endpoint& other) const {
        return std::tie(deviceId, socket) < std::tie(other.deviceId, other.socket);
    }

    bool operator==(const Endpoint& other) const {
        return deviceId == other.deviceId && socket == other.socket;
    }
};

struct InitialGuess {
    Endpoint from;
    Endpoint to;
    Extrinsics guess;
};

struct KnownDistance {
    Endpoint from;
    Endpoint to;
    float meters = 0.0f;
};

struct StereoPair {
    std::string deviceId;
    CameraBoardSocket leftSocket = CameraBoardSocket::AUTO;
    CameraBoardSocket rightSocket = CameraBoardSocket::AUTO;
};

std::string cameraInputName(const std::string& deviceId, CameraBoardSocket socket) {
    return "camera_" + deviceId + "_" + toString(socket);
}

bool finitePoint(const Point3f& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

}  // namespace

class MultiDeviceCalibration::Impl {
   public:
    enum class State { IDLE, COLLECTING, COMPLETE };

    struct Camera {
        std::string deviceId;
        CameraBoardSocket socket = CameraBoardSocket::AUTO;
        std::string inputName;
        CameraBoardSocket localOrigin = CameraBoardSocket::AUTO;
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        std::pair<unsigned, unsigned> resolution{0, 0};
        ImgTransformation transformation;
        std::shared_ptr<dcl::CameraSensorHandle> sensor;
#endif
    };

    mutable std::mutex mutex;
    std::atomic<State> state{State::IDLE};
    std::size_t sampleCount = 10;
    std::vector<Camera> cameras;
    std::map<std::string, CalibrationHandler> calibrationOverrides;
    std::map<std::string, CalibrationHandler> calibrations;
    std::vector<KnownDistance> knownDistances;
    std::vector<InitialGuess> initialGuesses;
    std::vector<StereoPair> stereoPairs;

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    std::unique_ptr<dcl::DynamicCalibration> dcl;
    std::shared_ptr<dcl::Device> dclDevice;
    std::map<std::string, std::vector<cv::Mat>> sampleImages;
    std::size_t collectedGroups = 0;
    bool dclInitialized = false;
#endif

    void requireIdle(const char* setting) const {
        DAI_CHECK_V(state == State::IDLE, "MultiDeviceCalibration {} cannot be changed unless the node is idle", setting);
    }

    void clearCycle() {
        calibrations.clear();
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        dcl.reset();
        dclDevice.reset();
        sampleImages.clear();
        collectedGroups = 0;
        dclInitialized = false;
        for(auto& camera : cameras) {
            camera.localOrigin = CameraBoardSocket::AUTO;
            camera.resolution = {0, 0};
            camera.transformation = {};
            camera.sensor.reset();
        }
#else
        for(auto& camera : cameras) camera.localOrigin = CameraBoardSocket::AUTO;
#endif
    }
};

MultiDeviceCalibration::MultiDeviceCalibration() : pimpl(spimpl::make_unique_impl<Impl>()) {}

MultiDeviceCalibration::~MultiDeviceCalibration() = default;

void MultiDeviceCalibration::buildInternal() {
    sync->out.link(syncInput);
    sync->setRunOnHost(true);
    for(auto& entry : inputs) {
        entry.second.setBlocking(false);
        entry.second.setMaxSize(2);
    }
}

void MultiDeviceCalibration::addCamera(std::string deviceId, CameraBoardSocket socket, Node::Output& cameraOutput) {
    DAI_CHECK_V(!deviceId.empty(), "MultiDeviceCalibration camera device ID must not be empty");
    DAI_CHECK_V(isConcreteSocket(socket), "MultiDeviceCalibration camera socket must be concrete, got {}", toString(socket));

    std::lock_guard<std::mutex> lock(pimpl->mutex);
    pimpl->requireIdle("camera registration");
    const auto duplicate = std::find_if(
        pimpl->cameras.begin(), pimpl->cameras.end(), [&](const Impl::Camera& camera) { return camera.deviceId == deviceId && camera.socket == socket; });
    DAI_CHECK_V(duplicate == pimpl->cameras.end(), "MultiDeviceCalibration camera ({}, {}) was already registered", deviceId, toString(socket));

    Impl::Camera camera;
    camera.deviceId = std::move(deviceId);
    camera.socket = socket;
    camera.inputName = cameraInputName(camera.deviceId, socket);
    pimpl->cameras.push_back(camera);
    auto& input = inputs[pimpl->cameras.back().inputName];
    input.setBlocking(false);
    input.setMaxSize(2);
    cameraOutput.link(input);
}

void MultiDeviceCalibration::setSampleCount(std::size_t sampleCount) {
    DAI_CHECK_V(sampleCount >= 1, "MultiDeviceCalibration sampleCount must be at least 1, got {}", sampleCount);
    std::lock_guard<std::mutex> lock(pimpl->mutex);
    pimpl->requireIdle("sampleCount");
    pimpl->sampleCount = sampleCount;
}

std::size_t MultiDeviceCalibration::getSampleCount() const {
    std::lock_guard<std::mutex> lock(pimpl->mutex);
    return pimpl->sampleCount;
}

void MultiDeviceCalibration::setKnownDistance(
    std::string fromDeviceId, CameraBoardSocket fromSocket, std::string toDeviceId, CameraBoardSocket toSocket, float distance, LengthUnit unit) {
    DAI_CHECK_V(!fromDeviceId.empty() && !toDeviceId.empty(), "MultiDeviceCalibration known-distance device IDs must not be empty");
    DAI_CHECK_V(isConcreteSocket(fromSocket) && isConcreteSocket(toSocket), "MultiDeviceCalibration known-distance sockets must be concrete");
    DAI_CHECK_V(fromDeviceId != toDeviceId, "MultiDeviceCalibration known distance must connect different devices");
    DAI_CHECK_V(unit != LengthUnit::CUSTOM, "MultiDeviceCalibration known distance uses unsupported CUSTOM length units");
    DAI_CHECK_V(std::isfinite(distance) && distance > 0.0f, "MultiDeviceCalibration known distance must be finite and positive, got {}", distance);
    const auto meters = distance * getDistanceUnitScale(LengthUnit::METER, unit);
    DAI_CHECK_V(std::isfinite(meters) && meters > 0.0f, "MultiDeviceCalibration known distance conversion produced an invalid value");

    std::lock_guard<std::mutex> lock(pimpl->mutex);
    pimpl->requireIdle("known distance");
    const Endpoint from{std::move(fromDeviceId), fromSocket};
    const Endpoint to{std::move(toDeviceId), toSocket};
    const auto existing = std::find_if(pimpl->knownDistances.begin(), pimpl->knownDistances.end(), [&](const KnownDistance& known) {
        return (known.from == from && known.to == to) || (known.from == to && known.to == from);
    });
    if(existing != pimpl->knownDistances.end()) {
        existing->from = from;
        existing->to = to;
        existing->meters = meters;
    } else {
        pimpl->knownDistances.push_back({from, to, meters});
    }
}

void MultiDeviceCalibration::setInitialGuess(
    std::string fromDeviceId, CameraBoardSocket fromSocket, std::string toDeviceId, CameraBoardSocket toSocket, const Extrinsics& guess) {
    DAI_CHECK_V(!fromDeviceId.empty() && !toDeviceId.empty(), "MultiDeviceCalibration initial-guess device IDs must not be empty");
    DAI_CHECK_V(isConcreteSocket(fromSocket) && isConcreteSocket(toSocket), "MultiDeviceCalibration initial-guess sockets must be concrete");
    DAI_CHECK_V(fromDeviceId != toDeviceId, "MultiDeviceCalibration initial guess must connect different devices");
    DAI_CHECK_V(guess.toDeviceId == toDeviceId,
                "MultiDeviceCalibration initial guess destination device '{}' does not match explicit destination '{}'",
                guess.toDeviceId,
                toDeviceId);
    DAI_CHECK_V(guess.toCameraSocket == toSocket,
                "MultiDeviceCalibration initial guess destination socket {} does not match explicit destination {}",
                toString(guess.toCameraSocket),
                toString(toSocket));
    DAI_CHECK_V(isConcreteSocket(guess.toCameraSocket), "MultiDeviceCalibration initial guess destination socket must be concrete");
    DAI_CHECK_V(finitePoint(guess.translation), "MultiDeviceCalibration initial guess translation must be finite");
    DAI_CHECK_V(guess.lengthUnit != LengthUnit::CUSTOM, "MultiDeviceCalibration initial guess uses unsupported CUSTOM length units");
    try {
        matrix::validateRotationMatrix3x3(guess.rotationMatrix);
    } catch(const std::exception&) {
        throw std::invalid_argument("MultiDeviceCalibration initial guess rotation must be a finite proper rigid rotation");
    }

    std::lock_guard<std::mutex> lock(pimpl->mutex);
    pimpl->requireIdle("initial guess");
    const Endpoint from{std::move(fromDeviceId), fromSocket};
    const Endpoint to{std::move(toDeviceId), toSocket};
    const auto existing = std::find_if(
        pimpl->initialGuesses.begin(), pimpl->initialGuesses.end(), [&](const InitialGuess& initial) { return initial.from == from && initial.to == to; });
    if(existing != pimpl->initialGuesses.end()) {
        existing->guess = guess;
    } else {
        pimpl->initialGuesses.push_back({from, to, guess});
    }
}

void MultiDeviceCalibration::setStereoPair(std::string deviceId, CameraBoardSocket leftSocket, CameraBoardSocket rightSocket) {
    DAI_CHECK_V(!deviceId.empty(), "MultiDeviceCalibration stereo-pair device ID must not be empty");
    DAI_CHECK_V(isConcreteSocket(leftSocket) && isConcreteSocket(rightSocket), "MultiDeviceCalibration stereo-pair sockets must be concrete");
    DAI_CHECK_V(leftSocket != rightSocket, "MultiDeviceCalibration stereo pair must contain two different sockets");

    std::lock_guard<std::mutex> lock(pimpl->mutex);
    pimpl->requireIdle("stereo pair");
    const auto existing = std::find_if(pimpl->stereoPairs.begin(), pimpl->stereoPairs.end(), [&](const StereoPair& pair) { return pair.deviceId == deviceId; });
    const StereoPair pair{std::move(deviceId), leftSocket, rightSocket};
    if(existing != pimpl->stereoPairs.end()) {
        *existing = pair;
    } else {
        pimpl->stereoPairs.push_back(pair);
    }
}

void MultiDeviceCalibration::setDeviceCalibration(std::string deviceId, const CalibrationHandler& calibration) {
    DAI_CHECK_V(!deviceId.empty(), "MultiDeviceCalibration device calibration needs a non-empty device ID");
    std::lock_guard<std::mutex> lock(pimpl->mutex);
    pimpl->requireIdle("device calibration");
    pimpl->calibrationOverrides.insert_or_assign(std::move(deviceId), calibration);
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

namespace {

using Transform = std::vector<std::vector<float>>;

Transform identityTransform() {
    return {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
}

Transform inverted(Transform transform) {
    matrix::invertSe3Matrix4x4InPlace(transform);
    return transform;
}

std::vector<float> cameraCenter(const Transform& transform) {
    const auto rotation = matrix::extractRotationMatrix(transform);
    const auto translation = matrix::extractTranslationVector(transform);
    std::vector<float> center(3, 0.0f);
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) center[i] -= rotation[j][i] * translation[j];
    }
    return center;
}

bool isFiniteTransform(const Transform& transform) {
    if(transform.size() != 4 || std::any_of(transform.begin(), transform.end(), [](const auto& row) { return row.size() != 4; })) {
        return false;
    }
    for(const auto& row : transform) {
        for(const auto value : row) {
            if(!std::isfinite(value)) return false;
        }
    }
    try {
        matrix::validateRotationMatrix3x3(matrix::extractRotationMatrix(transform));
    } catch(const std::exception&) {
        return false;
    }
    return std::abs(transform[3][0]) < 1e-5f && std::abs(transform[3][1]) < 1e-5f && std::abs(transform[3][2]) < 1e-5f
           && std::abs(transform[3][3] - 1.0f) < 1e-5f;
}

}  // namespace

bool MultiDeviceCalibration::initializeCycle(const std::shared_ptr<MessageGroup>& firstGroup, std::string& error) {
    std::set<std::string> deviceIds;
    for(const auto& camera : pimpl->cameras) deviceIds.insert(camera.deviceId);
    if(pimpl->cameras.size() < 2) {
        error = "at least two cameras must be registered";
        return false;
    }
    if(deviceIds.size() < 2) {
        error = "at least two different devices must be registered";
        return false;
    }

    pimpl->calibrations = pimpl->calibrationOverrides;
    const auto addLiveCalibration = [&](const std::shared_ptr<Device>& device) {
        if(!device) return;
        const auto deviceId = device->getDeviceId();
        if(deviceIds.count(deviceId) == 0 || pimpl->calibrations.count(deviceId) != 0) return;
        pimpl->calibrations.emplace(deviceId, device->getCalibration());
    };
    for(const auto& [unused, device] : inputs.getSourceDevices()) {
        (void)unused;
        addLiveCalibration(device);
    }
    for(const auto& device : getParentPipeline().getDevices()) addLiveCalibration(device);

    for(const auto& deviceId : deviceIds) {
        if(pimpl->calibrations.count(deviceId) == 0) {
            error = "no explicit or live calibration is available for device " + deviceId;
            return false;
        }
    }

    for(auto& camera : pimpl->cameras) {
        try {
            CameraBoardSocket origin = CameraBoardSocket::AUTO;
            pimpl->calibrations.at(camera.deviceId).getExtrinsicsToOrigin(camera.socket, false, origin);
            if(!isConcreteSocket(origin)) {
                error = "device " + camera.deviceId + " has no concrete local calibration origin for " + toString(camera.socket);
                return false;
            }
            camera.localOrigin = origin;
        } catch(const std::exception& ex) {
            error = "cannot determine the local calibration origin for " + camera.deviceId + "/" + toString(camera.socket) + ": " + ex.what();
            return false;
        }
    }

    for(const auto& initial : pimpl->initialGuesses) {
        const auto fromIt = std::find_if(pimpl->cameras.begin(), pimpl->cameras.end(), [&](const Impl::Camera& camera) {
            return camera.deviceId == initial.from.deviceId && camera.localOrigin == initial.from.socket;
        });
        const auto toIt = std::find_if(pimpl->cameras.begin(), pimpl->cameras.end(), [&](const Impl::Camera& camera) {
            return camera.deviceId == initial.to.deviceId && camera.localOrigin == initial.to.socket;
        });
        if(fromIt == pimpl->cameras.end() || toIt == pimpl->cameras.end()) {
            error = "initial guesses must use each endpoint device's local calibration-origin socket";
            return false;
        }
    }

    for(auto& camera : pimpl->cameras) {
        const auto frame = firstGroup->get<ImgFrame>(camera.inputName);
        if(!frame) {
            error = "the first synchronized group is missing " + camera.inputName;
            return false;
        }
        const auto image = frame->getCvFrame();
        if(image.empty()) {
            error = "the first synchronized group contains an empty image for " + camera.inputName;
            return false;
        }
        camera.resolution = {frame->getWidth(), frame->getHeight()};
        camera.transformation = frame->getTransformation();
    }

    pimpl->dcl = std::make_unique<dcl::DynamicCalibration>();
    pimpl->dclDevice = pimpl->dcl->addDevice();

    std::optional<MultiDeviceCalibrationHandler> initialHandler;
    if(!pimpl->initialGuesses.empty()) {
        std::vector<MultiDeviceExtrinsics> edges;
        edges.reserve(pimpl->initialGuesses.size());
        for(const auto& initial : pimpl->initialGuesses) {
            MultiDeviceExtrinsics edge;
            edge.fromDeviceId = initial.from.deviceId;
            edge.fromSocket = initial.from.socket;
            edge.extrinsics = initial.guess;
            edges.push_back(std::move(edge));
        }
        try {
            initialHandler.emplace(std::move(edges));
        } catch(const std::exception& ex) {
            error = std::string("initial-guess graph is invalid: ") + ex.what();
            return false;
        }
    }

    for(auto& camera : pimpl->cameras) {
        Transform originToWorld = identityTransform();
        if(initialHandler) {
            const auto originPose = initialHandler->getExtrinsicsToOrigin(camera.deviceId, camera.localOrigin);
            if(originPose) {
                originToWorld = matrix::toVecMatrix4x4(originPose->getTransformationMatrix(false, LengthUnit::METER));
            }
        }

        const auto originToCamera = pimpl->calibrations.at(camera.deviceId).getCameraExtrinsics(camera.localOrigin, camera.socket, false, LengthUnit::METER);
        const auto cameraToWorld = matrix::matMul(originToCamera, inverted(originToWorld));
        const auto calibration = DclUtils::createDclCalibration(camera.transformation.getIntrinsicMatrix(),
                                                                camera.transformation.getDistortionCoefficients(),
                                                                matrix::extractRotationMatrix(cameraToWorld),
                                                                matrix::extractTranslationVector(cameraToWorld),
                                                                camera.transformation.getDistortionModel());
        camera.sensor = pimpl->dcl->addSensor(pimpl->dclDevice, calibration, dcl::resolution_t{camera.resolution.first, camera.resolution.second});
    }

    pimpl->dclInitialized = true;
    return true;
}

bool MultiDeviceCalibration::loadCompleteGroup(const std::shared_ptr<MessageGroup>& group, std::string& error) {
    if(!group || !pimpl->dclInitialized) {
        error = "DCL is not initialized";
        return false;
    }

    dcl::DeviceImageList images;
    images.reserve(pimpl->cameras.size());
    std::vector<std::pair<std::string, cv::Mat>> retained;
    retained.reserve(pimpl->cameras.size());
    dcl::timestamp_t timestamp = 0;
    bool timestampInitialized = false;
    for(const auto& camera : pimpl->cameras) {
        const auto frame = group->get<ImgFrame>(camera.inputName);
        if(!frame) {
            error = "synchronized group is incomplete";
            return false;
        }
        const auto image = frame->getCvFrame();
        if(image.empty()) {
            error = "synchronized group contains an empty image for " + camera.inputName;
            return false;
        }
        if(!timestampInitialized) {
            timestamp = static_cast<dcl::timestamp_t>(frame->getTimestamp().time_since_epoch().count());
            timestampInitialized = true;
        }
        images.emplace_back(camera.sensor, DclUtils::cvMatToImageData(image));
        retained.emplace_back(camera.inputName, image.clone());
    }

    const auto loaded = pimpl->dcl->loadImages(images, timestamp);
    if(!loaded.passed()) {
        error = loaded.errorMessage();
        return false;
    }
    for(auto& [inputName, image] : retained) pimpl->sampleImages[inputName].push_back(std::move(image));
    ++pimpl->collectedGroups;
    return true;
}

#endif

namespace {

void sendFailure(MultiDeviceCalibration& node, const std::string& info, const std::string& reason) {
    auto result = std::make_shared<MultiDeviceCalibrationResult>();
    result->handler.reset();
    result->passed = false;
    result->complete = false;
    result->info = info;
    MultiDeviceCalibrationResult::EdgeDiagnostic diagnostic;
    diagnostic.rejectionReason = reason;
    result->diagnostics.push_back(std::move(diagnostic));
    node.calibrationOutput.send(std::move(result));
}

}  // namespace

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

namespace {

struct CandidateEdge {
    std::string fromDeviceId;
    CameraBoardSocket fromSocket = CameraBoardSocket::AUTO;
    std::string toDeviceId;
    CameraBoardSocket toSocket = CameraBoardSocket::AUTO;
    Transform transform;
    MultiDeviceCalibrationResult::EdgeDiagnostic diagnostic;
};

struct ScaleInfo {
    bool valid = false;
    std::string source;
    double targetDistanceMeters = 0.0;
    double residualMeters = 0.0;
    Endpoint destinationCamera;
    Endpoint sourceCamera;
    std::string reason;
};

struct StereoView {
    std::vector<cv::Mat> left;
    std::vector<cv::Mat> right;
    cv::Matx33d intrinsicsLeft = cv::Matx33d::eye();
    cv::Matx33d intrinsicsRight = cv::Matx33d::eye();
    cv::Matx44d leftFromRight = cv::Matx44d::eye();
    std::vector<float> distortionLeft;
    std::vector<float> distortionRight;
};

struct StereoScaleEstimate {
    bool valid = false;
    double distanceMeters = 0.0;
    double residualMeters = 0.0;
    std::string reason;
};

StereoScaleEstimate estimateStereoScale(const StereoView& first, const StereoView& second) {
    impl::StereoDeviceViews firstViews;
    firstViews.left = first.left;
    firstViews.right = first.right;
    firstViews.intrinsicsLeft = first.intrinsicsLeft;
    firstViews.intrinsicsRight = first.intrinsicsRight;
    firstViews.leftFromRight = first.leftFromRight;
    firstViews.distortionLeft.assign(first.distortionLeft.begin(), first.distortionLeft.end());
    firstViews.distortionRight.assign(first.distortionRight.begin(), first.distortionRight.end());

    impl::StereoDeviceViews secondViews;
    secondViews.left = second.left;
    secondViews.right = second.right;
    secondViews.intrinsicsLeft = second.intrinsicsLeft;
    secondViews.intrinsicsRight = second.intrinsicsRight;
    secondViews.leftFromRight = second.leftFromRight;
    secondViews.distortionLeft.assign(second.distortionLeft.begin(), second.distortionLeft.end());
    secondViews.distortionRight.assign(second.distortionRight.begin(), second.distortionRight.end());

    const auto recovered = impl::estimateInterDeviceScale(firstViews, secondViews);
    StereoScaleEstimate result;
    result.valid = recovered.observable;
    result.distanceMeters = recovered.distanceMeters;
    result.residualMeters = recovered.rmseMeters;
    result.reason = recovered.note;
    return result;
}

std::optional<float> knownDistanceFor(const std::vector<KnownDistance>& knownDistances,
                                      const std::string& deviceA,
                                      CameraBoardSocket socketA,
                                      const std::string& deviceB,
                                      CameraBoardSocket socketB) {
    for(const auto& known : knownDistances) {
        if((known.from.deviceId == deviceA && known.from.socket == socketA && known.to.deviceId == deviceB && known.to.socket == socketB)
           || (known.from.deviceId == deviceB && known.from.socket == socketB && known.to.deviceId == deviceA && known.to.socket == socketA)) {
            return known.meters;
        }
    }
    return std::nullopt;
}

Extrinsics makeLocalOriginEdge(const Transform& transform, const std::string& toDeviceId, CameraBoardSocket toSocket) {
    const auto translation = matrix::extractTranslationVector(transform);
    Extrinsics edge(matrix::extractRotationMatrix(transform), Point3f(translation[0], translation[1], translation[2]), toSocket, LengthUnit::METER);
    edge.toDeviceId = toDeviceId;
    return edge;
}

struct ScaledTransform {
    bool valid = false;
    Transform transform;
    std::string reason;
};

ScaledTransform scaleTransformToCameraDistance(const Transform& raw,
                                               const CalibrationHandler& destinationCalibration,
                                               CameraBoardSocket destinationOrigin,
                                               CameraBoardSocket destinationCamera,
                                               const CalibrationHandler& sourceCalibration,
                                               CameraBoardSocket sourceOrigin,
                                               CameraBoardSocket sourceCamera,
                                               double targetDistanceMeters) {
    ScaledTransform result;
    if(!isFiniteTransform(raw) || !std::isfinite(targetDistanceMeters) || targetDistanceMeters <= 0.0) {
        result.reason = "invalid pose or target metric distance";
        return result;
    }

    try {
        const auto destinationOffset = destinationOrigin == destinationCamera
                                           ? identityTransform()
                                           : destinationCalibration.getCameraExtrinsics(destinationOrigin, destinationCamera, false, LengthUnit::METER);
        const auto sourceOffset =
            sourceOrigin == sourceCamera ? identityTransform() : sourceCalibration.getCameraExtrinsics(sourceOrigin, sourceCamera, false, LengthUnit::METER);
        const auto centerDestination = cameraCenter(destinationOffset);
        const auto centerSource = cameraCenter(matrix::matMul(sourceOffset, inverted(raw)));
        const auto translation = matrix::extractTranslationVector(raw);

        std::vector<float> offset(3, 0.0f);
        for(int i = 0; i < 3; ++i) offset[i] = centerSource[i] - translation[i] - centerDestination[i];
        const auto dot = [](const std::vector<float>& left, const std::vector<float>& right) {
            return std::inner_product(left.begin(), left.end(), right.begin(), 0.0);
        };
        const double a = dot(translation, translation);
        const double b = 2.0 * dot(translation, offset);
        const double c = dot(offset, offset) - targetDistanceMeters * targetDistanceMeters;
        const double discriminant = b * b - 4.0 * a * c;
        if(a <= 1e-12 || discriminant < 0.0 || !std::isfinite(discriminant)) {
            result.reason = "metric distance cannot be matched by the estimated translation direction";
            return result;
        }
        const double rootDiscriminant = std::sqrt(discriminant);
        const double first = (-b + rootDiscriminant) / (2.0 * a);
        const double second = (-b - rootDiscriminant) / (2.0 * a);
        std::vector<double> positiveRoots;
        for(const auto root : {first, second}) {
            if(std::isfinite(root) && root > 0.0
               && std::none_of(positiveRoots.begin(), positiveRoots.end(), [root](double existing) { return std::abs(existing - root) <= 1e-6; })) {
                positiveRoots.push_back(root);
            }
        }
        if(positiveRoots.size() != 1) {
            result.reason = positiveRoots.empty() ? "metric distance implies no positive scale" : "metric distance implies an ambiguous positive scale";
            return result;
        }
        result.transform = raw;
        for(int i = 0; i < 3; ++i) result.transform[i][3] *= static_cast<float>(positiveRoots.front());
        result.valid = isFiniteTransform(result.transform);
        if(!result.valid) result.reason = "scaled pose is not a finite proper rigid transform";
    } catch(const std::exception& ex) {
        result.reason = std::string("failed to normalize metric scale: ") + ex.what();
    }
    return result;
}

Transform transformForCandidate(const CandidateEdge& edge, const std::string& fromDeviceId, const std::string& toDeviceId) {
    if(edge.fromDeviceId == fromDeviceId && edge.toDeviceId == toDeviceId) return edge.transform;
    return inverted(edge.transform);
}

}  // namespace

void MultiDeviceCalibration::estimateAndEmit() {
    auto result = std::make_shared<MultiDeviceCalibrationResult>();
    std::vector<CandidateEdge> candidates;
    std::vector<std::string> notes;
    std::set<std::pair<std::string, std::string>> imageConnectedDevicePairs;
    double dataConfidence = 0.0;

    const auto cameraOrigin = [&]() {
        std::map<std::string, CameraBoardSocket> origins;
        for(const auto& camera : pimpl->cameras) {
            const auto [it, inserted] = origins.emplace(camera.deviceId, camera.localOrigin);
            if(!inserted && it->second != camera.localOrigin) {
                notes.push_back("device " + camera.deviceId + " has inconsistent local calibration origins");
            }
        }
        return origins;
    }();

    std::vector<std::size_t> component(pimpl->cameras.size());
    std::iota(component.begin(), component.end(), 0);
    const auto findRoot = [&component](std::size_t index) {
        std::size_t root = index;
        while(component[root] != root) root = component[root];
        while(component[index] != index) {
            const auto next = component[index];
            component[index] = root;
            index = next;
        }
        return root;
    };
    const auto unite = [&component, &findRoot](std::size_t left, std::size_t right) {
        const auto leftRoot = findRoot(left);
        const auto rightRoot = findRoot(right);
        if(leftRoot != rightRoot) component[leftRoot] = rightRoot;
    };

    for(std::size_t i = 0; i < pimpl->cameras.size(); ++i) {
        for(std::size_t j = i + 1; j < pimpl->cameras.size(); ++j) {
            auto confidence = pimpl->dcl->computeDataConfidence(pimpl->cameras[i].sensor, pimpl->cameras[j].sensor);
            if(confidence.passed() && std::isfinite(confidence.value) && confidence.value > 0.0) unite(i, j);
        }
    }

    std::map<std::size_t, std::vector<std::size_t>> components;
    for(std::size_t i = 0; i < pimpl->cameras.size(); ++i) components[findRoot(i)].push_back(i);

    for(const auto& [unusedRoot, indices] : components) {
        (void)unusedRoot;
        std::set<std::string> componentDevices;
        for(const auto index : indices) componentDevices.insert(pimpl->cameras[index].deviceId);
        if(componentDevices.size() < 2) continue;
        for(auto first = componentDevices.begin(); first != componentDevices.end(); ++first) {
            for(auto second = std::next(first); second != componentDevices.end(); ++second) {
                imageConnectedDevicePairs.emplace(*first, *second);
            }
        }

        std::map<std::pair<std::string, std::string>, ScaleInfo> scaleInfoByPair;
        const auto validFactoryBaseline = [&](const Impl::Camera& left, const Impl::Camera& right) -> std::optional<double> {
            try {
                const auto baseline = pimpl->calibrations.at(left.deviceId).getBaselineDistance(left.socket, right.socket, false, LengthUnit::METER);
                if(std::isfinite(baseline) && baseline > 0.0f) return static_cast<double>(baseline);
            } catch(const std::exception&) {
                // The pair is not a usable factory-calibrated stereo pair.
            }
            return std::nullopt;
        };
        const auto stereoPairFor = [&](const std::string& deviceId) -> std::pair<std::optional<std::pair<std::size_t, std::size_t>>, std::string> {
            const auto explicitPair =
                std::find_if(pimpl->stereoPairs.begin(), pimpl->stereoPairs.end(), [&](const StereoPair& pair) { return pair.deviceId == deviceId; });
            if(explicitPair != pimpl->stereoPairs.end()) {
                const auto left = std::find_if(pimpl->cameras.begin(), pimpl->cameras.end(), [&](const Impl::Camera& camera) {
                    return camera.deviceId == deviceId && camera.socket == explicitPair->leftSocket;
                });
                const auto right = std::find_if(pimpl->cameras.begin(), pimpl->cameras.end(), [&](const Impl::Camera& camera) {
                    return camera.deviceId == deviceId && camera.socket == explicitPair->rightSocket;
                });
                if(left == pimpl->cameras.end() || right == pimpl->cameras.end()) {
                    return {std::nullopt, "explicit stereo pair is not fully registered"};
                }
                if(!validFactoryBaseline(*left, *right)) return {std::nullopt, "explicit stereo pair has no valid factory baseline"};
                return {std::make_pair(static_cast<std::size_t>(std::distance(pimpl->cameras.begin(), left)),
                                       static_cast<std::size_t>(std::distance(pimpl->cameras.begin(), right))),
                        {}};
            }

            std::vector<std::pair<std::size_t, std::size_t>> eligible;
            for(std::size_t leftIndex = 0; leftIndex < pimpl->cameras.size(); ++leftIndex) {
                if(pimpl->cameras[leftIndex].deviceId != deviceId) continue;
                for(std::size_t rightIndex = leftIndex + 1; rightIndex < pimpl->cameras.size(); ++rightIndex) {
                    if(pimpl->cameras[rightIndex].deviceId != deviceId) continue;
                    if(validFactoryBaseline(pimpl->cameras[leftIndex], pimpl->cameras[rightIndex])) eligible.emplace_back(leftIndex, rightIndex);
                }
            }
            if(eligible.size() == 1) return {eligible.front(), {}};
            if(eligible.empty()) return {std::nullopt, "no registered stereo pair has valid factory calibration"};
            return {std::nullopt, "automatic stereo-pair selection is ambiguous"};
        };
        const auto buildStereoView = [&](std::size_t leftIndex, std::size_t rightIndex) {
            StereoView view;
            const auto& left = pimpl->cameras[leftIndex];
            const auto& right = pimpl->cameras[rightIndex];
            const auto leftIntrinsics = left.transformation.getIntrinsicMatrix();
            const auto rightIntrinsics = right.transformation.getIntrinsicMatrix();
            for(int row = 0; row < 3; ++row) {
                for(int column = 0; column < 3; ++column) {
                    view.intrinsicsLeft(row, column) = leftIntrinsics[row][column];
                    view.intrinsicsRight(row, column) = rightIntrinsics[row][column];
                }
            }
            view.distortionLeft = left.transformation.getDistortionCoefficients();
            view.distortionRight = right.transformation.getDistortionCoefficients();
            const auto leftFromRight = inverted(pimpl->calibrations.at(left.deviceId).getCameraExtrinsics(left.socket, right.socket, false, LengthUnit::METER));
            for(int row = 0; row < 4; ++row)
                for(int column = 0; column < 4; ++column) view.leftFromRight(row, column) = leftFromRight[row][column];
            view.left = pimpl->sampleImages.at(left.inputName);
            view.right = pimpl->sampleImages.at(right.inputName);
            return view;
        };

        for(auto first = componentDevices.begin(); first != componentDevices.end(); ++first) {
            for(auto second = std::next(first); second != componentDevices.end(); ++second) {
                ScaleInfo scaleInfo;
                for(const auto firstIndex : indices) {
                    const auto& firstCamera = pimpl->cameras[firstIndex];
                    if(firstCamera.deviceId != *first) continue;
                    for(const auto secondIndex : indices) {
                        const auto& secondCamera = pimpl->cameras[secondIndex];
                        if(secondCamera.deviceId != *second) continue;
                        if(const auto distance =
                               knownDistanceFor(pimpl->knownDistances, firstCamera.deviceId, firstCamera.socket, secondCamera.deviceId, secondCamera.socket)) {
                            scaleInfo.valid = true;
                            scaleInfo.source = "known distance";
                            scaleInfo.targetDistanceMeters = *distance;
                            scaleInfo.destinationCamera = {firstCamera.deviceId, firstCamera.socket};
                            scaleInfo.sourceCamera = {secondCamera.deviceId, secondCamera.socket};
                            break;
                        }
                    }
                    if(scaleInfo.valid) break;
                }
                if(!scaleInfo.valid) {
                    const auto firstStereo = stereoPairFor(*first);
                    const auto secondStereo = stereoPairFor(*second);
                    if(firstStereo.first && secondStereo.first) {
                        const auto firstView = buildStereoView(firstStereo.first->first, firstStereo.first->second);
                        const auto secondView = buildStereoView(secondStereo.first->first, secondStereo.first->second);
                        const auto stereoScale = estimateStereoScale(firstView, secondView);
                        if(stereoScale.valid) {
                            scaleInfo.valid = true;
                            scaleInfo.source = "stereo";
                            scaleInfo.targetDistanceMeters = stereoScale.distanceMeters;
                            scaleInfo.residualMeters = stereoScale.residualMeters;
                            scaleInfo.destinationCamera = {pimpl->cameras[firstStereo.first->first].deviceId, pimpl->cameras[firstStereo.first->first].socket};
                            scaleInfo.sourceCamera = {pimpl->cameras[secondStereo.first->first].deviceId, pimpl->cameras[secondStereo.first->first].socket};
                        } else {
                            scaleInfo.reason = "stereo scale recovery failed: " + stereoScale.reason;
                        }
                    } else {
                        scaleInfo.reason = firstStereo.second + "; " + secondStereo.second;
                    }
                }
                if(!scaleInfo.valid) notes.push_back("no usable metric scale between " + *first + " and " + *second + ": " + scaleInfo.reason);
                scaleInfoByPair.emplace(std::make_pair(*first, *second), std::move(scaleInfo));
            }
        }

        std::vector<std::shared_ptr<const dcl::CameraSensorHandle>> sensors;
        sensors.reserve(indices.size());
        for(const auto index : indices) sensors.push_back(pimpl->cameras[index].sensor);

        std::vector<dcl::EdgeBaseline> baselines;
        for(std::size_t i = 0; i < indices.size(); ++i) {
            for(std::size_t j = i + 1; j < indices.size(); ++j) {
                const auto& first = pimpl->cameras[indices[i]];
                const auto& second = pimpl->cameras[indices[j]];
                std::optional<float> distance = knownDistanceFor(pimpl->knownDistances, first.deviceId, first.socket, second.deviceId, second.socket);
                if(!distance && first.deviceId == second.deviceId) {
                    try {
                        const auto baseline = pimpl->calibrations.at(first.deviceId).getBaselineDistance(first.socket, second.socket, false, LengthUnit::METER);
                        if(std::isfinite(baseline) && baseline > 0.0f) distance = baseline;
                    } catch(const std::exception&) {
                        // This pair has no usable factory baseline.
                    }
                }
                if(distance) baselines.push_back({i, j, static_cast<double>(*distance)});
            }
        }

        auto dclResult = pimpl->dcl->findNewCalibration(sensors, dcl::PerformanceMode::DEFAULT, false, baselines);
        if(!dclResult.passed()) {
            for(auto first = componentDevices.begin(); first != componentDevices.end(); ++first) {
                for(auto second = std::next(first); second != componentDevices.end(); ++second) {
                    MultiDeviceCalibrationResult::EdgeDiagnostic diagnostic;
                    diagnostic.fromDeviceId = *second;
                    diagnostic.fromSocket = cameraOrigin.at(*second);
                    diagnostic.toDeviceId = *first;
                    diagnostic.toSocket = cameraOrigin.at(*first);
                    diagnostic.rejectionReason = "DynamicCalibration failed: " + dclResult.errorMessage();
                    result->diagnostics.push_back(std::move(diagnostic));
                }
            }
            notes.push_back("DynamicCalibration failed for an image-connected component: " + dclResult.errorMessage());
            continue;
        }
        if(dclResult.value.calibrations.size() != sensors.size()) {
            notes.push_back("DynamicCalibration returned a calibration count mismatch");
            continue;
        }
        if(std::isfinite(dclResult.value.dataConfidence)) dataConfidence = std::max(dataConfidence, dclResult.value.dataConfidence);

        std::map<std::string, Transform> originTransforms;
        for(std::size_t k = 0; k < indices.size(); ++k) {
            const auto& camera = pimpl->cameras[indices[k]];
            const auto cameraTransform = DclUtils::calibrationHandleToTransform(dclResult.value.calibrations[k]);
            const auto originToCamera =
                camera.localOrigin == camera.socket
                    ? identityTransform()
                    : pimpl->calibrations.at(camera.deviceId).getCameraExtrinsics(camera.localOrigin, camera.socket, false, LengthUnit::METER);
            const auto originTransform = matrix::matMul(cameraTransform, inverted(originToCamera));
            const auto existing = originTransforms.find(camera.deviceId);
            if(existing == originTransforms.end()) {
                originTransforms.emplace(camera.deviceId, originTransform);
            } else if(!isFiniteTransform(existing->second) || !isFiniteTransform(originTransform)) {
                notes.push_back("DynamicCalibration returned an invalid pose for device " + camera.deviceId);
            }
        }

        const double sampsonError = dclResult.value.sampsonErrorNew;
        for(auto first = componentDevices.begin(); first != componentDevices.end(); ++first) {
            for(auto second = std::next(first); second != componentDevices.end(); ++second) {
                MultiDeviceCalibrationResult::EdgeDiagnostic diagnostic;
                diagnostic.fromDeviceId = *second;
                diagnostic.fromSocket = cameraOrigin.at(*second);
                diagnostic.toDeviceId = *first;
                diagnostic.toSocket = cameraOrigin.at(*first);
                diagnostic.dclConfidence = dclResult.value.dataConfidence;
                diagnostic.reprojectionError = sampsonError;
                diagnostic.sampsonError = sampsonError;

                const auto scaleInfoIt = scaleInfoByPair.find({*first, *second});
                if(scaleInfoIt == scaleInfoByPair.end() || !scaleInfoIt->second.valid) {
                    diagnostic.rejectionReason = scaleInfoIt == scaleInfoByPair.end()
                                                     ? "no usable metric scale: supply a known camera-center distance or a validated stereo pair"
                                                     : scaleInfoIt->second.reason;
                    result->diagnostics.push_back(std::move(diagnostic));
                    continue;
                }
                const auto& scaleInfo = scaleInfoIt->second;
                diagnostic.scaleSource = scaleInfo.source;
                diagnostic.scaleResidual = scaleInfo.residualMeters;

                const auto fromTransform = originTransforms.find(*second);
                const auto toTransform = originTransforms.find(*first);
                if(fromTransform == originTransforms.end() || toTransform == originTransforms.end() || !isFiniteTransform(fromTransform->second)
                   || !isFiniteTransform(toTransform->second) || !std::isfinite(sampsonError)) {
                    diagnostic.rejectionReason = "DynamicCalibration returned a non-finite or non-rigid pose";
                    result->diagnostics.push_back(std::move(diagnostic));
                    continue;
                }

                const auto rawTransform = matrix::matMul(toTransform->second, inverted(fromTransform->second));
                const auto scaled = scaleTransformToCameraDistance(rawTransform,
                                                                   pimpl->calibrations.at(*first),
                                                                   cameraOrigin.at(*first),
                                                                   scaleInfo.destinationCamera.socket,
                                                                   pimpl->calibrations.at(*second),
                                                                   cameraOrigin.at(*second),
                                                                   scaleInfo.sourceCamera.socket,
                                                                   scaleInfo.targetDistanceMeters);
                if(!scaled.valid) {
                    diagnostic.rejectionReason = scaled.reason;
                    result->diagnostics.push_back(std::move(diagnostic));
                    continue;
                }

                CandidateEdge candidate;
                candidate.fromDeviceId = *second;
                candidate.fromSocket = cameraOrigin.at(*second);
                candidate.toDeviceId = *first;
                candidate.toSocket = cameraOrigin.at(*first);
                candidate.transform = scaled.transform;
                candidate.diagnostic = diagnostic;
                candidate.diagnostic.accepted = true;
                candidates.push_back(std::move(candidate));
            }
        }
    }

    const auto qualityLess = [](const CandidateEdge& left, const CandidateEdge& right) {
        if(left.diagnostic.dclConfidence != right.diagnostic.dclConfidence) return left.diagnostic.dclConfidence > right.diagnostic.dclConfidence;
        if(left.diagnostic.reprojectionError != right.diagnostic.reprojectionError)
            return left.diagnostic.reprojectionError < right.diagnostic.reprojectionError;
        if(left.diagnostic.scaleResidual != right.diagnostic.scaleResidual) return left.diagnostic.scaleResidual < right.diagnostic.scaleResidual;
        return std::tie(left.fromDeviceId, left.fromSocket, left.toDeviceId, left.toSocket)
               < std::tie(right.fromDeviceId, right.fromSocket, right.toDeviceId, right.toSocket);
    };
    std::sort(candidates.begin(), candidates.end(), qualityLess);

    std::map<std::string, std::string> parent;
    const auto root = [&parent](const std::string& deviceId) {
        std::string current = deviceId;
        while(parent.count(current) != 0 && parent[current] != current) current = parent[current];
        return current;
    };
    std::vector<std::size_t> selected;
    for(std::size_t index = 0; index < candidates.size(); ++index) {
        auto& candidate = candidates[index];
        const auto fromRoot = root(candidate.fromDeviceId);
        const auto toRoot = root(candidate.toDeviceId);
        if(fromRoot == toRoot) {
            candidate.diagnostic.accepted = false;
            candidate.diagnostic.rejectionReason = "not selected: spanning-tree cycle";
            continue;
        }
        if(parent.count(candidate.fromDeviceId) == 0) parent[candidate.fromDeviceId] = candidate.fromDeviceId;
        if(parent.count(candidate.toDeviceId) == 0) parent[candidate.toDeviceId] = candidate.toDeviceId;
        parent[fromRoot] = toRoot;
        selected.push_back(index);
    }

    std::vector<MultiDeviceExtrinsics> graph;
    std::map<std::string, std::vector<std::pair<std::string, std::size_t>>> adjacency;
    for(const auto index : selected) {
        const auto& candidate = candidates[index];
        adjacency[candidate.fromDeviceId].push_back({candidate.toDeviceId, index});
        adjacency[candidate.toDeviceId].push_back({candidate.fromDeviceId, index});
    }
    std::set<std::string> visited;
    for(const auto& [deviceId, unused] : adjacency) {
        (void)unused;
        if(!visited.insert(deviceId).second) continue;
        std::set<std::string> componentDevices;
        std::vector<std::string> stack{deviceId};
        while(!stack.empty()) {
            const auto current = stack.back();
            stack.pop_back();
            componentDevices.insert(current);
            auto neighbours = adjacency[current];
            std::sort(neighbours.begin(), neighbours.end());
            for(const auto& [neighbour, unusedIndex] : neighbours) {
                (void)unusedIndex;
                if(visited.insert(neighbour).second) stack.push_back(neighbour);
            }
        }
        const auto componentRoot = *std::min_element(componentDevices.begin(), componentDevices.end(), [&](const auto& left, const auto& right) {
            return std::tie(left, cameraOrigin.at(left)) < std::tie(right, cameraOrigin.at(right));
        });

        std::set<std::string> treeVisited{componentRoot};
        std::vector<std::string> treeStack{componentRoot};
        while(!treeStack.empty()) {
            const auto current = treeStack.back();
            treeStack.pop_back();
            auto neighbours = adjacency[current];
            std::sort(neighbours.begin(), neighbours.end());
            for(const auto& [neighbour, candidateIndex] : neighbours) {
                if(!treeVisited.insert(neighbour).second) continue;
                const auto& candidate = candidates[candidateIndex];
                const auto transform = transformForCandidate(candidate, neighbour, current);
                MultiDeviceExtrinsics edge;
                edge.fromDeviceId = neighbour;
                edge.fromSocket = cameraOrigin.at(neighbour);
                edge.extrinsics = makeLocalOriginEdge(transform, current, cameraOrigin.at(current));
                graph.push_back(std::move(edge));
                treeStack.push_back(neighbour);
            }
        }
    }

    std::set<std::string> registeredDevices;
    for(const auto& camera : pimpl->cameras) registeredDevices.insert(camera.deviceId);
    for(auto first = registeredDevices.begin(); first != registeredDevices.end(); ++first) {
        for(auto second = std::next(first); second != registeredDevices.end(); ++second) {
            if(imageConnectedDevicePairs.count({*first, *second}) != 0) continue;
            MultiDeviceCalibrationResult::EdgeDiagnostic diagnostic;
            diagnostic.fromDeviceId = *second;
            diagnostic.fromSocket = cameraOrigin.at(*second);
            diagnostic.toDeviceId = *first;
            diagnostic.toSocket = cameraOrigin.at(*first);
            diagnostic.rejectionReason = "devices are not connected by valid shared-scene image measurements";
            result->diagnostics.push_back(std::move(diagnostic));
            notes.push_back("no image-connected calibration component between " + *first + " and " + *second);
        }
    }
    for(const auto& candidate : candidates) result->diagnostics.push_back(candidate.diagnostic);
    std::stable_sort(result->diagnostics.begin(), result->diagnostics.end(), [](const auto& left, const auto& right) {
        return std::tie(left.fromDeviceId, left.fromSocket, left.toDeviceId, left.toSocket, left.accepted, left.rejectionReason)
               < std::tie(right.fromDeviceId, right.fromSocket, right.toDeviceId, right.toSocket, right.accepted, right.rejectionReason);
    });
    result->dataConfidence = dataConfidence;
    if(!notes.empty()) {
        std::ostringstream info;
        for(std::size_t index = 0; index < notes.size(); ++index) {
            if(index != 0) info << "; ";
            info << notes[index];
        }
        result->info = info.str();
    }
    if(!graph.empty()) {
        result->handler.emplace(std::move(graph));
        result->passed = true;
        std::set<std::string> represented;
        for(const auto& diagnostic : result->diagnostics) {
            if(diagnostic.accepted) {
                represented.insert(diagnostic.fromDeviceId);
                represented.insert(diagnostic.toDeviceId);
            }
        }
        for(const auto& camera : pimpl->cameras) {
            if(represented.count(camera.deviceId) == 0) {
                result->complete = false;
                break;
            }
            result->complete = true;
        }
    }
    if(!result->handler) {
        result->passed = false;
        result->complete = false;
        if(result->info.empty()) result->info = "no usable validated metric edge exists";
    }
    calibrationOutput.send(std::move(result));
}

#endif

void MultiDeviceCalibration::run() {
#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    while(mainLoop()) {
        auto control = inputControl.get<MultiDeviceCalibrationControl>();
        if(!control) continue;
        if(std::holds_alternative<MultiDeviceCalibrationControl::Commands::Start>(control->command)) {
            std::lock_guard<std::mutex> lock(pimpl->mutex);
            if(pimpl->state != Impl::State::IDLE) continue;
            pimpl->state = Impl::State::COMPLETE;
            sendFailure(*this, "MultiDeviceCalibration requires OpenCV support", "opencv support unavailable");
        } else if(std::holds_alternative<MultiDeviceCalibrationControl::Commands::Reset>(control->command)) {
            std::lock_guard<std::mutex> lock(pimpl->mutex);
            pimpl->clearCycle();
            pimpl->state = Impl::State::IDLE;
        }
    }
#else
    while(mainLoop()) {
        if(pimpl->state == Impl::State::IDLE) {
            auto control = inputControl.get<MultiDeviceCalibrationControl>();
            if(!control) continue;
            if(std::holds_alternative<MultiDeviceCalibrationControl::Commands::Start>(control->command)) {
                std::lock_guard<std::mutex> lock(pimpl->mutex);
                if(pimpl->state != Impl::State::IDLE) continue;
                pimpl->state = Impl::State::COLLECTING;
                pimpl->clearCycle();
                std::set<std::string> deviceIds;
                for(const auto& camera : pimpl->cameras) deviceIds.insert(camera.deviceId);
                if(pimpl->cameras.size() < 2 || deviceIds.size() < 2) {
                    pimpl->state = Impl::State::COMPLETE;
                    sendFailure(*this, "MultiDeviceCalibration needs at least two cameras on two devices", "insufficient registered cameras");
                }
            } else if(std::holds_alternative<MultiDeviceCalibrationControl::Commands::Reset>(control->command)) {
                std::lock_guard<std::mutex> lock(pimpl->mutex);
                pimpl->clearCycle();
                pimpl->state = Impl::State::IDLE;
            }
            continue;
        }

        if(auto control = inputControl.tryGet<MultiDeviceCalibrationControl>()) {
            if(std::holds_alternative<MultiDeviceCalibrationControl::Commands::Stop>(control->command)) {
                std::lock_guard<std::mutex> lock(pimpl->mutex);
                if(pimpl->state == Impl::State::COLLECTING) {
                    pimpl->clearCycle();
                    sendFailure(*this, "stopped", "stopped");
                    pimpl->state = Impl::State::IDLE;
                }
            } else if(std::holds_alternative<MultiDeviceCalibrationControl::Commands::Reset>(control->command)) {
                std::lock_guard<std::mutex> lock(pimpl->mutex);
                pimpl->clearCycle();
                pimpl->state = Impl::State::IDLE;
            }
        }

        if(pimpl->state != Impl::State::COLLECTING) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        auto group = syncInput.tryGet<MessageGroup>();
        if(!group) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        bool complete = true;
        for(const auto& camera : pimpl->cameras) {
            const auto frame = group->get<ImgFrame>(camera.inputName);
            if(!frame || frame->getCvFrame().empty()) {
                complete = false;
                break;
            }
        }
        if(!complete) continue;

        std::string error;
        try {
            if(!pimpl->dclInitialized && !initializeCycle(group, error)) {
                pimpl->state = Impl::State::COMPLETE;
                sendFailure(*this, error, error);
                continue;
            }
            if(!loadCompleteGroup(group, error)) {
                continue;
            }

            if(pimpl->collectedGroups >= getSampleCount()) {
                estimateAndEmit();
                pimpl->state = Impl::State::COMPLETE;
            }
        } catch(const std::exception& ex) {
            pimpl->state = Impl::State::COMPLETE;
            sendFailure(*this, std::string("MultiDeviceCalibration failed: ") + ex.what(), "calibration exception");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
#endif
}

}  // namespace node
}  // namespace dai
