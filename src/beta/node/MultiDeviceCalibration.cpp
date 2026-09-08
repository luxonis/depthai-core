#include "depthai/beta/node/MultiDeviceCalibration.hpp"

#include <DynamicCalibration.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "beta/node/MultiDeviceCalibrationUtils.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/calib3d.hpp>
    #include <opencv2/core.hpp>
#endif

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationControl.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "pipeline/node/DynamicCalibrationUtils.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
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
    std::map<std::string, std::shared_ptr<dcl::Device>> dclDevices;
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
        dclDevices.clear();
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
    const auto reverse = std::find_if(
        pimpl->initialGuesses.begin(), pimpl->initialGuesses.end(), [&](const InitialGuess& initial) { return initial.from == to && initial.to == from; });
    DAI_CHECK_V(reverse == pimpl->initialGuesses.end(),
                "MultiDeviceCalibration initial guess between {} and {} is already registered in the opposite direction",
                from.deviceId,
                to.deviceId);
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

using detail::MultiDeviceTransform;

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

    for(const auto& known : pimpl->knownDistances) {
        const auto from = std::find_if(pimpl->cameras.begin(), pimpl->cameras.end(), [&](const Impl::Camera& camera) {
            return camera.deviceId == known.from.deviceId && camera.socket == known.from.socket;
        });
        const auto to = std::find_if(pimpl->cameras.begin(), pimpl->cameras.end(), [&](const Impl::Camera& camera) {
            return camera.deviceId == known.to.deviceId && camera.socket == known.to.socket;
        });
        if(from == pimpl->cameras.end() || to == pimpl->cameras.end()) {
            error = "known-distance endpoints must both be registered cameras";
            return false;
        }
    }

    for(const auto& stereo : pimpl->stereoPairs) {
        const auto left = std::find_if(pimpl->cameras.begin(), pimpl->cameras.end(), [&](const Impl::Camera& camera) {
            return camera.deviceId == stereo.deviceId && camera.socket == stereo.leftSocket;
        });
        const auto right = std::find_if(pimpl->cameras.begin(), pimpl->cameras.end(), [&](const Impl::Camera& camera) {
            return camera.deviceId == stereo.deviceId && camera.socket == stereo.rightSocket;
        });
        if(left == pimpl->cameras.end() || right == pimpl->cameras.end()) {
            error = "explicit stereo-pair endpoints must both be registered cameras on device " + stereo.deviceId;
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
    for(const auto& deviceId : deviceIds) pimpl->dclDevices.emplace(deviceId, pimpl->dcl->addDevice());

    for(auto& camera : pimpl->cameras) {
        const auto originToCamera = pimpl->calibrations.at(camera.deviceId).getCameraExtrinsics(camera.localOrigin, camera.socket, false, LengthUnit::METER);
        const auto calibration = dai::node::DclUtils::createDclCalibration(camera.transformation.getIntrinsicMatrix(),
                                                                           camera.transformation.getDistortionCoefficients(),
                                                                           matrix::extractRotationMatrix(originToCamera),
                                                                           matrix::extractTranslationVector(originToCamera),
                                                                           camera.transformation.getDistortionModel());
        camera.sensor =
            pimpl->dcl->addSensor(pimpl->dclDevices.at(camera.deviceId), calibration, dcl::resolution_t{camera.resolution.first, camera.resolution.second});
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
        images.emplace_back(camera.sensor, dai::node::DclUtils::cvMatToImageData(image));
    }

    const auto loaded = pimpl->dcl->loadImages(images, timestamp);
    if(!loaded.passed()) {
        error = loaded.errorMessage();
        return false;
    }
    ++pimpl->collectedGroups;
    return true;
}

#endif

namespace {

void sendFailure(MultiDeviceCalibration& node, const std::string& info) {
    auto result = std::make_shared<MultiDeviceCalibrationResult>();
    result->handler.reset();
    result->passed = false;
    result->info = info;
    node.calibrationOutput.send(std::move(result));
}

}  // namespace

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

namespace {

Extrinsics makeLocalOriginEdge(const MultiDeviceTransform& transform, const std::string& toDeviceId, CameraBoardSocket toSocket) {
    const auto translation = matrix::extractTranslationVector(transform);
    Extrinsics edge(matrix::extractRotationMatrix(transform), Point3f(translation[0], translation[1], translation[2]), toSocket, LengthUnit::METER);
    edge.toDeviceId = toDeviceId;
    return edge;
}

MultiDeviceTransform transformFromDclPose(const dcl::MultiDevicePose& pose) {
    cv::Mat rotationVector(3, 1, CV_64F);
    for(std::size_t axis = 0; axis < 3; ++axis) rotationVector.at<double>(static_cast<int>(axis)) = pose.rvec[axis];
    cv::Mat rotation;
    cv::Rodrigues(rotationVector, rotation);

    MultiDeviceTransform transform = detail::identityTransform();
    for(std::size_t row = 0; row < 3; ++row) {
        for(std::size_t column = 0; column < 3; ++column)
            transform[row][column] = static_cast<float>(rotation.at<double>(static_cast<int>(row), static_cast<int>(column)));
        transform[row][3] = static_cast<float>(pose.tvecMeters[row]);
    }
    return transform;
}

dcl::MultiDevicePose makeDclPose(const std::shared_ptr<const dcl::Device>& device, const MultiDeviceTransform& transform) {
    cv::Mat rotation(3, 3, CV_64F);
    for(std::size_t row = 0; row < 3; ++row) {
        for(std::size_t column = 0; column < 3; ++column) rotation.at<double>(static_cast<int>(row), static_cast<int>(column)) = transform[row][column];
    }
    cv::Mat rotationVector;
    cv::Rodrigues(rotation, rotationVector);

    dcl::MultiDevicePose pose;
    pose.device = device;
    for(std::size_t axis = 0; axis < 3; ++axis) {
        pose.rvec[axis] = rotationVector.at<double>(static_cast<int>(axis));
        pose.tvecMeters[axis] = transform[axis][3];
    }
    return pose;
}

}  // namespace

void MultiDeviceCalibration::estimateAndEmit() {
    auto result = std::make_shared<MultiDeviceCalibrationResult>();
    if(pimpl->dclDevices.empty()) {
        result->info = "no DCL devices were registered";
        calibrationOutput.send(std::move(result));
        return;
    }
    const auto& [referenceDeviceId, referenceDevice] = *pimpl->dclDevices.begin();

    dcl::MultiDeviceCalibrationRequest request;
    request.referenceDevice = referenceDevice;
    request.sensors.reserve(pimpl->cameras.size());
    std::map<Endpoint, std::size_t> sensorIndices;
    std::map<std::string, CameraBoardSocket> deviceOrigins;
    for(const auto& camera : pimpl->cameras) {
        sensorIndices.emplace(Endpoint{camera.deviceId, camera.socket}, request.sensors.size());
        request.sensors.push_back(camera.sensor);
        deviceOrigins.emplace(camera.deviceId, camera.localOrigin);
    }

    request.knownDistances.reserve(pimpl->knownDistances.size());
    for(const auto& known : pimpl->knownDistances) {
        const auto from = sensorIndices.find(known.from);
        const auto to = sensorIndices.find(known.to);
        if(from == sensorIndices.end() || to == sensorIndices.end()) {
            result->info = "known-distance endpoints must both be registered cameras";
            calibrationOutput.send(std::move(result));
            return;
        }
        request.knownDistances.push_back(dcl::KnownDistanceConstraint{request.sensors[from->second], request.sensors[to->second], known.meters});
    }

    request.stereoPairAllowlist.reserve(pimpl->stereoPairs.size());
    for(const auto& stereo : pimpl->stereoPairs) {
        const auto left = sensorIndices.find({stereo.deviceId, stereo.leftSocket});
        const auto right = sensorIndices.find({stereo.deviceId, stereo.rightSocket});
        if(left == sensorIndices.end() || right == sensorIndices.end()) continue;
        request.stereoPairAllowlist.emplace_back(request.sensors[left->second], request.sensors[right->second]);
    }

    std::vector<detail::PairwiseDeviceTransform> pairwiseInitialGuesses;
    pairwiseInitialGuesses.reserve(pimpl->initialGuesses.size());
    for(const auto& initial : pimpl->initialGuesses) {
        const auto initialTransform = initial.guess.getTransformationMatrix(false, LengthUnit::METER);
        MultiDeviceTransform fromToTo = detail::identityTransform();
        for(std::size_t row = 0; row < 4; ++row) {
            for(std::size_t column = 0; column < 4; ++column) fromToTo[row][column] = initialTransform[row][column];
        }
        pairwiseInitialGuesses.push_back(detail::PairwiseDeviceTransform{initial.from.deviceId, initial.to.deviceId, std::move(fromToTo)});
    }
    const auto referenceToDevice = detail::makeReferenceRelativeTransforms(referenceDeviceId, pairwiseInitialGuesses);
    for(const auto& [deviceId, transform] : referenceToDevice) {
        if(deviceId == referenceDeviceId) continue;
        const auto device = pimpl->dclDevices.find(deviceId);
        if(device != pimpl->dclDevices.end()) request.initialGuesses.push_back(makeDclPose(device->second, transform));
    }

    auto calibrated = pimpl->dcl->findMultiDeviceCalibration(request);
    if(!calibrated.passed()) {
        result->info = "DynamicCalibration failed: " + calibrated.errorMessage();
        calibrationOutput.send(std::move(result));
        return;
    }

    std::vector<MultiDeviceExtrinsics> graph;
    for(const auto& dclPose : calibrated.value.poses) {
        const auto device = std::find_if(pimpl->dclDevices.begin(), pimpl->dclDevices.end(), [&](const auto& entry) { return entry.second == dclPose.device; });
        if(device == pimpl->dclDevices.end() || device->first == referenceDeviceId) continue;

        MultiDeviceExtrinsics edge;
        edge.fromDeviceId = device->first;
        edge.fromSocket = deviceOrigins.at(device->first);
        edge.extrinsics =
            makeLocalOriginEdge(detail::inverseRigidTransform(transformFromDclPose(dclPose)), referenceDeviceId, deviceOrigins.at(referenceDeviceId));
        graph.push_back(std::move(edge));
    }

    result->dataConfidence = calibrated.value.dataConfidence;
    result->sampsonError = calibrated.value.sampsonError;
    if(graph.size() + 1 == pimpl->dclDevices.size()) {
        result->handler.emplace(std::move(graph));
        result->passed = true;
    } else {
        result->info = "DynamicCalibration returned an incomplete device pose set";
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
            sendFailure(*this, "MultiDeviceCalibration requires OpenCV support");
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
                    sendFailure(*this, "MultiDeviceCalibration needs at least two cameras on two devices");
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
                    sendFailure(*this, "stopped");
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
                sendFailure(*this, error);
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
            sendFailure(*this, std::string("MultiDeviceCalibration failed: ") + ex.what());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
#endif
}

}  // namespace node
}  // namespace beta
}  // namespace dai
