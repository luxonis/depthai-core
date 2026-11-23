#include "depthai/beta/node/MultiDeviceCalibration.hpp"

#include <DynamicCalibration.hpp>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "beta/node/MultiDeviceCalibrationUtils.hpp"
#include "utility/ImgFrameView.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/calib3d.hpp>
    #include <opencv2/core.hpp>
#endif

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationControl.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/node/DynamicCalibrationUtils.hpp"
#include "pipeline/node/WorldConstraintsValidation.hpp"
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

/// "0, 4" for a list of marker ids, "none" when empty.
std::string joinIds(const std::vector<int32_t>& ids) {
    if(ids.empty()) return "none";
    std::string joined;
    for(const auto id : ids) joined += (joined.empty() ? "" : ", ") + std::to_string(id);
    return joined;
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
/// Marker observations are kept in the sets of the DynamicCalibration node, which key cameras by socket. Devices
/// share socket names, so here the key is the position of the camera in the registration order.
CameraBoardSocket observationKey(std::size_t cameraIndex) {
    return static_cast<CameraBoardSocket>(static_cast<int32_t>(cameraIndex));
}
#endif

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
        detail::MultiDeviceTransform originToCamera;
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
    std::vector<MultiDeviceExtrinsics> initialGuesses;
    // initialGuesses converted to local-origin-to-local-origin transforms for the current cycle
    std::vector<detail::PairwiseDeviceTransform> originGuesses;
    std::vector<StereoPair> stereoPairs;
    std::optional<WorldConstraints> worldConstraints;

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    std::unique_ptr<dcl::DynamicCalibration> dcl;
    std::map<std::string, std::shared_ptr<dcl::Device>> dclDevices;
    std::size_t collectedGroups = 0;
    bool dclInitialized = false;
    /// A socket that a marker distance refers to but that supplies no images: it keeps its place inside its device.
    struct AuxiliaryCamera {
        std::string deviceId;
        CameraBoardSocket socket = CameraBoardSocket::AUTO;
        detail::MultiDeviceTransform originToCamera;
        dai::node::detail::ObservationCamera model;
        std::shared_ptr<dcl::CameraSensorHandle> sensor;
    };
    // Keyed by observationKey(cameras.size() + position)
    std::vector<AuxiliaryCamera> auxiliaryCameras;
    // worldConstraints with the socket of every socket distance replaced by the observationKey() of its camera
    std::optional<WorldConstraints> keyedWorldConstraints;
    // Marker observations of the groups loaded in the current cycle, cameras keyed by observationKey()
    std::deque<MarkerObservationSet> markerObservations;
    std::set<int32_t> markerSizeWarningIssued;
    bool markerDetectionWarningIssued = false;
    std::optional<std::chrono::steady_clock::time_point> markersMissingSince;
    std::optional<std::chrono::steady_clock::time_point> markersMissingReportedAt;

    /// Warns when groups keep being skipped because a configured marker is not seen by two cameras: after 5 s and
    /// then every 30 s, as the DynamicCalibration node does.
    void trackMissingMarkers(const CoverageData::MarkerCoverage& coverage, spdlog::async_logger& logger) {
        constexpr std::chrono::seconds kMarkersMissingWarnAfter{5};
        constexpr std::chrono::seconds kMarkersMissingWarnPeriod{30};
        const auto now = std::chrono::steady_clock::now();
        if(coverage.accepted) {
            if(markersMissingReportedAt) logger.info("Every configured marker is seen by two cameras again");
            markersMissingSince.reset();
            markersMissingReportedAt.reset();
            return;
        }
        if(!markersMissingSince) markersMissingSince = now;
        const bool due =
            markersMissingReportedAt ? now - *markersMissingReportedAt >= kMarkersMissingWarnPeriod : now - *markersMissingSince >= kMarkersMissingWarnAfter;
        if(!due) return;
        markersMissingReportedAt = now;
        std::string tooSmall;
        for(const auto& [id, sizePx] : coverage.tooSmallPx) tooSmall += fmt::format("{}{} ({:.0f} px)", tooSmall.empty() ? "" : ", ", id, sizePx);
        const bool noneSeen = coverage.seenBy.empty() && coverage.tooSmallPx.empty();
        logger.warn(
            "No synchronized group accepted for {} s, markers not seen by two cameras: {}{}. Calibration waits until every configured marker is visible{}",
            std::chrono::duration_cast<std::chrono::seconds>(now - *markersMissingSince).count(),
            joinIds(coverage.missingMarkers),
            tooSmall.empty() ? "" : fmt::format(", too small (minimum {:.0f} px): {}", dai::node::DclUtils::kMinMarkerSizePx, tooSmall),
            noneSeen ? fmt::format("; none of them has been detected, check markerType{} and the marker ids",
                                   worldConstraints && worldConstraints->markerType == MarkerType::ARUCO ? " / arucoDictionary" : "")
                     : "");
    }
#endif

    void requireIdle(const char* setting) const {
        DAI_CHECK_V(state == State::IDLE, "MultiDeviceCalibration {} cannot be changed unless the node is idle", setting);
    }

    void clearCycle() {
        calibrations.clear();
        originGuesses.clear();
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        dcl.reset();
        dclDevices.clear();
        collectedGroups = 0;
        dclInitialized = false;
        markerObservations.clear();
        auxiliaryCameras.clear();
        keyedWorldConstraints.reset();
        markerSizeWarningIssued.clear();
        markerDetectionWarningIssued = false;
        markersMissingSince.reset();
        markersMissingReportedAt.reset();
        for(auto& camera : cameras) {
            camera.localOrigin = CameraBoardSocket::AUTO;
            camera.resolution = {0, 0};
            camera.transformation = {};
            camera.originToCamera.clear();
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

void MultiDeviceCalibration::addCamera(Node::Output& cameraOutput) {
    auto* camera = dynamic_cast<dai::node::Camera*>(&cameraOutput.getParent());
    DAI_CHECK_V(
        camera != nullptr,
        "MultiDeviceCalibration::addCamera(output) needs an output of a Camera node, got '{}'; use addCamera(deviceId, socket, output) for other outputs",
        cameraOutput.getParent().getName());
    const auto device = camera->getDevice();
    DAI_CHECK_V(device != nullptr,
                "MultiDeviceCalibration::addCamera(output): the Camera node has no device; create it with pipeline.create<Camera>(device) or use "
                "addCamera(deviceId, socket, output)");
    addCamera(device->getDeviceId(), camera->getBoardSocket(), cameraOutput);
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

void MultiDeviceCalibration::setInitialGuess(const MultiDeviceExtrinsics& guess) {
    const auto& fromDeviceId = guess.fromDeviceId;
    const auto& toDeviceId = guess.extrinsics.toDeviceId;
    DAI_CHECK_V(!fromDeviceId.empty() && !toDeviceId.empty(), "MultiDeviceCalibration initial guess needs fromDeviceId and extrinsics.toDeviceId");
    DAI_CHECK_V(fromDeviceId != toDeviceId, "MultiDeviceCalibration initial guess must connect different devices");
    DAI_CHECK_V(isConcreteSocket(guess.fromSocket) && isConcreteSocket(guess.extrinsics.toCameraSocket),
                "MultiDeviceCalibration initial guess sockets must be concrete, got {} -> {}",
                toString(guess.fromSocket),
                toString(guess.extrinsics.toCameraSocket));
    DAI_CHECK_V(matrix::isFinitePoint3f(guess.extrinsics.translation), "MultiDeviceCalibration initial guess translation must be finite");
    DAI_CHECK_V(guess.extrinsics.lengthUnit != LengthUnit::CUSTOM, "MultiDeviceCalibration initial guess uses unsupported CUSTOM length units");
    try {
        matrix::validateRotationMatrix3x3(guess.extrinsics.rotationMatrix);
    } catch(const std::exception&) {
        throw std::invalid_argument("MultiDeviceCalibration initial guess rotation must be a finite proper rigid rotation");
    }

    std::lock_guard<std::mutex> lock(pimpl->mutex);
    pimpl->requireIdle("initial guess");
    // One guess per device pair, in either direction: the last one set wins
    const auto existing = std::find_if(pimpl->initialGuesses.begin(), pimpl->initialGuesses.end(), [&](const MultiDeviceExtrinsics& initial) {
        return (initial.fromDeviceId == fromDeviceId && initial.extrinsics.toDeviceId == toDeviceId)
               || (initial.fromDeviceId == toDeviceId && initial.extrinsics.toDeviceId == fromDeviceId);
    });
    if(existing != pimpl->initialGuesses.end()) {
        *existing = guess;
    } else {
        pimpl->initialGuesses.push_back(guess);
    }
}

void MultiDeviceCalibration::setInitialGuesses(const std::vector<MultiDeviceExtrinsics>& guesses) {
    for(const auto& guess : guesses) setInitialGuess(guess);
}

void MultiDeviceCalibration::setInitialGuesses(const MultiDeviceCalibrationHandler& previous) {
    setInitialGuesses(previous.getGraph());
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

void MultiDeviceCalibration::setWorldConstraints(const WorldConstraints& constraints) {
    constraints.validate();
    if(constraints.empty()) {
        throw std::invalid_argument("MultiDeviceCalibration::setWorldConstraints: at least one marker distance or marker size is required");
    }
    for(const auto& distance : constraints.socketDistances) {
        if(distance.deviceId.empty()) {
            throw std::invalid_argument(fmt::format(
                "MultiDeviceCalibration::setWorldConstraints: the distance between marker {} and {} needs the deviceId of the device the socket belongs to",
                distance.marker,
                toString(distance.socket)));
        }
    }
    std::lock_guard<std::mutex> lock(pimpl->mutex);
    pimpl->requireIdle("world constraints");
    pimpl->worldConstraints = constraints;
}

void MultiDeviceCalibration::setWorldConstraintsFromJson(const std::string& json) {
    setWorldConstraints(WorldConstraints::fromJsonString(json));
}

void MultiDeviceCalibration::setWorldConstraintsFromFile(const std::filesystem::path& path) {
    setWorldConstraints(WorldConstraints::fromFile(path));
}

void MultiDeviceCalibration::clearWorldConstraints() {
    std::lock_guard<std::mutex> lock(pimpl->mutex);
    pimpl->requireIdle("world constraints");
    pimpl->worldConstraints.reset();
}

std::optional<WorldConstraints> MultiDeviceCalibration::getWorldConstraints() const {
    std::lock_guard<std::mutex> lock(pimpl->mutex);
    return pimpl->worldConstraints;
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

    // Convert the initial guesses, given between arbitrary sockets, to local-origin-to-local-origin
    // transforms: originGuess = (toSocket -> toOrigin) * guess * (fromOrigin -> fromSocket)
    std::map<std::string, CameraBoardSocket> originBySocketDevice;
    for(const auto& camera : pimpl->cameras) originBySocketDevice.emplace(camera.deviceId, camera.localOrigin);
    pimpl->originGuesses.clear();
    for(const auto& initial : pimpl->initialGuesses) {
        const auto& fromDeviceId = initial.fromDeviceId;
        const auto& toDeviceId = initial.extrinsics.toDeviceId;
        const auto fromOrigin = originBySocketDevice.find(fromDeviceId);
        const auto toOrigin = originBySocketDevice.find(toDeviceId);
        if(fromOrigin == originBySocketDevice.end() || toOrigin == originBySocketDevice.end()) {
            error = "initial guess " + fromDeviceId + " -> " + toDeviceId + " references a device without registered cameras";
            return false;
        }
        const auto socketToOrigin = [&](const std::string& deviceId, CameraBoardSocket socket, CameraBoardSocket origin, bool towardsOrigin) {
            if(socket == origin) return detail::identityTransform();
            const auto& calibration = pimpl->calibrations.at(deviceId);
            return towardsOrigin ? calibration.getCameraExtrinsics(socket, origin, false, LengthUnit::METER)
                                 : calibration.getCameraExtrinsics(origin, socket, false, LengthUnit::METER);
        };
        try {
            const auto fromOriginToFromSocket = socketToOrigin(fromDeviceId, initial.fromSocket, fromOrigin->second, false);
            const auto toSocketToToOrigin = socketToOrigin(toDeviceId, initial.extrinsics.toCameraSocket, toOrigin->second, true);
            const auto guess = matrix::toVecMatrix4x4(initial.extrinsics.getTransformationMatrix(false, LengthUnit::METER));
            pimpl->originGuesses.push_back(detail::PairwiseDeviceTransform{
                fromDeviceId, toDeviceId, detail::composeRigidTransforms(toSocketToToOrigin, detail::composeRigidTransforms(guess, fromOriginToFromSocket))});
        } catch(const std::exception& ex) {
            error = "initial guess " + fromDeviceId + "/" + toString(initial.fromSocket) + " -> " + toDeviceId + "/"
                    + toString(initial.extrinsics.toCameraSocket) + " uses a socket unknown to the device calibration: " + ex.what();
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
        const auto image = frame->getFrame();
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
        camera.originToCamera = originToCamera;
        const auto calibration = dai::node::DclUtils::createDclCalibration(camera.transformation.getIntrinsicMatrix(),
                                                                           camera.transformation.getDistortionCoefficients(),
                                                                           matrix::extractRotationMatrix(originToCamera),
                                                                           matrix::extractTranslationVector(originToCamera),
                                                                           camera.transformation.getDistortionModel());
        camera.sensor =
            pimpl->dcl->addSensor(pimpl->dclDevices.at(camera.deviceId), calibration, dcl::resolution_t{camera.resolution.first, camera.resolution.second});
    }

    // Socket distances name a device and a socket: a registered camera, or any other calibrated socket of a
    // registered device, which then keeps its place inside the device.
    if(pimpl->worldConstraints) {
        auto keyed = *pimpl->worldConstraints;
        for(auto& distance : keyed.socketDistances) {
            const auto what = fmt::format("the distance between marker {} and {}/{}", distance.marker, distance.deviceId, toString(distance.socket));
            if(deviceIds.count(distance.deviceId) == 0) {
                error = "world constraints: " + what + " refers to a device without registered cameras";
                return false;
            }
            const auto matches = [&](const auto& camera) { return camera.deviceId == distance.deviceId && camera.socket == distance.socket; };
            std::size_t key = 0;
            const auto registered = std::find_if(pimpl->cameras.begin(), pimpl->cameras.end(), matches);
            const auto auxiliary = std::find_if(pimpl->auxiliaryCameras.begin(), pimpl->auxiliaryCameras.end(), matches);
            if(registered != pimpl->cameras.end()) {
                key = static_cast<std::size_t>(registered - pimpl->cameras.begin());
            } else if(auxiliary != pimpl->auxiliaryCameras.end()) {
                key = pimpl->cameras.size() + static_cast<std::size_t>(auxiliary - pimpl->auxiliaryCameras.begin());
            } else {
                const auto& calibration = pimpl->calibrations.at(distance.deviceId);
                const auto& cameraData = calibration.getEepromData().cameraData;
                const auto info = cameraData.find(distance.socket);
                if(info == cameraData.end()) {
                    error = "world constraints: " + what + " refers to a socket without calibration on that device";
                    return false;
                }
                Impl::AuxiliaryCamera camera;
                camera.deviceId = distance.deviceId;
                camera.socket = distance.socket;
                try {
                    const auto localOrigin = std::find_if(pimpl->cameras.begin(), pimpl->cameras.end(), [&](const auto& other) {
                                                 return other.deviceId == distance.deviceId;
                                             })->localOrigin;
                    camera.originToCamera = calibration.getCameraExtrinsics(localOrigin, distance.socket, false, LengthUnit::METER);
                    camera.model = {calibration.getCameraIntrinsics(distance.socket),
                                    calibration.getDistortionCoefficients(distance.socket),
                                    calibration.getDistortionModel(distance.socket)};
                } catch(const std::exception& ex) {
                    error = "world constraints: " + what + " refers to a socket whose calibration cannot be read: " + ex.what();
                    return false;
                }
                std::array<std::array<float, 3>, 3> intrinsics{};
                for(std::size_t row = 0; row < 3; ++row) {
                    for(std::size_t col = 0; col < 3; ++col) intrinsics[row][col] = camera.model.intrinsics.at(row).at(col);
                }
                const auto dclCalibration = dai::node::DclUtils::createDclCalibration(intrinsics,
                                                                                      camera.model.distortion,
                                                                                      matrix::extractRotationMatrix(camera.originToCamera),
                                                                                      matrix::extractTranslationVector(camera.originToCamera),
                                                                                      camera.model.distortionModel);
                camera.sensor =
                    pimpl->dcl->addSensor(pimpl->dclDevices.at(camera.deviceId),
                                          dclCalibration,
                                          dcl::resolution_t{std::max<unsigned>(1U, info->second.width), std::max<unsigned>(1U, info->second.height)});
                key = pimpl->cameras.size() + pimpl->auxiliaryCameras.size();
                pimpl->auxiliaryCameras.push_back(std::move(camera));
                ThreadedNode::pimpl->logger->debug("Marker constraints: {}/{} is not a registered camera, using its place in the device calibration",
                                                   distance.deviceId,
                                                   toString(distance.socket));
            }
            distance.socket = observationKey(key);
        }
        pimpl->keyedWorldConstraints = std::move(keyed);
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
        const auto image = utility::getCvFrameView(*frame);
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

    // Collected now, kept only when DCL accepts the group below. A group that does not show every required marker
    // to two cameras is not loaded at all: with constraints configured, every counted group carries them.
    std::optional<MarkerObservationSet> pendingObservations;
    if(pimpl->worldConstraints) {
        const auto& constraints = *pimpl->worldConstraints;
        auto& logger = *ThreadedNode::pimpl->logger;
        MarkerObservationSet observations;
        std::map<int32_t, float> tooSmallPx;
        for(std::size_t idx = 0; idx < images.size(); ++idx) {
            std::map<int32_t, double> tooSmall;
            auto detected = dai::node::DclUtils::detectConfiguredMarkers(images[idx].second, constraints, &tooSmall);
            for(const auto& [markerId, sizePx] : tooSmall) {
                auto& recorded = tooSmallPx[markerId];
                recorded = std::max(recorded, static_cast<float>(sizePx));
                // Once per marker: a configured marker that is seen but too small is otherwise a silent loss of constraints.
                if(pimpl->markerSizeWarningIssued.insert(markerId).second) {
                    logger.warn(
                        "Marker {} is only {:.1f} px across in {}/{}, below the {:.0f} px minimum, and is ignored: move the camera closer or print it "
                        "larger",
                        markerId,
                        sizePx,
                        pimpl->cameras[idx].deviceId,
                        toString(pimpl->cameras[idx].socket),
                        dai::node::DclUtils::kMinMarkerSizePx);
                }
            }
            if(!detected.passed()) {
                if(!pimpl->markerDetectionWarningIssued) {
                    logger.warn("Marker detection failed, world constraints are not applied: {}", detected.errorMessage());
                    pimpl->markerDetectionWarningIssued = true;
                }
                continue;
            }
            for(const auto& [markerId, detection] : detected.value) {
                observations.centers[markerId][observationKey(idx)] = {static_cast<float>(detection.center[0]), static_cast<float>(detection.center[1])};
                if(!constraints.markerSizes.empty()) {
                    observations.outlines[markerId][observationKey(idx)] = dai::node::DclUtils::toMarkerOutline(detection);
                }
            }
        }
        const auto coverage = dai::node::DclUtils::markerCoverage(constraints, observations, tooSmallPx, pimpl->markerObservations.size());
        pimpl->trackMissingMarkers(coverage, logger);
        if(!coverage.accepted) {
            error = "markers not seen by two cameras: " + joinIds(coverage.missingMarkers);
            logger.trace("Synchronized group skipped, {}", error);
            return false;
        }
        pendingObservations = std::move(observations);
    }

    const auto loaded = pimpl->dcl->loadImages(images, timestamp);
    if(!loaded.passed()) {
        error = loaded.errorMessage();
        return false;
    }
    if(pendingObservations) {
        pimpl->markerObservations.push_back(std::move(*pendingObservations));
        while(pimpl->markerObservations.size() > dai::node::DclUtils::kMaxMarkerObservationSets) {
            pimpl->markerObservations.pop_front();
        }
    }
    ++pimpl->collectedGroups;
    return true;
}

#endif

namespace {

void sendFailure(MultiDeviceCalibration& node, const std::string& info) {
    auto result = std::make_shared<MultiDeviceCalibrationResult>();
    result->graph.reset();
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

    const auto referenceToDevice = detail::makeReferenceRelativeTransforms(referenceDeviceId, pimpl->originGuesses);
    for(const auto& [deviceId, transform] : referenceToDevice) {
        if(deviceId == referenceDeviceId) continue;
        const auto device = pimpl->dclDevices.find(deviceId);
        if(device != pimpl->dclDevices.end()) request.initialGuesses.push_back(makeDclPose(device->second, transform));
    }

    // Marker distances and sizes give the calibration a metric reference that the image features cannot provide.
    if(pimpl->keyedWorldConstraints && !pimpl->markerObservations.empty()) {
        const auto sensorForKey = [&](CameraBoardSocket key) -> std::shared_ptr<const dcl::CameraSensorHandle> {
            const auto index = static_cast<std::size_t>(static_cast<int32_t>(key));
            if(index < pimpl->cameras.size()) return pimpl->cameras[index].sensor;
            const auto auxiliary = index - pimpl->cameras.size();
            return auxiliary < pimpl->auxiliaryCameras.size() ? pimpl->auxiliaryCameras[auxiliary].sensor : nullptr;
        };
        dai::node::DclUtils::WorldConstraintsSummary summary;
        request.worldConstraints = dai::node::DclUtils::buildWorldConstraints(*pimpl->keyedWorldConstraints, pimpl->markerObservations, sensorForKey, &summary);
        ThreadedNode::pimpl->logger->info("Marker constraints: {} frame sets, {} scene points, {} marker distances, {} socket distances, {} marker sizes",
                                          summary.observationSets,
                                          summary.points,
                                          summary.pointDistances,
                                          summary.sensorDistances,
                                          summary.markerSizes);
    }

    auto calibrated = pimpl->dcl->findMultiDeviceCalibration(request);
    if(!calibrated.passed()) {
        result->info = "DynamicCalibration failed: " + calibrated.errorMessage();
        calibrationOutput.send(std::move(result));
        return;
    }

    if(!request.worldConstraints.empty()) {
        // The measurements are fitted, not enforced: a result that does not reproduce them is not a calibration.
        std::string mismatch;
        try {
            std::map<std::string, MultiDeviceTransform> referenceToDevice;
            for(const auto& dclPose : calibrated.value.poses) {
                const auto device =
                    std::find_if(pimpl->dclDevices.begin(), pimpl->dclDevices.end(), [&](const auto& entry) { return entry.second == dclPose.device; });
                if(device != pimpl->dclDevices.end()) referenceToDevice.emplace(device->first, transformFromDclPose(dclPose));
            }
            std::map<CameraBoardSocket, dai::node::detail::PosedObservationCamera> cameras;
            for(std::size_t idx = 0; idx < pimpl->cameras.size(); ++idx) {
                const auto& camera = pimpl->cameras[idx];
                const auto pose = referenceToDevice.find(camera.deviceId);
                if(pose == referenceToDevice.end()) continue;
                const auto intrinsics = camera.transformation.getIntrinsicMatrix();
                dai::node::detail::PosedObservationCamera posed;
                for(const auto& row : intrinsics) posed.model.intrinsics.emplace_back(row.begin(), row.end());
                posed.model.distortion = camera.transformation.getDistortionCoefficients();
                posed.model.distortionModel = camera.transformation.getDistortionModel();
                posed.baseToCamera = detail::composeRigidTransforms(camera.originToCamera, pose->second);
                cameras.emplace(observationKey(idx), std::move(posed));
            }
            for(std::size_t idx = 0; idx < pimpl->auxiliaryCameras.size(); ++idx) {
                const auto& camera = pimpl->auxiliaryCameras[idx];
                const auto pose = referenceToDevice.find(camera.deviceId);
                if(pose == referenceToDevice.end()) continue;
                cameras.emplace(observationKey(pimpl->cameras.size() + idx),
                                dai::node::detail::PosedObservationCamera{camera.model, detail::composeRigidTransforms(camera.originToCamera, pose->second)});
            }
            auto residuals = dai::node::detail::worldConstraintResiduals(*pimpl->keyedWorldConstraints, pimpl->markerObservations, cameras);
            // Reported in configuration order: marker distances, socket distances, marker sizes. The keyed copy does
            // not know the names of the sockets.
            for(std::size_t idx = 0; idx < pimpl->worldConstraints->socketDistances.size(); ++idx) {
                const auto& distance = pimpl->worldConstraints->socketDistances[idx];
                residuals.at(pimpl->worldConstraints->markerDistances.size() + idx).what =
                    fmt::format("the distance between marker {} and {}/{}", distance.marker, distance.deviceId, toString(distance.socket));
            }
            for(const auto& residual : residuals) {
                ThreadedNode::pimpl->logger->debug("Marker constraints: {} configured {:.4f} m, reconstructed {}",
                                                   residual.what,
                                                   residual.configured,
                                                   residual.measured ? fmt::format("{:.4f} m", *residual.measured) : std::string("not available"));
                if(residual.fits()) continue;
                if(!mismatch.empty()) mismatch += "; ";
                mismatch +=
                    residual.measured
                        ? fmt::format(
                              "{} is configured as {:.3f} m but is {:.3f} m in the calibrated result", residual.what, residual.configured, *residual.measured)
                        : fmt::format("{} could not be reconstructed from the marker observations", residual.what);
            }
        } catch(const std::exception& ex) {
            mismatch = std::string("constraint validation failed: ") + ex.what();
        }
        if(mismatch.empty()) {
            // Measurements that agree with the images leave the epipolar error of the feature solve almost unchanged.
            const auto intrinsics = pimpl->cameras.front().transformation.getIntrinsicMatrix();
            const auto focalPx = 0.5 * (static_cast<double>(intrinsics[0][0]) + static_cast<double>(intrinsics[1][1]));
            const auto featuresPx = calibrated.value.featureSampsonError * focalPx;
            const auto constrainedPx = calibrated.value.sampsonError * focalPx;
            if(dai::node::detail::epipolarErrorContradictsConstraints(featuresPx, constrainedPx)) {
                mismatch = fmt::format("the epipolar error of the image features changes from {:.2f} px to {:.2f} px", featuresPx, constrainedPx);
            }
        }
        if(mismatch.empty()) {
            // Registered cameras with a fixed baseline make the image features metric on their own. The measurements
            // may refine the device poses that those give, not contradict them.
            constexpr double maxRotationChangeDeg = 5.0;
            auto featureRequest = request;
            featureRequest.worldConstraints = {};
            auto features = pimpl->dcl->findMultiDeviceCalibration(featureRequest);
            if(features.passed()) {
                const auto distanceOf = [](const dcl::MultiDevicePose& pose) {
                    return std::sqrt(pose.tvecMeters[0] * pose.tvecMeters[0] + pose.tvecMeters[1] * pose.tvecMeters[1]
                                     + pose.tvecMeters[2] * pose.tvecMeters[2]);
                };
                // Angle between two rotations given as rotation vectors, through their quaternions.
                const auto rotationBetweenDeg = [](const dcl::MultiDevicePose& first, const dcl::MultiDevicePose& second) {
                    const auto quaternionOf = [](const dcl::MultiDevicePose& pose) {
                        const double angle = std::sqrt(pose.rvec[0] * pose.rvec[0] + pose.rvec[1] * pose.rvec[1] + pose.rvec[2] * pose.rvec[2]);
                        const double scale = angle > 1e-12 ? std::sin(0.5 * angle) / angle : 0.5;
                        return std::array<double, 4>{std::cos(0.5 * angle), scale * pose.rvec[0], scale * pose.rvec[1], scale * pose.rvec[2]};
                    };
                    const auto a = quaternionOf(first);
                    const auto b = quaternionOf(second);
                    const double dot = std::abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
                    return 2.0 * std::acos(std::min(1.0, dot)) * 180.0 / 3.14159265358979323846;
                };
                for(const auto& constrained : calibrated.value.poses) {
                    const auto device =
                        std::find_if(pimpl->dclDevices.begin(), pimpl->dclDevices.end(), [&](const auto& entry) { return entry.second == constrained.device; });
                    const auto fromFeatures = std::find_if(features.value.poses.begin(), features.value.poses.end(), [&](const dcl::MultiDevicePose& pose) {
                        return pose.device == constrained.device;
                    });
                    if(device == pimpl->dclDevices.end() || device->first == referenceDeviceId || fromFeatures == features.value.poses.end()) continue;
                    const dai::node::detail::ConstraintResidual distance{"", distanceOf(*fromFeatures), distanceOf(constrained)};
                    const double rotationChangeDeg = rotationBetweenDeg(*fromFeatures, constrained);
                    if(rotationChangeDeg > maxRotationChangeDeg) {
                        if(!mismatch.empty()) mismatch += "; ";
                        mismatch += fmt::format("the rotation between devices {} and {} differs by {:.1f} deg from the one the image features give alone",
                                                referenceDeviceId,
                                                device->first,
                                                rotationChangeDeg);
                        continue;
                    }
                    if(distance.fits()) continue;
                    if(!mismatch.empty()) mismatch += "; ";
                    mismatch += fmt::format("the distance between devices {} and {} is {:.3f} m with the measurements but {:.3f} m from the image features",
                                            referenceDeviceId,
                                            device->first,
                                            *distance.measured,
                                            distance.configured);
                }
            }
        }
        if(!mismatch.empty()) {
            result->info = "World constraints could not be validated: " + mismatch + ". Check the measured values (meters) and marker ids";
            ThreadedNode::pimpl->logger->warn("{}", result->info);
            result->dataConfidence = calibrated.value.dataConfidence;
            result->sampsonError = calibrated.value.sampsonError;
            calibrationOutput.send(std::move(result));
            return;
        }
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
        result->graph = MultiDeviceCalibrationHandler(std::move(graph)).getGraph();
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
            if(!frame || frame->getFrame().empty()) {
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
