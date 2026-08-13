#include "depthai/pipeline/node/host/MultiDeviceCalibration.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <DynamicCalibration.hpp>
#include <InterDeviceScaleEstimator.hpp>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/MultiDeviceCalibrationResult.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/node/DynamicCalibrationUtils.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace node {

namespace {

using Transform = std::vector<std::vector<float>>;

Transform identityTransform() {
    return {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
}

Transform inverted(Transform transform) {
    matrix::invertSe3Matrix4x4InPlace(transform);
    return transform;
}

/// Center of the camera whose pose w.r.t. the reference frame is `transform`, expressed in the reference frame.
std::vector<float> cameraCenter(const Transform& transform) {
    const auto rotation = matrix::extractRotationMatrix(transform);
    const auto translation = matrix::extractTranslationVector(transform);
    std::vector<float> center(3, 0.0f);
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            center[i] -= rotation[j][i] * translation[j];
        }
    }
    return center;
}

float dot(const std::vector<float>& a, const std::vector<float>& b) {
    return std::inner_product(a.begin(), a.end(), b.begin(), 0.0f);
}

std::string frameKey(const CoordinateFrame& frame) {
    return frame.deviceId + "_" + toString(frame.socket);
}

/// A camera stream the rig is estimated from.
struct CameraStream {
    CoordinateFrame frame;
    std::string inputName;
    /// Pose of this camera w.r.t. the frame the whole rig is expressed in, in meters. Initial guess, refined by DCL.
    Transform toRigBase = identityTransform();
    std::pair<unsigned, unsigned> resolution{0, 0};
    ImgTransformation transformation;
    std::shared_ptr<dcl::CameraSensorHandle> sensor;
};

}  // namespace

class MultiDeviceCalibration::Impl {
   public:
    dcl::DynamicCalibration dcl;
    std::map<std::string, std::shared_ptr<dcl::Device>> dclDevices;

    std::vector<CameraStream> cameras;
    std::vector<RigEdge> initialGuesses;
    /// Known distances between camera centers, in meters, keyed by the frame pair.
    std::map<std::pair<CoordinateFrame, CoordinateFrame>, float> knownDistances;
    /// Metric distances recovered from the scene, in meters, keyed by the device reference frame pair. Recomputed each
    /// estimation cycle; a user `setKnownDistance()` always takes precedence over these.
    std::map<std::pair<CoordinateFrame, CoordinateFrame>, float> autoKnownDistances;
    /// Calibration of every device involved, read from the live devices.
    std::map<std::string, CalibrationHandler> calibrations;
    /// Frame every device is represented by in the rig, i.e. its first registered camera.
    std::map<std::string, CoordinateFrame> deviceReference;

    size_t sampleCount = 10;
    bool continuous = false;
    bool estimateScale = true;
    DynamicCalibrationControl::PerformanceMode performanceMode = DynamicCalibrationControl::PerformanceMode::DEFAULT;
    MultiDeviceCalibration::Method method = MultiDeviceCalibration::Method::DYNAMIC_CALIBRATION;
    /// Frame the whole rig is expressed in, i.e. the first registered camera.
    CoordinateFrame baseFrame;
    /// Try several strategies and keep the best-scoring one instead of trusting a single solve.
    bool autoStrategy = true;
    /// Yaw perturbations (degrees) of the initial guess explored by the automatic strategy.
    std::vector<float> guessYawOffsets = {0.0f, -30.0f, 30.0f, -60.0f, 60.0f};

    std::optional<float> findKnownDistance(const CoordinateFrame& a, const CoordinateFrame& b) const {
        const auto direct = knownDistances.find({a, b});
        if(direct != knownDistances.end()) return direct->second;
        const auto reverse = knownDistances.find({b, a});
        if(reverse != knownDistances.end()) return reverse->second;
        const auto autoDirect = autoKnownDistances.find({a, b});
        if(autoDirect != autoKnownDistances.end()) return autoDirect->second;
        const auto autoReverse = autoKnownDistances.find({b, a});
        if(autoReverse != autoKnownDistances.end()) return autoReverse->second;
        return std::nullopt;
    }

    /// Pose of camera `frame` w.r.t. the frame its device is represented by, in meters.
    Transform referenceToCamera(const CoordinateFrame& frame) const {
        const auto& reference = deviceReference.at(frame.deviceId);
        if(reference == frame) return identityTransform();
        return calibrations.at(frame.deviceId).getCameraExtrinsics(reference.socket, frame.socket, false, LengthUnit::METER);
    }

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    /// Initial guesses with an extra yaw rotation (about the reference vertical axis) applied to `deviceId`'s edge.
    MultiDeviceCalibrationHandler guessesWithYaw(const std::string& deviceId, float yawDegrees) const {
        auto edges = initialGuesses;
        if(yawDegrees != 0.0f) {
            const float yaw = yawDegrees * 0.017453292519943295f;  // pi / 180
            const Transform rotate = {
                {std::cos(yaw), 0.0f, std::sin(yaw), 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {-std::sin(yaw), 0.0f, std::cos(yaw), 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
            for(auto& edge : edges) {
                if(edge.from.deviceId != deviceId) continue;
                const auto current = matrix::toVecMatrix4x4(edge.transform.getTransformationMatrix(false, LengthUnit::CENTIMETER));
                const auto rotated = matrix::matMul(rotate, current);
                edge.transform.setTransformationMatrix(rotated, LengthUnit::CENTIMETER);
                edge.transform.setReferenceFrame(edge.to);
            }
        }
        return MultiDeviceCalibrationHandler(MultiDeviceCalibrationData{1, 0, edges, {}});
    }

    /// Pose of camera `frame` w.r.t. the rig base, composed from the (possibly perturbed) guesses, in meters.
    Transform seedToRigBase(const CoordinateFrame& frame, const MultiDeviceCalibrationHandler& guesses) const {
        const auto& reference = deviceReference.at(frame.deviceId);
        Transform referenceToRigBase = identityTransform();
        if(reference != baseFrame) {
            referenceToRigBase = matrix::toVecMatrix4x4(guesses.getTransform(reference, baseFrame, LengthUnit::METER));
        }
        return matrix::matMul(referenceToCamera(frame), inverted(referenceToRigBase));
    }

    /// Known metric baselines within a subset of cameras, indexed by their position in the subset.
    std::vector<dcl::EdgeBaseline> baselineEdges(const std::vector<size_t>& subset) const {
        std::vector<dcl::EdgeBaseline> edges;
        for(size_t i = 0; i < subset.size(); ++i) {
            for(size_t j = i + 1; j < subset.size(); ++j) {
                const auto& a = cameras[subset[i]].frame;
                const auto& b = cameras[subset[j]].frame;
                std::optional<float> distance = findKnownDistance(a, b);
                if(!distance.has_value() && a.deviceId == b.deviceId) {
                    distance = calibrations.at(a.deviceId).getBaselineDistance(a.socket, b.socket, true, LengthUnit::METER);
                }
                if(!distance.has_value() || *distance <= 0.0f) continue;
                edges.push_back({i, j, static_cast<double>(*distance)});
            }
        }
        return edges;
    }
#endif
};

MultiDeviceCalibration::MultiDeviceCalibration() : pimpl(spimpl::make_unique_impl<Impl>()) {}
MultiDeviceCalibration::~MultiDeviceCalibration() = default;

void MultiDeviceCalibration::buildInternal() {
    sync->out.link(syncInput);
    // The streams come from several devices, so they can only be brought together on the host
    sync->setRunOnHost(true);
}

void MultiDeviceCalibration::addCamera(const CoordinateFrame& frame, Node::Output& source) {
    DAI_CHECK_V(frame.isQualified(), "MultiDeviceCalibration cameras must specify both a device id and a camera socket, got {}", toString(frame));
    const auto duplicate = std::find_if(pimpl->cameras.begin(), pimpl->cameras.end(), [&frame](const CameraStream& camera) { return camera.frame == frame; });
    DAI_CHECK_V(duplicate == pimpl->cameras.end(), "Camera {} was already registered on this MultiDeviceCalibration node", toString(frame));

    CameraStream camera;
    camera.frame = frame;
    camera.inputName = frameKey(frame);
    pimpl->cameras.push_back(camera);

    auto& input = inputs[camera.inputName];
    input.setBlocking(false);
    input.setMaxSize(2);
    source.link(input);
}

std::shared_ptr<MultiDeviceCalibration> MultiDeviceCalibration::build(const std::vector<std::pair<CoordinateFrame, Node::Output*>>& sources) {
    for(const auto& [frame, source] : sources) {
        DAI_CHECK_V(source != nullptr, "MultiDeviceCalibration source for {} is null", toString(frame));
        addCamera(frame, *source);
    }
    return std::static_pointer_cast<MultiDeviceCalibration>(shared_from_this());
}

void MultiDeviceCalibration::setInitialGuess(const CoordinateFrame& from, const CoordinateFrame& to, const Extrinsics& guess) {
    RigEdge edge;
    edge.from = from;
    edge.to = to;
    edge.transform = guess;
    edge.transform.setReferenceFrame(to);
    edge.source = "initial-guess";
    pimpl->initialGuesses.push_back(edge);
}

void MultiDeviceCalibration::setDeviceCalibration(const std::string& deviceId, const CalibrationHandler& calibration) {
    DAI_CHECK_V(!deviceId.empty(), "MultiDeviceCalibration device calibration needs a non-empty device id");
    pimpl->calibrations.insert_or_assign(deviceId, calibration);
}

void MultiDeviceCalibration::setKnownDistance(const CoordinateFrame& from, const CoordinateFrame& to, float distance, LengthUnit unit) {
    DAI_CHECK_V(distance > 0.0f, "MultiDeviceCalibration known distance must be positive, got {}", distance);
    pimpl->knownDistances[{from, to}] = distance * getDistanceUnitScale(LengthUnit::METER, unit);
}

void MultiDeviceCalibration::setSampleCount(size_t sampleCount) {
    DAI_CHECK_V(sampleCount >= 1, "MultiDeviceCalibration needs at least one image set, got {}", sampleCount);
    pimpl->sampleCount = sampleCount;
}

void MultiDeviceCalibration::setContinuous(bool continuous) {
    pimpl->continuous = continuous;
}

void MultiDeviceCalibration::setPerformanceMode(DynamicCalibrationControl::PerformanceMode mode) {
    pimpl->performanceMode = mode;
    pimpl->autoStrategy = false;
}

void MultiDeviceCalibration::setAutoStrategy(bool enable) {
    pimpl->autoStrategy = enable;
}

void MultiDeviceCalibration::setGuessYawSweep(const std::vector<float>& offsetsDegrees) {
    pimpl->guessYawOffsets = offsetsDegrees.empty() ? std::vector<float>{0.0f} : offsetsDegrees;
}

void MultiDeviceCalibration::setEstimateInterDeviceScale(bool enable) {
    pimpl->estimateScale = enable;
}

void MultiDeviceCalibration::setMethod(Method method) {
    pimpl->method = method;
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

}  // namespace node
}  // namespace dai

    #include <opencv2/core.hpp>

namespace dai {
namespace node {

void MultiDeviceCalibration::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    auto& cameras = pimpl->cameras;

    DAI_CHECK_V(cameras.size() >= 2, "MultiDeviceCalibration needs at least two cameras, got {}. Register them with build() or addCamera().", cameras.size());
    std::set<std::string> deviceIds;
    for(const auto& camera : cameras) {
        deviceIds.insert(camera.frame.deviceId);
    }
    DAI_CHECK_V(deviceIds.size() >= 2,
                "MultiDeviceCalibration estimates transformations between different devices, but all its cameras are on device {}. Use the "
                "DynamicCalibration node for a single device.",
                *deviceIds.begin());

    // Per-device calibration is authoritative. An explicitly supplied one (setDeviceCalibration, e.g. for a recorded
    // session) takes precedence; otherwise it is read from the live device assigned to the pipeline.
    for(const auto& device : getParentPipeline().getAllAssignedDevices()) {
        if(deviceIds.count(device->getDeviceId()) == 0) continue;
        pimpl->calibrations.emplace(device->getDeviceId(), device->getCalibration());
    }
    for(const auto& deviceId : deviceIds) {
        DAI_CHECK_V(pimpl->calibrations.count(deviceId) == 1,
                    "MultiDeviceCalibration was given cameras of device {}, but neither is that device assigned to the pipeline nor was its "
                    "calibration supplied with setDeviceCalibration() - required to replay a recorded session.",
                    deviceId);
    }

    // The rig is expressed w.r.t. the first registered camera; the initial pose of every other camera is composed from
    // the user supplied inter-device guesses and the intra-device calibration of its own device.
    const auto baseFrame = cameras.front().frame;
    pimpl->baseFrame = baseFrame;
    for(const auto& camera : cameras) {
        pimpl->deviceReference.emplace(camera.frame.deviceId, camera.frame);
    }
    // The feature-tracks method estimates the geometry from scratch, so it needs neither an initial guess nor the
    // dynamic calibration library seeding below.
    if(pimpl->method == Method::DYNAMIC_CALIBRATION) {
        const MultiDeviceCalibrationHandler guesses(MultiDeviceCalibrationData{1, 0, pimpl->initialGuesses, {}});
        for(auto& camera : cameras) {
            const auto& reference = pimpl->deviceReference.at(camera.frame.deviceId);
            Transform referenceToRigBase = identityTransform();
            if(reference != baseFrame) {
                DAI_CHECK_V(guesses.canTransform(reference, baseFrame),
                            "MultiDeviceCalibration has no initial guess connecting {} to {}. Supply a rough one with setInitialGuess().",
                            toString(reference),
                            toString(baseFrame));
                referenceToRigBase = matrix::toVecMatrix4x4(guesses.getTransform(reference, baseFrame, LengthUnit::METER));
            }
            // T_camera<-rigBase == T_camera<-deviceReference * T_deviceReference<-rigBase
            camera.toRigBase = matrix::matMul(pimpl->referenceToCamera(camera.frame), inverted(referenceToRigBase));
        }
    } else {
        for(auto& camera : cameras) {
            camera.toRigBase = pimpl->referenceToCamera(camera.frame);
        }
    }

    // Wait for the first synchronized set to learn the resolutions and intrinsics actually produced
    logger->info("Waiting for the first synchronized image set of {} cameras", cameras.size());
    std::shared_ptr<MessageGroup> group;
    {
        auto blockEvent = this->inputBlockEvent();
        group = syncInput.get<MessageGroup>();
    }
    if(group == nullptr) return;
    for(auto& camera : cameras) {
        const auto frame = group->get<ImgFrame>(camera.inputName);
        DAI_CHECK_V(frame != nullptr, "MultiDeviceCalibration is missing an image of camera {}", toString(camera.frame));
        camera.resolution = {frame->getWidth(), frame->getHeight()};
        camera.transformation = frame->getTransformation();
    }

    for(const auto& deviceId : deviceIds) {
        pimpl->dclDevices.emplace(deviceId, pimpl->dcl.addDevice());
    }
    for(auto& camera : cameras) {
        const auto calibration = DclUtils::createDclCalibration(camera.transformation.getIntrinsicMatrix(),
                                                                camera.transformation.getDistortionCoefficients(),
                                                                matrix::extractRotationMatrix(camera.toRigBase),
                                                                matrix::extractTranslationVector(camera.toRigBase),
                                                                camera.transformation.getDistortionModel());
        // Preserve physical device ownership in DCL. The socket-less overload
        // avoids socket-number collisions between devices.
        camera.sensor = pimpl->dcl.addSensor(
            pimpl->dclDevices.at(camera.frame.deviceId), calibration, dcl::resolution_t{camera.resolution.first, camera.resolution.second});
    }

    size_t loaded = 0;
    while(mainLoop()) {
        if(group == nullptr) {
            auto blockEvent = this->inputBlockEvent();
            group = syncInput.get<MessageGroup>();
            if(group == nullptr) continue;
        }

        dcl::DeviceImageList images;
        images.reserve(cameras.size());
        dcl::timestamp_t timestamp = 0;
        bool complete = true;
        for(const auto& camera : cameras) {
            const auto frame = group->get<ImgFrame>(camera.inputName);
            if(frame == nullptr) {
                logger->trace("Missing image of camera {} in the synchronized group", toString(camera.frame));
                complete = false;
                break;
            }
            if(timestamp == 0) {
                timestamp = static_cast<dcl::timestamp_t>(frame->getTimestamp().time_since_epoch().count());
            }
            auto cvFrame = frame->getCvFrame();
            images.emplace_back(camera.sensor, DclUtils::cvMatToImageData(cvFrame));
        }
        group = nullptr;
        if(!complete) continue;
        auto loadResult = pimpl->dcl.loadImages(images, timestamp);
        if(!loadResult.passed()) {
            logger->trace("Failed to load the synchronized image set: {}", loadResult.errorMessage());
            continue;
        }
        if(++loaded < pimpl->sampleCount) continue;

        loaded = 0;
        if(pimpl->method == Method::FEATURE_TRACKS) {
            estimateFromStereoGraph();
        } else {
            estimateFromCalibrationGraph();
        }
        if(!pimpl->continuous) {
            logger->info("Rig calibration emitted, MultiDeviceCalibration is done. Use setContinuous(true) to keep estimating.");
            return;
        }
    }
}

void MultiDeviceCalibration::estimateFromCalibrationGraph() {
    auto& logger = ThreadedNode::pimpl->logger;
    const auto& cameras = pimpl->cameras;
    const size_t numCameras = cameras.size();

    // Cameras that never observed a common scene cannot be related, so decompose the set into the components DCL can
    // actually solve - it expects a single connected one.
    std::vector<size_t> component(numCameras);
    std::iota(component.begin(), component.end(), 0);
    const std::function<size_t(size_t)> root = [&component, &root](size_t index) {
        return component[index] == index ? index : component[index] = root(component[index]);
    };
    for(size_t i = 0; i < numCameras; ++i) {
        for(size_t j = i + 1; j < numCameras; ++j) {
            auto confidence = pimpl->dcl.computeDataConfidence({cameras[i].sensor, cameras[j].sensor});
            if(!confidence.passed() || confidence.value <= 0.0) continue;
            component[root(i)] = root(j);
        }
    }
    std::map<size_t, std::vector<size_t>> components;
    for(size_t i = 0; i < numCameras; ++i) {
        components[root(i)].push_back(i);
    }

    // Metric inter-device scale from the shared scene: for each device that exposes a stereo pair (both cameras
    // registered), triangulate scene points metrically using its factory baseline, then robustly align the two
    // devices' metric point clouds to recover the true inter-device distance. It is fed to the solve exactly like a
    // known distance, so the scale is observed from the scene instead of inherited from the initial guess.
    pimpl->autoKnownDistances.clear();
    std::map<std::string, std::vector<const CameraStream*>> deviceStreams;
    for(const auto& camera : cameras) {
        deviceStreams[camera.frame.deviceId].push_back(&camera);
    }
    const auto stereoSensors = [&](const std::string& deviceId) {
        const auto& streams = deviceStreams.at(deviceId);
        return dcl::StereoSensorPair{streams[0]->sensor, streams[1]->sensor};
    };
    const auto recoverInterDeviceScale = [&](const std::string& deviceA, const std::string& deviceB, std::vector<std::string>& notes) {
        if(!pimpl->estimateScale) return;
        const auto& refA = pimpl->deviceReference.at(deviceA);
        const auto& refB = pimpl->deviceReference.at(deviceB);
        if(pimpl->findKnownDistance(refA, refB).has_value()) return;  // user distance or already recovered
        if(deviceStreams[deviceA].size() < 2 || deviceStreams[deviceB].size() < 2) return;
        const auto calibration = pimpl->dcl.estimateInterDeviceCalibration({stereoSensors(deviceA), stereoSensors(deviceB)});
        if(calibration.passed() && !calibration.value.poses.empty()) {
            const auto& result = calibration.value.poses.front().quality;
            pimpl->autoKnownDistances[{refA, refB}] = static_cast<float>(result.distanceMeters);
            notes.push_back(fmt::format("recovered the metric distance between {} and {} from the scene: {:.1f} cm ({} inliers, rmse {:.1f} cm)",
                                        deviceA,
                                        deviceB,
                                        result.distanceMeters * 100.0,
                                        result.inliers,
                                        result.rmseMeters * 100.0));
            logger->info("Recovered inter-device distance {} <-> {} from the scene: {:.1f} cm", deviceA, deviceB, result.distanceMeters * 100.0);
        } else {
            notes.push_back(fmt::format("could not recover the metric scale between {} and {} from the scene", deviceA, deviceB));
            logger->info("Inter-device scale {} <-> {} unobservable from the scene", deviceA, deviceB);
        }
    };

    MultiDeviceCalibrationData rig;
    rig.timestamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    std::vector<std::string> notes;
    double dataConfidence = 0.0;

    // A single solve can land in a poor local minimum or fail a coverage gate, so instead of trusting one attempt the
    // node sweeps several strategies and keeps, per device, the edge that scores best on the dynamic calibration
    // library's own metrics. The attempted performance modes, whether the initial camera centers are kept and the yaw
    // perturbations of the initial guess make up the search; an explicit setPerformanceMode() narrows it to one solve.
    std::vector<DynamicCalibrationControl::PerformanceMode> modes;
    if(pimpl->autoStrategy) {
        modes = {DynamicCalibrationControl::PerformanceMode::DEFAULT,
                 DynamicCalibrationControl::PerformanceMode::RELAXED_COVERAGE,
                 DynamicCalibrationControl::PerformanceMode::OPTIMIZE_PERFORMANCE,
                 DynamicCalibrationControl::PerformanceMode::STATIC_SCENERY,
                 DynamicCalibrationControl::PerformanceMode::SKIP_CHECKS};
    } else {
        modes = {pimpl->performanceMode};
    }
    // keepCameraCenters=true is incompatible with the known baseline edges the scale relies on, so the centers are
    // always solved; the strategies vary instead by performance mode, joint vs. pairwise and the guess perturbation.
    const std::vector<bool> keepCentersOptions = {false};
    const std::vector<float> yawOffsets = pimpl->autoStrategy ? pimpl->guessYawOffsets : std::vector<float>{0.0f};

    for(const auto& [componentRoot, indices] : components) {
        (void)componentRoot;
        std::set<std::string> componentDevices;
        for(const auto index : indices) {
            componentDevices.insert(cameras[index].frame.deviceId);
        }
        if(componentDevices.size() < 2) {
            notes.push_back(fmt::format("cameras of device {} did not observe a scene shared with another device", *componentDevices.begin()));
            continue;
        }

        const std::vector<std::string> componentDeviceList(componentDevices.begin(), componentDevices.end());
        for(size_t i = 0; i < componentDeviceList.size(); ++i) {
            for(size_t j = i + 1; j < componentDeviceList.size(); ++j) {
                recoverInterDeviceScale(componentDeviceList[i], componentDeviceList[j], notes);
            }
        }

        const auto baseDeviceId = pimpl->baseFrame.deviceId;
        if(componentDevices.count(baseDeviceId) == 0) {
            notes.push_back(fmt::format("devices [{}] did not share a scene with the reference device {}", fmt::join(componentDevices, ", "), baseDeviceId));
            continue;
        }

        {
            std::vector<std::shared_ptr<const dcl::CameraSensorHandle>> componentSensors;
            for(const auto index : indices) componentSensors.push_back(cameras[index].sensor);
            auto confidence = pimpl->dcl.computeDataConfidence(componentSensors);
            if(confidence.passed()) dataConfidence = std::max(dataConfidence, confidence.value);
        }

        std::map<std::string, std::vector<size_t>> byDevice;
        for(const auto index : indices) byDevice[cameras[index].frame.deviceId].push_back(index);

        const auto& baseReference = pimpl->deviceReference.at(baseDeviceId);

        // Only the inter-device edges are the result - the intra-device geometry stays with the device. Emitting them
        // as a star around the reference device keeps the rig a forest and every edge independent.
        for(const auto& deviceId : componentDevices) {
            if(deviceId == baseDeviceId) continue;
            const auto& deviceReferenceFrame = pimpl->deviceReference.at(deviceId);

            struct AttemptMetadata {
                std::vector<size_t> subset;
                std::string description;
            };
            std::vector<dcl::CalibrationAttempt> attempts;
            std::vector<AttemptMetadata> attemptMetadata;
            const auto addAttempt = [&](const std::vector<size_t>& subset,
                                        float yawDegrees,
                                        DynamicCalibrationControl::PerformanceMode mode,
                                        bool keepCenters,
                                        const char* approach) {
                const auto guesses = pimpl->guessesWithYaw(deviceId, yawDegrees);
                dcl::CalibrationAttempt attempt;
                attempt.sensors.reserve(subset.size());
                attempt.initialCalibrations.reserve(subset.size());
                attempt.mode = DclUtils::daiPerformanceModeToDclPerformanceMode(mode);
                attempt.keepCameraCenters = keepCenters;
                attempt.keptBaselineEdges = pimpl->baselineEdges(subset);
                for(const auto index : subset) {
                    const auto toRigBase = pimpl->seedToRigBase(cameras[index].frame, guesses);
                    attempt.sensors.push_back(cameras[index].sensor);
                    attempt.initialCalibrations.push_back(DclUtils::createDclCalibration(cameras[index].transformation.getIntrinsicMatrix(),
                                                                                         cameras[index].transformation.getDistortionCoefficients(),
                                                                                         matrix::extractRotationMatrix(toRigBase),
                                                                                         matrix::extractTranslationVector(toRigBase),
                                                                                         cameras[index].transformation.getDistortionModel()));
                }
                attempts.push_back(std::move(attempt));
                attemptMetadata.push_back(
                    {subset, fmt::format("{}, mode {}, keepCenters {}, yaw {:+.0f}deg", approach, static_cast<int>(mode), keepCenters, yawDegrees)});
            };

            // Pairwise: only the reference device and this one, which is robust when not every pair shares a scene.
            std::vector<size_t> pairSubset = byDevice.at(baseDeviceId);
            pairSubset.insert(pairSubset.end(), byDevice.at(deviceId).begin(), byDevice.at(deviceId).end());
            for(const float yaw : yawOffsets) {
                for(const auto mode : modes) {
                    for(const bool keepCenters : keepCentersOptions) {
                        addAttempt(pairSubset, yaw, mode, keepCenters, "pairwise");
                    }
                }
            }

            // Joint: solve all cameras of the component together, letting the other devices constrain this one.
            if(componentDevices.size() > 2) {
                const std::vector<size_t> jointSubset(indices.begin(), indices.end());
                for(const auto mode : modes) {
                    for(const bool keepCenters : keepCentersOptions) {
                        addAttempt(jointSubset, 0.0f, mode, keepCenters, "joint");
                    }
                }
            }

            auto best = pimpl->dcl.findBestCalibration(attempts);
            if(!best.passed() || best.value.attemptIndex >= attemptMetadata.size()) {
                notes.push_back(fmt::format("no strategy produced a valid calibration for device {}", deviceId));
                logger->warn("No strategy produced a valid calibration for device {}", deviceId);
                continue;
            }

            const auto& selectedMetadata = attemptMetadata[best.value.attemptIndex];
            std::map<CoordinateFrame, Transform> estimated;
            for(size_t k = 0; k < selectedMetadata.subset.size(); ++k) {
                estimated.emplace(cameras[selectedMetadata.subset[k]].frame, DclUtils::calibrationHandleToTransform(best.value.calibration.calibrations[k]));
            }
            const auto baseIt = estimated.find(baseReference);
            const auto deviceIt = estimated.find(deviceReferenceFrame);
            if(baseIt == estimated.end() || deviceIt == estimated.end()) {
                notes.push_back(fmt::format("selected strategy did not return both reference frames for device {}", deviceId));
                continue;
            }

            auto transform = matrix::matMul(baseIt->second, inverted(deviceIt->second));
            const auto scale = resolveScale(transform, baseReference, deviceReferenceFrame, notes);
            for(int i = 0; i < 3; ++i) {
                transform[i][3] *= scale;
            }
            auto translation = matrix::extractTranslationVector(transform);
            for(auto& value : translation) {
                value *= getDistanceUnitScale(LengthUnit::CENTIMETER, LengthUnit::METER);
            }

            RigEdge edge;
            edge.from = deviceReferenceFrame;
            edge.to = baseReference;
            edge.transform =
                Extrinsics(matrix::extractRotationMatrix(transform), Point3f(translation[0], translation[1], translation[2]), baseReference.socket);
            edge.transform.setReferenceFrame(baseReference);
            edge.timestamp = rig.timestamp;
            edge.source = "multi-device-calibration";
            rig.edges.push_back(edge);
            notes.push_back(fmt::format("device {}: {} (confidence {:.3f}, Sampson {:.4f})",
                                        deviceId,
                                        selectedMetadata.description,
                                        best.value.calibrationConfidence,
                                        best.value.calibration.sampsonErrorNew));
            logger->info("Device {} rig edge from {} (confidence {:.3f}, Sampson error {:.4f})",
                         deviceId,
                         selectedMetadata.description,
                         best.value.calibrationConfidence,
                         best.value.calibration.sampsonErrorNew);
        }
    }

    const auto info = fmt::format("{}", fmt::join(notes, "; "));
    if(rig.edges.empty()) {
        logger->warn("No inter-device transformation could be estimated: {}", info);
        rigCalibration.send(std::make_shared<MultiDeviceCalibrationResult>(info));
        return;
    }
    logger->info("Estimated {} inter-device transformation(s){}{}", rig.edges.size(), info.empty() ? "" : ", ", info);
    rigCalibration.send(std::make_shared<MultiDeviceCalibrationResult>(rig, dataConfidence, info));
}

void MultiDeviceCalibration::estimateFromStereoGraph() {
    auto& logger = ThreadedNode::pimpl->logger;
    const auto& cameras = pimpl->cameras;

    struct DeviceCameras {
        std::string deviceId;
        size_t leftIndex = 0;
        size_t rightIndex = 0;
        bool hasRight = false;
    };
    std::vector<DeviceCameras> devices;
    const auto deviceIndex = [&devices](const std::string& deviceId) -> DeviceCameras& {
        for(auto& device : devices)
            if(device.deviceId == deviceId) return device;
        devices.push_back(DeviceCameras{deviceId, 0, 0, false});
        return devices.back();
    };
    for(size_t i = 0; i < cameras.size(); ++i) {
        auto& device = deviceIndex(cameras[i].frame.deviceId);
        if(cameras[i].frame == pimpl->deviceReference.at(cameras[i].frame.deviceId)) {
            device.leftIndex = i;
        } else if(!device.hasRight) {
            device.rightIndex = i;
            device.hasRight = true;
        }
    }
    for(const auto& device : devices) {
        DAI_CHECK_V(device.hasRight,
                    "MultiDeviceCalibration FEATURE_TRACKS needs a stereo pair (two cameras) per device, but device {} has only one registered camera",
                    device.deviceId);
    }

    const std::string baseDeviceId = pimpl->baseFrame.deviceId;
    const auto baseDevice = std::find_if(devices.begin(), devices.end(), [&baseDeviceId](const auto& device) { return device.deviceId == baseDeviceId; });
    const size_t baseDeviceIndex = static_cast<size_t>(std::distance(devices.begin(), baseDevice));
    std::vector<dcl::StereoSensorPair> stereoDevices;
    stereoDevices.reserve(devices.size());
    for(const auto& device : devices) {
        stereoDevices.push_back({cameras[device.leftIndex].sensor, cameras[device.rightIndex].sensor});
    }

    const auto calibration = pimpl->dcl.estimateInterDeviceCalibration(stereoDevices, baseDeviceIndex);
    std::vector<std::string> notes;

    MultiDeviceCalibrationData rig;
    rig.timestamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    const auto& baseReference = pimpl->deviceReference.at(baseDeviceId);
    if(calibration.passed()) {
        for(const auto& pose : calibration.value.poses) {
            Transform transform(4, std::vector<float>(4));
            for(int row = 0; row < 4; ++row)
                for(int column = 0; column < 4; ++column) transform[row][column] = static_cast<float>(pose.baseFromDevice(row, column));

            const auto& device = devices[pose.deviceIndex];
            const auto& reference = devices[pose.referenceIndex];
            RigEdge edge;
            edge.from = pimpl->deviceReference.at(device.deviceId);
            edge.to = baseReference;
            edge.transform = Extrinsics(transform, baseReference.socket, LengthUnit::METER);
            edge.transform.setReferenceFrame(baseReference);
            edge.timestamp = rig.timestamp;
            edge.source = "dcl-feature-tracks";
            rig.edges.push_back(edge);

            notes.push_back(fmt::format("device {} <- {}: {} correspondences, {} inliers, rigid-fit RMSE {:.3f} m, |t| {:.3f} m",
                                        reference.deviceId,
                                        device.deviceId,
                                        pose.quality.correspondences,
                                        pose.quality.inliers,
                                        pose.quality.rmseMeters,
                                        pose.quality.distanceMeters));
        }
        for(const auto deviceIndex : calibration.value.disconnectedDeviceIndices) {
            notes.push_back(fmt::format("device {} could not be connected to the reference device {}", devices[deviceIndex].deviceId, baseDeviceId));
        }
    } else {
        notes.push_back("DCL rejected the inter-device calibration input");
    }

    const auto info = fmt::format("{}", fmt::join(notes, "; "));
    if(rig.edges.empty()) {
        logger->warn("No inter-device transformation could be estimated: {}", info);
        rigCalibration.send(std::make_shared<MultiDeviceCalibrationResult>(info));
        return;
    }
    logger->info("Estimated {} inter-device transformation(s) from DCL feature tracks{}{}", rig.edges.size(), info.empty() ? "" : ", ", info);
    rigCalibration.send(std::make_shared<MultiDeviceCalibrationResult>(rig, 1.0, info));
}

float MultiDeviceCalibration::resolveScale(const std::vector<std::vector<float>>& transform,
                                           const CoordinateFrame& baseReference,
                                           const CoordinateFrame& reference,
                                           std::vector<std::string>& notes) const {
    // The translation between cameras of different devices is only observable up to scale, so it has to be fixed by a
    // known distance. Scaling the edge translation by `s` moves the whole device along the edge, so the distance
    // between a camera of the base device (center `centerA`, unaffected) and one of the moved device is
    // ||s * t + v|| - one quadratic equation in `s`.
    std::vector<std::pair<std::pair<CoordinateFrame, CoordinateFrame>, float>> combined(pimpl->knownDistances.begin(), pimpl->knownDistances.end());
    combined.insert(combined.end(), pimpl->autoKnownDistances.begin(), pimpl->autoKnownDistances.end());
    for(const auto& [pair, distance] : combined) {
        for(const auto& ordered : {std::pair{pair.first, pair.second}, std::pair{pair.second, pair.first}}) {
            const auto& a = ordered.first;
            const auto& b = ordered.second;
            if(a.deviceId != baseReference.deviceId || b.deviceId != reference.deviceId) continue;

            // Centers expressed in the base device's reference frame
            const auto centerA = cameraCenter(pimpl->referenceToCamera(a));
            const auto translation = matrix::extractTranslationVector(transform);
            const auto centerB = cameraCenter(matrix::matMul(pimpl->referenceToCamera(b), inverted(transform)));

            std::vector<float> offset(3);
            for(int i = 0; i < 3; ++i) {
                offset[i] = centerB[i] - translation[i] - centerA[i];
            }
            const float a2 = dot(translation, translation);
            const float b2 = 2.0f * dot(translation, offset);
            const float c2 = dot(offset, offset) - distance * distance;
            const float discriminant = b2 * b2 - 4.0f * a2 * c2;
            if(a2 <= 0.0f || discriminant < 0.0f) {
                notes.push_back(fmt::format("the known distance between {} and {} cannot be matched, keeping the estimated scale", toString(a), toString(b)));
                return 1.0f;
            }
            const float rootDiscriminant = std::sqrt(discriminant);
            const float first = (-b2 + rootDiscriminant) / (2.0f * a2);
            const float second = (-b2 - rootDiscriminant) / (2.0f * a2);
            const float scale = std::abs(first - 1.0f) <= std::abs(second - 1.0f) ? first : second;
            if(scale <= 0.0f) {
                notes.push_back(
                    fmt::format("the known distance between {} and {} implies a non-positive scale, keeping the estimated one", toString(a), toString(b)));
                return 1.0f;
            }
            return scale;
        }
    }

    notes.push_back(
        fmt::format("no known distance fixes the scale between {} and {}, so the magnitude of the initial guess is kept - supply one with setKnownDistance()",
                    toString(baseReference),
                    toString(reference)));
    return 1.0f;
}

#else

void MultiDeviceCalibration::run() {
    DAI_CHECK_V(false, "MultiDeviceCalibration requires depthai to be built with OpenCV support");
}

void MultiDeviceCalibration::estimateFromCalibrationGraph() {}

void MultiDeviceCalibration::estimateFromStereoGraph() {}

float MultiDeviceCalibration::resolveScale(const std::vector<std::vector<float>>&,
                                           const CoordinateFrame&,
                                           const CoordinateFrame&,
                                           std::vector<std::string>&) const {
    return 1.0f;
}

#endif

}  // namespace node
}  // namespace dai
