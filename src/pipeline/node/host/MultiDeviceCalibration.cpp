#include "depthai/pipeline/node/host/MultiDeviceCalibration.hpp"

#include <fmt/format.h>
#include <fmt/ranges.h>

#include <DynamicCalibration.hpp>
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
#include "pipeline/node/MultiDeviceScaleEstimation.hpp"
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
    std::shared_ptr<dcl::Device> dclDevice;

    std::vector<CameraStream> cameras;
    std::vector<RigEdge> initialGuesses;
    /// Known distances between camera centers, in meters, keyed by the frame pair.
    std::map<std::pair<CoordinateFrame, CoordinateFrame>, float> knownDistances;
    /// Metric distances recovered from the scene, in meters, keyed by the device reference frame pair. Recomputed each
    /// estimation cycle; a user `setKnownDistance()` always takes precedence over these.
    std::map<std::pair<CoordinateFrame, CoordinateFrame>, float> autoKnownDistances;
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    /// Synchronized images retained for the current estimation cycle, keyed by camera input name, used to recover the
    /// metric inter-device scale from the shared scene.
    std::map<std::string, std::vector<cv::Mat>> sampleImages;
#endif
    /// Calibration of every device involved, read from the live devices.
    std::map<std::string, CalibrationHandler> calibrations;
    /// Frame every device is represented by in the rig, i.e. its first registered camera.
    std::map<std::string, CoordinateFrame> deviceReference;

    size_t sampleCount = 10;
    bool continuous = false;
    bool estimateScale = true;
    DynamicCalibrationControl::PerformanceMode performanceMode = DynamicCalibrationControl::PerformanceMode::DEFAULT;

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
}

void MultiDeviceCalibration::setEstimateInterDeviceScale(bool enable) {
    pimpl->estimateScale = enable;
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

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
    for(const auto& camera : cameras) {
        pimpl->deviceReference.emplace(camera.frame.deviceId, camera.frame);
    }
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

    pimpl->dclDevice = pimpl->dcl.addDevice();
    for(auto& camera : cameras) {
        const auto calibration = DclUtils::createDclCalibration(camera.transformation.getIntrinsicMatrix(),
                                                                camera.transformation.getDistortionCoefficients(),
                                                                matrix::extractRotationMatrix(camera.toRigBase),
                                                                matrix::extractTranslationVector(camera.toRigBase),
                                                                camera.transformation.getDistortionModel());
        // All physical devices are registered as a single DCL device on purpose - DCL rejects sensor sets spanning
        // several of its devices, and the socket-less overload avoids the socket collisions between our devices.
        camera.sensor = pimpl->dcl.addSensor(pimpl->dclDevice, calibration, dcl::resolution_t{camera.resolution.first, camera.resolution.second});
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
        std::vector<std::pair<std::string, cv::Mat>> retained;
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
            if(pimpl->estimateScale) retained.emplace_back(camera.inputName, cvFrame.clone());
        }
        group = nullptr;
        if(!complete) continue;
        for(auto& [inputName, image] : retained) {
            pimpl->sampleImages[inputName].push_back(std::move(image));
        }

        auto loadResult = pimpl->dcl.loadImages(images, timestamp);
        if(!loadResult.passed()) {
            logger->trace("Failed to load the synchronized image set: {}", loadResult.errorMessage());
            continue;
        }
        if(++loaded < pimpl->sampleCount) continue;

        loaded = 0;
        estimate();
        if(!pimpl->continuous) {
            logger->info("Rig calibration emitted, MultiDeviceCalibration is done. Use setContinuous(true) to keep estimating.");
            return;
        }
    }
}

void MultiDeviceCalibration::estimate() {
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
            auto confidence = pimpl->dcl.computeDataConfidence(cameras[i].sensor, cameras[j].sensor);
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
    const auto toMatx33 = [](const std::array<std::array<float, 3>, 3>& m) {
        return cv::Matx33d(m[0][0], m[0][1], m[0][2], m[1][0], m[1][1], m[1][2], m[2][0], m[2][1], m[2][2]);
    };
    const auto buildViews = [&](const std::string& deviceId) {
        impl::StereoDeviceViews views;
        const auto& streams = deviceStreams.at(deviceId);
        const CameraStream* left = streams[0];
        const CameraStream* right = streams[1];
        views.intrinsicsLeft = toMatx33(left->transformation.getIntrinsicMatrix());
        views.intrinsicsRight = toMatx33(right->transformation.getIntrinsicMatrix());
        for(const float c : left->transformation.getDistortionCoefficients()) views.distortionLeft.push_back(c);
        for(const float c : right->transformation.getDistortionCoefficients()) views.distortionRight.push_back(c);
        // getCameraExtrinsics(a, b) is T_b<-a, so the left-from-right stereo transform is its inverse.
        const Transform leftFromRight = inverted(pimpl->calibrations.at(deviceId).getCameraExtrinsics(left->frame.socket, right->frame.socket, false, LengthUnit::METER));
        for(int i = 0; i < 4; ++i)
            for(int j = 0; j < 4; ++j) views.leftFromRight(i, j) = leftFromRight[i][j];
        views.left = pimpl->sampleImages[left->inputName];
        views.right = pimpl->sampleImages[right->inputName];
        return views;
    };
    const auto recoverInterDeviceScale = [&](const std::string& deviceA, const std::string& deviceB, std::vector<std::string>& notes) {
        if(!pimpl->estimateScale) return;
        const auto& refA = pimpl->deviceReference.at(deviceA);
        const auto& refB = pimpl->deviceReference.at(deviceB);
        if(pimpl->findKnownDistance(refA, refB).has_value()) return;  // user distance or already recovered
        if(deviceStreams[deviceA].size() < 2 || deviceStreams[deviceB].size() < 2) return;
        const impl::StereoDeviceViews viewsA = buildViews(deviceA);
        const impl::StereoDeviceViews viewsB = buildViews(deviceB);
        if(viewsA.left.size() < 2 || viewsB.left.size() < 2) return;
        const auto result = impl::estimateInterDeviceScale(viewsA, viewsB);
        if(result.observable) {
            pimpl->autoKnownDistances[{refA, refB}] = static_cast<float>(result.distanceMeters);
            notes.push_back(fmt::format("recovered the metric distance between {} and {} from the scene: {:.1f} cm ({} inliers, rmse {:.1f} cm)",
                                        deviceA,
                                        deviceB,
                                        result.distanceMeters * 100.0,
                                        result.inliers,
                                        result.rmseMeters * 100.0));
            logger->info("Recovered inter-device distance {} <-> {} from the scene: {:.1f} cm", deviceA, deviceB, result.distanceMeters * 100.0);
        } else {
            notes.push_back(fmt::format("could not recover the metric scale between {} and {} from the scene ({})", deviceA, deviceB, result.note));
            logger->info("Inter-device scale {} <-> {} unobservable from the scene: {}", deviceA, deviceB, result.note);
        }
    };

    MultiDeviceCalibrationData rig;
    rig.timestamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    std::vector<std::string> notes;
    double dataConfidence = 0.0;

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

        std::vector<std::shared_ptr<const dcl::CameraSensorHandle>> sensors;
        sensors.reserve(indices.size());
        for(const auto index : indices) {
            sensors.push_back(cameras[index].sensor);
        }

        // Metric scale: the distances that are known upfront. Distances between cameras of one device come from the
        // device calibration, the inter-device ones have to be supplied by the user.
        std::vector<dcl::EdgeBaseline> keptBaselineEdges;
        for(size_t i = 0; i < indices.size(); ++i) {
            for(size_t j = i + 1; j < indices.size(); ++j) {
                const auto& a = cameras[indices[i]].frame;
                const auto& b = cameras[indices[j]].frame;
                std::optional<float> distance = pimpl->findKnownDistance(a, b);
                if(!distance.has_value() && a.deviceId == b.deviceId) {
                    distance = pimpl->calibrations.at(a.deviceId).getBaselineDistance(a.socket, b.socket, true, LengthUnit::METER);
                }
                if(!distance.has_value() || *distance <= 0.0f) continue;
                keptBaselineEdges.push_back({i, j, static_cast<double>(*distance)});
            }
        }

        auto confidence = pimpl->dcl.computeDataConfidence(sensors);
        if(confidence.passed()) {
            dataConfidence = std::max(dataConfidence, confidence.value);
        }

        auto result = pimpl->dcl.findNewCalibration(
            sensors, DclUtils::daiPerformanceModeToDclPerformanceMode(pimpl->performanceMode), /*keepCameraCenters=*/false, keptBaselineEdges);
        if(!result.passed()) {
            notes.push_back(fmt::format("estimation failed for devices [{}]: {}", fmt::join(componentDevices, ", "), result.errorMessage()));
            logger->warn("Rig estimation failed for devices [{}]: {}", fmt::join(componentDevices, ", "), result.errorMessage());
            continue;
        }
        if(result.value.calibrations.size() != sensors.size()) {
            notes.push_back("the dynamic calibration library returned an unexpected number of calibrations");
            continue;
        }

        // Poses of the component's cameras w.r.t. its first camera, as estimated
        std::map<CoordinateFrame, Transform> estimated;
        for(size_t i = 0; i < indices.size(); ++i) {
            estimated.emplace(cameras[indices[i]].frame, DclUtils::calibrationHandleToTransform(result.value.calibrations[i]));
        }

        // Only the inter-device edges are the result - the intra-device geometry stays with the device. Emitting them
        // as a star around the component's base device keeps the rig a forest and every edge independent.
        const auto& baseReference = pimpl->deviceReference.at(cameras[indices.front()].frame.deviceId);
        const auto& baseTransform = estimated.at(baseReference);
        for(const auto& deviceId : componentDevices) {
            const auto& reference = pimpl->deviceReference.at(deviceId);
            if(reference == baseReference) continue;
            const auto found = estimated.find(reference);
            if(found == estimated.end()) {
                notes.push_back(fmt::format("device {} is only represented by cameras outside of the solved set", deviceId));
                continue;
            }

            // T_baseReference<-reference
            auto transform = matrix::matMul(baseTransform, inverted(found->second));
            const auto scale = resolveScale(transform, baseReference, reference, notes);
            for(int i = 0; i < 3; ++i) {
                transform[i][3] *= scale;
            }

            auto translation = matrix::extractTranslationVector(transform);
            for(auto& value : translation) {
                value *= getDistanceUnitScale(LengthUnit::CENTIMETER, LengthUnit::METER);
            }

            RigEdge edge;
            edge.from = reference;
            edge.to = baseReference;
            edge.transform =
                Extrinsics(matrix::extractRotationMatrix(transform), Point3f(translation[0], translation[1], translation[2]), baseReference.socket);
            edge.transform.setReferenceFrame(baseReference);
            edge.timestamp = rig.timestamp;
            edge.source = "multi-device-calibration";
            rig.edges.push_back(edge);
        }
    }

    const auto info = fmt::format("{}", fmt::join(notes, "; "));
    pimpl->sampleImages.clear();
    if(rig.edges.empty()) {
        logger->warn("No inter-device transformation could be estimated: {}", info);
        rigCalibration.send(std::make_shared<MultiDeviceCalibrationResult>(info));
        return;
    }
    logger->info("Estimated {} inter-device transformation(s){}{}", rig.edges.size(), info.empty() ? "" : ", ", info);
    rigCalibration.send(std::make_shared<MultiDeviceCalibrationResult>(rig, dataConfidence, info));
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
        for(const auto ordered : {std::pair{pair.first, pair.second}, std::pair{pair.second, pair.first}}) {
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

void MultiDeviceCalibration::estimate() {}

float MultiDeviceCalibration::resolveScale(const std::vector<std::vector<float>>&,
                                           const CoordinateFrame&,
                                           const CoordinateFrame&,
                                           std::vector<std::string>&) const {
    return 1.0f;
}

#endif

}  // namespace node
}  // namespace dai
