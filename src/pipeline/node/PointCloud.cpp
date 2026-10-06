#include "depthai/pipeline/node/PointCloud.hpp"

#include <spdlog/logger.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <exception>
#include <future>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>

#ifdef DEPTHAI_ENABLE_KOMPUTE
    #include "kompute/Kompute.hpp"
#endif

#include "depthai/beta/device/MultiDeviceCalibrationHandler.hpp"
#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/common/Point3fRGBA.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "device/CalibrationHandler.hpp"
#include "pipeline/Pipeline.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/utilities/Alignment/AlignmentUtilities.hpp"

#ifdef DEPTHAI_ENABLE_KOMPUTE
    #include "depthai/shaders/depth2pointcloud.hpp"
#endif
#include "utility/PimplImpl.hpp"

namespace dai {
namespace node {

// ── Impl: apply / get methods ──

void PointCloud::Impl::setLogger(const std::shared_ptr<::spdlog::logger>& log) {
    logger = log;
}

bool PointCloud::Impl::usesGPU() const {
    return computeMethod == ComputeMethod::GPU;
}

void PointCloud::Impl::computePointCloudDense(const uint8_t* depthData, std::vector<Point3f>& points) {
    if(!intrinsicsSet) {
        throw std::runtime_error("Intrinsics not set");
    }

    points.resize(size);

    switch(computeMethod) {
        case ComputeMethod::CPU:
            computePointCloudDenseCPU(depthData, points);
            break;
        case ComputeMethod::CPU_MT:
            computePointCloudDenseCPUMT(depthData, points);
            break;
        case ComputeMethod::GPU:
            // A platform backend is used through computeDenseOnGpu; this is the Kompute path
            if(hasDistortion) {
                if(!gpuDistortionFallbackWarned) {
                    if(logger) logger->warn("GPU compute does not support depth undistortion yet, falling back to CPU");
                    gpuDistortionFallbackWarned = true;
                }
                computePointCloudDenseCPU(depthData, points);
            } else {
                computePointCloudDenseGPU(depthData, points);
            }
            break;
    }
}

void PointCloud::Impl::computePointCloudDenseColored(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& points) {
    if(!intrinsicsSet) {
        throw std::runtime_error("Intrinsics not set");
    }

    points.resize(size);

    switch(computeMethod) {
        case ComputeMethod::CPU:
            computePointCloudDenseColoredCPU(depthData, colorData, points);
            break;
        case ComputeMethod::CPU_MT:
            computePointCloudDenseColoredCPUMT(depthData, colorData, points);
            break;
        case ComputeMethod::GPU:
            // Kompute path doesn't support color yet, fall back to CPU
            if(logger) logger->warn("GPU compute does not support colorization yet, falling back to CPU");
            computePointCloudDenseColoredCPU(depthData, colorData, points);
            break;
    }
}

const Point3f* PointCloud::Impl::computeDenseOnGpu(const uint8_t* depthData, const std::shared_ptr<Memory>& depthMemory) {
    if(!gpuBackend) return nullptr;
    if(!intrinsicsSet) throw std::runtime_error("Intrinsics not set");
    try {
        const auto* dense = gpuBackend->computeDense(gpuGeometry(), depthData, depthMemory);
        if(dense == nullptr) throw std::runtime_error("backend returned no points");
        return dense;
    } catch(const std::exception& ex) {
        dropGpuBackend(ex.what());
        return nullptr;
    }
}

const Point3fRGBA* PointCloud::Impl::computeDenseColoredOnGpu(const uint8_t* depthData,
                                                              const std::shared_ptr<Memory>& depthMemory,
                                                              const uint8_t* colorData,
                                                              const std::shared_ptr<Memory>& colorMemory) {
    if(!gpuBackend) return nullptr;
    if(!intrinsicsSet) throw std::runtime_error("Intrinsics not set");
    try {
        const auto* dense = gpuBackend->computeDenseColored(gpuGeometry(), depthData, depthMemory, colorData, colorMemory);
        if(dense == nullptr) throw std::runtime_error("backend returned no points");
        return dense;
    } catch(const std::exception& ex) {
        dropGpuBackend(ex.what());
        return nullptr;
    }
}

template <typename PointT>
void PointCloud::Impl::gatherPoints(const PointT* dense, size_t count, bool organized, std::vector<PointT>& points) {
    points.resize(count);
    if(organized) {
        std::copy(dense, dense + count, points.begin());
        return;
    }
    size_t kept = 0;
    for(size_t i = 0; i < count; ++i) {
        if(dense[i].z > 0.0f) points[kept++] = dense[i];
    }
    points.resize(kept);
}

template void PointCloud::Impl::gatherPoints(const Point3f* dense, size_t count, bool organized, std::vector<Point3f>& points);
template void PointCloud::Impl::gatherPoints(const Point3fRGBA* dense, size_t count, bool organized, std::vector<Point3fRGBA>& points);

template <typename PointT>
void PointCloud::Impl::applyTransformation(std::vector<PointT>& points) {
    if(!hasExtrinsics) {
        if(logger) {
            logger->debug("No extrinsics set, skipping transformation");
        }
        return;
    }

    if(logger) {
        logger->debug("Applying coordinate system transformation");
    }

    switch(computeMethod) {
        case ComputeMethod::CPU:
            transformPointsCPU(points);
            break;
        case ComputeMethod::CPU_MT:
            transformPointsCPU(points);
            break;
        case ComputeMethod::GPU:
            transformPointsCPU(points);
            break;
    }
}

template <typename PointT>
void PointCloud::Impl::compactValidPoints(std::vector<PointT>& points) {
    size_t kept = 0;
    for(size_t i = 0; i < points.size(); ++i) {
        if(points[i].z > 0.0f) {
            if(kept != i) points[kept] = points[i];
            ++kept;
        }
    }
    points.resize(kept);
}

template void PointCloud::Impl::compactValidPoints(std::vector<Point3f>& points);
template void PointCloud::Impl::compactValidPoints(std::vector<Point3fRGBA>& points);

// Explicit template instantiations
template void PointCloud::Impl::applyTransformation(std::vector<Point3f>& points);
template void PointCloud::Impl::applyTransformation(std::vector<Point3fRGBA>& points);

template <typename PointT>
std::vector<PointT> PointCloud::Impl::filterValidPoints(const std::vector<PointT>& densePoints) {
    std::vector<PointT> sparsePoints;
    // Reserve half capacity as a heuristic - typically ~50% of depth pixels have valid (z > 0) data
    sparsePoints.reserve(densePoints.size() / 2);

    for(const auto& p : densePoints) {
        if(p.z > 0.0f) {
            sparsePoints.push_back(p);
        }
    }

    return sparsePoints;
}

// Explicit template instantiations
template std::vector<Point3f> PointCloud::Impl::filterValidPoints(const std::vector<Point3f>& densePoints);
template std::vector<Point3fRGBA> PointCloud::Impl::filterValidPoints(const std::vector<Point3fRGBA>& densePoints);

// ── Impl: CPU / GPU implementations ──

void PointCloud::Impl::calcPointsChunkDense(const uint8_t* depthData, std::vector<Point3f>& points, unsigned int startRow, unsigned int endRow) {
    const float scale = scaleFactor;

    for(unsigned int row = startRow; row < endRow; row++) {
        unsigned int rowStart = row * width;
        for(unsigned int col = 0; col < width; col++) {
            size_t i = rowStart + col;

            uint16_t depthValue;
            std::memcpy(&depthValue, depthData + i * sizeof(uint16_t), sizeof(depthValue));
            float z = static_cast<float>(depthValue) * scale;

            float xCoord = 0.0f;
            float yCoord = 0.0f;

            if(z > 0.0f) {
                const auto& ray = undistortedRays[i];
                xCoord = ray.x * z;
                yCoord = ray.y * z;
            }

            points[i] = Point3f{xCoord, yCoord, z};
        }
    }
}

void PointCloud::Impl::computePointCloudDenseCPU(const uint8_t* depthData, std::vector<Point3f>& points) {
    calcPointsChunkDense(depthData, points, 0, height);
}

void PointCloud::Impl::computePointCloudDenseCPUMT(const uint8_t* depthData, std::vector<Point3f>& points) {
    if(threadNum == 0) {
        if(logger) logger->warn("threadNum is 0, falling back to single-threaded computation");
        computePointCloudDenseCPU(depthData, points);
        return;
    }
    unsigned int rowsPerThread = height / threadNum;
    std::vector<std::future<void>> futures;

    auto processRows = [&](unsigned int startRow, unsigned int endRow) { calcPointsChunkDense(depthData, points, startRow, endRow); };

    for(uint32_t t = 0; t < threadNum; ++t) {
        unsigned int startRow = t * rowsPerThread;
        unsigned int endRow = (t == threadNum - 1) ? height : (startRow + rowsPerThread);
        futures.emplace_back(std::async(std::launch::async, processRows, startRow, endRow));
    }

    for(auto& f : futures) {
        f.get();
    }
}

void PointCloud::Impl::calcPointsChunkDenseColored(
    const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& points, unsigned int startRow, unsigned int endRow) {
    const float scale = scaleFactor;

    for(unsigned int row = startRow; row < endRow; row++) {
        unsigned int rowStart = row * width;
        for(unsigned int col = 0; col < width; col++) {
            size_t i = rowStart + col;

            uint16_t depthValue;
            std::memcpy(&depthValue, depthData + i * sizeof(uint16_t), sizeof(depthValue));
            float z = static_cast<float>(depthValue) * scale;

            float xCoord = 0.0f;
            float yCoord = 0.0f;

            if(z > 0.0f) {
                const auto& ray = undistortedRays[i];
                xCoord = ray.x * z;
                yCoord = ray.y * z;
            }

            uint8_t r = colorData[i * 3 + 0];
            uint8_t g = colorData[i * 3 + 1];
            uint8_t b = colorData[i * 3 + 2];

            points[i] = Point3fRGBA{xCoord, yCoord, z, r, g, b};
        }
    }
}

void PointCloud::Impl::computePointCloudDenseColoredCPU(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& points) {
    calcPointsChunkDenseColored(depthData, colorData, points, 0, height);
}

void PointCloud::Impl::computePointCloudDenseColoredCPUMT(const uint8_t* depthData, const uint8_t* colorData, std::vector<Point3fRGBA>& points) {
    if(threadNum == 0) {
        if(logger) logger->warn("threadNum is 0, falling back to single-threaded computation");
        computePointCloudDenseColoredCPU(depthData, colorData, points);
        return;
    }
    unsigned int rowsPerThread = height / threadNum;
    std::vector<std::future<void>> futures;

    auto processRows = [&](unsigned int startRow, unsigned int endRow) { calcPointsChunkDenseColored(depthData, colorData, points, startRow, endRow); };

    for(uint32_t t = 0; t < threadNum; ++t) {
        unsigned int startRow = t * rowsPerThread;
        unsigned int endRow = (t == threadNum - 1) ? height : (startRow + rowsPerThread);
        futures.emplace_back(std::async(std::launch::async, processRows, startRow, endRow));
    }

    for(auto& f : futures) {
        f.get();
    }
}

template <typename PointT>
void PointCloud::Impl::transformPointsCPU(std::vector<PointT>& points) {
    // Both points and extrinsics translations are in the same unit (target unit)
    // No conversion needed - just apply the transformation directly

    if(logger) {
        logger->debug("Applying transformation to {} points", points.size());
    }

    size_t transformedCount = 0;
    for(auto& p : points) {
        if(p.z > 0.0f) {
            // Standard 4x4 transformation: R*p + t
            float x = extrinsics[0][0] * p.x + extrinsics[0][1] * p.y + extrinsics[0][2] * p.z + extrinsics[0][3];
            float y = extrinsics[1][0] * p.x + extrinsics[1][1] * p.y + extrinsics[1][2] * p.z + extrinsics[1][3];
            float z = extrinsics[2][0] * p.x + extrinsics[2][1] * p.y + extrinsics[2][2] * p.z + extrinsics[2][3];

            p.x = x;
            p.y = y;
            p.z = z;
            transformedCount++;
        }
    }

    if(logger) {
        logger->debug("Transformed {} valid points (z > 0)", transformedCount);
    }
}

// Explicit template instantiations
template void PointCloud::Impl::transformPointsCPU(std::vector<Point3f>& points);
template void PointCloud::Impl::transformPointsCPU(std::vector<Point3fRGBA>& points);

void PointCloud::Impl::dropGpuBackend(const std::string& reason) {
    if(logger) logger->error("GPU point cloud computation failed ({}) -- computing on the CPU from now on", reason);
    gpuBackend.reset();
    computeMethod = ComputeMethod::CPU;
}

PointCloudGpuBackend::Geometry PointCloud::Impl::gpuGeometry() const {
    PointCloudGpuBackend::Geometry geometry;
    geometry.width = width;
    geometry.height = height;
    geometry.depthScale = scaleFactor;
    geometry.rays = undistortedRays.data();
    geometry.raysVersion = raysVersion;
    geometry.hasTransform = hasExtrinsics;
    if(hasExtrinsics) {
        for(size_t r = 0; r < 4; ++r) {
            for(size_t c = 0; c < 4; ++c) geometry.transform[r][c] = extrinsics[r][c];
        }
    }
    return geometry;
}

void PointCloud::Impl::initializeGPU(uint32_t device) {
#ifdef DEPTHAI_ENABLE_KOMPUTE
    // Reset any stale Kompute state before creating a new manager
    algo.reset();
    depthTensor.reset();
    intrinsicsTensor.reset();
    xyzTensor.reset();
    tensors.clear();
    algoInitialized = false;
    tensorsInitialized = false;

    mgr = std::make_shared<kp::Manager>(device);
    shader = std::vector<uint32_t>(shaders::DEPTH2POINTCLOUD_COMP_SPV.begin(), shaders::DEPTH2POINTCLOUD_COMP_SPV.end());
    computeMethod = ComputeMethod::GPU;
#else
    (void)device;
    throw std::runtime_error("GPU point cloud computation is not available in this build (no Kompute support and no platform GPU backend)");
#endif
}

void PointCloud::Impl::computePointCloudDenseGPU(const uint8_t* depthData, std::vector<Point3f>& points) {
#ifdef DEPTHAI_ENABLE_KOMPUTE
    std::vector<float> xyzOut;
    xyzOut.resize(size * 3);

    const float scale = scaleFactor;

    std::vector<float> depthDataFloat(size);
    for(size_t i = 0; i < size; i++) {
        uint16_t depthValue;
        std::memcpy(&depthValue, depthData + i * sizeof(uint16_t), sizeof(depthValue));
        depthDataFloat[i] = static_cast<float>(depthValue);
    }

    std::vector<float> intrinsics = {fx, fy, cx, cy, scale, static_cast<float>(width), static_cast<float>(height)};

    if(!tensorsInitialized) {
        depthTensor = mgr->tensor(depthDataFloat);
        intrinsicsTensor = mgr->tensor(intrinsics);
        xyzTensor = mgr->tensor(xyzOut);
        tensorsInitialized = true;
    } else {
        depthTensor->setData(depthDataFloat);
        intrinsicsTensor->setData(intrinsics);
    }

    if(!algoInitialized) {
        tensors.emplace_back(depthTensor);
        tensors.emplace_back(intrinsicsTensor);
        tensors.emplace_back(xyzTensor);
        algo = mgr->algorithm(tensors, shader);
        algoInitialized = true;
    }

    mgr->sequence()->record<kp::OpSyncDevice>(tensors)->record<kp::OpAlgoDispatch>(algo)->record<kp::OpSyncLocal>(tensors)->eval();

    xyzOut = xyzTensor->vector<float>();

    for(size_t i = 0; i < size; i++) {
        points[i].x = xyzOut[i * 3 + 0];
        points[i].y = xyzOut[i * 3 + 1];
        points[i].z = xyzOut[i * 3 + 2];
    }
#else
    (void)depthData;
    (void)points;
    throw std::runtime_error("Kompute not enabled in this build");
#endif
}

// ── Impl: intrinsics / extrinsics setters ──

void PointCloud::Impl::setLengthUnit(dai::LengthUnit lengthUnit) {
    // Check if unit actually changed
    bool unitChanged = (targetLengthUnit != lengthUnit);

    targetLengthUnit = lengthUnit;
    lengthUnitMultiplier = getLengthUnitMultiplier(lengthUnit);

    // Depth values from sensor are in millimeters (uint16_t raw values)
    // scaleFactor converts from mm to target unit
    scaleFactor = lengthUnitMultiplier / getLengthUnitMultiplier(LengthUnit::MILLIMETER);

    if(logger) {
        logger->info("Set length unit: multiplier={}, scaleFactor={} (mm->target), unit changed: {}", lengthUnitMultiplier, scaleFactor, unitChanged);
    }
}

void PointCloud::Impl::useCPU() {
    computeMethod = ComputeMethod::CPU;
}

void PointCloud::Impl::useCPUMT(uint32_t numThreads) {
    threadNum = std::max(1u, numThreads);
    computeMethod = ComputeMethod::CPU_MT;
}

void PointCloud::Impl::useGPU(uint32_t device, std::shared_ptr<PointCloudGpuBackend> backend) {
    gpuDistortionFallbackWarned = false;
    gpuBackend = std::move(backend);
    if(gpuBackend) {
        // Platform backend (e.g. OpenCL on an RVC4 device) takes precedence over the built-in path
        computeMethod = ComputeMethod::GPU;
        return;
    }
    initializeGPU(device);
}

void PointCloud::Impl::setIntrinsics(float fx, float fy, float cx, float cy, unsigned int width, unsigned int height) {
#ifdef DEPTHAI_ENABLE_KOMPUTE
    const bool resolutionChanged = intrinsicsSet && (this->width != width || this->height != height);
#endif
    if(fx == 0.0f || fy == 0.0f) {
        throw std::runtime_error("Focal lengths fx and fy must be non-zero");
    }
    this->fx = fx;
    this->fy = fy;
    this->cx = cx;
    this->cy = cy;
    this->width = width;
    this->height = height;
    size = this->width * this->height;
    intrinsicsSet = true;
    cacheUndistortedRays();
#ifdef DEPTHAI_ENABLE_KOMPUTE
    if(resolutionChanged) {
        tensorsInitialized = false;
        algoInitialized = false;
        tensors.clear();
    }
#endif
}

void PointCloud::Impl::setDistortion(CameraModel model, std::vector<float> coefficients) {
    const bool nextHasDistortion = hasNonZeroDistortion(coefficients);
    if(nextHasDistortion && model != CameraModel::Perspective && model != CameraModel::Fisheye) {
        throw std::invalid_argument(std::string("PointCloud does not support distortion model: ") + std::string(toString(model)));
    }

    const bool distortionStateChanged = hasDistortion != nextHasDistortion;
    distortionModel = model;
    distortionCoefficients = std::move(coefficients);
    hasDistortion = nextHasDistortion;
    if(distortionStateChanged) gpuDistortionFallbackWarned = false;
    cacheUndistortedRays();
}

void PointCloud::Impl::cacheUndistortedRays() {
    undistortedRays.clear();
    ++raysVersion;
    if(!intrinsicsSet) return;

    undistortedRays.resize(size);
    for(unsigned int row = 0; row < height; ++row) {
        for(unsigned int col = 0; col < width; ++col) {
            const size_t i = static_cast<size_t>(row) * width + col;
            const auto ray = undistortPoint({(col - cx) / fx, (row - cy) / fy, 1.0f}, distortionModel, distortionCoefficients);
            undistortedRays[i] = {ray[0] / ray[2], ray[1] / ray[2]};
        }
    }
}

void PointCloud::Impl::clearExtrinsics() {
    hasExtrinsics = false;
}

void PointCloud::Impl::setExtrinsics(const std::vector<std::vector<float>>& transformMatrix) {
    if(transformMatrix.size() != 4) {
        throw std::runtime_error("Transformation matrix must be 4x4");
    }
    for(size_t i = 0; i < 4; ++i) {
        if(transformMatrix[i].size() != 4) {
            throw std::runtime_error("Transformation matrix must be 4x4");
        }
    }
    extrinsics = transformMatrix;
    hasExtrinsics = true;

    if(logger) {
        logger->info("Extrinsics transformation matrix set:");
        for(size_t i = 0; i < 4; i++) {
            logger->info("  [{:8.4f}, {:8.4f}, {:8.4f}, {:8.4f}]", extrinsics[i][0], extrinsics[i][1], extrinsics[i][2], extrinsics[i][3]);
        }
    }
}

// PointCloud main class implementations

// Regular construction (also without a device, e.g. in a host-only pipeline): the Sync subnode is created here.
PointCloud::PointCloud()
    : DeviceNodeCRTP<DeviceNode, PointCloud, PointCloudProperties>(),
      initialConfig(std::make_shared<PointCloudConfig>(properties.initialConfig)),
      pimplPointCloud() {}

// Construction from serialized properties (configure mode): subnodes come from the pipeline schema.
PointCloud::PointCloud(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, PointCloud, PointCloudProperties>(std::move(props)),
      initialConfig(std::make_shared<PointCloudConfig>(properties.initialConfig)),
      pimplPointCloud() {}

PointCloud::~PointCloud() = default;

namespace {

constexpr char STREAM_KEY_SEPARATOR = '/';

std::string makeStreamKey(const char* base, const std::string& name) {
    if(name.empty()) return base;
    return std::string(base) + STREAM_KEY_SEPARATOR + name;
}

// Returns the stream name when `key` is a depth stream key ("depth" -> "", "depth/<name>" -> "<name>")
std::optional<std::string> streamNameFromDepthKey(const std::string& key) {
    const std::string base = PointCloud::depthInputName;
    if(key == base) return std::string();
    if(key.size() > base.size() + 1 && key.compare(0, base.size(), base) == 0 && key[base.size()] == STREAM_KEY_SEPARATOR) {
        return key.substr(base.size() + 1);
    }
    return std::nullopt;
}

}  // namespace

std::string PointCloud::getDepthInputKey(const std::string& name) {
    return makeStreamKey(depthInputName, name);
}

std::string PointCloud::getColorInputKey(const std::string& name) {
    return makeStreamKey(colorInputName, name);
}

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
Node::Input& PointCloud::getColorInput() {
    return getColorInput("");
}

Node::Input& PointCloud::getDepthInput(const std::string& name) {
    const auto key = getDepthInputKey(name);
    const bool created = !syncInputs.has(key);
    auto& input = syncInputs[key];
    if(created) {
        input.setBlocking(false);
        input.setMaxSize(4);
    }
    return input;
}

Node::Input& PointCloud::getColorInput(const std::string& name) {
    const auto key = getColorInputKey(name);
    const bool created = !syncInputs.has(key);
    auto& input = syncInputs[key];
    if(created) {
        input.setBlocking(false);
        input.setMaxSize(4);
    }
    return input;
}

std::vector<std::string> PointCloud::getDepthInputNames() const {
    // Sorted so that the default stream ("depth") comes before the named ones ("depth/<name>")
    std::vector<std::string> names;
    for(const auto& entry : syncInputs) {
        if(auto name = streamNameFromDepthKey(entry.first.second)) {
            names.push_back(*name);
        }
    }
    std::sort(names.begin(), names.end());
    return names;
}

void PointCloud::buildStage1() {
    DeviceNodeCRTP::buildStage1();

    // Depth streams from several devices can only be paired with clocks that are comparable
    // across devices. With the default timestamp source that means the Sync subnode has to run
    // on the host (Sync::buildStage1 rejects the device placement otherwise).
    std::set<const Device*> sourceDevices;
    for(const auto& entry : syncInputs.getSourceDevices()) {
        if(entry.second != nullptr) sourceDevices.insert(entry.second.get());
    }
    if(sourceDevices.size() > 1 && !sync->runOnHost() && sync->getTimestampSource() == Sync::TimestampSource::DEFAULT) {
        pimpl->logger->info("PointCloud: depth streams come from {} devices, running the Sync subnode on the host", sourceDevices.size());
        sync->setRunOnHost(true);
    }
}

void PointCloud::postBuildStage() {
    DeviceNodeCRTP::postBuildStage();

    // The Sync subnode waits for every one of its inputs. Entries that exist but were never linked
    // (the default depth input when only named streams are used, color inputs created but not linked)
    // are removed so that they do not stall the synchronization. Erasing invalidates the references
    // to those inputs (inputDepth for the default stream), which is why this only happens at build time.
    std::vector<std::pair<std::string, std::string>> unused;
    size_t connectedDepthStreams = 0;
    for(const auto& entry : syncInputs) {
        const auto& key = entry.first.second;
        const bool isDepth = streamNameFromDepthKey(key).has_value();
        if(entry.second.isConnected()) {
            if(isDepth) ++connectedDepthStreams;
            continue;
        }
        unused.push_back(entry.first);
    }
    if(connectedDepthStreams == 0) return;  // Nothing linked yet (e.g. inputs fed later); keep the default behaviour
    for(const auto& key : unused) {
        pimpl->logger->debug("PointCloud: ignoring unlinked input '{}'", key.second);
        syncInputs.erase(key);
    }
}

void PointCloud::buildInternal() {
    // Wire Sync subnode output to our private inSync input.
    // Color mode detection happens in run() because buildInternal() is called
    // at node creation time, before the user has linked inputs.
    sync->out.link(inSync);
    sync->setRunOnHost(false);

    // Configure depth sync input queue
    inputDepth.setBlocking(false);
    inputDepth.setMaxSize(4);
    inSync.setBlocking(false);
    inSync.setMaxSize(4);
}
#endif

PointCloud::Properties& PointCloud::getProperties() {
    if(device && !runOnHostVar) {
        auto platform = device->getPlatform();
        if(platform == Platform::RVC2) {
            throw std::runtime_error("PointCloud node is not supported on RVC2 devices. Use setRunOnHost(true) instead.");
        }
    }
    properties.initialConfig = *initialConfig;
    return properties;
}

void PointCloud::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

void PointCloud::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool PointCloud::runOnHost() const {
    return runOnHostVar;
}

// The compute method is stored in the properties so that it reaches a node running on the device; it is applied to
// the Impls when the node starts (applyComputeSettings), where a missing GPU can be handled with a CPU fallback.
void PointCloud::useCPU() {
    properties.computeMethod = Properties::ComputeMethod::CPU;
}

void PointCloud::useCPUMT(uint32_t numThreads) {
    properties.computeMethod = Properties::ComputeMethod::CPU_MT;
    properties.numThreads = std::max(1u, numThreads);
}

void PointCloud::useGPU(uint32_t device) {
    properties.computeMethod = Properties::ComputeMethod::GPU;
    properties.gpuDevice = device;
}

std::shared_ptr<PointCloudGpuBackend> PointCloud::createGpuBackend(uint32_t gpuDevice, std::shared_ptr<::spdlog::logger> logger) {
    (void)gpuDevice;
    (void)logger;
    return nullptr;
}

void PointCloud::applyComputeSettings(Impl& impl) {
    impl.setLogger(pimpl->logger);  // so that a failing GPU backend can report why
    switch(properties.computeMethod) {
        case Properties::ComputeMethod::CPU:
            impl.useCPU();
            break;
        case Properties::ComputeMethod::CPU_MT:
            impl.useCPUMT(properties.numThreads);
            break;
        case Properties::ComputeMethod::GPU:
            try {
                impl.useGPU(properties.gpuDevice, createGpuBackend(properties.gpuDevice, pimpl->logger));
                pimpl->logger->info("PointCloud: computing on GPU device {}", properties.gpuDevice);
            } catch(const std::exception& ex) {
                pimpl->logger->warn("PointCloud: GPU compute requested but not available ({}) -- falling back to CPU", ex.what());
                impl.useCPU();
            }
            break;
    }
}

void PointCloud::setTargetCoordinateSystem(CameraBoardSocket targetCamera) {
    initialConfig->setTargetCoordinateSystem(targetCamera);
}

void PointCloud::setTargetCoordinateSystem(HousingCoordinateSystem housingCS) {
    initialConfig->setTargetCoordinateSystem(housingCS);
}

void PointCloud::setTargetCoordinateSystem(CameraBoardSocket targetCamera, bool useSpecTranslation) {
    initialConfig->setTargetCoordinateSystem(targetCamera, useSpecTranslation);
}

void PointCloud::setTargetCoordinateSystem(HousingCoordinateSystem housingCS, bool useSpecTranslation) {
    initialConfig->setTargetCoordinateSystem(housingCS, useSpecTranslation);
}

void PointCloud::setTargetCoordinateSystem(const std::string& targetDeviceId, CameraBoardSocket targetCamera) {
    initialConfig->setTargetCoordinateSystem(targetDeviceId, targetCamera);
}

void PointCloud::setTargetCoordinateSystem(const std::string& targetDeviceId, HousingCoordinateSystem housingCS) {
    initialConfig->setTargetCoordinateSystem(targetDeviceId, housingCS);
}

void PointCloud::setDeviceCalibration(const std::string& deviceId, const CalibrationHandler& calibration) {
    deviceCalibrations[deviceId] = calibration;
    for(auto& entry : depthStreams) entry.second.initialized = false;
}

std::string PointCloud::OutputCoordinateSystem::describe() const {
    std::string description = "device '" + deviceId + "' ";
    if(housing != HousingCoordinateSystem::AUTO) return description + "housing " + toString(housing);
    return description + "socket " + std::string(toString(socket));
}

// ── Depth stream bookkeeping ──

PointCloud::DepthStream& PointCloud::getDepthStream(const std::string& name) {
    auto it = depthStreams.find(name);
    if(it != depthStreams.end()) return it->second;

    DepthStream stream;
    if(!name.empty()) {
        // Additional streams get their own deprojection state, computed the same way as the default one
        stream.impl = std::make_unique<Impl>();
        applyComputeSettings(*stream.impl);
    }
    return depthStreams.emplace(name, std::move(stream)).first->second;
}

PointCloud::Impl& PointCloud::getImpl(DepthStream& stream) {
    return stream.impl ? *stream.impl : *pimplPointCloud;
}

std::vector<PointCloud::StreamFrames> PointCloud::collectStreamFrames(MessageGroup& group) {
    // MessageGroup keys are sorted: the default stream "depth" precedes "depth/<name>" entries
    std::vector<StreamFrames> frames;
    for(const auto& entry : group.group) {
        const auto name = streamNameFromDepthKey(entry.first);
        if(!name) continue;
        auto depthFrame = std::dynamic_pointer_cast<ImgFrame>(entry.second);
        if(!depthFrame) {
            pimpl->logger->warn("PointCloud: failed to extract depth frame '{}' from MessageGroup -- skipping", entry.first);
            return {};
        }
        frames.push_back({*name, std::move(depthFrame), nullptr});
    }
    // Color frames are optional: look them up without creating entries in the group
    for(auto& stream : frames) {
        const auto colorEntry = group.group.find(getColorInputKey(stream.name));
        if(colorEntry != group.group.end()) {
            stream.color = std::dynamic_pointer_cast<ImgFrame>(colorEntry->second);
        }
    }
    return frames;
}

bool PointCloud::hasTransformationChanged(DepthStream& stream, const ImgFrame& frame) {
    if(stream.lastTransformation && stream.lastTransformation->isEqualTransformation(frame.transformation)) return false;
    pimpl->logger->debug("Frame transformation changed, reinitializing...");
    stream.lastTransformation = frame.transformation;
    return true;
}

bool PointCloud::isValidDepthFrame(const ImgFrame& depthFrame) {
    const auto width = depthFrame.getWidth();
    const auto height = depthFrame.getHeight();
    // Validate that the buffer contains packed uint16_t depth data
    const auto expectedBytes = static_cast<std::size_t>(width) * height * sizeof(uint16_t);
    if(depthFrame.getType() != ImgFrame::Type::RAW16 || depthFrame.getData().size() != expectedBytes) {
        pimpl->logger->warn("PointCloud: unexpected depth frame (type={}, size={}, expected {} bytes for {}x{} RAW16) -- skipping frame",
                            static_cast<int>(depthFrame.getType()),
                            depthFrame.getData().size(),
                            expectedBytes,
                            width,
                            height);
        return false;
    }
    return true;
}

bool PointCloud::haveCommonTargetCoordinateSystem(const std::vector<StreamFrames>& frames) {
    // Every stream is transformed into its own output coordinate system: the one its frame extrinsics
    // point to, or the configured target resolved for that stream. The merged cloud is only meaningful
    // when all of those coincide (same device and socket/housing, e.g. the common origin of a
    // multi-device calibration or an explicit target device). Unknown parts (AUTO sockets) are
    // treated as compatible, like Extrinsics::hasCompatibleCoordinateSystem.
    const auto compatible = [](const OutputCoordinateSystem& a, const OutputCoordinateSystem& b) {
        const bool sameSocket = a.socket == CameraBoardSocket::AUTO || b.socket == CameraBoardSocket::AUTO || a.socket == b.socket;
        return a.deviceId == b.deviceId && sameSocket && a.housing == b.housing;
    };
    const auto& reference = getDepthStream(frames.front().name).outputCoordinateSystem;
    for(size_t i = 1; i < frames.size(); ++i) {
        const auto& other = getDepthStream(frames[i].name).outputCoordinateSystem;
        if(!compatible(reference, other)) {
            if(!coordinateSystemMismatchWarned) {
                pimpl->logger->warn(
                    "PointCloud: depth stream '{}' is expressed relative to {} while depth stream '{}' is expressed relative to {}. The streams cannot "
                    "be merged until they share a target coordinate system (set a multi-device calibration on the pipeline for depth from several "
                    "devices, or select the target device explicitly with setTargetCoordinateSystem(deviceId, ...)) -- dropping synced groups",
                    frames.front().name,
                    reference.describe(),
                    frames[i].name,
                    other.describe());
                coordinateSystemMismatchWarned = true;
            }
            return false;
        }
    }
    if(coordinateSystemMismatchWarned) {
        pimpl->logger->info("PointCloud: depth streams share the target coordinate system {} again", reference.describe());
        coordinateSystemMismatchWarned = false;
    }
    return true;
}

static void logMatrix4x4(const std::shared_ptr<spdlog::logger>& logger,
                         spdlog::level::level_enum level,
                         const std::string& name,
                         const std::vector<std::vector<float>>& m) {
    logger->log(level, "{}:", name);
    for(int i = 0; i < 4; i++) {
        logger->log(level, "  [{:8.4f}, {:8.4f}, {:8.4f}, {:8.4f}]", m[i][0], m[i][1], m[i][2], m[i][3]);
    }
}

// ── PointCloud helper methods ──

CalibrationHandler PointCloud::getCalibrationFor(const std::string& deviceId) {
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    // Explicit overrides (offline streams, tests) win over live device calibrations
    const auto overrideIt = deviceCalibrations.find(deviceId);
    if(overrideIt != deviceCalibrations.end()) {
        return overrideIt->second;
    }
    // The frame extrinsics name the device owning the reference coordinate system. With a
    // multi-device calibration that is the origin device, which is not necessarily the device
    // this node was created with. An explicit target device may be any device of the pipeline.
    if(!deviceId.empty()) {
        if(device && device->getDeviceId() == deviceId) {
            return device->getCalibration();
        }
        for(const auto& pipelineDevice : getParentPipeline().getDevices()) {
            if(pipelineDevice && pipelineDevice->getDeviceId() == deviceId) {
                return pipelineDevice->getCalibration();
            }
        }
        if(device) {
            throw std::runtime_error("PointCloud: device '" + deviceId
                                     + "' is not part of the pipeline. Its calibration is required to transform the cloud to a camera or housing "
                                       "coordinate system of that device (set it explicitly with setDeviceCalibration for offline streams).");
        }
    }
#else
    (void)deviceId;  // On device every frame comes from this device
#endif
    if(device) {
        return device->getCalibration();
    }
    return getParentPipeline().getCalibrationData();
}

void PointCloud::setIntrinsicsFromFrame(Impl& impl, const ImgFrame& frame) {
    const auto width = frame.getWidth();
    const auto height = frame.getHeight();
    const auto intrinsics = frame.transformation.getIntrinsicMatrix();
    const float fx = intrinsics[0][0];
    const float fy = intrinsics[1][1];
    const float cx = intrinsics[0][2];
    const float cy = intrinsics[1][2];
    pimpl->logger->debug("Setting intrinsics: fx={}, fy={}, cx={}, cy={}, size={}x{}", fx, fy, cx, cy, width, height);
    impl.setIntrinsics(fx, fy, cx, cy, width, height);
}

std::string PointCloud::getReferenceDeviceId(const Extrinsics& frameExtrinsics) {
    if(!frameExtrinsics.toDeviceId.empty()) return frameExtrinsics.toDeviceId;
    // Frames without a device ID (older firmware, synthetic frames) are expressed relative to the node's own device
    if(device) return device->getDeviceId();
    return {};
}

std::vector<std::vector<float>> PointCloud::getReferenceToDeviceOrigin(const std::string& referenceDeviceId,
                                                                       CameraBoardSocket referenceSocket,
                                                                       const std::string& targetDeviceId,
                                                                       CameraBoardSocket& targetOriginSocket,
                                                                       LengthUnit unit) {
#ifdef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    (void)referenceSocket;
    (void)targetOriginSocket;
    (void)unit;
    throw std::runtime_error("PointCloud: the target coordinate system lives on device '" + targetDeviceId
                             + "' while the depth frame is expressed relative to device '" + referenceDeviceId
                             + "'. Targets on another device are only supported when the node runs on the host (setRunOnHost(true)).");
#else
    // ref camera -> local origin of the reference device -> common origin of the multi-device calibration -> local origin of the target device
    const auto graph = getParentPipeline().getMultiDeviceCalibration();
    if(!graph.has_value()) {
        throw std::runtime_error("PointCloud: the target coordinate system lives on device '" + targetDeviceId
                                 + "' while the depth frame is expressed relative to device '" + referenceDeviceId
                                 + "'. Transforming between devices requires a multi-device calibration on the pipeline "
                                   "(Pipeline::setMultiDeviceCalibration) that connects both devices.");
    }
    const beta::MultiDeviceCalibrationHandler handler(*graph);
    const auto referenceOrigin = handler.getDeviceSocket(referenceDeviceId);
    const auto targetOrigin = handler.getDeviceSocket(targetDeviceId);
    if(!referenceOrigin.has_value() || !targetOrigin.has_value()) {
        throw std::runtime_error("PointCloud: device '" + (referenceOrigin.has_value() ? targetDeviceId : referenceDeviceId)
                                 + "' is not part of the multi-device calibration of the pipeline, so the point cloud cannot be transformed from device '"
                                 + referenceDeviceId + "' to device '" + targetDeviceId + "'.");
    }
    const auto referenceOriginToCommon = handler.getExtrinsicsToOrigin(referenceDeviceId, *referenceOrigin);
    const auto targetOriginToCommon = handler.getExtrinsicsToOrigin(targetDeviceId, *targetOrigin);
    if(!referenceOriginToCommon.has_value() || !targetOriginToCommon.has_value() || referenceOriginToCommon->toDeviceId != targetOriginToCommon->toDeviceId
       || referenceOriginToCommon->toCameraSocket != targetOriginToCommon->toCameraSocket) {
        throw std::runtime_error(
            "PointCloud: devices '" + referenceDeviceId + "' and '" + targetDeviceId
            + "' are not connected by the multi-device calibration of the pipeline, so the point cloud cannot be transformed between them.");
    }

    // The reference camera is usually the local origin already (the device rebased the frame onto it); otherwise go through the local calibration
    std::vector<std::vector<float>> T_ref_to_referenceOrigin = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    if(referenceSocket != *referenceOrigin) {
        T_ref_to_referenceOrigin = getCalibrationFor(referenceDeviceId).getCameraExtrinsics(referenceSocket, *referenceOrigin, false, unit);
    }
    const auto T_referenceOrigin_to_common = matrix::toVecMatrix4x4(referenceOriginToCommon->getTransformationMatrix(false, unit));
    auto T_common_to_targetOrigin = matrix::toVecMatrix4x4(targetOriginToCommon->getTransformationMatrix(false, unit));
    matrix::invertSe3Matrix4x4InPlace(T_common_to_targetOrigin);

    targetOriginSocket = *targetOrigin;
    return matrix::matMul(T_common_to_targetOrigin, matrix::matMul(T_referenceOrigin_to_common, T_ref_to_referenceOrigin));
#endif
}

void PointCloud::setCoordinateTransformation(DepthStream& stream, const ImgFrame& depthFrame, const PointCloudConfig& config) {
    auto& impl = getImpl(stream);
    auto unit = impl.targetLengthUnit;
    auto coordSystemType = config.getCoordinateSystemType();
    auto targetCameraSocket = config.getTargetCameraSocket();
    auto targetHousingCS = config.getTargetHousingCS();
    auto useSpecTranslation = config.getUseSpecTranslation();

    // Read T_frame→ref from the depth frame's extrinsics
    // After rectification, the rotation already accounts for the rectification change.
    auto frameExtrinsics = depthFrame.transformation.getExtrinsics();
    auto refCamera = frameExtrinsics.toCameraSocket;
    if(refCamera == CameraBoardSocket::AUTO) {
        throw std::runtime_error(
            "PointCloud: depth frame extrinsics toCameraSocket is AUTO. "
            "Ensure the depth frame has valid extrinsics with a specific camera socket set.");
    }
    auto T_frame_to_ref = matrix::toVecMatrix4x4(frameExtrinsics.getTransformationMatrix(useSpecTranslation, unit));
    logMatrix4x4(pimpl->logger, spdlog::level::debug, "T_frame_to_ref", T_frame_to_ref);

    // The target socket / housing is looked up on the configured device, by default the one owning the reference camera.
    // When the two differ, the reference is first carried over to the local calibration origin of the target device.
    const std::string referenceDeviceId = getReferenceDeviceId(frameExtrinsics);
    const bool explicitTargetDevice = !config.getTargetDeviceId().empty();
    const std::string targetDeviceId = explicitTargetDevice ? config.getTargetDeviceId() : referenceDeviceId;
    const bool crossDevice = coordSystemType != PointCloudConfig::CoordinateSystemType::DEFAULT && targetDeviceId != referenceDeviceId;
    CameraBoardSocket targetRefCamera = refCamera;
    std::vector<std::vector<float>> T_ref_to_targetRef = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    if(crossDevice) {
        T_ref_to_targetRef = getReferenceToDeviceOrigin(referenceDeviceId, refCamera, targetDeviceId, targetRefCamera, unit);
        logMatrix4x4(pimpl->logger, spdlog::level::debug, "T_ref_to_target_device_origin", T_ref_to_targetRef);
    }

    // Compute the target transform based on coordSystemType
    // The final transform is: T_ref→target * T_frame→ref
    std::optional<std::vector<std::vector<float>>> T_final = std::nullopt;
    std::optional<Extrinsics> targetExtrinsics = std::nullopt;
    // Without a target the points stay in the coordinate system the frame extrinsics point to
    OutputCoordinateSystem outputCoordinateSystem{frameExtrinsics.toDeviceId, refCamera, HousingCoordinateSystem::AUTO};

    switch(coordSystemType) {
        case PointCloudConfig::CoordinateSystemType::CAMERA_SOCKET: {
            pimpl->logger->info("Using CAMERA_SOCKET transformation to {} of device '{}', via ref camera {} of device '{}'",
                                toString(targetCameraSocket),
                                targetDeviceId,
                                toString(refCamera),
                                referenceDeviceId);
            auto calibHandler = getCalibrationFor(targetDeviceId);
            auto T_targetRef_to_target = calibHandler.getCameraExtrinsics(targetRefCamera, targetCameraSocket, useSpecTranslation, unit);
            auto T_ref_to_target = matrix::matMul(T_targetRef_to_target, T_ref_to_targetRef);
            T_final = matrix::matMul(T_ref_to_target, T_frame_to_ref);
            targetExtrinsics = Extrinsics(T_ref_to_target, targetCameraSocket, unit);
            outputCoordinateSystem = {targetDeviceId, targetCameraSocket, HousingCoordinateSystem::AUTO};
            break;
        }

        case PointCloudConfig::CoordinateSystemType::HOUSING: {
            pimpl->logger->info("Using HOUSING transformation to housing {} of device '{}', via ref camera {} of device '{}'",
                                toString(targetHousingCS),
                                targetDeviceId,
                                toString(refCamera),
                                referenceDeviceId);
            auto calibHandler = getCalibrationFor(targetDeviceId);
            auto T_targetRef_to_housing = calibHandler.getHousingCalibration(targetRefCamera, targetHousingCS, true, unit);
            auto T_ref_to_housing = matrix::matMul(T_targetRef_to_housing, T_ref_to_targetRef);
            T_final = matrix::matMul(T_ref_to_housing, T_frame_to_ref);
            targetExtrinsics = Extrinsics(T_ref_to_housing, CameraBoardSocket::AUTO, unit);
            outputCoordinateSystem = {targetDeviceId, CameraBoardSocket::AUTO, targetHousingCS};
            break;
        }

        case PointCloudConfig::CoordinateSystemType::DEFAULT: {
            auto configMat = matrix::toVecMatrix4x4(config.getTransformationMatrix());

            if(!matrix::isIdentity4x4(configMat)) {
                pimpl->logger->info("Applying custom transform composed with frame extrinsics");
                T_final = matrix::matMul(configMat, T_frame_to_ref);
                targetExtrinsics = Extrinsics(configMat, refCamera, unit);
            } else if(!matrix::isIdentity4x4(T_frame_to_ref)) {
                pimpl->logger->info("Applying frame extrinsics (T_frame_to_ref)");
                T_final = T_frame_to_ref;
            } else {
                pimpl->logger->debug("No coordinate system transformation applied (identity)");
            }
            break;
        }
    }

    // The target coordinate system lives on the explicitly selected device, otherwise on the device that owns the reference camera
    if(targetExtrinsics) {
        targetExtrinsics->toDeviceId = explicitTargetDevice ? targetDeviceId : frameExtrinsics.toDeviceId;
    }
    stream.targetExtrinsics = std::move(targetExtrinsics);
    stream.outputCoordinateSystem = std::move(outputCoordinateSystem);

    // Apply the final transform
    if(T_final) {
        impl.setExtrinsics(*T_final);
    } else {
        impl.clearExtrinsics();
    }
}

// ── Main initialize method ──

void PointCloud::initialize(DepthStream& stream, const ImgFrame& depthFrame, const PointCloudConfig& config) {
    pimpl->logger->debug("PointCloud::initialize() called");
    auto& impl = getImpl(stream);
    impl.setLogger(pimpl->logger);

    // Set length unit before intrinsics/extrinsics (they depend on it)
    impl.setLengthUnit(config.getLengthUnit());

    // Set camera intrinsics from the depth frame
    setIntrinsicsFromFrame(impl, depthFrame);
    impl.setDistortion(depthFrame.transformation.getDistortionModel(), depthFrame.transformation.getDistortionCoefficients());

    // Compute and apply coordinate transformation (frame extrinsics + target transform)
    setCoordinateTransformation(stream, depthFrame, config);

    stream.initialized = true;
    pimpl->logger->info("PointCloud::initialize() completed");
}

//------------------------------------------------------------------
// Processing helpers (depth-only and colorized paths)
//------------------------------------------------------------------

void PointCloud::computeDepthOnly(Impl& impl, const ImgFrame& depthFrame, bool organized, std::vector<Point3f>& points) {
    // GPU backend: the dense points stay in GPU-mapped memory and are gathered (compacted for sparse output) into the
    // scratch buffer in one pass; the backend has applied the extrinsics already.
    if(const auto* dense = impl.computeDenseOnGpu(depthFrame.getData().data(), depthFrame.data)) {
        Impl::gatherPoints(dense, static_cast<size_t>(depthFrame.getWidth()) * depthFrame.getHeight(), organized, points);
        return;
    }
    // CPU: the dense cloud is written into the reused scratch buffer (resize keeps its capacity), invalid points are
    // compacted away in place for sparse output and the transformation is applied in place as well
    impl.computePointCloudDense(depthFrame.getData().data(), points);
    if(!organized) Impl::compactValidPoints(points);
    impl.applyTransformation(points);
}

bool PointCloud::canColorize(const ImgFrame& depthFrame, const ImgFrame& colorFrame) {
    const auto width = depthFrame.getWidth();
    const auto height = depthFrame.getHeight();

    // Validate color frame - fall back to depth-only on failure
    if(colorFrame.getWidth() != width || colorFrame.getHeight() != height) {
        pimpl->logger->warn("PointCloud: color frame size ({}x{}) does not match depth ({}x{}) -- skipping colorization",
                            colorFrame.getWidth(),
                            colorFrame.getHeight(),
                            width,
                            height);
        return false;
    }
    if(colorFrame.getType() != ImgFrame::Type::RGB888i) {
        pimpl->logger->warn("PointCloud: color frame type ({}) is not RGB888i -- skipping colorization", static_cast<int>(colorFrame.getType()));
        return false;
    }
    return true;
}

void PointCloud::computeColorized(Impl& impl, const ImgFrame& depthFrame, const ImgFrame& colorFrame, bool organized, std::vector<Point3fRGBA>& points) {
    // Check extrinsics mismatches (non-fatal)
    if(!depthFrame.transformation.getExtrinsics().isEqualExtrinsics(colorFrame.transformation.getExtrinsics())) {
        pimpl->logger->warn("PointCloud: depth and color extrinsics differ -- colorization may be misaligned");
    }

    // Check intrinsics and distortion mismatches (non-fatal)
    if(!depthFrame.transformation.isAlignedTo(colorFrame.transformation)) {
        pimpl->logger->warn("PointCloud: depth and color transformations are not aligned (intrinsics/distortion differ) -- colorization may be misaligned");
    }

    if(const auto* dense = impl.computeDenseColoredOnGpu(depthFrame.getData().data(), depthFrame.data, colorFrame.getData().data(), colorFrame.data)) {
        Impl::gatherPoints(dense, static_cast<size_t>(depthFrame.getWidth()) * depthFrame.getHeight(), organized, points);
        return;
    }
    impl.computePointCloudDenseColored(depthFrame.getData().data(), colorFrame.getData().data(), points);
    if(!organized) Impl::compactValidPoints(points);
    impl.applyTransformation(points);
}

template <typename PointT>
size_t PointCloud::computeStreams(const std::vector<StreamFrames>& frames, bool organized, PointCloudData& output) {
    constexpr bool colored = std::is_same_v<PointT, Point3fRGBA>;
    const size_t count = frames.size();
    std::vector<DepthStream*> streams;
    streams.reserve(count);
    for(const auto& stream : frames) streams.push_back(&getDepthStream(stream.name));
    auto bufferOf = [](DepthStream& stream) -> std::vector<PointT>& {
        if constexpr(colored) {
            return stream.coloredPoints;
        } else {
            return stream.points;
        }
    };

    // Each stream computes into its own scratch buffer so that the streams can be processed independently
    auto computeOne = [&](size_t i) {
        auto& stream = *streams[i];
        if constexpr(colored) {
            computeColorized(getImpl(stream), *frames[i].depth, *frames[i].color, organized, stream.coloredPoints);
        } else {
            computeDepthOnly(getImpl(stream), *frames[i].depth, organized, stream.points);
        }
    };

    // One thread per additional stream on the host. Every stream owns its Impl (intrinsics, undistortion cache,
    // extrinsics, worker threads of CPU_MT), so the streams do not share mutable state. GPU Impls stay sequential:
    // their Vulkan resources are not meant to be driven from several threads at once.
    bool parallel = count > 1;
#ifdef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    parallel = false;
#else
    for(auto* stream : streams) parallel = parallel && !getImpl(*stream).usesGPU();
#endif
    if(parallel) {
        std::vector<std::future<void>> workers;
        workers.reserve(count - 1);
        for(size_t i = 1; i < count; ++i) {
            workers.push_back(std::async(std::launch::async, computeOne, i));
        }
        // The first stream is computed on this thread; errors of the workers are collected after all of them finished
        std::exception_ptr error;
        try {
            computeOne(0);
        } catch(...) {
            error = std::current_exception();
        }
        for(auto& worker : workers) {
            try {
                worker.get();
            } catch(...) {
                if(!error) error = std::current_exception();
            }
        }
        if(error) std::rethrow_exception(error);
    } else {
        for(size_t i = 0; i < count; ++i) computeOne(i);
    }

    // Gather the streams, in stream order, into the output buffer with a single allocation
    size_t total = 0;
    for(auto* stream : streams) total += bufferOf(*stream).size();
    std::vector<uint8_t> data(total * sizeof(PointT));
    size_t offset = 0;
    for(auto* stream : streams) {
        const auto& points = bufferOf(*stream);
        const size_t bytes = points.size() * sizeof(PointT);
        if(bytes > 0) std::memcpy(data.data() + offset, points.data(), bytes);
        offset += bytes;
    }
    output.setData(std::move(data));
    output.setColor(colored);
    return total;
}

//------------------------------------------------------------------
// Main run loop
//------------------------------------------------------------------

void PointCloud::run() {
#ifdef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    // On device, apply the deserialized config from properties. On the host initialConfig is the
    // source of truth (properties are only refreshed from it when the pipeline is serialized).
    *initialConfig = properties.initialConfig;
#endif

    applyComputeSettings(*pimplPointCloud);
    for(auto& entry : depthStreams) {
        if(entry.second.impl) applyComputeSettings(*entry.second.impl);
    }
    pimpl->logger->info("PointCloud node started");

    // EEPROM id of the device this node runs on (falls back to the pipeline default device when the node has none),
    // so calibration changes are tracked on the node's own device rather than the master's.
    auto readEepromId = [&]() -> uint32_t { return device ? device->getProperties().eepromId : getParentPipeline().getEepromId(); };
    uint32_t currentEepromId = readEepromId();
    auto latestConfig = initialConfig;

    while(mainLoop()) {
        if(outputPointCloud.getQueueConnections().empty() && outputPointCloud.getConnections().empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Get synced frames from MessageGroup
        std::shared_ptr<MessageGroup> group;
        {
            auto blockEvent = this->inputBlockEvent();
            group = inSync.get<MessageGroup>();
        }
        if(!group) continue;

        // One entry per depth stream (default stream first), with its optional color frame
        auto frames = collectStreamFrames(*group);
        if(frames.empty()) {
            pimpl->logger->warn("PointCloud: no depth frame in MessageGroup -- skipping");
            continue;
        }
        const bool multiStream = frames.size() > 1;

        // Check for runtime config update
        auto newConfig = inputConfig.tryGet<PointCloudConfig>();
        if(newConfig) {
            latestConfig = newConfig;
            for(auto& entry : depthStreams) entry.second.initialized = false;
        }

        // Read organized mode from config
        const bool organized = latestConfig->getOrganized();

        const uint32_t latestEepromId = readEepromId();
        if(latestEepromId > currentEepromId) {
            pimpl->logger->debug("Calibration data changed (ID: {} -> {}), reinitializing...", currentEepromId, latestEepromId);
            for(auto& entry : depthStreams) entry.second.initialized = false;
            currentEepromId = latestEepromId;
        }

        bool skipGroup = false;
        for(const auto& stream : frames) {
            auto& state = getDepthStream(stream.name);
            if(hasTransformationChanged(state, *stream.depth)) state.initialized = false;
            if(!state.initialized) {
                try {
                    initialize(state, *stream.depth, *latestConfig);
                } catch(const std::exception& ex) {
                    // Typically a coordinate system that cannot be resolved yet (missing multi-device calibration or device calibration).
                    // Keep the node alive and retry with the next group, the situation may be fixed at runtime.
                    if(!initializationFailedWarned) {
                        pimpl->logger->error(
                            "PointCloud: cannot initialize depth stream '{}': {} -- dropping synced groups until it succeeds", stream.name, ex.what());
                        initializationFailedWarned = true;
                    }
                    skipGroup = true;
                    break;
                }
                if(initializationFailedWarned) {
                    pimpl->logger->info("PointCloud: depth stream '{}' initialized", stream.name);
                    initializationFailedWarned = false;
                }
            }
            if(!isValidDepthFrame(*stream.depth)) {
                skipGroup = true;
                break;
            }
        }
        if(skipGroup) continue;

        if(multiStream && !haveCommonTargetCoordinateSystem(frames)) continue;

        // The output is colorized only when every stream carries a usable color frame
        bool colorize = true;
        bool anyColor = false;
        for(const auto& stream : frames) {
            const bool streamColor = stream.color != nullptr && canColorize(*stream.depth, *stream.color);
            anyColor = anyColor || streamColor;
            colorize = colorize && streamColor;
        }
        if(anyColor && !colorize && !mixedColorWarned) {
            pimpl->logger->warn("PointCloud: not every depth stream has a usable color frame -- the merged point cloud is not colorized");
            mixedColorWarned = true;
        }

        // Create PointCloudData
        auto pc = std::make_shared<PointCloudData>();
        const auto& primary = frames.front();
        pc->setInstanceNum(primary.depth->getInstanceNum());
        const auto& primaryState = getDepthStream(primary.name);

        // Compute and merge the points of all streams (concurrently on the host)
        const auto computeStart = std::chrono::steady_clock::now();
        const size_t totalPoints = colorize ? computeStreams<Point3fRGBA>(frames, organized, *pc) : computeStreams<Point3f>(frames, organized, *pc);
        computeTimeSum += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - computeStart).count();
        if(++computeTimeCount >= COMPUTE_TIME_LOG_INTERVAL) {
            pimpl->logger->debug("PointCloud: compute {:.2f} ms per group on average over {} groups ({} streams, {} points, {})",
                                 computeTimeSum / computeTimeCount,
                                 computeTimeCount,
                                 frames.size(),
                                 totalPoints,
                                 pimplPointCloud->usesGPU() ? "GPU" : "CPU");
            computeTimeSum = 0.0;
            computeTimeCount = 0;
        }

        // Output layout
        if(!organized) {
            pc->setWidth(static_cast<unsigned int>(totalPoints));
            pc->setHeight(1);
        } else if(!multiStream) {
            pc->setWidth(primary.depth->getWidth());
            pc->setHeight(primary.depth->getHeight());
        } else {
            // Organized clouds of equal width are stacked row-wise; otherwise the layout degrades to a single row
            bool sameWidth = true;
            unsigned int totalHeight = 0;
            for(const auto& stream : frames) {
                sameWidth = sameWidth && stream.depth->getWidth() == primary.depth->getWidth();
                totalHeight += stream.depth->getHeight();
            }
            if(sameWidth) {
                pc->setWidth(primary.depth->getWidth());
                pc->setHeight(totalHeight);
            } else {
                if(!organizedLayoutWarned) {
                    pimpl->logger->warn(
                        "PointCloud: organized output requested but the depth streams differ in width -- the merged cloud is stored as a single row");
                    organizedLayoutWarned = true;
                }
                pc->setWidth(static_cast<unsigned int>(totalPoints));
                pc->setHeight(1);
            }
        }

        // Metadata and transformation of the output cloud
        if(!multiStream) {
            pc->setBufferMetadataFrom(primary.depth);
            // Preserve the source ImgTransformation as metadata. The distortion model and
            // coefficients describe the image used to generate the cloud; the 3D points have already
            // been distortion-compensated.
            auto outputTransformation = primary.depth->getTransformation();
            if(primaryState.targetExtrinsics) {
                outputTransformation.setExtrinsics(*primaryState.targetExtrinsics);
            }
            pc->setTransformation(outputTransformation);
        } else {
            // Timestamps follow the newest frame of the group (Sync stamps the group that way)
            pc->setBufferMetadataFrom(group);
            // A merged cloud has no single source image: the transformation only carries the output size and the
            // coordinate system the points are expressed in (identity to the common reference unless a target was configured)
            ImgTransformation outputTransformation(pc->getWidth(), pc->getHeight());
            if(primaryState.targetExtrinsics) {
                outputTransformation.setExtrinsics(*primaryState.targetExtrinsics);
            } else {
                const auto reference = primary.depth->transformation.getExtrinsics();
                const std::vector<std::vector<float>> identityMatrix = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
                Extrinsics identity(identityMatrix, reference.toCameraSocket, latestConfig->getLengthUnit());
                identity.toDeviceId = reference.toDeviceId;
                outputTransformation.setExtrinsics(identity);
            }
            pc->setTransformation(outputTransformation);
        }

        pc->updateBoundingBox();

        {
            auto blockEvent = this->outputBlockEvent();
            outputPointCloud.send(pc);

            if(!passthroughDepth.getQueueConnections().empty() || !passthroughDepth.getConnections().empty()) {
                for(const auto& stream : frames) {
                    passthroughDepth.send(stream.depth);
                }
            }
        }
    }
}

}  // namespace node
}  // namespace dai

// Explicit template instantiation for Pimpl
template class dai::Pimpl<dai::node::PointCloud::Impl>;
