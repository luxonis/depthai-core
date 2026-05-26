#include "depthai/pipeline/node/PointCloud.hpp"

#include <spdlog/logger.h>

#include <chrono>
#include <cstring>
#include <future>
#include <thread>

#ifdef DEPTHAI_ENABLE_KOMPUTE
    #include "kompute/Kompute.hpp"
#endif

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

#ifdef DEPTHAI_ENABLE_KOMPUTE
    #include "depthai/shaders/depth2pointcloud.hpp"
#endif
#include "utility/PimplImpl.hpp"

namespace dai {
namespace node {

// ── Impl: apply / get methods ──

void PointCloud::Impl::setLogger(std::shared_ptr<::spdlog::logger> log) {
    logger = log;
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
            computePointCloudDenseGPU(depthData, points);
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
            // GPU path doesn't support color yet, fall back to CPU
            if(logger) logger->warn("GPU compute does not support colorization yet, falling back to CPU");
            computePointCloudDenseColoredCPU(depthData, colorData, points);
            break;
    }
}

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
                xCoord = (col - cx) * z / fx;
                yCoord = (row - cy) * z / fy;
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
                xCoord = (col - cx) * z / fx;
                yCoord = (row - cy) * z / fy;
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
    throw std::runtime_error("Kompute not enabled in this build");
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

void PointCloud::Impl::useGPU(uint32_t device) {
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
#ifdef DEPTHAI_ENABLE_KOMPUTE
    if(resolutionChanged) {
        tensorsInitialized = false;
        algoInitialized = false;
        tensors.clear();
    }
#endif
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
PointCloud::PointCloud() : pimplPointCloud() {}

PointCloud::~PointCloud() = default;

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
Node::Input& PointCloud::getColorInput() {
    return syncInputs[colorInputName];
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

void PointCloud::useCPU() {
    pimplPointCloud->useCPU();
}

void PointCloud::useCPUMT(uint32_t numThreads) {
    pimplPointCloud->useCPUMT(numThreads);
}

void PointCloud::useGPU(uint32_t device) {
    pimplPointCloud->useGPU(device);
}

void PointCloud::setTargetCoordinateSystem(CameraBoardSocket targetCamera, bool useSpecTranslation) {
    initialConfig->setTargetCoordinateSystem(targetCamera, useSpecTranslation);
}

void PointCloud::setTargetCoordinateSystem(HousingCoordinateSystem housingCS, bool useSpecTranslation) {
    initialConfig->setTargetCoordinateSystem(housingCS, useSpecTranslation);
}

bool PointCloud::hasTransformationChanged(const ImgFrame& frame) {
    if(lastTransformation_ && lastTransformation_->isEqualTransformation(frame.transformation)) return false;
    pimpl->logger->debug("Frame transformation changed, reinitializing...");
    lastTransformation_ = frame.transformation;
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

void PointCloud::setIntrinsicsFromFrame(const ImgFrame& frame) {
    const auto width = frame.getWidth();
    const auto height = frame.getHeight();
    const auto intrinsics = frame.transformation.getIntrinsicMatrix();
    const float fx = intrinsics[0][0];
    const float fy = intrinsics[1][1];
    const float cx = intrinsics[0][2];
    const float cy = intrinsics[1][2];
    pimpl->logger->debug("Setting intrinsics: fx={}, fy={}, cx={}, cy={}, size={}x{}", fx, fy, cx, cy, width, height);
    pimplPointCloud->setIntrinsics(fx, fy, cx, cy, width, height);
}

void PointCloud::setCoordinateTransformation(const ImgFrame& depthFrame, const PointCloudConfig& config) {
    auto unit = pimplPointCloud->targetLengthUnit;
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

    // Get calibration data
    CalibrationHandler calibHandler;
    if(device) {
        calibHandler = device->getCalibration();
    } else {
        calibHandler = getParentPipeline().getCalibrationData();
    }

    // Compute the target transform based on coordSystemType
    // The final transform is: T_ref→target * T_frame→ref
    std::optional<std::vector<std::vector<float>>> T_final = std::nullopt;

    switch(coordSystemType) {
        case PointCloudConfig::CoordinateSystemType::CAMERA_SOCKET: {
            pimpl->logger->info("Using CAMERA_SOCKET transformation to {}, via ref camera {}", toString(targetCameraSocket), toString(refCamera));
            auto T_ref_to_target = calibHandler.getCameraExtrinsics(refCamera, targetCameraSocket, useSpecTranslation, unit);
            T_final = matrix::matMul(T_ref_to_target, T_frame_to_ref);
            targetExtrinsics_ = Extrinsics(T_ref_to_target, targetCameraSocket, unit);
            break;
        }

        case PointCloudConfig::CoordinateSystemType::HOUSING: {
            pimpl->logger->info("Using HOUSING transformation to housing {}, via ref camera {}", static_cast<int>(targetHousingCS), toString(refCamera));
            auto T_ref_to_housing = calibHandler.getHousingCalibration(refCamera, targetHousingCS, true, unit);
            T_final = matrix::matMul(T_ref_to_housing, T_frame_to_ref);
            targetExtrinsics_ = Extrinsics(T_ref_to_housing, CameraBoardSocket::AUTO, unit);
            break;
        }

        case PointCloudConfig::CoordinateSystemType::DEFAULT: {
            auto configMat = matrix::toVecMatrix4x4(config.getTransformationMatrix());

            if(!matrix::isIdentity4x4(configMat)) {
                pimpl->logger->info("Applying custom transform composed with frame extrinsics");
                T_final = matrix::matMul(configMat, T_frame_to_ref);
                targetExtrinsics_ = Extrinsics(configMat, refCamera, unit);
            } else if(!matrix::isIdentity4x4(T_frame_to_ref)) {
                pimpl->logger->info("Applying frame extrinsics (T_frame_to_ref)");
                T_final = T_frame_to_ref;
                targetExtrinsics_ = std::nullopt;
            } else {
                pimpl->logger->debug("No coordinate system transformation applied (identity)");
                targetExtrinsics_ = std::nullopt;
            }
            break;
        }
    }

    // Apply the final transform
    if(T_final) {
        pimplPointCloud->setExtrinsics(*T_final);
    } else {
        pimplPointCloud->clearExtrinsics();
    }
}

// ── Main initialize method ──

void PointCloud::initialize(const ImgFrame& depthFrame, const PointCloudConfig& config) {
    pimpl->logger->debug("PointCloud::initialize() called");
    pimplPointCloud->setLogger(pimpl->logger);

    // Set length unit before intrinsics/extrinsics (they depend on it)
    pimplPointCloud->setLengthUnit(config.getLengthUnit());

    // Set camera intrinsics from the depth frame
    setIntrinsicsFromFrame(depthFrame);

    // Compute and apply coordinate transformation (frame extrinsics + target transform)
    setCoordinateTransformation(depthFrame, config);

    initialized = true;
    pimpl->logger->info("PointCloud::initialize() completed");
}

//------------------------------------------------------------------
// Processing helpers (depth-only and colorized paths)
//------------------------------------------------------------------

void PointCloud::processDepthOnly(std::shared_ptr<ImgFrame> depthFrame, std::shared_ptr<PointCloudData> pc, bool organized) {
    const auto width = depthFrame->getWidth();
    const auto height = depthFrame->getHeight();
    const auto* depthData = depthFrame->getData().data();

    std::vector<Point3f> points;

    if(organized) {
        pimplPointCloud->computePointCloudDense(depthData, points);
        pimplPointCloud->applyTransformation(points);
        pc->setWidth(width);
        pc->setHeight(height);
    } else {
        std::vector<Point3f> densePoints;
        pimplPointCloud->computePointCloudDense(depthData, densePoints);
        points = pimplPointCloud->filterValidPoints(densePoints);
        pimplPointCloud->applyTransformation(points);
        pc->setWidth(static_cast<unsigned int>(points.size()));
        pc->setHeight(1);
    }

    pc->setPoints(std::move(points));
}

void PointCloud::processColorized(std::shared_ptr<ImgFrame> depthFrame,
                                  std::shared_ptr<ImgFrame> colorFrame,
                                  std::shared_ptr<PointCloudData> pc,
                                  bool organized) {
    const auto width = depthFrame->getWidth();
    const auto height = depthFrame->getHeight();

    // Validate color frame - early return to depth-only on failure
    if(colorFrame->getWidth() != width || colorFrame->getHeight() != height) {
        pimpl->logger->warn("PointCloud: color frame size ({}x{}) does not match depth ({}x{}) -- skipping colorization",
                            colorFrame->getWidth(),
                            colorFrame->getHeight(),
                            width,
                            height);
        processDepthOnly(depthFrame, pc, organized);
        return;
    }
    if(colorFrame->getType() != ImgFrame::Type::RGB888i) {
        pimpl->logger->warn("PointCloud: color frame type ({}) is not RGB888i -- skipping colorization", static_cast<int>(colorFrame->getType()));
        processDepthOnly(depthFrame, pc, organized);
        return;
    }

    // Warn about extrinsics mismatches (non-fatal)
    {
        auto depthExtrinsics = depthFrame->transformation.getExtrinsics();
        auto colorExtrinsics = colorFrame->transformation.getExtrinsics();
        if(depthExtrinsics.toCameraSocket != colorExtrinsics.toCameraSocket) {
            pimpl->logger->warn("PointCloud: color extrinsics toCameraSocket ({}) does not match depth ({}) -- colorization may be misaligned",
                                toString(colorExtrinsics.toCameraSocket),
                                toString(depthExtrinsics.toCameraSocket));
        } else if(depthExtrinsics.rotationMatrix != colorExtrinsics.rotationMatrix || depthExtrinsics.translation.x != colorExtrinsics.translation.x
                  || depthExtrinsics.translation.y != colorExtrinsics.translation.y || depthExtrinsics.translation.z != colorExtrinsics.translation.z) {
            pimpl->logger->warn(
                "PointCloud: depth and color extrinsics differ (same toCameraSocket={} but different rotation/translation) "
                "-- colorization may be misaligned",
                toString(depthExtrinsics.toCameraSocket));
        }
    }

    const auto* depthData = depthFrame->getData().data();
    const auto* colorData = colorFrame->getData().data();
    std::vector<Point3fRGBA> coloredPoints;

    if(organized) {
        pimplPointCloud->computePointCloudDenseColored(depthData, colorData, coloredPoints);
        pimplPointCloud->applyTransformation(coloredPoints);
        pc->setWidth(width);
        pc->setHeight(height);
    } else {
        std::vector<Point3fRGBA> denseColored;
        pimplPointCloud->computePointCloudDenseColored(depthData, colorData, denseColored);
        coloredPoints = pimplPointCloud->filterValidPoints(denseColored);
        pimplPointCloud->applyTransformation(coloredPoints);
        pc->setWidth(static_cast<unsigned int>(coloredPoints.size()));
        pc->setHeight(1);
    }

    pc->setPointsRGB(std::move(coloredPoints));
}

//------------------------------------------------------------------
// Main run loop
//------------------------------------------------------------------

void PointCloud::run() {
    // Detect color mode at runtime — the color entry in syncInputs only exists
    // if the user accessed getColorInput() and linked to it before pipeline.start().
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    colorMode = syncInputs.has(colorInputName);
    if(colorMode) {
        syncInputs[colorInputName].setBlocking(false);
        syncInputs[colorInputName].setMaxSize(4);
    }
#endif
    // On device, apply the deserialized config from properties
    *initialConfig = properties.initialConfig;

    pimpl->logger->info("PointCloud node started (colorMode={})", colorMode);

    uint32_t currentEepromId = getParentPipeline().getEepromId();
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

        auto depthFrame = group->get<ImgFrame>(depthInputName);
        if(!depthFrame) {
            pimpl->logger->warn("PointCloud: failed to extract depth frame from MessageGroup -- skipping");
            continue;
        }

        auto colorFrame = colorMode ? group->get<ImgFrame>(colorInputName) : nullptr;

        // Check for runtime config update
        auto newConfig = inputConfig.tryGet<PointCloudConfig>();
        if(newConfig) {
            latestConfig = newConfig;
            initialized = false;
        }

        // Read organized mode from config
        const bool organized = latestConfig->getOrganized();

        const uint32_t latestEepromId = getParentPipeline().getEepromId();
        if(latestEepromId > currentEepromId) {
            pimpl->logger->debug("Calibration data changed (ID: {} -> {}), reinitializing...", currentEepromId, latestEepromId);
            initialized = false;
            currentEepromId = latestEepromId;
        }

        if(hasTransformationChanged(*depthFrame)) initialized = false;

        if(!initialized) {
            initialize(*depthFrame, *latestConfig);
        }

        const auto width = depthFrame->getWidth();
        const auto height = depthFrame->getHeight();

        // Validate that the buffer contains packed uint16_t depth data
        const auto expectedBytes = static_cast<std::size_t>(width) * height * sizeof(uint16_t);
        if(depthFrame->getType() != ImgFrame::Type::RAW16 || depthFrame->getData().size() != expectedBytes) {
            pimpl->logger->warn("PointCloud: unexpected depth frame (type={}, size={}, expected {} bytes for {}x{} RAW16) -- skipping frame",
                                static_cast<int>(depthFrame->getType()),
                                depthFrame->getData().size(),
                                expectedBytes,
                                width,
                                height);
            continue;
        }

        // Create PointCloudData
        auto pc = std::make_shared<PointCloudData>();
        pc->setTimestamp(depthFrame->getTimestamp());
        pc->setTimestampDevice(depthFrame->getTimestampDevice());
        pc->setSequenceNum(depthFrame->getSequenceNum());
        pc->setInstanceNum(depthFrame->getInstanceNum());
        auto outputTransformation = depthFrame->getTransformation();
        if(targetExtrinsics_) {
            outputTransformation.setExtrinsics(*targetExtrinsics_);
        }
        pc->setTransformation(outputTransformation);

        if(colorFrame) {
            processColorized(depthFrame, colorFrame, pc, organized);
        } else {
            processDepthOnly(depthFrame, pc, organized);
        }

        pc->updateBoundingBox();

        {
            auto blockEvent = this->outputBlockEvent();
            outputPointCloud.send(pc);

            if(!passthroughDepth.getQueueConnections().empty() || !passthroughDepth.getConnections().empty()) {
                passthroughDepth.send(depthFrame);
            }
        }
    }
}

}  // namespace node
}  // namespace dai

// Explicit template instantiation for Pimpl
template class dai::Pimpl<dai::node::PointCloud::Impl>;
