#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <utility>

#include "../DsImgDispatcher.hpp"
#include "../ImageManip.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/properties/ImageManipProperties.hpp"
#include "depthai/utility/ImageManipImpl.hpp"
#include "depthai/utility/OffsetMemory.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "fastcv/fastcv.h"
#include "gpu/ImageManipGPUMat.hpp"
#include "gpu/ImageManipGPUMesh.hpp"
#include "pipeline/DmaFramePool.hpp"
#include "pipeline/NodeTiming.hpp"
#include "utilities/AcceleratorMemory.hpp"
#include "utilities/FastCvUtils.hpp"
#include "utilities/GpuAvailability.hpp"

namespace dai::gate {

using namespace impl;

constexpr float SIZE_EQUALITY_THRESHOLD_MULTIPLIER = 0.001;
constexpr std::size_t AUTO_GPU_MIN_FRAME_WIDTH = 1000;
constexpr std::size_t AUTO_GPU_MIN_FRAME_HEIGHT = 1000;

inline std::array<float, 9> flatten(std::array<std::array<float, 3>, 3> mat) {
    return {mat[0][0], mat[0][1], mat[0][2], mat[1][0], mat[1][1], mat[1][2], mat[2][0], mat[2][1], mat[2][2]};
}

std::array<std::array<float, 3>, 3> getUndistortBaseIntrinsics(const ImgTransformation& transformation, size_t width, size_t height, std::optional<float> alpha) {
    const auto intrinsics = transformation.getIntrinsicMatrix();
    const auto distortionCoefficients = transformation.getDistortionCoefficients();
    if(!alpha.has_value() || distortionCoefficients.empty() || transformation.getDistortionModel() != CameraModel::Perspective) {
        return intrinsics;
    }

    auto cameraMatrix = matrix::matrix3x3ToCvMat(intrinsics);
    cv::Mat distCoeffs(1, static_cast<int>(distortionCoefficients.size()), CV_32FC1);
    std::memcpy(distCoeffs.data, distortionCoefficients.data(), distortionCoefficients.size() * sizeof(float));
    auto newCameraMatrix =
        cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(static_cast<int>(width), static_cast<int>(height)), alpha.value());
    return matrix::cvMatToMatrix3x3(newCameraMatrix);
}

uint32_t getNumPlanes(ImgFrame::Type imgType) {
    switch(imgType) {
        case ImgFrame::Type::RGB888i:
        case ImgFrame::Type::BGR888i:
        case ImgFrame::Type::RAW8:
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::RAW16:
            return 1;
        case ImgFrame::Type::NV12:
            return 2;
        case ImgFrame::Type::BGR888p:
        case ImgFrame::Type::RGB888p:
        case ImgFrame::Type::YUV420p:
            return 3;
        case ImgFrame::Type::YUV422i:
        case ImgFrame::Type::YUV444p:
        case ImgFrame::Type::YUV422p:
        case ImgFrame::Type::YUV400p:
        case ImgFrame::Type::RGBA8888:
        case ImgFrame::Type::RGB161616:
        case ImgFrame::Type::LUT2:
        case ImgFrame::Type::LUT4:
        case ImgFrame::Type::LUT16:
        case ImgFrame::Type::RAW14:
        case ImgFrame::Type::RAW12:
        case ImgFrame::Type::RAW10:
        case ImgFrame::Type::PACK10:
        case ImgFrame::Type::PACK12:
        case ImgFrame::Type::YUV444i:
        case ImgFrame::Type::NV21:
        case ImgFrame::Type::BITSTREAM:
        case ImgFrame::Type::HDR:
        case ImgFrame::Type::RGBF16F16F16p:
        case ImgFrame::Type::BGRF16F16F16p:
        case ImgFrame::Type::RGBF16F16F16i:
        case ImgFrame::Type::BGRF16F16F16i:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            throw std::runtime_error("Unsupported image format. Only YUV420p, RGB888p, BGR888p, RGB888i, BGR888i, RAW8, RAW16, NV12, GRAY8 are supported");
    }
    throw std::runtime_error("Unsupported image format");
}

namespace {

constexpr std::size_t UNDISTORT_MESH_STEP = 8;

bool validUndistortMatrix(const std::array<float, 9>& matrix, float epsilon = 1e-6) {
    auto floatEq = [epsilon](float a, float b) { return std::fabs(a - b) <= epsilon; };
    return !floatEq(matrix[0], 0.f) && floatEq(matrix[1], 0.f) && floatEq(matrix[3], 0.f) && !floatEq(matrix[4], 0.f) && floatEq(matrix[6], 0.f)
           && floatEq(matrix[7], 0.f) && floatEq(matrix[8], 1.f);
}

std::size_t getUndistortMeshDimension(std::size_t dstSize) {
    if(dstSize < 2) {
        throw std::runtime_error("WarpGPUMesh undistortion requires output dimensions of at least 2 pixels");
    }

    const auto meshCells = (dstSize - 1 + UNDISTORT_MESH_STEP - 1) / UNDISTORT_MESH_STEP;
    return std::min(dstSize, meshCells + 1);
}

float sampleFloatMap(const cv::Mat& map, float x, float y) {
    const int width = map.cols;
    const int height = map.rows;
    const int x0 = static_cast<int>(std::floor(x));
    const int y0 = static_cast<int>(std::floor(y));
    const int x1 = std::min(x0 + 1, width - 1);
    const int y1 = std::min(y0 + 1, height - 1);
    const float tx = x - static_cast<float>(x0);
    const float ty = y - static_cast<float>(y0);
    const float v00 = map.at<float>(y0, x0);
    const float v10 = map.at<float>(y0, x1);
    const float v01 = map.at<float>(y1, x0);
    const float v11 = map.at<float>(y1, x1);
    return v00 * (1.0f - tx) * (1.0f - ty) + v10 * tx * (1.0f - ty) + v01 * (1.0f - tx) * ty + v11 * tx * ty;
}

std::pair<std::shared_ptr<const std::vector<float>>, std::shared_ptr<const std::vector<float>>> getMeshFromUndistortMaps(const cv::Mat& mapX,
                                                                                                                         const cv::Mat& mapY,
                                                                                                                         std::size_t meshWidth,
                                                                                                                         std::size_t meshHeight) {
    if(mapX.empty() || mapY.empty()) {
        throw std::runtime_error("Empty undistortion maps for WarpGPUMesh");
    }
    if(mapX.size() != mapY.size()) {
        throw std::runtime_error("Different sizes of undistortion maps for WarpGPUMesh");
    }
    if(mapX.type() != CV_32FC1 || mapY.type() != CV_32FC1) {
        throw std::runtime_error("Invalid undistortion map types for WarpGPUMesh");
    }

    auto meshX = std::make_shared<std::vector<float>>();
    auto meshY = std::make_shared<std::vector<float>>();
    meshX->reserve(meshWidth * meshHeight);
    meshY->reserve(meshWidth * meshHeight);
    const auto dstWidth = static_cast<std::size_t>(mapX.cols);
    const auto dstHeight = static_cast<std::size_t>(mapX.rows);

    for(std::size_t y = 0; y < meshHeight; ++y) {
        const float dstY = meshHeight > 1 ? static_cast<float>(y) * static_cast<float>(dstHeight - 1) / static_cast<float>(meshHeight - 1) : 0.0f;
        for(std::size_t x = 0; x < meshWidth; ++x) {
            const float dstX = meshWidth > 1 ? static_cast<float>(x) * static_cast<float>(dstWidth - 1) / static_cast<float>(meshWidth - 1) : 0.0f;
            meshX->push_back(sampleFloatMap(mapX, dstX, dstY));
            meshY->push_back(sampleFloatMap(mapY, dstX, dstY));
        }
    }

    return {meshX, meshY};
}

class UndistortWarpMeshImpl : public Undistort {
    WarpGPUMesh warpGpu;
    std::vector<MeshConfig> planeConfigs;

    void initMaps(std::array<float, 9> cameraMatrix,
                  std::array<float, 9> newCameraMatrix,
                  std::vector<float> distCoeffs,
                  dai::ImgFrame::Type type,
                  uint32_t srcWidth,
                  uint32_t srcHeight,
                  uint32_t dstWidth,
                  uint32_t dstHeight) {
        this->cameraMatrix = std::move(cameraMatrix);
        this->newCameraMatrix = std::move(newCameraMatrix);
        this->distCoeffs = std::move(distCoeffs);
        this->type = type;
        this->srcWidth = srcWidth;
        this->srcHeight = srcHeight;
        this->dstWidth = dstWidth;
        this->dstHeight = dstHeight;
        this->planeConfigs.clear();

        cv::Mat cvCameraMatrix(3, 3, CV_32F, this->cameraMatrix.data());
        cv::Mat cvNewCameraMatrix(3, 3, CV_32F, this->newCameraMatrix.data());

        cv::Mat mapX;
        cv::Mat mapY;
        cv::initUndistortRectifyMap(cvCameraMatrix, this->distCoeffs, cv::Mat(), cvNewCameraMatrix, cv::Size(dstWidth, dstHeight), CV_32FC1, mapX, mapY);

        auto addPlaneConfig = [&](const cv::Mat& planeMapX,
                                  const cv::Mat& planeMapY,
                                  std::size_t planeSrcWidth,
                                  std::size_t planeSrcHeight,
                                  std::size_t planeDstWidth,
                                  std::size_t planeDstHeight,
                                  std::uint32_t channels,
                                  std::uint32_t bytesPerChannel,
                                  std::array<std::uint16_t, 4> borderValue) {
            MeshConfig config;
            config.meshWidth = getUndistortMeshDimension(planeDstWidth);
            config.meshHeight = getUndistortMeshDimension(planeDstHeight);
            auto [meshX, meshY] = getMeshFromUndistortMaps(planeMapX, planeMapY, config.meshWidth, config.meshHeight);
            config.meshX = std::move(meshX);
            config.meshY = std::move(meshY);
            config.srcWidth = planeSrcWidth;
            config.srcHeight = planeSrcHeight;
            config.dstWidth = planeDstWidth;
            config.dstHeight = planeDstHeight;
            config.channels = channels;
            config.bytesPerChannel = bytesPerChannel;
            config.borderValue = borderValue;
            planeConfigs.push_back(std::move(config));
            return planeConfigs.back();
        };

        switch(type) {
            case ImgFrame::Type::RGB888i:
            case ImgFrame::Type::BGR888i:
                addPlaneConfig(mapX, mapY, srcWidth, srcHeight, dstWidth, dstHeight, 3, 1, {0, 0, 0, 0});
                break;
            case ImgFrame::Type::BGR888p:
            case ImgFrame::Type::RGB888p: {
                auto config = addPlaneConfig(mapX, mapY, srcWidth, srcHeight, dstWidth, dstHeight, 1, 1, {0, 0, 0, 0});
                planeConfigs.push_back(config);
                planeConfigs.push_back(config);
                break;
            }
            case ImgFrame::Type::YUV420p:
            case ImgFrame::Type::NV12: {
                addPlaneConfig(mapX, mapY, srcWidth, srcHeight, dstWidth, dstHeight, 1, 1, {0, 0, 0, 0});

                cv::Mat cvCameraMatrixHalf = cvCameraMatrix.clone();
                cv::Mat cvNewCameraMatrixHalf = cvNewCameraMatrix.clone();
                cvCameraMatrixHalf.at<float>(0, 0) /= 2;
                cvCameraMatrixHalf.at<float>(1, 1) /= 2;
                cvCameraMatrixHalf.at<float>(0, 2) /= 2;
                cvCameraMatrixHalf.at<float>(1, 2) /= 2;
                cvNewCameraMatrixHalf.at<float>(0, 0) /= 2;
                cvNewCameraMatrixHalf.at<float>(1, 1) /= 2;
                cvNewCameraMatrixHalf.at<float>(0, 2) /= 2;
                cvNewCameraMatrixHalf.at<float>(1, 2) /= 2;

                cv::Mat mapXHalf;
                cv::Mat mapYHalf;
                cv::initUndistortRectifyMap(cvCameraMatrixHalf,
                                            this->distCoeffs,
                                            cv::Mat(),
                                            cvNewCameraMatrixHalf,
                                            cv::Size(dstWidth / 2, dstHeight / 2),
                                            CV_32FC1,
                                            mapXHalf,
                                            mapYHalf);
                if(type == ImgFrame::Type::NV12) {
                    addPlaneConfig(mapXHalf, mapYHalf, srcWidth / 2, srcHeight / 2, dstWidth / 2, dstHeight / 2, 2, 1, {127, 127, 0, 0});
                } else {
                    auto config = addPlaneConfig(mapXHalf, mapYHalf, srcWidth / 2, srcHeight / 2, dstWidth / 2, dstHeight / 2, 1, 1, {127, 0, 0, 0});
                    planeConfigs.push_back(config);
                }
                break;
            }
            case ImgFrame::Type::RAW8:
            case ImgFrame::Type::GRAY8:
                addPlaneConfig(mapX, mapY, srcWidth, srcHeight, dstWidth, dstHeight, 1, 1, {0, 0, 0, 0});
                break;
            case ImgFrame::Type::RAW16:
                addPlaneConfig(mapX, mapY, srcWidth, srcHeight, dstWidth, dstHeight, 1, 2, {0, 0, 0, 0});
                break;
            default:
                throw std::runtime_error("Unsupported image format for GPU undistortion backend");
        }

        warpGpu.setLogger(this->logger);
        if(!warpGpu.init(static_cast<int>(planeConfigs.size()))) {
            throw std::runtime_error("Failed initializing WarpGPUMesh for undistortion");
        }
    }

   public:
    using Undistort::Undistort;

    BuildStatus build(std::array<float, 9> cameraMatrix,
                      std::array<float, 9> newCameraMatrix,
                      std::vector<float> distCoeffs,
                      dai::ImgFrame::Type type,
                      uint32_t srcWidth,
                      uint32_t srcHeight,
                      uint32_t dstWidth,
                      uint32_t dstHeight) override {
        if(distCoeffs.empty()) {
            planeConfigs.clear();
            return BuildStatus::NOT_USED;
        }

        if(!validUndistortMatrix(cameraMatrix)) {
            logger->error("Invalid camera matrix provided for undistortion, will not be applied ({},{},{},{},{},{},{},{},{})",
                          cameraMatrix[0],
                          cameraMatrix[1],
                          cameraMatrix[2],
                          cameraMatrix[3],
                          cameraMatrix[4],
                          cameraMatrix[5],
                          cameraMatrix[6],
                          cameraMatrix[7],
                          cameraMatrix[8]);
            planeConfigs.clear();
            return BuildStatus::ERROR;
        }

        try {
            if(!validUndistortMatrix(newCameraMatrix)) {
                if(type != this->type || srcWidth != this->srcWidth || srcHeight != this->srcHeight || distCoeffs != this->distCoeffs
                   || cameraMatrix != this->cameraMatrix || this->dstWidth != srcWidth || this->dstHeight != srcHeight
                   || this->newCameraMatrix != cameraMatrix) {  // Undistort alpha is not applied here: fine for resizers only
                    initMaps(cameraMatrix, cameraMatrix, std::move(distCoeffs), type, srcWidth, srcHeight, srcWidth, srcHeight);
                    return BuildStatus::TWO_SHOT;
                }
                return BuildStatus::NOT_BUILT;
            }

            if(type != this->type || srcWidth != this->srcWidth || srcHeight != this->srcHeight || dstWidth != this->dstWidth || dstHeight != this->dstHeight
               || distCoeffs != this->distCoeffs || cameraMatrix != this->cameraMatrix || newCameraMatrix != this->newCameraMatrix) {
                initMaps(cameraMatrix, newCameraMatrix, std::move(distCoeffs), type, srcWidth, srcHeight, dstWidth, dstHeight);
                return BuildStatus::ONE_SHOT;
            }
            return BuildStatus::NOT_BUILT;
        } catch(const std::exception& ex) {
            logger->warn("ImageManip | Undistort WarpGPUMesh | {}", ex.what());
            planeConfigs.clear();
            return BuildStatus::ERROR;
        }
    }

    void undistort(const UndistortInput& srcI, const UndistortInput& dstI, uint32_t plane) override {
        if(planeConfigs.empty()) {
            throw std::runtime_error("UndistortWarpMeshImpl: mesh configuration is not initialized");
        }

        MeshConfig config = planeConfigs[plane];
        if(srcI.width != config.srcWidth || srcI.height != config.srcHeight || dstI.width != config.dstWidth || dstI.height != config.dstHeight
           || srcI.channels != config.channels || dstI.channels != config.channels || srcI.bpp != config.bytesPerChannel
           || dstI.bpp != config.bytesPerChannel) {
            throw std::runtime_error("UndistortWarpMeshImpl: plane shape does not match the configured mesh");
        }

        config.srcStrideBytes = srcI.stride;
        config.dstStrideBytes = dstI.stride;

        if(!warpGpu.buildPlane(static_cast<int>(plane), config)) {
            throw std::runtime_error("UndistortWarpMeshImpl: failed to build GPU mesh plane");
        }
        warpGpu.enqueueWarpPlane(static_cast<int>(plane), srcI.data, dstI.data);
    }

    void finish() override {
        warpGpu.warp();
    }
};

}  // namespace

class WarpQ : public Warp {
    enum class OptOp : uint8_t {
        PERSPECTIVE,
        AFFINE,
        CROP,
        SCALE_DOWN,
        SCALE_UP,
        SCALE,
    } optOp = OptOp::PERSPECTIVE;

    using Backend = dai::ImageManipProperties::Backend;

    Backend backend;

    bool gpuEnabled = false;
    std::variant<std::monostate, WarpGPUMat, WarpGPUMesh> warpGpu = std::monostate{};

    bool hwScalerEnabled = true;
    bool dsImgInitialized = false;
    ImgFrame::Type dsImgType = ImgFrame::Type::NONE;
    bool firstFrameOver = false;
    DsImgDispatcher dsImg;
    std::shared_ptr<AcceleratorMemory> dsImgAux;
    FrameSpecs dsImgAuxSpecs;
    bool canUseAccelerator();
    bool hwDownscale(const std::shared_ptr<OffsetMemory>& src, std::shared_ptr<OffsetMemory> dst, const ImgFrame::Type type);
    bool fastCvDownscaleu8(const std::shared_ptr<OffsetMemory> src,
                           std::shared_ptr<OffsetMemory> dst,
                           size_t srcWidth,
                           size_t srcHeight,
                           size_t srcStride,
                           size_t dstWidth,
                           size_t dstHeight,
                           size_t dstStride,
                           uint16_t numChannels);

    bool fastCvEnabled = true;
    bool fastCvInitialized = false;

    std::shared_ptr<DmaHeapMemory> auxFrame;

    std::unique_ptr<Undistort> undistortImpl;

    void transform(const std::shared_ptr<OffsetMemory> srcData,
                   std::shared_ptr<OffsetMemory> dstData,
                   const size_t srcWidth,
                   const size_t srcHeight,
                   const size_t srcStride,
                   const size_t dstWidth,
                   const size_t dstHeight,
                   const size_t dstStride,
                   const uint16_t numChannels,
                   const uint16_t bpp,
                   const std::array<std::array<float, 3>, 3> matrix,
                   const std::vector<uint32_t>& backgroundColor);
    bool shouldUseGpuForFrame(std::size_t dstFrameWidth, std::size_t dstFrameHeight) const;
    bool shouldUseGpuWarp() const;
    bool enqueueGpuTransform(int plane,
                             const std::shared_ptr<OffsetMemory> srcData,
                             std::shared_ptr<OffsetMemory> dstData,
                             const size_t srcWidth,
                             const size_t srcHeight,
                             const size_t srcStride,
                             const size_t dstWidth,
                             const size_t dstHeight,
                             const size_t dstStride,
                             const uint16_t numChannels,
                             const uint16_t bpp,
                             const std::array<std::array<float, 3>, 3> matrix,
                             const std::vector<uint32_t>& backgroundColor);

   public:
    void init(ImageManipProperties& props) override {
        fastCvEnabled = props.performanceMode != ImageManipProperties::PerformanceMode::PERFORMANCE;
        if(fastCvEnabled && !fastCvInitialized) {
            fastCvInitialized = true;
            fcvOperationMode operMode = FASTCV_OP_PERFORMANCE;  // FASTCV_OP_CPU_OFFLOAD;
            switch(props.performanceMode) {
                case ImageManipProperties::PerformanceMode::BALANCED:
                    operMode = FASTCV_OP_PERFORMANCE;
                    break;
                case ImageManipProperties::PerformanceMode::LOW_POWER:
                    operMode = FASTCV_OP_LOW_POWER;
                    break;
                default:
                    break;
            }
            fcvInit(operMode);
        }

        backend = props.backend;
        gpuEnabled = props.backend == ImageManipProperties::Backend::GPU || props.backend == ImageManipProperties::Backend::AUTO && utils::hasGPU();
        hwScalerEnabled = props.backend == ImageManipProperties::Backend::HW;
        if(hwScalerEnabled) dsImg.registerNode();
        if(gpuEnabled) {
            warpGpu.emplace<WarpGPUMat>();
            std::get<WarpGPUMat>(warpGpu).setLogger(this->logger);
        }
        logger->info("GPU enabled: {}, hardware accelerator enabled: {}", gpuEnabled, hwScalerEnabled);
    }
    void build(const FrameSpecs srcFrameSpecs,
               const FrameSpecs dstFrameSpecs,
               const ImgFrame::Type type,
               const std::array<std::array<float, 3>, 3> matrix,
               std::vector<std::array<std::array<float, 2>, 4>> srcCorners) override;
    void buildUndistort(bool enable,
                        const std::array<float, 9>& cameraMatrix,
                        const std::array<float, 9>& newCameraMatrix,
                        const std::vector<float>& distCoeffs,
                        const std::optional<float> alpha,
                        const ImgFrame::Type type,
                        const uint32_t srcWidth,
                        const uint32_t srcHeight,
                        const uint32_t dstWidth,
                        const uint32_t dstHeight) override;

    void apply(const std::shared_ptr<OffsetMemory>& src, std::shared_ptr<OffsetMemory> dst) override;
};

class ColorChangeQ : public ColorChange {
    std::shared_ptr<AcceleratorMemory> ccAuxFrame;

   public:
    void build(const FrameSpecs srcFrameSpecs, const FrameSpecs dstFrameSpecs, const ImgFrame::Type typeFrom, const ImgFrame::Type typeTo);

    void apply(const std::shared_ptr<OffsetMemory>& src, std::shared_ptr<OffsetMemory> dst);
};

void setDstMetadata(const impl::ImageManipOperations<DmaHeapMemory, ColorChangeQ, WarpQ>& manip, const ImgFrame& srcFrame, ImgFrame& dstFrame) {
    auto outType = manip.getOutputFrameType();
    auto dstSpecs = manip.getOutputFrameSpecs(outType);
    dstFrame.setBufferMetadataFrom(&srcFrame);
    dstFrame.sourceFb = srcFrame.sourceFb;
    dstFrame.cam = srcFrame.cam;
    dstFrame.instanceNum = srcFrame.instanceNum;
    dstFrame.category = srcFrame.category;
    dstFrame.event = srcFrame.event;
    dstFrame.fb.height = dstSpecs.height;
    dstFrame.fb.width = dstSpecs.width;
    dstFrame.fb.stride = dstSpecs.p1Stride;
    dstFrame.fb.p1Offset = dstSpecs.p1Offset;
    dstFrame.fb.p2Offset = dstSpecs.p2Offset;
    dstFrame.fb.p3Offset = dstSpecs.p3Offset;
    dstFrame.setType(outType);

    if(manip.undistortEnabled()) {
        const auto sourceSize = srcFrame.transformation.getSourceSize();
        const auto undistortBaseIntrinsics =
            getUndistortBaseIntrinsics(srcFrame.transformation, srcFrame.getWidth(), srcFrame.getHeight(), manip.getUndistortAlpha());
        const auto undistortMatrix = impl::matmul(undistortBaseIntrinsics, srcFrame.transformation.getSourceIntrinsicMatrixInv());
        const auto totalMatrix = impl::matmul(manip.getMatrix(), undistortMatrix);

        dstFrame.transformation = ImgTransformation(sourceSize.first,  // From source metadata, so the full source-to-output transform applies directly
                                                    sourceSize.second,
                                                    srcFrame.transformation.getSourceIntrinsicMatrix(),
                                                    srcFrame.transformation.getDistortionModel(),
                                                    {},
                                                    srcFrame.transformation.getExtrinsics());
        dstFrame.transformation.addSrcCrops(srcFrame.transformation.getSrcCrops());
        dstFrame.transformation.addSrcCrops(manip.getSrcCrops());
        dstFrame.transformation.addTransformation(totalMatrix);
    } else {
        dstFrame.transformation = srcFrame.transformation;
        dstFrame.transformation.addTransformation(manip.getMatrix());
        dstFrame.transformation.addSrcCrops(manip.getSrcCrops());
    }
    dstFrame.transformation.setSize(dstSpecs.width, dstSpecs.height);
}

void ImageManip::run() {
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");
    impl::ImageManipOperations<DmaHeapMemory, ColorChangeQ, WarpQ> manip(properties, logger);
    std::shared_ptr<DmaFramePool> pool = nullptr;

    impl::loop<ImageManip>(
        *this,
        properties.initialConfig,
        logger,
        [this](auto tAbsoluteBeginning, auto tGotInput, auto tProcessed, auto tAbsoluteEnd) {
            dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
        },
        [&](const ImageManipConfig& config, const ImgFrame& frame) {
            auto start = std::chrono::steady_clock::now();
            auto srcFrameSpecs = impl::getSrcFrameSpecs(frame.fb);
            manip.build(config.base, config.outputFrameType, srcFrameSpecs, frame.getType());
            auto undistortBaseIntrinsics = getUndistortBaseIntrinsics(frame.transformation, frame.getWidth(), frame.getHeight(), std::nullopt);
            auto newCameraMatrix = impl::matmul(manip.getMatrix(), undistortBaseIntrinsics);
            manip.buildUndistort(config.base.undistort,
                                 flatten(frame.transformation.getIntrinsicMatrix()),
                                 flatten(newCameraMatrix),
                                 frame.transformation.getDistortionCoefficients(),
                                 std::nullopt,
                                 frame.getType(),
                                 frame.getWidth(),
                                 frame.getHeight(),
                                 manip.getOutputWidth(),
                                 manip.getOutputHeight());
            logger->trace("ImageManip took '{}' ms",
                          std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count());
            return manip.getOutputSize();
        },
        [&](size_t outputSize) {
            if(pool == nullptr || pool->getFrameDataSize() < outputSize) {
                pool = DmaFramePool::create(properties.numFramesPool, outputSize);
            }
            auto frame = pool->tryGet();
            if(!frame) {
                auto newPoolSize = std::min(properties.maxPoolSize / (unsigned int)outputSize, 30u);  // 30: more frames is overkill even for HFR
                if(newPoolSize > pool->getCapacity()) {
                    pool = DmaFramePool::create(newPoolSize, outputSize);
                    logger->info("ImageManip | pool ran out of frames, resizing pool to {} frames with a total size of {}B", newPoolSize, newPoolSize * outputSize);
                }
                frame = pool->get();  // Memory use can exceed maxPoolSize here: the frames of the old pool are still live
            }
            return frame;
        },
        [&](const std::shared_ptr<OffsetMemory>& src, std::shared_ptr<OffsetMemory> dst) {
            auto start = std::chrono::steady_clock::now();
            std::shared_ptr<OffsetMemory> srcMem = src;
            {
                auto t1 = std::chrono::steady_clock::now();
                auto accelMem = AcceleratorMemory::convert(ConvertedOffsetMemory::cast<Memory>(src));
                if(accelMem->getData().data() != src->getData().data()) {
                    srcMem = accelMem;
                }
                auto t2 = std::chrono::steady_clock::now();
            }
            auto out = manip.apply(srcMem, std::move(dst));
            logger->trace("ImageManip | benchmark | apply | {}us",
                          std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start).count());
            return out;
        },
        [&](const ImgFrame& srcFrame, ImgFrame& dstFrame) { setDstMetadata(manip, srcFrame, dstFrame); });
}

void ManipResizer::runThreadedResizer(int numFramesPool,
                                      std::function<std::shared_ptr<ImgFrame>(std::shared_ptr<ImgFrame>)> prepareSend,
                                      bool useCpu,
                                      std::shared_ptr<spdlog::async_logger> pLogger) {
    isRunning = true;
    fcvInit(FASTCV_OP_PERFORMANCE);
    auto logger = std::make_shared<spdlog::async_logger>(*pLogger);
    this->resizerThread = std::make_unique<std::thread>([this, numFramesPool, logger, prepareSend, useCpu]() {
        ImageManipProperties properties;
        properties.initialConfig = cfg;
        if(useCpu) {  // Undistort on the GPU: AUTO undistorts small frames on the CPU, about 3x slower than the GPU mesh
            properties.backend = ImageManipProperties::Backend::CPU;
        } else if(cfg.base.undistort && utils::hasGPU()) {
            properties.backend = ImageManipProperties::Backend::GPU;
        } else {
            properties.backend = ImageManipProperties::Backend::AUTO;
        }
        properties.performanceMode = ImageManipProperties::PerformanceMode::PERFORMANCE;
        impl::ImageManipOperations<DmaHeapMemory, ColorChangeQ, WarpQ> manip(properties, logger);

        std::shared_ptr<DmaFramePool> pool = nullptr;

        bool first = true;
        try {
            while(isRunning) {
                auto msg = inQ.get<dai::ImgFrame>();
                if(!msg) continue;
                if(first) {
                    first = false;
                    auto srcFrameSpecs = impl::getSrcFrameSpecs(msg->fb);
                    manip.build(cfg.base, cfg.outputFrameType, srcFrameSpecs, msg->getType());
                    auto undistortBaseIntrinsics =
                        getUndistortBaseIntrinsics(msg->transformation, msg->getWidth(), msg->getHeight(), alphaScaling);
                    auto newCameraMatrix = impl::matmul(manip.getMatrix(), undistortBaseIntrinsics);
                    manip.buildUndistort(cfg.base.undistort,
                                         flatten(msg->transformation.getIntrinsicMatrix()),
                                         flatten(newCameraMatrix),
                                         msg->transformation.getDistortionCoefficients(),
                                         alphaScaling,
                                         msg->getType(),
                                         msg->getWidth(),
                                         msg->getHeight(),
                                         manip.getOutputWidth(),
                                         manip.getOutputHeight());
                    pool = DmaFramePool::create(numFramesPool, manip.getOutputSize());
                }
                auto dst = pool->get();
                dst->data->setSize(dst->data->getMaxSize());

                std::shared_ptr<OffsetMemory> srcMem = ConvertedOffsetMemory::convert(msg->data);
                {
                    auto t1 = std::chrono::steady_clock::now();
                    auto accelMem = AcceleratorMemory::convert(ConvertedOffsetMemory::cast<Memory>(srcMem));
                    if(accelMem->getData().data() != srcMem->getData().data()) {
                        srcMem = accelMem;
                    }
                    auto t2 = std::chrono::steady_clock::now();
                }
                manip.apply(srcMem, ConvertedOffsetMemory::convert(dst->data));
                setDstMetadata(manip, *msg, *dst);

                outQ.send(prepareSend(dst));
            }
        } catch(const std::exception& e) {
            logger->error("Resizer encountered an error: {}", e.what());
        }
    });
}

void ManipResizer::stop() {
    isRunning = false;
}
void ManipResizer::join() {
    if(resizerThread != nullptr && resizerThread->joinable()) resizerThread->join();
}

static void printImgFrame(const ImgFrame& img) {
    printf(
        "fb: p1: %p, size: %ld, w: %d, h: %d, stride: %d, "
        "planeStride: %d, Bpp: %lf, type: %d, p1_ofs: %d, p2_ofs: %d\n",
        img.getData().data(),
        img.getData().size(),
        img.getWidth(),
        img.getHeight(),
        img.getStride(),
        img.getPlaneStride(),
        img.getBytesPerPixel(),
        (int)img.getType(),
        img.fb.p1Offset,
        img.fb.p2Offset);
}

inline bool float_eq(float a, float b) {
    return fabs(a - b) <= 1e-6f;
}
inline bool pxl_eq(float a, float b) {
    return fabs(a - b) <= 1e-1f;
}

std::size_t ceilDiv(std::size_t value, std::size_t divisor) {
    return (value + divisor - 1) / divisor;
}

bool matrixIsAfine(const std::array<std::array<float, 3>, 3>& matrix) {
    return float_eq(matrix[2][0], 0.f) && float_eq(matrix[2][1], 0.f) && float_eq(matrix[2][2], 1.f);
}
bool matrixIsCroppedScale(const std::array<std::array<float, 3>, 3>& matrix) {
    return float_eq(matrix[0][1], 0.f) && float_eq(matrix[1][0], 0.f) && matrixIsAfine(matrix) && matrix[0][0] > 0.f && matrix[1][1] > 0.f;
}
bool matrixIsScale(const std::array<std::array<float, 3>, 3>& matrix) {
    return float_eq(matrix[0][2], 0.f) && float_eq(matrix[1][2], 0.f) && matrixIsCroppedScale(matrix);
}
bool matrixIsCrop(const std::array<std::array<float, 3>, 3>& matrix) {
    return float_eq(matrix[0][0], 1.f) && float_eq(matrix[1][1], 1.f) && matrixIsCroppedScale(matrix);
}

void WarpQ::build(const FrameSpecs srcFrameSpecs,
                  const FrameSpecs dstFrameSpecs,
                  const ImgFrame::Type type,
                  const std::array<std::array<float, 3>, 3> matrix,
                  std::vector<std::array<std::array<float, 2>, 4>> srcCorners) {
    this->matrix = matrix;
    this->type = type;
    this->srcSpecs = srcFrameSpecs;
    this->dstSpecs = dstFrameSpecs;

    const uint32_t inWidth = srcFrameSpecs.width;
    const uint32_t inHeight = srcFrameSpecs.height;
    this->sourceMinX = 0;
    this->sourceMaxX = inWidth;
    this->sourceMinY = 0;
    this->sourceMaxY = inHeight;
    for(const auto& corners : srcCorners) {
        auto [minx, maxx, miny, maxy] = getOuterRect(std::vector<std::array<float, 2>>(corners.begin(), corners.end()));
        this->sourceMinX = std::max(this->sourceMinX, (size_t)std::floor(std::max(minx, 0.f)));
        this->sourceMinY = std::max(this->sourceMinY, (size_t)std::floor(std::max(miny, 0.f)));
        this->sourceMaxX = std::min(this->sourceMaxX, (size_t)std::ceil(maxx));
        this->sourceMaxY = std::min(this->sourceMaxY, (size_t)std::ceil(maxy));
    }
    if(this->sourceMinX >= this->sourceMaxX || this->sourceMinY >= this->sourceMaxY) throw std::runtime_error("Initial crop is outside the source image");

    if(matrixIsCrop(this->matrix))
        optOp = OptOp::CROP;
    else if(matrixIsCroppedScale(this->matrix)) {
        if(this->matrix[0][0] >= 1.f && this->matrix[1][1] >= 1.f)
            optOp = OptOp::SCALE_UP;
        else if(this->matrix[0][0] <= 1.f && this->matrix[1][1] <= 1.f)
            optOp = OptOp::SCALE_DOWN;
        else
            optOp = OptOp::SCALE;
    } else if(matrixIsAfine(this->matrix))
        optOp = OptOp::AFFINE;
    else
        optOp = OptOp::PERSPECTIVE;

    if(optOp == OptOp::AFFINE || optOp == OptOp::PERSPECTIVE) {
        const std::size_t padding = (type == ImgFrame::Type::NV12 || type == ImgFrame::Type::YUV420p) ? 2 : 1;
        this->sourceMaxX = std::min(this->sourceMaxX + padding, static_cast<std::size_t>(inWidth));
        this->sourceMaxY = std::min(this->sourceMaxY + padding, static_cast<std::size_t>(inHeight));
    }

    if(gpuEnabled && shouldUseGpuForFrame(this->dstSpecs.width, this->dstSpecs.height)) {
        gpuEnabled = std::get<WarpGPUMat>(warpGpu).init(getNumPlanes(this->type));
        if(!gpuEnabled) {
            logger->warn("GPU backend is selected but the GPU is not available. Using the CPU backend instead.");
        }
    }
    if(optOp == OptOp::SCALE_DOWN && canUseAccelerator() && (this->sourceMinX > 0u || this->sourceMinY > 0u)) {
        uint32_t srcContentWidth = this->sourceMaxX - this->sourceMinX;
        uint32_t srcContentHeight = this->sourceMaxY - this->sourceMinY;
        auto imgSpecs = impl::getDstFrameSpecs(srcContentWidth, srcContentHeight, this->type);
        dsImgAuxSpecs = imgSpecs;
        dsImgAux = AcceleratorMemory::create(impl::getAlignedOutputFrameSize(this->type, imgSpecs.width, imgSpecs.height));
    }
}

void WarpQ::buildUndistort(bool enable,
                           const std::array<float, 9>& cameraMatrix,
                           const std::array<float, 9>& newCameraMatrix,
                           const std::vector<float>& distCoeffs,
                           const std::optional<float> alpha,
                           const ImgFrame::Type type,
                           const uint32_t srcWidth,
                           const uint32_t srcHeight,
                           const uint32_t dstWidth,
                           const uint32_t dstHeight) {
    if(enable) {
        this->alphaScaling = alpha;

        auto buildWithOpenCv = [&]() {
            if(dynamic_cast<UndistortOpenCvImpl*>(undistortImpl.get()) == nullptr) {
                undistortImpl = std::make_unique<UndistortOpenCvImpl>(this->logger);
            }
            return undistortImpl->build(cameraMatrix, newCameraMatrix, distCoeffs, type, srcWidth, srcHeight, dstWidth, dstHeight);
        };

        Undistort::BuildStatus undistortStatus = Undistort::BuildStatus::NOT_USED;
        if(gpuEnabled && shouldUseGpuForFrame(dstWidth, dstHeight)) {
            if(dynamic_cast<UndistortWarpMeshImpl*>(undistortImpl.get()) == nullptr) {
                undistortImpl = std::make_unique<UndistortWarpMeshImpl>(this->logger);
            }
            undistortStatus = undistortImpl->build(cameraMatrix, newCameraMatrix, distCoeffs, type, srcWidth, srcHeight, dstWidth, dstHeight);
            if(undistortStatus == Undistort::BuildStatus::ERROR) {
                this->logger->warn("ImageManip | Undistort WarpGPUMesh unavailable, falling back to OpenCV");
                undistortStatus = buildWithOpenCv();
            }
        } else {
            undistortStatus = buildWithOpenCv();
        }
        if(undistortStatus == Undistort::BuildStatus::ONE_SHOT || undistortStatus == Undistort::BuildStatus::TWO_SHOT) {
            const bool onGpu = dynamic_cast<UndistortWarpMeshImpl*>(undistortImpl.get()) != nullptr;
            this->logger->debug("ImageManip | Undistortion backend: {}", onGpu ? "GPU" : "CPU");
        }

        switch(undistortStatus) {
            case Undistort::BuildStatus::ONE_SHOT:
                this->enableUndistort = true;
                this->undistortOneShot = true;
                break;
            case Undistort::BuildStatus::TWO_SHOT:
                this->enableUndistort = true;
                this->undistortOneShot = false;
                this->alphaScaling = std::nullopt;  // Alpha scaling is not supported for two-shot undistortion
                break;
            case Undistort::BuildStatus::NOT_BUILT:
                break;
            case Undistort::BuildStatus::NOT_USED:
            case Undistort::BuildStatus::ERROR:
                this->enableUndistort = false;
                this->undistortOneShot = false;
                break;
        }

        if(this->enableUndistort && !this->undistortOneShot) {
            auto frameSize = getAlignedOutputFrameSize(type, srcWidth, srcHeight);
            if(!auxFrame || auxFrame->getOffsetSize() < frameSize) {
                auxFrame = std::make_shared<DmaHeapMemory>(frameSize);
            }
        }
    } else {
        undistortImpl = nullptr;
        this->enableUndistort = false;
    }
}

void simpleSetBackground(uint8_t* dst,
                         const size_t dstWidth,
                         const size_t dstHeight,
                         const size_t dstStride,
                         const size_t bpp,
                         const size_t numChannels,
                         const size_t offsetLeft,
                         const size_t offsetRight,
                         const size_t offsetTop,
                         const size_t offsetBottom,
                         const std::vector<uint32_t>& _background,
                         spdlog::logger& logger) {
    const size_t elWidth = numChannels * bpp;
    if(offsetLeft > dstWidth || offsetRight > dstWidth - offsetLeft || offsetTop > dstHeight || offsetBottom > dstHeight - offsetTop
       || elWidth == 0 || dstWidth > dstStride / elWidth) {
        logger.error("Invalid background geometry: output {}x{} (stride {}), offsets l{} r{} t{} b{}",
                     dstWidth,
                     dstHeight,
                     dstStride,
                     offsetLeft,
                     offsetRight,
                     offsetTop,
                     offsetBottom);
        throw std::runtime_error("Background geometry lies outside the destination image");
    }
    const size_t dstSize = dstHeight * dstStride;
    const size_t contentHeight = dstHeight - offsetTop - offsetBottom;
    const size_t contentWidth = dstWidth - offsetLeft - offsetRight;
    uint8_t* dstStart = dst + offsetTop * dstStride + offsetLeft;
    std::vector<uint32_t> background = _background;
    if(background.size() == 0) {
        background.resize(numChannels);
        for(auto i = 0U; i < numChannels; ++i) background[i] = 0;
    }
    const bool uniformBackground = numChannels == 1 || numChannels == 2 && background[0] == background[1]
                                   || numChannels == 3 && background[0] == background[1] && background[1] == background[2];
    if(contentWidth == 0 || contentHeight == 0) {  // The border fills below need at least one content row and column
        if(uniformBackground) {
            memset(dst, background[0], dstSize);
        } else {
            for(auto i = 0U; i < dstHeight; ++i) {
                uint8_t* rowStart = dst + i * dstStride;
                for(auto j = 0U; j < dstWidth; ++j) {
                    for(auto k = 0U; k < elWidth; ++k) {
                        *(rowStart + j * elWidth + k) = background[k];
                    }
                }
            }
        }
        return;
    }
    if(uniformBackground) {
        if(offsetTop + offsetLeft > 0) memset(dst, background[0], offsetTop * dstStride + offsetLeft * elWidth);
        if(offsetRight + offsetBottom > 0)
            memset(dst + dstSize - (offsetBottom + 1) * dstStride + (dstWidth - offsetRight) * elWidth,
                   background[0],
                   (offsetBottom + 1) * dstStride - (dstWidth - offsetRight) * elWidth);
        if(offsetLeft > 0 || offsetRight > 0) {
            for(auto i = 0U; i < contentHeight; ++i) {
                uint8_t* rowStart = dst + (offsetTop + i) * dstStride;
                if(i == 0U && offsetRight > 0) {
                    memset(rowStart + (dstWidth - offsetRight) * elWidth, background[0], offsetRight * elWidth);
                } else if(i == contentHeight - 1 && offsetLeft > 0) {
                    memset(rowStart, background[0], offsetLeft * elWidth);
                } else {
                    if(offsetLeft > 0) memset(rowStart, background[0], offsetLeft * elWidth);
                    if(offsetRight > 0) memset(rowStart + (dstWidth - offsetRight) * elWidth, background[0], offsetRight * elWidth);
                }
            }
        }
    } else {
        if(offsetTop > 0) {
            for(auto i = 0U; i < offsetTop; ++i) {
                uint8_t* rowStart = dst + i * dstStride;
                for(auto j = 0U; j < dstWidth; ++j) {
                    for(auto k = 0U; k < elWidth; ++k) {
                        *(rowStart + j * elWidth + k) = background[k];
                    }
                }
            }
        }
        if(offsetBottom > 0) {
            for(auto i = dstHeight - offsetBottom; i < dstHeight; ++i) {
                uint8_t* rowStart = dst + i * dstStride;
                for(auto j = 0U; j < dstWidth; ++j) {
                    for(auto k = 0U; k < elWidth; ++k) {
                        *(rowStart + j * elWidth + k) = background[k];
                    }
                }
            }
        }
        if(offsetLeft > 0 || offsetRight > 0) {
            for(auto i = 0U; i < contentHeight; ++i) {
                uint8_t* rowStart = dst + (offsetTop + i) * dstStride;
                if(offsetLeft > 0)
                    for(auto j = 0U; j < offsetLeft; ++j) {
                        for(auto k = 0U; k < elWidth; ++k) {
                            *(rowStart + j * elWidth + k) = background[k];
                        }
                    }
                if(offsetRight > 0)
                    for(auto j = dstWidth - offsetRight; j < dstWidth; ++j) {
                        for(auto k = 0U; k < elWidth; ++k) {
                            *(rowStart + j * elWidth + k) = background[k];
                        }
                    }
            }
        }
    }
}

bool WarpQ::fastCvDownscaleu8(const std::shared_ptr<OffsetMemory> src,
                              std::shared_ptr<OffsetMemory> dst,
                              size_t srcWidth,
                              size_t srcHeight,
                              size_t srcStride,
                              size_t dstWidth,
                              size_t dstHeight,
                              size_t dstStride,
                              uint16_t numChannels) {
    if(!fastCvEnabled) return false;
    if((float)srcWidth / (float)dstWidth > 20.f || (float)srcHeight / (float)dstHeight > 20.f) {  // Limitation of downscaler
        return false;
    }
    if(numChannels == 1)
        fcvRun([&]() { fcvScaleDownMNu8(src->getData().data(), srcWidth, srcHeight, srcStride, dst->getData().data(), dstWidth, dstHeight, dstStride); });
    else if(numChannels == 2)
        fcvRun([&]() {
            fcvScaleDownMNInterleaveu8(src->getData().data(), srcWidth, srcHeight, srcStride, dst->getData().data(), dstWidth, dstHeight, dstStride);
        });
    else
        return false;
    return true;
}

void copyData(const uint8_t* __restrict__ src,
              uint8_t* __restrict__ dst,
              const size_t width,
              const size_t height,
              const size_t srcStride,
              const size_t dstStride,
              const size_t bpp,
              const size_t numChannels) {
    for(auto i = 0U; i < height; ++i) {
        const uint8_t* srcStart = src + i * srcStride;
        std::copy(srcStart, srcStart + width * bpp * numChannels, dst + i * dstStride);
    }
}
void crop(const uint8_t* __restrict__ src,
          uint8_t* __restrict__ dst,
          const size_t srcStride,
          const size_t dstStride,
          const size_t offsetX,
          const size_t offsetY,
          const size_t width,
          const size_t height,
          const size_t bpp,
          const size_t numChannels) {
    size_t srcOffset = offsetY * srcStride + offsetX * bpp * numChannels;
    copyData(src + srcOffset, dst, width, height, srcStride, dstStride, bpp, numChannels);
}

bool WarpQ::hwDownscale(const std::shared_ptr<OffsetMemory>& src, std::shared_ptr<OffsetMemory> dst, const ImgFrame::Type type) {
    float offsetX = this->matrix[0][0] * (float)this->sourceMinX + this->matrix[0][1] * (float)this->sourceMinY + this->matrix[0][2];
    float offsetY = this->matrix[1][0] * (float)this->sourceMinX + this->matrix[1][1] * (float)this->sourceMinY + this->matrix[1][2];
    float scaleX = this->matrix[0][0];
    float scaleY = this->matrix[1][1];
    size_t appliedSrcWidth = this->sourceMaxX - this->sourceMinX;
    size_t appliedSrcHeight = this->sourceMaxY - this->sourceMinY;
    float scaleWidth = scaleX * (float)appliedSrcWidth;
    float scaleHeight = scaleY * (float)appliedSrcHeight;
    size_t dstContentWidth = fabs(scaleWidth - (float)this->dstSpecs.width) <= std::max(scaleWidth * SIZE_EQUALITY_THRESHOLD_MULTIPLIER, 0.5f)
                                 ? this->dstSpecs.width
                                 : roundf(scaleWidth);
    size_t dstContentHeight = fabs(scaleHeight - (float)this->dstSpecs.height) <= std::max(scaleHeight * SIZE_EQUALITY_THRESHOLD_MULTIPLIER, 0.5f)
                                  ? this->dstSpecs.height
                                  : roundf(scaleHeight);

    if(!pxl_eq(offsetX, 0.f) || !pxl_eq(offsetY, 0.f)) return false;  // Apparently the accelerator does not do offsets very well

    if((double)dstContentHeight / (double)appliedSrcHeight < 1. / 8 || (double)dstContentWidth / (double)appliedSrcWidth < 1. / 8)
        return false;  // The accelerator can't handle a ratio lower than 1/8

    switch(type) {
        case ImgFrame::Type::NV12: {
            if(this->srcSpecs.p1Stride % 32 != 0 || this->srcSpecs.p2Stride % 16 != 0) return false;
            auto inputFrame = std::make_shared<ImgFrame>();
            auto outputFrame = std::make_shared<ImgFrame>();
            {
                size_t srcOffsetP1 = this->sourceMinY * this->srcSpecs.p1Stride + this->sourceMinX;
                size_t srcOffsetP2 = (this->sourceMinY / 2) * this->srcSpecs.p2Stride + this->sourceMinX;
                inputFrame->data = src->offset(srcOffsetP1);
                inputFrame->fb.p2Offset = this->srcSpecs.p2Offset - srcOffsetP1 + srcOffsetP2;
                inputFrame->setSize(appliedSrcWidth, appliedSrcHeight);
                inputFrame->setType(this->type);
                inputFrame->setStride(this->srcSpecs.p1Stride);
            }
            {
                size_t dstOffsetP1 = (size_t)offsetY * this->dstSpecs.p1Stride + (size_t)offsetX;
                size_t dstOffsetP2 = (size_t)(offsetY / 2.f) * this->dstSpecs.p2Stride + (size_t)offsetX;
                outputFrame->data = dst->offset(dstOffsetP1);
                outputFrame->fb.p2Offset = this->dstSpecs.p2Offset - dstOffsetP1 + dstOffsetP2;
                outputFrame->setSize(dstContentWidth, dstContentHeight);
                outputFrame->setType(this->type);
                outputFrame->setStride(this->dstSpecs.p1Stride);
            }
            if(this->sourceMinX > 0u || this->sourceMinY > 0u) {
                inputFrame->data = dsImgAux;
                inputFrame->fb.p2Offset = dsImgAuxSpecs.p2Offset;
                inputFrame->setStride(dsImgAuxSpecs.p1Stride);
                crop(src->offset(this->srcSpecs.p1Offset)->getOffsetData().data(),
                     dsImgAux->offset(dsImgAuxSpecs.p1Offset)->getOffsetData().data(),
                     this->srcSpecs.p1Stride,
                     dsImgAuxSpecs.p1Stride,
                     this->sourceMinX,
                     this->sourceMinY,
                     appliedSrcWidth,
                     appliedSrcHeight,
                     1,
                     1);
                crop(src->offset(this->srcSpecs.p2Offset)->getOffsetData().data(),
                     dsImgAux->offset(dsImgAuxSpecs.p2Offset)->getOffsetData().data(),
                     this->srcSpecs.p2Stride,
                     dsImgAuxSpecs.p2Stride,
                     this->sourceMinX / 2,
                     this->sourceMinY / 2,
                     appliedSrcWidth / 2,
                     appliedSrcHeight / 2,
                     1,
                     2);
            }
            dsImg.Run(inputFrame, outputFrame, true);
        } break;
        case ImgFrame::Type::YUV420p: {
            if(this->srcSpecs.p1Stride % 32 != 0 || this->srcSpecs.p2Stride % 16 != 0 || this->srcSpecs.p3Stride % 16 != 0) return false;
            auto inputFrame = std::make_shared<ImgFrame>();
            auto outputFrame = std::make_shared<ImgFrame>();
            {
                size_t srcOffsetP1 = this->sourceMinY * this->srcSpecs.p1Stride + this->sourceMinX;
                size_t srcOffsetP2 = (this->sourceMinY / 2) * this->srcSpecs.p2Stride + this->sourceMinX / 2;
                size_t srcOffsetP3 = (this->sourceMinY / 2) * this->srcSpecs.p3Stride + this->sourceMinX / 2;
                inputFrame->data = src->offset(srcOffsetP1);
                inputFrame->fb.p2Offset = this->srcSpecs.p2Offset - srcOffsetP1 + srcOffsetP2;
                inputFrame->fb.p3Offset = this->srcSpecs.p3Offset - srcOffsetP1 + srcOffsetP3;
                inputFrame->setSize(appliedSrcWidth, appliedSrcHeight);
                inputFrame->setType(this->type);
                inputFrame->setStride(this->srcSpecs.p1Stride);
            }
            {
                size_t dstOffsetP1 = (size_t)offsetY * this->dstSpecs.p1Stride + (size_t)offsetX;
                size_t dstOffsetP2 = (size_t)(offsetY / 2.f) * this->dstSpecs.p2Stride + (size_t)offsetX / 2;
                size_t dstOffsetP3 = (size_t)(offsetY / 2.f) * this->dstSpecs.p3Stride + (size_t)offsetX / 2;
                outputFrame->data = dst->offset(dstOffsetP1);
                outputFrame->fb.p2Offset = this->dstSpecs.p2Offset - dstOffsetP1 + dstOffsetP2;
                outputFrame->fb.p3Offset = this->dstSpecs.p3Offset - dstOffsetP1 + dstOffsetP3;
                outputFrame->setSize(dstContentWidth, dstContentHeight);
                outputFrame->setType(this->type);
                outputFrame->setStride(this->dstSpecs.p1Stride);
            }
            if(this->sourceMinX > 0u || this->sourceMinY > 0u) {
                inputFrame->data = dsImgAux;
                inputFrame->fb.p2Offset = dsImgAuxSpecs.p2Offset;
                inputFrame->setStride(dsImgAuxSpecs.p1Stride);
                crop(src->offset(this->srcSpecs.p1Offset)->getOffsetData().data(),
                     dsImgAux->offset(dsImgAuxSpecs.p1Offset)->getOffsetData().data(),
                     this->srcSpecs.p1Stride,
                     dsImgAuxSpecs.p1Stride,
                     this->sourceMinX,
                     this->sourceMinY,
                     appliedSrcWidth,
                     appliedSrcHeight,
                     1,
                     1);
                crop(src->offset(this->srcSpecs.p2Offset)->getOffsetData().data(),
                     dsImgAux->offset(dsImgAuxSpecs.p2Offset)->getOffsetData().data(),
                     this->srcSpecs.p2Stride,
                     dsImgAuxSpecs.p2Stride,
                     this->sourceMinX / 2,
                     this->sourceMinY / 2,
                     appliedSrcWidth / 2,
                     appliedSrcHeight / 2,
                     1,
                     1);
                crop(src->offset(this->srcSpecs.p3Offset)->getOffsetData().data(),
                     dsImgAux->offset(dsImgAuxSpecs.p3Offset)->getOffsetData().data(),
                     this->srcSpecs.p3Stride,
                     dsImgAuxSpecs.p3Stride,
                     this->sourceMinX / 2,
                     this->sourceMinY / 2,
                     appliedSrcWidth / 2,
                     appliedSrcHeight / 2,
                     1,
                     1);
            }
            dsImg.Run(inputFrame, outputFrame, true);
        } break;
        case ImgFrame::Type::RAW8:
        case ImgFrame::Type::GRAY8: {
            if(this->srcSpecs.p1Stride % 32 != 0) return false;
            auto inputFrame = std::make_shared<ImgFrame>();
            auto outputFrame = std::make_shared<ImgFrame>();
            {
                size_t srcOffsetP1 = this->sourceMinY * this->srcSpecs.p1Stride + this->sourceMinX;
                inputFrame->data = src->offset(srcOffsetP1);
                inputFrame->setSize(appliedSrcWidth, appliedSrcHeight);
                inputFrame->setType(this->type);
                inputFrame->setStride(this->srcSpecs.p1Stride);
            }
            {
                size_t dstOffsetP1 = (size_t)offsetY * this->dstSpecs.p1Stride + (size_t)offsetX;
                outputFrame->data = dst->offset(dstOffsetP1);
                outputFrame->setSize(dstContentWidth, dstContentHeight);
                outputFrame->setType(this->type);
                outputFrame->setStride(this->dstSpecs.p1Stride);
            }
            if(this->sourceMinX > 0u || this->sourceMinY > 0u) {
                inputFrame->data = dsImgAux;
                inputFrame->fb.p2Offset = dsImgAuxSpecs.p2Offset;
                inputFrame->setStride(dsImgAuxSpecs.p1Stride);
                crop(src->offset(this->srcSpecs.p1Offset)->getOffsetData().data(),
                     dsImgAux->offset(dsImgAuxSpecs.p1Offset)->getOffsetData().data(),
                     this->srcSpecs.p1Stride,
                     dsImgAuxSpecs.p1Stride,
                     this->sourceMinX,
                     this->sourceMinY,
                     appliedSrcWidth,
                     appliedSrcHeight,
                     1,
                     1);
            }
            dsImg.Run(inputFrame, outputFrame, true);
        } break;
        default:
            break;
    }
    return true;
}

bool WarpQ::canUseAccelerator() {
    bool supportedType = this->type == ImgFrame::Type::YUV420p || this->type == ImgFrame::Type::NV12 || this->type == ImgFrame::Type::RAW8
                         || this->type == ImgFrame::Type::GRAY8;  // The accelerator type cannot change after initialization
    return hwScalerEnabled && supportedType && (!dsImgInitialized && !firstFrameOver || dsImgInitialized && dsImgType == this->type);
}

void WarpQ::apply(const std::shared_ptr<OffsetMemory>& src, std::shared_ptr<OffsetMemory> dst) {
    auto undistortDst = this->isIdentityWarp() || this->undistortOneShot ? dst : auxFrame;
    auto undistortSpecs =
        this->isIdentityWarp() || this->undistortOneShot ? this->dstSpecs : getDstFrameSpecs(this->srcSpecs.width, this->srcSpecs.height, this->type);
    auto warpSrc = this->enableUndistort ? auxFrame : src;
    auto warpSrcSpecs = this->enableUndistort ? undistortSpecs : this->srcSpecs;
    bool gpuWarpPending = false;
    int gpuPlane = 0;
    auto transformPlane = [&](const std::shared_ptr<OffsetMemory>& srcPlane,
                              std::shared_ptr<OffsetMemory> dstPlane,
                              const size_t planeSrcWidth,
                              const size_t planeSrcHeight,
                              const size_t planeSrcStride,
                              const size_t planeDstWidth,
                              const size_t planeDstHeight,
                              const size_t planeDstStride,
                              const uint16_t planeNumChannels,
                              const uint16_t planeBpp,
                              const std::array<std::array<float, 3>, 3>& planeMatrix,
                              const std::vector<uint32_t>& planeBackground) {
        if(shouldUseGpuWarp()
           && enqueueGpuTransform(gpuPlane,
                                  srcPlane,
                                  dstPlane,
                                  planeSrcWidth,
                                  planeSrcHeight,
                                  planeSrcStride,
                                  planeDstWidth,
                                  planeDstHeight,
                                  planeDstStride,
                                  planeNumChannels,
                                  planeBpp,
                                  planeMatrix,
                                  planeBackground)) {
            gpuWarpPending = true;
        } else {
            auto srcOffset = srcPlane->getOffset();  // ConvertedOffsetMemory drops the offset: set it again on the accelerator buffer
            auto accelSrcPlane = AcceleratorMemory::convert(ConvertedOffsetMemory::cast<Memory>(srcPlane));
            if(accelSrcPlane == nullptr) {
                throw std::runtime_error("Warp: Invalid source buffer");
            }
            if(accelSrcPlane->getOffset() == 0) accelSrcPlane->setOffset(srcOffset);
            auto dstOffset = dstPlane->getOffset();
            auto accelDstPlane = AcceleratorMemory::convertZeroCopy(ConvertedOffsetMemory::cast<Memory>(dstPlane));
            if(accelDstPlane == nullptr) {
                throw std::runtime_error("Warp: Invalid destination buffer");
            }
            if(accelDstPlane->getOffset() == 0) accelDstPlane->setOffset(dstOffset);
            transform(accelSrcPlane,
                      accelDstPlane,
                      planeSrcWidth,
                      planeSrcHeight,
                      planeSrcStride,
                      planeDstWidth,
                      planeDstHeight,
                      planeDstStride,
                      planeNumChannels,
                      planeBpp,
                      planeMatrix,
                      planeBackground);
        }
        ++gpuPlane;
    };
    try {
        switch(this->type) {
            case ImgFrame::Type::RGB888i:
            case ImgFrame::Type::BGR888i:
                if(this->enableUndistort) {
                    this->undistortImpl->undistort(
                        {this->srcSpecs.width, this->srcSpecs.height, this->srcSpecs.p1Stride, 1, 3, src->offset(this->srcSpecs.p1Offset)},
                        {undistortSpecs.width, undistortSpecs.height, undistortSpecs.p1Stride, 1, 3, undistortDst->offset(undistortSpecs.p1Offset)},
                        0);
                    this->undistortImpl->finish();
                }
                if(!this->undistortOneShot && !this->isIdentityWarp()) {
                    transformPlane(warpSrc->offset(warpSrcSpecs.p1Offset),
                                   dst->offset(this->dstSpecs.p1Offset),
                                   warpSrcSpecs.width,
                                   warpSrcSpecs.height,
                                   warpSrcSpecs.p1Stride,
                                   this->dstSpecs.width,
                                   this->dstSpecs.height,
                                   this->dstSpecs.p1Stride,
                                   3,
                                   1,
                                   this->matrix,
                                   {this->backgroundColor[0], this->backgroundColor[1], this->backgroundColor[2]});
                }
                break;
            case ImgFrame::Type::BGR888p:
            case ImgFrame::Type::RGB888p:
                if(this->enableUndistort) {
                    {
                        this->undistortImpl->undistort(
                            {this->srcSpecs.width, this->srcSpecs.height, this->srcSpecs.p1Stride, 1, 1, src->offset(this->srcSpecs.p1Offset)},
                            {undistortSpecs.width, undistortSpecs.height, undistortSpecs.p1Stride, 1, 1, undistortDst->offset(undistortSpecs.p1Offset)},
                            0);
                    }
                    {
                        this->undistortImpl->undistort(
                            {this->srcSpecs.width, this->srcSpecs.height, this->srcSpecs.p2Stride, 1, 1, src->offset(this->srcSpecs.p2Offset)},
                            {undistortSpecs.width, undistortSpecs.height, undistortSpecs.p2Stride, 1, 1, undistortDst->offset(undistortSpecs.p2Offset)},
                            1);
                    }
                    {
                        this->undistortImpl->undistort(
                            {this->srcSpecs.width, this->srcSpecs.height, this->srcSpecs.p3Stride, 1, 1, src->offset(this->srcSpecs.p3Offset)},
                            {undistortSpecs.width, undistortSpecs.height, undistortSpecs.p3Stride, 1, 1, undistortDst->offset(undistortSpecs.p3Offset)},
                            2);
                    }
                    this->undistortImpl->finish();
                }
                if(!this->undistortOneShot && !this->isIdentityWarp()) {
                    transformPlane(warpSrc->offset(warpSrcSpecs.p1Offset),
                                   dst->offset(this->dstSpecs.p1Offset),
                                   warpSrcSpecs.width,
                                   warpSrcSpecs.height,
                                   warpSrcSpecs.p1Stride,
                                   this->dstSpecs.width,
                                   this->dstSpecs.height,
                                   this->dstSpecs.p1Stride,
                                   1,
                                   1,
                                   this->matrix,
                                   {this->backgroundColor[0]});
                    transformPlane(warpSrc->offset(warpSrcSpecs.p2Offset),
                                   dst->offset(this->dstSpecs.p2Offset),
                                   warpSrcSpecs.width,
                                   warpSrcSpecs.height,
                                   warpSrcSpecs.p2Stride,
                                   this->dstSpecs.width,
                                   this->dstSpecs.height,
                                   this->dstSpecs.p2Stride,
                                   1,
                                   1,
                                   this->matrix,
                                   {this->backgroundColor[1]});
                    transformPlane(warpSrc->offset(warpSrcSpecs.p3Offset),
                                   dst->offset(this->dstSpecs.p3Offset),
                                   warpSrcSpecs.width,
                                   warpSrcSpecs.height,
                                   warpSrcSpecs.p3Stride,
                                   this->dstSpecs.width,
                                   this->dstSpecs.height,
                                   this->dstSpecs.p3Stride,
                                   1,
                                   1,
                                   this->matrix,
                                   {this->backgroundColor[2]});
                }
                break;
            case ImgFrame::Type::YUV420p: {
                if(this->enableUndistort) {
                    {
                        this->undistortImpl->undistort(
                            {this->srcSpecs.width, this->srcSpecs.height, this->srcSpecs.p1Stride, 1, 1, src->offset(this->srcSpecs.p1Offset)},
                            {undistortSpecs.width, undistortSpecs.height, undistortSpecs.p1Stride, 1, 1, undistortDst->offset(undistortSpecs.p1Offset)},
                            0);
                    }
                    {
                        this->undistortImpl->undistort(
                            {this->srcSpecs.width / 2, this->srcSpecs.height / 2, this->srcSpecs.p2Stride, 1, 1, src->offset(this->srcSpecs.p2Offset)},
                            {undistortSpecs.width / 2, undistortSpecs.height / 2, undistortSpecs.p2Stride, 1, 1, undistortDst->offset(undistortSpecs.p2Offset)},
                            1);
                    }
                    {
                        this->undistortImpl->undistort(
                            {this->srcSpecs.width / 2, this->srcSpecs.height / 2, this->srcSpecs.p3Stride, 1, 1, src->offset(this->srcSpecs.p3Offset)},
                            {undistortSpecs.width / 2, undistortSpecs.height / 2, undistortSpecs.p3Stride, 1, 1, undistortDst->offset(undistortSpecs.p3Offset)},
                            2);
                    }
                    this->undistortImpl->finish();
                }
                if(!this->undistortOneShot && !this->isIdentityWarp()) {
                    bool doPlanewise = true;
                    if(optOp == OptOp::SCALE_DOWN && canUseAccelerator() && !this->enableUndistort) {
                        if(!dsImgInitialized) {
                            dsImgInitialized = true;
                            dsImgType = this->type;
                            DsImgInit dsInit = {.colorFormat = this->type};
                            int DsImgErr = dsImg.Init(dsInit);
                            if(DsImgErr) {
                                throw std::runtime_error("Failed initializing downscaler image node: " + std::to_string((int)DsImgErr));
                            }

                            DsImgConfig DsImgCfg = {.eDsInterpolation = ScaledownInterpolation::BILINEAR};
                            dsImg.Config(DsImgCfg);
                        }
                        doPlanewise = !hwDownscale(src, dst, this->type);
                    }
                    if(doPlanewise) {
                        transformPlane(warpSrc->offset(warpSrcSpecs.p1Offset),
                                       dst->offset(this->dstSpecs.p1Offset),
                                       warpSrcSpecs.width,
                                       warpSrcSpecs.height,
                                       warpSrcSpecs.p1Stride,
                                       this->dstSpecs.width,
                                       this->dstSpecs.height,
                                       this->dstSpecs.p1Stride,
                                       1,
                                       1,
                                       this->matrix,
                                       {this->backgroundColor[0]});
                        transformPlane(warpSrc->offset(warpSrcSpecs.p2Offset),
                                       dst->offset(this->dstSpecs.p2Offset),
                                       warpSrcSpecs.width / 2,
                                       warpSrcSpecs.height / 2,
                                       warpSrcSpecs.p2Stride,
                                       this->dstSpecs.width / 2,
                                       this->dstSpecs.height / 2,
                                       this->dstSpecs.p2Stride,
                                       1,
                                       1,
                                       this->matrix,
                                       {this->backgroundColor[1]});
                        transformPlane(warpSrc->offset(warpSrcSpecs.p3Offset),
                                       dst->offset(this->dstSpecs.p3Offset),
                                       warpSrcSpecs.width / 2,
                                       warpSrcSpecs.height / 2,
                                       warpSrcSpecs.p3Stride,
                                       this->dstSpecs.width / 2,
                                       this->dstSpecs.height / 2,
                                       this->dstSpecs.p3Stride,
                                       1,
                                       1,
                                       this->matrix,
                                       {this->backgroundColor[2]});
                    }
                }
                break;
            }
            case ImgFrame::Type::NV12: {
                if(this->enableUndistort) {
                    {
                        this->undistortImpl->undistort(
                            {this->srcSpecs.width, this->srcSpecs.height, this->srcSpecs.p1Stride, 1, 1, src->offset(this->srcSpecs.p1Offset)},
                            {undistortSpecs.width, undistortSpecs.height, undistortSpecs.p1Stride, 1, 1, undistortDst->offset(undistortSpecs.p1Offset)},
                            0);
                    }
                    {
                        this->undistortImpl->undistort(
                            {this->srcSpecs.width / 2, this->srcSpecs.height / 2, this->srcSpecs.p2Stride, 1, 2, src->offset(this->srcSpecs.p2Offset)},
                            {undistortSpecs.width / 2, undistortSpecs.height / 2, undistortSpecs.p2Stride, 1, 2, undistortDst->offset(undistortSpecs.p2Offset)},
                            1);
                    }
                    this->undistortImpl->finish();
                }
                if(!this->undistortOneShot && !this->isIdentityWarp()) {
                    bool doPlanewise = true;
                    if(optOp == OptOp::SCALE_DOWN && canUseAccelerator() && !this->enableUndistort) {
                        if(!dsImgInitialized) {
                            dsImgInitialized = true;
                            dsImgType = this->type;
                            DsImgInit dsInit = {.colorFormat = this->type};
                            int DsImgErr = dsImg.Init(dsInit);
                            if(DsImgErr) {
                                throw std::runtime_error("Failed initializing downscaler image node: " + std::to_string((int)DsImgErr));
                            }

                            DsImgConfig DsImgCfg = {.eDsInterpolation = ScaledownInterpolation::BILINEAR};
                            dsImg.Config(DsImgCfg);
                        }
                        doPlanewise = !hwDownscale(src, dst, this->type);
                    }
                    if(doPlanewise) {
                        transformPlane(warpSrc->offset(warpSrcSpecs.p1Offset),
                                       dst->offset(this->dstSpecs.p1Offset),
                                       warpSrcSpecs.width,
                                       warpSrcSpecs.height,
                                       warpSrcSpecs.p1Stride,
                                       this->dstSpecs.width,
                                       this->dstSpecs.height,
                                       this->dstSpecs.p1Stride,
                                       1,
                                       1,
                                       this->matrix,
                                       {this->backgroundColor[0]});
                        transformPlane(warpSrc->offset(warpSrcSpecs.p2Offset),
                                       dst->offset(this->dstSpecs.p2Offset),
                                       warpSrcSpecs.width / 2,
                                       warpSrcSpecs.height / 2,
                                       warpSrcSpecs.p2Stride,
                                       this->dstSpecs.width / 2,
                                       this->dstSpecs.height / 2,
                                       this->dstSpecs.p2Stride,
                                       2,
                                       1,
                                       this->matrix,
                                       {this->backgroundColor[1], this->backgroundColor[2]});
                    }
                }
                break;
            }
            case ImgFrame::Type::RAW8:
            case ImgFrame::Type::GRAY8: {
                if(this->enableUndistort) {
                    {
                        this->undistortImpl->undistort(
                            {this->srcSpecs.width, this->srcSpecs.height, this->srcSpecs.p1Stride, 1, 1, src->offset(this->srcSpecs.p1Offset)},
                            {undistortSpecs.width, undistortSpecs.height, undistortSpecs.p1Stride, 1, 1, undistortDst->offset(undistortSpecs.p1Offset)},
                            0);
                    }
                    this->undistortImpl->finish();
                }
                if(!this->undistortOneShot && !this->isIdentityWarp()) {
                    bool doPlanewise = true;
                    if(optOp == OptOp::SCALE_DOWN && canUseAccelerator() && !this->enableUndistort) {
                        if(!dsImgInitialized) {
                            dsImgInitialized = true;
                            dsImgType = this->type;
                            DsImgInit dsInit = {.colorFormat = this->type};
                            int DsImgErr = dsImg.Init(dsInit);
                            if(DsImgErr) {
                                throw std::runtime_error("Failed initializing downscaler image node: " + std::to_string((int)DsImgErr));
                            }

                            DsImgConfig DsImgCfg = {.eDsInterpolation = ScaledownInterpolation::BILINEAR};
                            dsImg.Config(DsImgCfg);
                        }
                        doPlanewise = !hwDownscale(src, dst, this->type);
                    }
                    if(doPlanewise) {
                        transformPlane(warpSrc->offset(warpSrcSpecs.p1Offset),
                                       dst->offset(this->dstSpecs.p1Offset),
                                       warpSrcSpecs.width,
                                       warpSrcSpecs.height,
                                       warpSrcSpecs.p1Stride,
                                       this->dstSpecs.width,
                                       this->dstSpecs.height,
                                       this->dstSpecs.p1Stride,
                                       1,
                                       1,
                                       this->matrix,
                                       {this->backgroundColor[0]});
                    }
                }
                break;
            }
            case ImgFrame::Type::RAW16:
                if(this->enableUndistort) {
                    {
                        this->undistortImpl->undistort(
                            {this->srcSpecs.width, this->srcSpecs.height, this->srcSpecs.p1Stride, 2, 1, src->offset(this->srcSpecs.p1Offset)},
                            {undistortSpecs.width, undistortSpecs.height, undistortSpecs.p1Stride, 2, 1, undistortDst->offset(undistortSpecs.p1Offset)},
                            0);
                    }
                    this->undistortImpl->finish();
                }
                if(!this->undistortOneShot && !this->isIdentityWarp()) {
                    transformPlane(warpSrc->offset(warpSrcSpecs.p1Offset),
                                   dst->offset(this->dstSpecs.p1Offset),
                                   warpSrcSpecs.width,
                                   warpSrcSpecs.height,
                                   warpSrcSpecs.p1Stride,
                                   this->dstSpecs.width,
                                   this->dstSpecs.height,
                                   this->dstSpecs.p1Stride,
                                   1,
                                   2,
                                   this->matrix,
                                   {this->backgroundColor[0]});
                }
                break;
            case ImgFrame::Type::YUV422i:
            case ImgFrame::Type::YUV444p:
            case ImgFrame::Type::YUV422p:
            case ImgFrame::Type::YUV400p:
            case ImgFrame::Type::RGBA8888:
            case ImgFrame::Type::RGB161616:
            case ImgFrame::Type::LUT2:
            case ImgFrame::Type::LUT4:
            case ImgFrame::Type::LUT16:
            case ImgFrame::Type::RAW14:
            case ImgFrame::Type::RAW12:
            case ImgFrame::Type::RAW10:
            case ImgFrame::Type::PACK10:
            case ImgFrame::Type::PACK12:
            case ImgFrame::Type::YUV444i:
            case ImgFrame::Type::NV21:
            case ImgFrame::Type::BITSTREAM:
            case ImgFrame::Type::HDR:
            case ImgFrame::Type::RGBF16F16F16p:
            case ImgFrame::Type::BGRF16F16F16p:
            case ImgFrame::Type::RGBF16F16F16i:
            case ImgFrame::Type::BGRF16F16F16i:
            case ImgFrame::Type::GRAYF16:
            case ImgFrame::Type::RAW32:
            case ImgFrame::Type::NONE:
                throw std::runtime_error("Unsupported image format. Only YUV420p, RGB888p, BGR888p, RGB888i, BGR888i, RAW8, NV12, GRAY8 are supported");
                break;
        }
        if(gpuWarpPending) {
            std::get<WarpGPUMat>(warpGpu).warp();
            gpuWarpPending = false;
        }
    } catch(...) {
        if(this->enableUndistort) {  // Drain queued mesh work while src, dst and undistortDst still own the imported allocations
            try {
                this->undistortImpl->finish();
            } catch(const std::exception& ex) {
                this->logger->error("ImageManip undistort cleanup after error failed: {}", ex.what());
            } catch(...) {
                this->logger->error("ImageManip undistort cleanup after error failed");
            }
        }
        if(gpuWarpPending) {
            try {
                std::get<WarpGPUMat>(warpGpu).warp();
            } catch(const std::exception& ex) {
                this->logger->error("ImageManip GPU cleanup after error failed: {}", ex.what());
            } catch(...) {
                this->logger->error("ImageManip GPU cleanup after error failed");
            }
            gpuWarpPending = false;
        }
        throw;
    }
    if(hwScalerEnabled && !dsImgInitialized && !firstFrameOver) {
        hwScalerEnabled = false;
        try {
            dsImg.firstFrameOver();
        } catch(std::runtime_error& err) {
            this->logger->error(err.what());  // A timeout is fine: another node starts the session
        }
    }
    firstFrameOver = true;
}

bool WarpQ::shouldUseGpuWarp() const {
    if(!shouldUseGpuForFrame(this->dstSpecs.width, this->dstSpecs.height)) {
        return false;
    }

    return gpuEnabled
           && (optOp == OptOp::AFFINE || optOp == OptOp::PERSPECTIVE || optOp == OptOp::SCALE || optOp == OptOp::SCALE_UP || optOp == OptOp::SCALE_DOWN);
}

bool WarpQ::shouldUseGpuForFrame(std::size_t dstFrameWidth, std::size_t dstFrameHeight) const {
    if(backend == Backend::GPU) {
        return true;
    }

    if(backend != Backend::AUTO) {
        return false;
    }

    return dstFrameWidth > AUTO_GPU_MIN_FRAME_WIDTH && dstFrameHeight > AUTO_GPU_MIN_FRAME_HEIGHT;
}

bool WarpQ::enqueueGpuTransform(int plane,
                                const std::shared_ptr<OffsetMemory> srcData,
                                std::shared_ptr<OffsetMemory> dstData,
                                const size_t srcWidth,
                                const size_t srcHeight,
                                const size_t srcStride,
                                const size_t dstWidth,
                                const size_t dstHeight,
                                const size_t dstStride,
                                const uint16_t numChannels,
                                const uint16_t bpp,
                                const std::array<std::array<float, 3>, 3> matrix,
                                const std::vector<uint32_t>& background) {
    assert(background.size() == 0 || background.size() == numChannels);
    int ssF = this->srcSpecs.width / srcWidth;
    assert(ssF == (int)(this->srcSpecs.height / srcHeight) && (ssF == 1 || ssF == 2));
    const std::size_t planeSourceMinX = this->sourceMinX / ssF;
    const std::size_t planeSourceMinY = this->sourceMinY / ssF;
    const std::size_t planeSourceMaxX = std::min(ceilDiv(this->sourceMaxX, static_cast<std::size_t>(ssF)), srcWidth);
    const std::size_t planeSourceMaxY = std::min(ceilDiv(this->sourceMaxY, static_cast<std::size_t>(ssF)), srcHeight);

    std::array<std::array<float, 3>, 3> appliedMatrix = matrix;
    appliedMatrix[0][2] /= ssF;
    appliedMatrix[1][2] /= ssF;
    if(this->sourceMinX != 0 || this->sourceMinY != 0) {
        appliedMatrix[0][2] += appliedMatrix[0][0] * static_cast<float>(planeSourceMinX) + appliedMatrix[0][1] * static_cast<float>(planeSourceMinY);
        appliedMatrix[1][2] += appliedMatrix[1][0] * static_cast<float>(planeSourceMinX) + appliedMatrix[1][1] * static_cast<float>(planeSourceMinY);
        appliedMatrix[2][2] += appliedMatrix[2][0] * static_cast<float>(planeSourceMinX) + appliedMatrix[2][1] * static_cast<float>(planeSourceMinY);
    }

    size_t srcOffset = planeSourceMinY * srcStride + planeSourceMinX * bpp * numChannels;
    size_t appliedSrcWidth = std::max(planeSourceMaxX - planeSourceMinX, 1ul);
    size_t appliedSrcHeight = std::max(planeSourceMaxY - planeSourceMinY, 1ul);

    MatConfig config;
    config.matrix = appliedMatrix;
    config.srcWidth = appliedSrcWidth;
    config.srcHeight = appliedSrcHeight;
    config.srcStrideBytes = srcStride;
    config.dstWidth = dstWidth;
    config.dstHeight = dstHeight;
    config.dstStrideBytes = dstStride;
    config.channels = numChannels;
    config.bytesPerChannel = bpp;
    config.borderValue.fill(0);
    for(size_t channel = 0; channel < background.size() && channel < config.borderValue.size(); ++channel) {
        config.borderValue[channel] = static_cast<std::uint16_t>(std::min(background[channel], static_cast<uint32_t>(UINT16_MAX)));
    }

    if(!std::get<WarpGPUMat>(warpGpu).buildPlane(plane, config)) {
        gpuEnabled = false;
        this->logger->warn("ImageManip | GPU warp unavailable, falling back to CPU/OpenCV");
        return false;
    }

    try {
        std::get<WarpGPUMat>(warpGpu).enqueueWarpPlane(plane, srcData->offset(srcOffset), dstData);
    } catch(const std::runtime_error& e) {
        try {  // Enqueue can fail after it submitted work: drain while srcData and dstData still own the buffers
            std::get<WarpGPUMat>(warpGpu).warp();
        } catch(const std::exception& ex) {
            this->logger->error("ImageManip GPU cleanup after enqueue error failed: {}", ex.what());
        } catch(...) {
            this->logger->error("ImageManip GPU cleanup after enqueue error failed");
        }
        this->logger->error("ImageManip | Error in GPU warp, falling back to CPU: {}", e.what());
        gpuEnabled = false;
        return false;
    }
    return true;
}

void WarpQ::transform(const std::shared_ptr<OffsetMemory> srcData,
                      std::shared_ptr<OffsetMemory> dstData,
                      const size_t srcWidth,
                      const size_t srcHeight,
                      const size_t srcStride,
                      const size_t dstWidth,
                      const size_t dstHeight,
                      const size_t dstStride,
                      const uint16_t numChannels,
                      const uint16_t bpp,
                      const std::array<std::array<float, 3>, 3> matrix,
                      const std::vector<uint32_t>& background) {
    assert(background.size() == 0 || background.size() == numChannels);
    const uint8_t* src = srcData->getData().data();
    uint8_t* dst = dstData->getData().data();
    int ssF = this->srcSpecs.width / srcWidth;
    assert(ssF == (int)(this->srcSpecs.height / srcHeight) && (ssF == 1 || ssF == 2));  // Sanity check
    const std::size_t planeSourceMinX = this->sourceMinX / ssF;
    const std::size_t planeSourceMinY = this->sourceMinY / ssF;
    const std::size_t planeSourceMaxX = std::min(ceilDiv(this->sourceMaxX, static_cast<std::size_t>(ssF)), srcWidth);
    const std::size_t planeSourceMaxY = std::min(ceilDiv(this->sourceMaxY, static_cast<std::size_t>(ssF)), srcHeight);
    float appliedMatrix[9] = {
        matrix[0][0], matrix[0][1], matrix[0][2] / ssF, matrix[1][0], matrix[1][1], matrix[1][2] / ssF, matrix[2][0], matrix[2][1], matrix[2][2]};
    if(this->sourceMinX != 0 || this->sourceMinY != 0) {
        appliedMatrix[2] += appliedMatrix[0] * static_cast<float>(planeSourceMinX) + appliedMatrix[1] * static_cast<float>(planeSourceMinY);
        appliedMatrix[5] += appliedMatrix[3] * static_cast<float>(planeSourceMinX) + appliedMatrix[4] * static_cast<float>(planeSourceMinY);
        appliedMatrix[8] += appliedMatrix[6] * static_cast<float>(planeSourceMinX) + appliedMatrix[7] * static_cast<float>(planeSourceMinY);
    }
    float offsetX = std::clamp(appliedMatrix[2] / appliedMatrix[8], 0.f, static_cast<float>(dstWidth));  // Clamp: an out-of-frame shift would underflow size_t
    float offsetY = std::clamp(appliedMatrix[5] / appliedMatrix[8], 0.f, static_cast<float>(dstHeight));
    float scaleX = appliedMatrix[0];
    float scaleY = appliedMatrix[4];

    size_t srcOffset = planeSourceMinY * srcStride + planeSourceMinX * bpp * numChannels;
    size_t appliedSrcWidth = std::max(planeSourceMaxX - planeSourceMinX, 1ul);
    size_t appliedSrcHeight = std::max(planeSourceMaxY - planeSourceMinY, 1ul);

    size_t dstOffset = (size_t)offsetY * dstStride + (size_t)offsetX * bpp * numChannels;
    size_t dstContentWidth =
        std::min(std::max(fabs(scaleX * (float)appliedSrcWidth - (float)dstWidth) <= 0.5f ? dstWidth : roundf(scaleX * (float)appliedSrcWidth), 1.f),
                 (float)dstWidth - offsetX);
    size_t dstContentHeight =
        std::min(std::max(fabs(scaleY * (float)appliedSrcHeight - (float)dstHeight) <= 0.5f ? dstHeight : roundf(scaleY * (float)appliedSrcHeight), 1.f),
                 (float)dstHeight - offsetY);

    auto type = CV_8UC1;
    switch(numChannels) {
        case 1:
            switch(bpp) {
                case 1:
                    type = CV_8UC1;
                    break;
                case 2:
                    type = CV_16UC1;
                    break;
                default:
                    assert(false);
            }
            break;
        case 2:
            assert(bpp == 1);
            type = CV_8UC2;
            break;
        case 3:
            assert(bpp == 1);
            type = CV_8UC3;
            break;
        default:
            assert(false);
    }
    auto bg = numChannels == 1 ? cv::Scalar(background[0])
                               : (numChannels == 2 ? cv::Scalar(background[0], background[1]) : cv::Scalar(background[0], background[1], background[2]));

    auto planeOp = optOp;
    if(planeOp == OptOp::CROP
       && (appliedMatrix[2] / appliedMatrix[8] != offsetX || appliedMatrix[5] / appliedMatrix[8] != offsetY || offsetX != std::floor(offsetX)
           || offsetY != std::floor(offsetY) || appliedSrcWidth > dstWidth - static_cast<size_t>(offsetX)
           || appliedSrcHeight > dstHeight - static_cast<size_t>(offsetY))) {
        planeOp = OptOp::AFFINE;
    }

    switch(planeOp) {
        case OptOp::CROP: {
            if(appliedSrcWidth == dstWidth && appliedSrcHeight == dstHeight) {
                copyData(src + srcOffset, dst + dstOffset, appliedSrcWidth, appliedSrcHeight, srcStride, dstStride, bpp, numChannels);
            } else {
                simpleSetBackground(dst,
                                    dstWidth,
                                    dstHeight,
                                    dstStride,
                                    bpp,
                                    numChannels,
                                    (size_t)std::max(offsetX, 0.f),
                                    (size_t)std::max((long)dstWidth - (long)dstContentWidth - (long)std::max(offsetX, 0.f), 0l),
                                    (size_t)std::max(offsetY, 0.f),
                                    (size_t)std::max((long)dstHeight - (long)dstContentHeight - (long)std::max(offsetY, 0.f), 0l),
                                    background,
                                    *this->logger);

                copyData(src + srcOffset, dst + dstOffset, appliedSrcWidth, appliedSrcHeight, srcStride, dstStride, bpp, numChannels);
            }
        } break;
        case OptOp::SCALE_DOWN:
        case OptOp::SCALE_UP:
        case OptOp::SCALE: {
            if(dstContentWidth != dstWidth || dstContentHeight != dstHeight) {
                simpleSetBackground(dst,
                                    dstWidth,
                                    dstHeight,
                                    dstStride,
                                    bpp,
                                    numChannels,
                                    (size_t)std::max(offsetX, 0.f),
                                    (size_t)std::max((long)dstWidth - (long)dstContentWidth - (long)std::max(offsetX, 0.f), 0l),
                                    (size_t)std::max(offsetY, 0.f),
                                    (size_t)std::max((long)dstHeight - (long)dstContentHeight - (long)std::max(offsetY, 0.f), 0l),
                                    background,
                                    *this->logger);
            }

            bool useOpenCV = false;
            if(optOp == OptOp::SCALE_DOWN && bpp == 1 && numChannels <= 2
               && fastCvDownscaleu8(srcData->offset(srcOffset),
                                    dstData->offset(dstOffset),
                                    appliedSrcWidth,
                                    appliedSrcHeight,
                                    srcStride,
                                    dstContentWidth,
                                    dstContentHeight,
                                    dstStride,
                                    numChannels)) {
            } else if(fastCvEnabled && bpp == 1 && numChannels == 1) {
                int res = fcvRun([&]() {
                    return fcvScaleu8(src + srcOffset,
                                      appliedSrcWidth,
                                      appliedSrcHeight,
                                      srcStride,
                                      dst + dstOffset,
                                      dstContentWidth,
                                      dstContentHeight,
                                      dstStride,
                                      FASTCV_INTERPOLATION_TYPE_AREA);
                });
                if(res != 0) useOpenCV = true;
            } else {
                useOpenCV = true;
            }
            if(useOpenCV) {
                if(appliedSrcHeight == 0 || appliedSrcWidth == 0 || srcStride == 0)
                    throw std::runtime_error(fmt::format("Source dimension is zero ({}x{}, {})", appliedSrcWidth, appliedSrcHeight, srcStride));
                const cv::Mat cvSrc(appliedSrcHeight, appliedSrcWidth, type, const_cast<uint8_t*>(src + srcOffset), srcStride);
                cv::Mat cvDst(dstHeight, dstWidth, type, dst, dstStride);
                if(dstContentHeight == 0 || dstContentWidth == 0 || dstStride == 0)
                    throw std::runtime_error(fmt::format("Destination dimension is zero ({}x{}, {})", dstContentWidth, dstContentHeight, dstStride));
                cv::Mat cvDst2(dstContentHeight, dstContentWidth, type, dst + dstOffset, dstStride);

                bool fallbackToWarp = false;
                try {
                    cv::resize(cvSrc, cvDst2, cv::Size(dstContentWidth, dstContentHeight));
                } catch(const cv::Exception& e) {
                    fallbackToWarp = true;
                }
                if(fallbackToWarp) {
                    cv::Mat cvAffine(2, 3, CV_32F, appliedMatrix);
                    try {
                        cv::warpAffine(cvSrc,
                                       cvDst,
                                       cvAffine,
                                       cv::Size(dstWidth, dstHeight),
                                       cv::INTER_AREA,
                                       cv::BORDER_CONSTANT,
                                       bg);  // TODO(asahtik): Add support for different border types
                    } catch(const cv::Exception& e) {
                        throw std::runtime_error(fmt::format("cv::warpAffine failed: {}", e.what()));
                    }
                }
            }
        } break;
        case OptOp::AFFINE: {
            if(appliedSrcHeight == 0 || appliedSrcWidth == 0 || srcStride == 0)
                throw std::runtime_error(fmt::format("Source dimension is zero ({}x{}, {})", appliedSrcWidth, appliedSrcHeight, srcStride));
            const cv::Mat cvSrc(appliedSrcHeight, appliedSrcWidth, type, const_cast<uint8_t*>(src + srcOffset), srcStride);
            cv::Mat cvDst(dstHeight, dstWidth, type, dst, dstStride);
            cv::Mat cvAffine(2, 3, CV_32F, appliedMatrix);
            try {
                cv::warpAffine(cvSrc,
                               cvDst,
                               cvAffine,
                               cv::Size(dstWidth, dstHeight),
                               cv::INTER_AREA,
                               cv::BORDER_CONSTANT,
                               bg);  // TODO(asahtik): Add support for different border types
            } catch(const cv::Exception& e) {
                throw std::runtime_error(fmt::format("cv::warpAffine failed: {}", e.what()));
            }
        } break;
        case OptOp::PERSPECTIVE: {
            if(appliedSrcHeight == 0 || appliedSrcWidth == 0 || srcStride == 0)
                throw std::runtime_error(fmt::format("Source dimension is zero ({}x{}, {})", appliedSrcWidth, appliedSrcHeight, srcStride));
            const cv::Mat cvSrc(appliedSrcHeight, appliedSrcWidth, type, const_cast<uint8_t*>(src + srcOffset), srcStride);
            cv::Mat cvDst(dstHeight, dstWidth, type, dst, dstStride);
            cv::Mat cvPerspective(3, 3, CV_32F, appliedMatrix);
            try {
                cv::warpPerspective(cvSrc,
                                    cvDst,
                                    cvPerspective,
                                    cv::Size(dstWidth, dstHeight),
                                    cv::INTER_AREA,
                                    cv::BORDER_CONSTANT,
                                    bg);  // TODO(asahtik): Add support for different border types
            } catch(const cv::Exception& e) {
                throw std::runtime_error(fmt::format("cv::warpPerspective failed: {}", e.what()));
            }
        } break;
    }
}

void ColorChangeQ::build(const FrameSpecs srcFrameSpecs, const FrameSpecs dstFrameSpecs, const ImgFrame::Type typeFrom, const ImgFrame::Type typeTo) {
    from = typeFrom;
    to = typeTo;
    srcSpecs = srcFrameSpecs;
    dstSpecs = dstFrameSpecs;
    size_t newAuxFrameSize = ALIGN_UP(srcSpecs.height, DEPTHAI_HEIGHT_ALIGNMENT) * ALIGN_UP(3 * srcSpecs.width, DEPTHAI_STRIDE_ALIGNMENT);
    if(!ccAuxFrame || ccAuxFrame->getOffsetSize() < newAuxFrameSize) ccAuxFrame = AcceleratorMemory::create(newAuxFrameSize);
}

void ColorChangeQ::apply(const std::shared_ptr<OffsetMemory>& src, std::shared_ptr<OffsetMemory> dst) {
    float bpp;
    int numPlanes;
    getFrameTypeInfo(to, numPlanes, bpp);

    bool done = false;
    auto start = std::chrono::steady_clock::now();

    auto srcOffset = src->getOffset();
    auto accelSrc = AcceleratorMemory::convert(ConvertedOffsetMemory::cast<Memory>(src));
    if(accelSrc == nullptr) {
        throw std::runtime_error("ColorChange: Invalid source pointer");
    }
    if(accelSrc->getOffset() == 0) accelSrc->setOffset(srcOffset);
    auto dstOffset = dst->getOffset();
    auto accelDst = AcceleratorMemory::convertZeroCopy(ConvertedOffsetMemory::cast<Memory>(dst));
    if(accelDst == nullptr) {
        throw std::runtime_error("ColorChange: Invalid destination pointer");
    }
    if(accelDst->getOffset() == 0) accelDst->setOffset(dstOffset);

    switch(to) {
        case dai::ImgFrame::Type::RGB888p:
            done = colorConvertToRGB888p({accelSrc, accelDst, srcSpecs, dstSpecs, from, ccAuxFrame});
            break;
        case dai::ImgFrame::Type::BGR888p:
            done = colorConvertToBGR888p({accelSrc, accelDst, srcSpecs, dstSpecs, from, ccAuxFrame});
            break;
        case dai::ImgFrame::Type::RGB888i:
            done = colorConvertToRGB888i({accelSrc, accelDst, srcSpecs, dstSpecs, from, ccAuxFrame});
            break;
        case dai::ImgFrame::Type::BGR888i:
            done = colorConvertToBGR888i({accelSrc, accelDst, srcSpecs, dstSpecs, from, ccAuxFrame});
            break;
        case dai::ImgFrame::Type::NV12:
            done = colorConvertToNV12({accelSrc, accelDst, srcSpecs, dstSpecs, from, ccAuxFrame});
            break;
        case dai::ImgFrame::Type::YUV420p:
            done = colorConvertToYUV420p({accelSrc, accelDst, srcSpecs, dstSpecs, from, ccAuxFrame});
            break;
        case dai::ImgFrame::Type::GRAY8:
        case dai::ImgFrame::Type::RAW8:
            done = colorConvertToGRAY8({accelSrc, accelDst, srcSpecs, dstSpecs, from, ccAuxFrame});
            break;
        case ImgFrame::Type::YUV422i:
        case ImgFrame::Type::YUV444p:
        case ImgFrame::Type::YUV422p:
        case ImgFrame::Type::YUV400p:
        case ImgFrame::Type::RGBA8888:
        case ImgFrame::Type::RGB161616:
        case ImgFrame::Type::LUT2:
        case ImgFrame::Type::LUT4:
        case ImgFrame::Type::LUT16:
        case ImgFrame::Type::RAW16:
        case ImgFrame::Type::RAW14:
        case ImgFrame::Type::RAW12:
        case ImgFrame::Type::RAW10:
        case ImgFrame::Type::PACK10:
        case ImgFrame::Type::PACK12:
        case ImgFrame::Type::YUV444i:
        case ImgFrame::Type::NV21:
        case ImgFrame::Type::BITSTREAM:
        case ImgFrame::Type::HDR:
        case ImgFrame::Type::RGBF16F16F16p:
        case ImgFrame::Type::BGRF16F16F16p:
        case ImgFrame::Type::RGBF16F16F16i:
        case ImgFrame::Type::BGRF16F16F16i:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            break;
    }
    auto diff = std::chrono::steady_clock::now() - start;
    if(logger) logger->trace("ImageManip | colorConvert took {}ns", std::chrono::duration_cast<std::chrono::nanoseconds>(diff).count());

    if(!done) {
        if(logger) logger->error("Convert color from {} to {} not supported or failed.", (int)from, (int)to);
        std::copy(src->getOffsetData().data(),
                  src->getOffsetData().data() + (src->getOffsetSize() <= dst->getOffsetSize() ? src->getOffsetSize() : dst->getOffsetSize()),
                  dst->getOffsetData().data());
    }
}

}  // namespace dai::gate
