#include "ToF.hpp"

#include <chrono>
#include <cstring>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <vector>

#include "../ImgTransformationUtils.hpp"
#include "BleedingFilter.hpp"
#include "camera/HalBufferMemory.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/utility/OffsetMemory.hpp"
#include "depthai/utility/VectorMemory.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "pipeline/MultiDeviceCalibrationStore.hpp"
#include "pipeline/NodeTiming.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/node/Camera/Camera.hpp"
#include "utilities/ErrorMacros.hpp"
#include "utilities/OSUtils.hpp"

using namespace std::chrono;

namespace dai {
namespace gate {

void ToFBase::run() {
    auto& logger = pimpl->logger;
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");
    logger->debug("ToF RVC4 node starting (VD55H1 depthmapwrapper integration)");

    std::unique_ptr<tofVD55H1> vd55h1;
    auto bleedingFilter = std::make_unique<BleedingFilter>(BleedingFilter::Parameters{});
    bool enableBleedingFilter = properties.initialConfig.profile != ToFConfig::Profile::LOW_RANGE;
    std::vector<uint8_t> amplitudeScratch;
    auto amplitudeLut = tofVD55H1::makeFloat16ToUint8Log1pLut(49.32f, 1.0f);
    amplitudeLut[0x7c00] = 255;  // FP32 amplitudes overflowing FP16 must saturate to white, not zero
    uint64_t seqDepth = 0;
    bool initialized = false;
    bool wantAmplitude = false;
    bool wantConfidence = false;
    bool wantIntensity = false;
    std::optional<tofVD55H1::Float16ToUint8Lut> confidenceLut;
    std::optional<tofVD55H1::Float16ToUint8Lut> intensityLut;
    cv::Mat undistortMapX;
    cv::Mat undistortMapY;
    dai::ImgTransformation localTransform;  // Frames carry outputTransform, rebased to the multi-device origin; localTransform is device-local
    dai::ImgTransformation outputTransform;
    std::shared_ptr<Camera> sourceCamera;
    void* sensorErrorCbHandle = nullptr;
    const auto deviceId = getSerialNumber();
    auto& multiDeviceStore = MultiDeviceCalibrationStore::getInstance();
    uint32_t multiDeviceGeneration = 0;
    bool warnedForMultiDeviceMismatch = false;
    const auto refreshOutputTransform = [&]() {
        if(multiDeviceStore.getGeneration() == multiDeviceGeneration) {
            return;
        }
        const auto snapshot = multiDeviceStore.get();
        logger->debug("ToF: multi-device calibration changed (generation {} -> {})", multiDeviceGeneration, snapshot.generation);
        multiDeviceGeneration = snapshot.generation;
        warnedForMultiDeviceMismatch = false;
        outputTransform = prepareMultiDeviceTransformation(localTransform, deviceId, snapshot.handler.get(), warnedForMultiDeviceMismatch, logger);
    };

    const auto cleanupSensorErrorCallback = [&]() {
        if(sourceCamera && sensorErrorCbHandle != nullptr) {
            sourceCamera->sensorErrorCbRemove(sensorErrorCbHandle);
            sensorErrorCbHandle = nullptr;
        }
        sourceCamera.reset();
    };

    try {
        while(isRunning()) {
            std::shared_ptr<ImgFrame> inImage;
            bool timedOut = false;
            inImage = rawInput.get<ImgFrame>(std::chrono::milliseconds(200), timedOut);
            if(timedOut || !inImage) {
                continue;
            }
            if(!initialized) {
                if(!initialize(inImage, vd55h1, wantAmplitude, wantConfidence, wantIntensity, localTransform, sourceCamera, sensorErrorCbHandle)) {
                    logger->error("ToF: failed to initialize VD55H1 processor, stopping node");
                    break;
                }
                if(wantConfidence) confidenceLut.emplace(tofVD55H1::makeFloat16ToUint8Log1pLut(30.91f, 20.0f));
                if(wantIntensity) intensityLut.emplace(tofVD55H1::makeFloat16ToUint8Log1pLut(51.95f, 0.5f));
                if(properties.enableUndistortion) {
                    const auto K = localTransform.getIntrinsicMatrix();
                    cv::Mat cameraMatrix(3, 3, CV_32F);
                    std::memcpy(cameraMatrix.data, K.data(), sizeof(K));
                    cv::initUndistortRectifyMap(cameraMatrix,
                                                cv::Mat(localTransform.getDistortionCoefficients()),
                                                cv::noArray(),
                                                cameraMatrix,
                                                cv::Size(vd55h1->getDepthHeight(), vd55h1->getDepthWidth()),
                                                CV_32FC1,
                                                undistortMapX,
                                                undistortMapY);
                    localTransform.setDistortionCoefficients({});
                }
                initialized = true;
            }
            refreshOutputTransform();

            auto runtimeCfg = inputConfig.tryGet<ToFConfig>();
            if(runtimeCfg) {
                vd55h1->applyConfig(*runtimeCfg);
                enableBleedingFilter = runtimeCfg->profile != ToFConfig::Profile::LOW_RANGE;
                logger->debug("ToF: runtime config applied");
            }

            auto tAbsoluteBeginning = steady_clock::now();
            auto tGotInput = tAbsoluteBeginning;
            uint8_t* rawPtr = inImage->getData().data();
            if(!rawPtr) {
                logger->warn("ToF: null data pointer in incoming frame, skipping");
                continue;
            }

            logger->debug("ToF: processFrame seq={} size={}", inImage->getSequenceNum(), inImage->getData().size());
            int rawFd = -1;
            const auto memory = ConvertedOffsetMemory::convert(inImage->data);
            if(memory && memory->getOffset() == 0) {  // The IPP import starts at the allocation base and cannot represent an offset
                const auto hal = ConvertedOffsetMemory::cast<HalBufferMemory>(memory);
                if(hal && hal->privHnd && hal->getData().data() == rawPtr) rawFd = hal->privHnd->fd;
            }
            if(!vd55h1->processFrame(rawPtr, rawFd)) {
                logger->warn("ToF: processFrame failed for seq={}", inImage->getSequenceNum());
                continue;
            }

            if(sourceCamera) {
                const auto nextExposureNs = vd55h1->getNextExposureTimeNs();
                if(nextExposureNs > 0 && nextExposureNs <= 350000) {  // The wrapper falls back to 15 ms; valid VD55H1 AE predictions are at most 0.35 ms
                    const auto exposureUs = static_cast<uint32_t>((nextExposureNs + 500) / 1000);
                    const auto iso = inImage->cam.sensitivityIso > 0 ? static_cast<uint32_t>(inImage->cam.sensitivityIso) : 100u;
                    sourceCamera->setToFExposure(exposureUs, iso);
                }
            }

            const auto tProcessed = steady_clock::now();
            const uint32_t ippW = vd55h1->getDepthWidth();
            const uint32_t ippH = vd55h1->getDepthHeight();
            const uint32_t outW = ippH;
            const uint32_t outH = ippW;
            const size_t depthSizeB = vd55h1->getDepthSizeBytes();
            const size_t u8PlaneSizeB = static_cast<size_t>(outW) * outH;
            auto undistortOutput = [&](const std::shared_ptr<ImgFrame>& frame) {
                if(undistortMapX.empty()) return;
                const int cvType = frame->getType() == ImgFrame::Type::RAW16 ? CV_16UC1 : CV_8UC1;
                const cv::Mat image(outH, outW, cvType, frame->getData().data());
                auto remappedMemory = std::make_shared<VectorMemory>(std::vector<uint8_t>(frame->getData().size()));
                cv::Mat remapped(outH, outW, cvType, remappedMemory->getData().data());
                cv::remap(image, remapped, undistortMapX, undistortMapY, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
                frame->data = std::move(remappedMemory);
            };
            auto makeOutputFrame = [&](size_t size, ImgFrame::Type type) {
                auto frame = std::make_shared<ImgFrame>(size);
                frame->setMetadata(*inImage);
                frame->setWidth(outW);
                frame->setHeight(outH);
                frame->setType(type);
                frame->setStride(outW * frame->getBytesPerPixel());
                const auto [srcW, srcH] = outputTransform.getSourceSize();
                frame->setSourceSize(srcW, srcH);
                frame->transformation = outputTransform;
                return frame;
            };

            std::shared_ptr<ImgFrame> amplitudeFrame;  // Filtering needs amplitude even without a connected output; keep planes distorted until filtered
            uint8_t* amplitudeData = nullptr;
            if(wantAmplitude) {
                amplitudeFrame = makeOutputFrame(u8PlaneSizeB, ImgFrame::Type::RAW8);
                amplitudeData = amplitudeFrame->getData().data();
            } else if(enableBleedingFilter) {
                amplitudeScratch.resize(u8PlaneSizeB);
                amplitudeData = amplitudeScratch.data();
            }
            if(amplitudeData) {
                tofVD55H1::transposeFloat32ToUint8(reinterpret_cast<const float*>(vd55h1->getAmplitudePtr()), amplitudeData, ippW, ippH, amplitudeLut);
            }
            auto sendU8PlaneF16 = [&](auto& out, const void* src, const auto& lut) {
                auto frame = makeOutputFrame(u8PlaneSizeB, ImgFrame::Type::RAW8);
                tofVD55H1::transposeFloat16ToUint8(src, frame->getData().data(), ippW, ippH, lut);
                undistortOutput(frame);
                out.send(frame);
            };

            {
                auto depthFrame = makeOutputFrame(depthSizeB, ImgFrame::Type::RAW16);
                const auto* src = reinterpret_cast<const uint16_t*>(vd55h1->getDepthPtr());
                auto* dst = reinterpret_cast<uint16_t*>(depthFrame->getData().data());
                constexpr uint16_t DEPTH_MM_MASK = 0x1FFF;
                const auto& r2pLut = vd55h1->radialToPerpLut();
                for(uint32_t srcX = 0; srcX < ippW; ++srcX) {
                    uint32_t dstIdx = srcX * ippH;
                    for(uint32_t srcY = 0; srcY < ippH; ++srcY, ++dstIdx) {
                        const uint16_t depthMm = src[srcX + srcY * ippW] & DEPTH_MM_MASK;
                        dst[dstIdx] = (depthMm == 0 || depthMm >= 6500) ? 0 : static_cast<uint16_t>(depthMm * r2pLut[dstIdx] + 0.5f);
                    }
                }
                if(enableBleedingFilter) {
                    bleedingFilter->filterDepth(dst, amplitudeData, outW, outH, outW, outW);
                }
                undistortOutput(depthFrame);
                depth.send(depthFrame);
            }

            const auto tDepth = steady_clock::now();
            if(wantAmplitude) {
                undistortOutput(amplitudeFrame);
                amplitude.send(amplitudeFrame);
            }

            const auto tAmplitude = steady_clock::now();
            if(wantConfidence) {
                sendU8PlaneF16(confidence, vd55h1->getConfidencePtr(), *confidenceLut);
            }
            if(wantIntensity) {
                sendU8PlaneF16(intensity, vd55h1->getIntensityPtr(), *intensityLut);
            }

            const auto tDone = steady_clock::now();
            dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tGotInput, tProcessed, tDone);
            logger->debug("ToF: seq={} total={:.3f}ms ipp={:.3f}ms depth={:.3f}ms amplitude={:.3f}ms confidence_intensity={:.3f}ms",
                          seqDepth,
                          duration<double, std::milli>(tDone - tAbsoluteBeginning).count(),
                          duration<double, std::milli>(tProcessed - tAbsoluteBeginning).count(),
                          duration<double, std::milli>(tDepth - tProcessed).count(),
                          duration<double, std::milli>(tAmplitude - tDepth).count(),
                          duration<double, std::milli>(tDone - tAmplitude).count());
            seqDepth++;
        }
    } catch(...) {
        cleanupSensorErrorCallback();
        throw;
    }

    cleanupSensorErrorCallback();
    logger->info("ToF RVC4 node stopped");
}

bool ToFBase::initialize(const std::shared_ptr<ImgFrame>& inImage,
                         std::unique_ptr<tofVD55H1>& vd55h1,
                         bool& wantAmplitude,
                         bool& wantConfidence,
                         bool& wantIntensity,
                         dai::ImgTransformation& localTransform,
                         std::shared_ptr<Camera>& sourceCamera,
                         void*& sensorErrorCbHandle) {
    auto& logger = pimpl->logger;
    const uint32_t rawW = inImage->getWidth();
    const uint32_t rawH = inImage->getHeight();
    const size_t dataSize = inImage->getData().size();

    logger->debug("ToF: first frame received — {}x{}, type={}, dataSize={}", rawW, rawH, static_cast<int>(inImage->getType()), dataSize);

    wantAmplitude = !amplitude.getConnections().empty();
    wantConfidence = !confidence.getConnections().empty();
    wantIntensity = !intensity.getConnections().empty();
    logger->info("ToF: optional outputs — amplitude={} confidence={} intensity={}", wantAmplitude, wantConfidence, wantIntensity);

    try {
        vd55h1 = std::make_unique<tofVD55H1>(rawW, rawH, wantConfidence, wantIntensity, logger);
    } catch(const std::exception& e) {
        logger->error("ToF: failed to initialise VD55H1 processor: {}", e.what());
        return false;
    }

    vd55h1->applyConfig(properties.initialConfig);

    const uint32_t outW = vd55h1->getDepthHeight();
    const uint32_t outH = vd55h1->getDepthWidth();
    const auto socket = static_cast<dai::CameraBoardSocket>(inImage->getInstanceNum());
    auto setOutputTransform = [&](dai::ImgTransformation transform) { localTransform = std::move(transform); };

    if(!getParentPipeline().isCalibrationDataAvailable()) {
        logger->warn("ToF: no calibration data available; using identity transform (no radial-to-perpendicular correction)");
        setOutputTransform(dai::ImgTransformation(outW, outH));
        vd55h1->buildIdentityLut();
    } else {
        const auto calib = getParentPipeline().getCalibrationData();
        try {
            const int srcW = static_cast<int>(calib.getSourceWidth(socket));
            const int srcH = static_cast<int>(calib.getSourceHeight(socket));
            const auto K = dai::matrix::vectorMatrixToMatrix3x3(calib.getCameraIntrinsics(socket, srcW, srcH));
            const auto distModel = calib.getDistortionModel(socket);
            const auto distCoeffs = calib.getDistortionCoefficients(socket);

            dai::ImgTransformation transform(static_cast<size_t>(srcW), static_cast<size_t>(srcH), K, distModel, distCoeffs);
            try {
                dai::CameraBoardSocket originSocket = dai::CameraBoardSocket::AUTO;
                const auto cameraToOrigin = calib.getExtrinsicsToOrigin(socket, false, originSocket);
                dai::Extrinsics extr(cameraToOrigin, originSocket, dai::LengthUnit::CENTIMETER);
                transform = dai::ImgTransformation(static_cast<size_t>(srcW), static_cast<size_t>(srcH), K, distModel, distCoeffs, extr);
            } catch(const std::exception& e) {
                logger->debug("ToF: no extrinsics for socket {} ({}); using intrinsics-only transform", static_cast<int>(socket), e.what());
            }

            const auto [baseW, baseH] = transform.getSize();
            if(baseW > 0 && baseH > 0 && (baseW != outW || baseH != outH)) {
                transform.addScale(static_cast<float>(outW) / static_cast<float>(baseW), static_cast<float>(outH) / static_cast<float>(baseH));
                transform.setSize(outW, outH);
            }
            vd55h1->buildRadialToPerpLut(transform.getIntrinsicMatrix(), distCoeffs);
            setOutputTransform(std::move(transform));
        } catch(const std::exception& e) {
            logger->warn("ToF: failed to load calibration for socket {} ({}); using identity transform", static_cast<int>(socket), e.what());
            setOutputTransform(dai::ImgTransformation(outW, outH));
            vd55h1->buildIdentityLut();
        }
    }

    auto pipeline = getParentPipeline();
    for(const auto& connection : pipeline.getConnections()) {
        if(connection.inputId == id && connection.inputName == rawInput.getName() && connection.inputGroup == rawInput.getGroup()) {
            auto node = pipeline.getNode(connection.outputId);
            sourceCamera = std::dynamic_pointer_cast<Camera>(node);
            break;
        }
    }

    if(sourceCamera) {
        sensorErrorCbHandle = sourceCamera->sensorErrorCbAdd([](int state) {
            if(state == 1) {
                DAI_ERROR("TOF sensor triggered laser skin safety protocol, please remove objects blocking the emitter.");
            } else if(state == 0) {
                DAI_INFO("TOF sensor laser skin safety protocol cleared.");
            }
        });
    } else {
        logger->warn("ToF: failed to resolve upstream Camera for sensor error callback");
    }

    return true;
}

}  // namespace gate
}  // namespace dai
