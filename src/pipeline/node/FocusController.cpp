#include "depthai/pipeline/node/host/FocusController.hpp"

#include <depthai/pipeline/datatype/GateControl.hpp>
#include <depthai/pipeline/datatype/ImageManipConfig.hpp>
#include <depthai/pipeline/node/NeuralDepth.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace dai {
namespace node {

std::shared_ptr<FocusController> FocusController::build(float targetFps) {
    setTargetFps(targetFps);
    return std::static_pointer_cast<FocusController>(shared_from_this());
}

void FocusController::setTargetFps(float targetFps) {
    targetFps_ = targetFps;
}

void FocusController::setNeuralModel(DeviceModelZoo model) {
    neuralModel_ = model;
    neuralSize_ = NeuralDepth::getInputSize(model);
    switch(model) {
        case DeviceModelZoo::NEURAL_DEPTH_1248X780:
            neuralFps_ = 8.5f;
            break;
        case DeviceModelZoo::NEURAL_DEPTH_1056X660:
            neuralFps_ = 10.0f;
            break;
        case DeviceModelZoo::NEURAL_DEPTH_960X600:
            neuralFps_ = 14.0f;
            break;
        case DeviceModelZoo::NEURAL_DEPTH_864X540:
            neuralFps_ = 18.0f;
            break;
        case DeviceModelZoo::NEURAL_DEPTH_768X480:
            neuralFps_ = 22.0f;
            break;
        case DeviceModelZoo::NEURAL_DEPTH_576X360:
            neuralFps_ = 38.0f;
            break;
        case DeviceModelZoo::NEURAL_DEPTH_480X300:
            neuralFps_ = 56.0f;
            break;
        case DeviceModelZoo::NEURAL_DEPTH_384X240:
        case DeviceModelZoo::NEURAL_DEPTH_288X180:
        case DeviceModelZoo::NEURAL_DEPTH_192X120:
            neuralFps_ = 60.0f;
            break;
        default:
            neuralFps_ = 56.0f;
            break;
    }
}

void FocusController::setStereoSize(int width, int height) {
    stereoSize_ = {width, height};
}

void FocusController::setStereoFps(float fps) {
    stereoFps_ = fps;
}

std::shared_ptr<Buffer> FocusController::processGroup(std::shared_ptr<MessageGroup> in) {
    if(!in) {
        return nullptr;
    }

    auto leftImg = in->get<ImgFrame>("left");
    auto rightImg = in->get<ImgFrame>("right");
    if(!leftImg || !rightImg) {
        return nullptr;
    }

    const int frameWidth = static_cast<int>(leftImg->getWidth());
    const int frameHeight = static_cast<int>(leftImg->getHeight());
    if(frameWidth <= 0 || frameHeight <= 0) {
        return nullptr;
    }

    // Max disparity: 192 px for a 1 MP width, scaled linearly.
    const int maxD = static_cast<int>(std::round(192.0f * static_cast<float>(frameWidth) / 1280.0f));

    std::vector<ImgDetection> detections;
    if(auto dets = in->get<ImgDetections>("inputDetections")) {
        try {
            auto transformed = dets->transformTo(leftImg->getTransformation());
            detections = std::move(transformed.detections);
        } catch(...) {
            // If the source transformation is missing, assume detections are already in the left frame.
            detections = std::move(dets->detections);
        }
    }

    auto makeZeroFrame = [&](ImgFrame::Type type, int elemSize) {
        auto frame = std::make_shared<ImgFrame>();
        frame->setWidth(frameWidth);
        frame->setHeight(frameHeight);
        frame->setType(type);
        frame->setBufferMetadataFrom(leftImg.get());
        frame->setTransformation(leftImg->getTransformation());
        std::vector<std::uint8_t> data(frameWidth * frameHeight * elemSize, 0);
        frame->setData(std::move(data));
        return frame;
    };

    if(detections.empty()) {
        auto depth = makeZeroFrame(ImgFrame::Type::RAW16, 2);
        auto conf = makeZeroFrame(ImgFrame::Type::RAW8, 1);
        confidenceOut.send(conf);
        return depth;
    }

    struct CropInfo {
        int x, y, w, h;
        float detX, detY, detW, detH;
    };
    std::vector<CropInfo> crops;
    crops.reserve(detections.size());

    int largestCropW = 0;
    int largestCropH = 0;
    std::uint64_t totalArea = 0;

    for(const auto& d : detections) {
        std::array<float, 4> outer;
        try {
            outer = d.getOuterBoundingBox();
        } catch(...) {
            continue;
        }

        float fx = outer[0] * frameWidth;
        float fy = outer[1] * frameHeight;
        float fx2 = outer[2] * frameWidth;
        float fy2 = outer[3] * frameHeight;
        float fw = fx2 - fx;
        float fh = fy2 - fy;
        if(fw <= 0 || fh <= 0) {
            continue;
        }

        int cropX = static_cast<int>(std::floor(fx - maxD));
        int cropY = static_cast<int>(std::floor(fy));
        int cropW = static_cast<int>(std::ceil(fw + 2 * maxD));
        int cropH = static_cast<int>(std::ceil(fh));
        if(cropW <= 0 || cropH <= 0) {
            continue;
        }

        // Clamp to frame bounds and adjust the crop origin/width when it is partially outside.
        int cropX2 = std::min(frameWidth, cropX + cropW);
        int cropY2 = std::min(frameHeight, cropY + cropH);
        cropX = std::max(0, cropX);
        cropY = std::max(0, cropY);
        cropW = cropX2 - cropX;
        cropH = cropY2 - cropY;
        if(cropW <= 0 || cropH <= 0) {
            continue;
        }

        crops.push_back({cropX, cropY, cropW, cropH, fx, fy, fw, fh});
        totalArea += static_cast<std::uint64_t>(cropW) * static_cast<std::uint64_t>(cropH);
        largestCropW = std::max(largestCropW, cropW);
        largestCropH = std::max(largestCropH, cropH);
    }

    if(crops.empty()) {
        auto depth = makeZeroFrame(ImgFrame::Type::RAW16, 2);
        auto conf = makeZeroFrame(ImgFrame::Type::RAW8, 1);
        confidenceOut.send(conf);
        return depth;
    }

    // Backend selection per frame budget.
    const int N = static_cast<int>(crops.size());
    const float framePeriod = (targetFps_ > 0.0f) ? (1.0f / targetFps_) : (1.0f / 30.0f);

    const int neuralArea = neuralSize_.first * neuralSize_.second;
    const int neuralFrames = neuralArea > 0 ? static_cast<int>(totalArea / static_cast<std::uint64_t>(neuralArea)) : 0;
    const float neuralTime = static_cast<float>(std::max(N, neuralFrames)) / neuralFps_;

    const int stereoArea = stereoSize_.first * stereoSize_.second;
    const int stereoFrames = stereoArea > 0 ? static_cast<int>(totalArea / static_cast<std::uint64_t>(stereoArea)) : 0;
    const float stereoTime = static_cast<float>(std::max(N, stereoFrames)) / stereoFps_;

    struct GpuProfile {
        int w, h;
        float fps;
    };
    static constexpr GpuProfile GPU_PROFILES[] = {
        {640, 400, 55.0f},
        {1280, 800, 30.0f},
        {2592, 1944, 5.0f},
    };
    float gpuTime = std::numeric_limits<float>::max();
    for(const auto& p : GPU_PROFILES) {
        if(largestCropW <= p.w && largestCropH <= p.h) {
            const int gpuArea = p.w * p.h;
            const int gpuFrames = gpuArea > 0 ? static_cast<int>(totalArea / static_cast<std::uint64_t>(gpuArea)) : 0;
            gpuTime = static_cast<float>(std::max(N, gpuFrames)) / p.fps;
            break;
        }
    }

    Backend selected = Backend::NEURAL;
    float bestTime = neuralTime;
    if(stereoTime < bestTime) {
        bestTime = stereoTime;
        selected = Backend::STEREO;
    }
    if(gpuTime < bestTime) {
        bestTime = gpuTime;
        selected = Backend::GPU;
    }

    // Per-frame algorithm notification.
    switch(selected) {
        case Backend::NEURAL:
            // gateControlNeural opened per crop below.
            break;
        case Backend::STEREO:
            // gateControlStereo opened per crop below.
            break;
        case Backend::GPU:
            // gateControlGpu opened per crop below.
            break;
    }

    (void)framePeriod;  // Future logging; for now selection simply picks the fastest.

    cv::Mat fullDepth = cv::Mat::zeros(frameHeight, frameWidth, CV_16U);
    cv::Mat fullConf = cv::Mat::zeros(frameHeight, frameWidth, CV_8U);

    const std::chrono::milliseconds timeout(5000);

    // Feed the synchronized pair to the crop ImageManips once per group; the crop configs
    // below reuse it (setReusePreviousImage) for every crop, so both left and right crops
    // carry this pair's timestamp and the backend's left/right Sync can pair them.
    leftImage.send(leftImg);
    rightImage.send(rightImg);

    for(std::size_t idx = 0; idx < crops.size(); ++idx) {
        const auto& crop = crops[idx];
        const int outW = (selected == Backend::GPU) ? crop.w : (selected == Backend::STEREO) ? stereoSize_.first : neuralSize_.first;
        const int outH = (selected == Backend::GPU) ? crop.h : (selected == Backend::STEREO) ? stereoSize_.second : neuralSize_.second;

        auto cfg = std::make_shared<ImageManipConfig>();
        cfg->setReusePreviousImage(idx > 0);
        auto resizeMode = (selected == Backend::GPU) ? ImageManipConfig::ResizeMode::NONE : ImageManipConfig::ResizeMode::STRETCH;
        cfg->setOutputSize(static_cast<std::uint32_t>(outW), static_cast<std::uint32_t>(outH), resizeMode);
        cfg->base.center = false;
        cfg->addCrop(dai::Rect(static_cast<float>(crop.x), static_cast<float>(crop.y), static_cast<float>(crop.w), static_cast<float>(crop.h)), false);

        leftConfig.send(cfg);
        rightConfig.send(cfg);

        std::shared_ptr<GateControl> gateOpen = GateControl::openGate(1);
        if(selected == Backend::NEURAL) {
            gateControlNeural.send(gateOpen);
        } else if(selected == Backend::STEREO) {
            gateControlStereo.send(gateOpen);
        } else {
            gateControlGpu.send(gateOpen);
        }

        bool hasTimedOut = false;
        auto depthMsg = depthCrop.get<ImgFrame>(timeout, hasTimedOut);
        auto confMsg = confidenceCrop.get<ImgFrame>(timeout, hasTimedOut);
        if(!depthMsg || !confMsg) {
            continue;
        }

        // Detection region in the (possibly resized) crop output.
        int fullX = std::max(0, static_cast<int>(std::floor(crop.detX)));
        int fullY = std::max(0, static_cast<int>(std::floor(crop.detY)));
        int fullX2 = std::min(frameWidth, static_cast<int>(std::ceil(crop.detX + crop.detW)));
        int fullY2 = std::min(frameHeight, static_cast<int>(std::ceil(crop.detY + crop.detH)));
        if(fullX2 <= fullX || fullY2 <= fullY) {
            continue;
        }
        int fullW = fullX2 - fullX;
        int fullH = fullY2 - fullY;

        int srcX = fullX - crop.x;
        int srcY = fullY - crop.y;

        cv::Mat depthCropMat = depthMsg->getFrame();
        cv::Mat confCropMat = confMsg->getFrame();
        if(depthCropMat.empty() || confCropMat.empty()) {
            continue;
        }

        cv::Mat scaledDepth, scaledConf;
        if(static_cast<int>(depthCropMat.cols) != crop.w || static_cast<int>(depthCropMat.rows) != crop.h) {
            cv::resize(depthCropMat, scaledDepth, cv::Size(crop.w, crop.h), 0, 0, cv::INTER_NEAREST);
            cv::resize(confCropMat, scaledConf, cv::Size(crop.w, crop.h), 0, 0, cv::INTER_NEAREST);
        } else {
            scaledDepth = depthCropMat;
            scaledConf = confCropMat;
        }

        cv::Mat cropDepth(scaledDepth, cv::Rect(srcX, srcY, fullW, fullH));
        cv::Mat cropConf(scaledConf, cv::Rect(srcX, srcY, fullW, fullH));
        cv::Mat fullDepthRoi(fullDepth, cv::Rect(fullX, fullY, fullW, fullH));
        cv::Mat fullConfRoi(fullConf, cv::Rect(fullX, fullY, fullW, fullH));
        cropDepth.copyTo(fullDepthRoi);
        cropConf.copyTo(fullConfRoi);
    }

    auto depthImg = std::make_shared<ImgFrame>();
    depthImg->setWidth(frameWidth);
    depthImg->setHeight(frameHeight);
    depthImg->setType(ImgFrame::Type::RAW16);
    depthImg->setBufferMetadataFrom(leftImg.get());
    depthImg->setTransformation(leftImg->getTransformation());
    depthImg->setFrame(fullDepth);

    auto confImg = std::make_shared<ImgFrame>();
    confImg->setWidth(frameWidth);
    confImg->setHeight(frameHeight);
    confImg->setType(ImgFrame::Type::RAW8);
    confImg->setBufferMetadataFrom(leftImg.get());
    confImg->setTransformation(leftImg->getTransformation());
    confImg->setFrame(fullConf);

    confidenceOut.send(confImg);
    return depthImg;
}

}  // namespace node
}  // namespace dai
