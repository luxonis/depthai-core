#include "depthai/pipeline/node/host/FocusController.hpp"

#include <depthai/pipeline/datatype/GateControl.hpp>
#include <depthai/pipeline/datatype/ImageManipConfig.hpp>
#include <depthai/pipeline/node/NeuralDepth.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

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

std::vector<FocusController::Crop> FocusController::computeCrops(int frameWidth,
                                                                int frameHeight,
                                                                const std::vector<std::array<float, 4>>& normalizedBoxes) {
    std::vector<Crop> crops;
    crops.reserve(normalizedBoxes.size());
    if(frameWidth <= 0 || frameHeight <= 0) {
        return crops;
    }

    // Max disparity: 192 px for a 1 MP width, scaled linearly.
    const int maxD = static_cast<int>(std::round(192.0f * static_cast<float>(frameWidth) / 1280.0f));

    for(const auto& box : normalizedBoxes) {
        float fx = box[0] * frameWidth;
        float fy = box[1] * frameHeight;
        float fx2 = box[2] * frameWidth;
        float fy2 = box[3] * frameHeight;
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
    }

    return crops;
}

FocusController::CopyRegion FocusController::computeCopyRegion(const Crop& crop, int frameWidth, int frameHeight) {
    CopyRegion region{0, 0, 0, 0, 0, 0, false};
    if(crop.w <= 0 || crop.h <= 0 || frameWidth <= 0 || frameHeight <= 0) {
        return region;
    }

    // Detection box clamped to the frame; the focused output is written only over this region.
    int dstX = std::max(0, static_cast<int>(std::floor(crop.detX)));
    int dstY = std::max(0, static_cast<int>(std::floor(crop.detY)));
    int dstX2 = std::min(frameWidth, static_cast<int>(std::ceil(crop.detX + crop.detW)));
    int dstY2 = std::min(frameHeight, static_cast<int>(std::ceil(crop.detY + crop.detH)));
    if(dstX2 <= dstX || dstY2 <= dstY) {
        return region;
    }

    // Offset of the detection box inside the crop mat. Never negative because the crop origin is
    // at or before the detection box, but clamp defensively.
    int srcX = std::max(0, dstX - crop.x);
    int srcY = std::max(0, dstY - crop.y);
    dstX = crop.x + srcX;
    dstY = crop.y + srcY;

    // Clamp the copied region to whichever of the crop mat / frame runs out first. floor/ceil
    // rounding between detW/detH and the crop can otherwise make this one pixel too large.
    int w = std::min({dstX2 - dstX, crop.w - srcX, frameWidth - dstX});
    int h = std::min({dstY2 - dstY, crop.h - srcY, frameHeight - dstY});
    if(w <= 0 || h <= 0) {
        return region;
    }

    return CopyRegion{srcX, srcY, dstX, dstY, w, h, true};
}

float FocusController::depthFocalScale(float fxFull, float fxUsed, int outW, int cropW) {
    if(!(fxFull > 0.0f) || !(fxUsed > 0.0f) || outW <= 0 || cropW <= 0) {
        return 1.0f;
    }
    const float fxCorrect = fxFull * static_cast<float>(outW) / static_cast<float>(cropW);
    const float scale = fxCorrect / fxUsed;
    if(!std::isfinite(scale) || scale <= 0.0f) {
        return 1.0f;
    }
    return scale;
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

    // Horizontal focal length of the full rectified frame (in full-frame pixels). Used to
    // rescale each crop's depth to the crop's correct focal (see the per-crop correction below).
    const float fxFull = leftImg->getTransformation().getIntrinsicMatrix()[0][0];

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

    std::vector<std::array<float, 4>> boxes;
    boxes.reserve(detections.size());
    for(const auto& d : detections) {
        try {
            boxes.push_back(d.getOuterBoundingBox());
        } catch(...) {
            continue;
        }
    }

    const std::vector<Crop> crops = computeCrops(frameWidth, frameHeight, boxes);

    int largestCropW = 0;
    int largestCropH = 0;
    std::uint64_t totalArea = 0;
    for(const auto& crop : crops) {
        totalArea += static_cast<std::uint64_t>(crop.w) * static_cast<std::uint64_t>(crop.h);
        largestCropW = std::max(largestCropW, crop.w);
        largestCropH = std::max(largestCropH, crop.h);
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

    // The STEREO/GPU backends require different crop ImageManip output sizes than NEURAL, and the
    // left/right crop ImageManips can desync by a frame when the selected backend (hence output
    // size) changes between groups, delivering mismatched left/right sizes to the backend and
    // crashing it device-side (e.g. StereoDepth "Left 480x300, Right 640x400"). Until each backend
    // has its own fixed-size crop manips, pin the reliable NEURAL backend, which produces
    // correct per-crop depth for every crop size. The selection above is kept for future use.
    selected = Backend::NEURAL;

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

        cv::Mat depthCropMat = depthMsg->getFrame();
        cv::Mat confCropMat = confMsg->getFrame();
        if(depthCropMat.empty() || confCropMat.empty()) {
            continue;
        }

        // Rescale the backend depth to the crop's correct focal length. The backend estimates
        // disparity in the STRETCH-resized crop grid (outW wide) and converts it to depth using
        // the focal length carried by the crop frame's intrinsic (fxUsed). That focal is not the
        // crop's true focal, so depth comes out scaled by (fxCorrect / fxUsed) and the error
        // differs per crop width, producing depth discontinuities between regions.
        //
        // The correct focal for a crop of width crop.w (in full-frame px) resized to outW is
        // fxFull * outW / crop.w. Multiplying depth by fxCorrect / fxUsed restores metric depth
        // consistently across crop sizes. If the frame intrinsic is ever made crop-correct
        // upstream, fxUsed == fxCorrect and this becomes a no-op.
        const float fxUsed = depthMsg->getTransformation().getIntrinsicMatrix()[0][0];
        const float depthScale = depthFocalScale(fxFull, fxUsed, outW, crop.w);
        if(std::abs(depthScale - 1.0f) > 1e-3f) {
            depthCropMat.convertTo(depthCropMat, depthCropMat.type(), depthScale);
        }

        cv::Mat scaledDepth, scaledConf;
        if(static_cast<int>(depthCropMat.cols) != crop.w || static_cast<int>(depthCropMat.rows) != crop.h) {
            cv::resize(depthCropMat, scaledDepth, cv::Size(crop.w, crop.h), 0, 0, cv::INTER_NEAREST);
            cv::resize(confCropMat, scaledConf, cv::Size(crop.w, crop.h), 0, 0, cv::INTER_NEAREST);
        } else {
            scaledDepth = depthCropMat;
            scaledConf = confCropMat;
        }

        // Detection region mapped into the (resized) crop mat, clamped to both buffers.
        const CopyRegion region = computeCopyRegion(crop, frameWidth, frameHeight);
        if(!region.valid) {
            continue;
        }

        cv::Mat cropDepth(scaledDepth, cv::Rect(region.srcX, region.srcY, region.w, region.h));
        cv::Mat cropConf(scaledConf, cv::Rect(region.srcX, region.srcY, region.w, region.h));
        cv::Mat fullDepthRoi(fullDepth, cv::Rect(region.dstX, region.dstY, region.w, region.h));
        cv::Mat fullConfRoi(fullConf, cv::Rect(region.dstX, region.dstY, region.w, region.h));
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
