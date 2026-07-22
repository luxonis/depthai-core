#include "depthai/pipeline/node/host/FocusController.hpp"

#include <depthai/pipeline/datatype/ImageManipConfig.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
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

Node::Input& FocusController::depthCropTier(int tier) {
    switch(tier) {
        case 0:
            return depthCrop0;
        case 1:
            return depthCrop1;
        default:
            return depthCrop2;
    }
}

Node::Input& FocusController::confidenceCropTier(int tier) {
    switch(tier) {
        case 0:
            return confidenceCrop0;
        case 1:
            return confidenceCrop1;
        default:
            return confidenceCrop2;
    }
}

Node::Output& FocusController::leftConfigTier(int tier) {
    switch(tier) {
        case 0:
            return leftConfig0;
        case 1:
            return leftConfig1;
        default:
            return leftConfig2;
    }
}

Node::Output& FocusController::rightConfigTier(int tier) {
    switch(tier) {
        case 0:
            return rightConfig0;
        case 1:
            return rightConfig1;
        default:
            return rightConfig2;
    }
}

int FocusController::selectTier(int cropW, int cropH) {
    for(int tier = 0; tier < kNumTiers; ++tier) {
        if(kTiers[tier].w >= cropW && kTiers[tier].h >= cropH) {
            return tier;
        }
    }
    return kNumTiers - 1;
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

std::vector<FocusController::MergedCrop> FocusController::mergeCrops(const std::vector<Crop>& crops) {
    std::vector<MergedCrop> merged;
    merged.reserve(crops.size());
    for(const auto& c : crops) {
        if(c.w <= 0 || c.h <= 0) {
            continue;
        }
        merged.push_back({c.x, c.y, c.w, c.h, {{{c.detX, c.detY, c.detW, c.detH}}}});
    }

    // Two rectangles overlap when they intersect on both axes.
    auto overlaps = [](const MergedCrop& a, const MergedCrop& b) {
        return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
    };

    // Repeatedly fold any crop that overlaps an earlier one into it (union rectangle, combined
    // detection list) until a full pass makes no change. Crop counts are small, so O(n^2) per pass
    // is fine, and merging can create new overlaps that later passes resolve.
    bool mergedAny = true;
    while(mergedAny) {
        mergedAny = false;
        for(std::size_t i = 0; i < merged.size(); ++i) {
            for(std::size_t j = i + 1; j < merged.size();) {
                if(overlaps(merged[i], merged[j])) {
                    const int x2 = std::max(merged[i].x + merged[i].w, merged[j].x + merged[j].w);
                    const int y2 = std::max(merged[i].y + merged[i].h, merged[j].y + merged[j].h);
                    merged[i].x = std::min(merged[i].x, merged[j].x);
                    merged[i].y = std::min(merged[i].y, merged[j].y);
                    merged[i].w = x2 - merged[i].x;
                    merged[i].h = y2 - merged[i].y;
                    merged[i].dets.insert(merged[i].dets.end(), merged[j].dets.begin(), merged[j].dets.end());
                    merged.erase(merged.begin() + static_cast<std::ptrdiff_t>(j));
                    mergedAny = true;
                } else {
                    ++j;
                }
            }
        }
    }

    return merged;
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

    // Fold overlapping crops (disparity padding makes neighbouring detections overlap) into their
    // union so each region is inferred once instead of once per detection.
    std::vector<MergedCrop> merged = mergeCrops(crops);
    if(static_cast<int>(merged.size()) > kMaxCropsPerFrame) {
        merged.resize(kMaxCropsPerFrame);
    }

    if(merged.empty()) {
        auto depth = makeZeroFrame(ImgFrame::Type::RAW16, 2);
        auto conf = makeZeroFrame(ImgFrame::Type::RAW8, 1);
        confidenceOut.send(conf);
        return depth;
    }

    cv::Mat fullDepth = cv::Mat::zeros(frameHeight, frameWidth, CV_16U);
    cv::Mat fullConf = cv::Mat::zeros(frameHeight, frameWidth, CV_8U);

    const std::chrono::milliseconds timeout(5000);

    // Broadcast the synchronized pair to every tier's crop ImageManips once per group. The first
    // config sent to a tier consumes this frame (setReusePreviousImage(false)); later crops on the
    // same tier reuse it, so all of a tier's left/right crops share this timestamp and its backend
    // Sync can pair them. Tiers that get no crop this frame just hold the (non-blocking) latest.
    leftImage.send(leftImg);
    rightImage.send(rightImg);

    // Pick a single backend tier per frame: the smallest model that fits the largest crop this
    // frame. Every crop this frame uses that one model. This deliberately does NOT route each crop
    // to its own best-fit tier: RVC4 has a single NN engine, and alternating between differently
    // sized depth models within a frame forces an expensive model reload per crop (measured ~2.5x
    // slower for mixed-size scenes, and far worse alongside a detection network). Using one model
    // per frame keeps the model resident across the frame's crops while still using a smaller, faster
    // model when the whole scene is small and a larger one only when a crop needs the resolution.
    int maxW = 0;
    int maxH = 0;
    for(const auto& mc : merged) {
        maxW = std::max(maxW, mc.w);
        maxH = std::max(maxH, mc.h);
    }
    const int tier = selectTier(maxW, maxH);
    const int outW = kTiers[tier].w;

    // Phase 1 (dispatch): queue every crop's ImageManip config on the selected tier up front, so the
    // crop ImageManips and the tier's NeuralDepth backend run back-to-back (pipelined) instead of the
    // frame costing one full host<->device round-trip per crop, which was the dominant cost.
    for(std::size_t idx = 0; idx < merged.size(); ++idx) {
        const auto& mc = merged[idx];

        auto cfg = std::make_shared<ImageManipConfig>();
        cfg->setReusePreviousImage(idx > 0);
        cfg->setOutputSize(static_cast<std::uint32_t>(kTiers[tier].w), static_cast<std::uint32_t>(kTiers[tier].h), ImageManipConfig::ResizeMode::STRETCH);
        cfg->base.center = false;
        cfg->addCrop(dai::Rect(static_cast<float>(mc.x), static_cast<float>(mc.y), static_cast<float>(mc.w), static_cast<float>(mc.h)), false);

        leftConfigTier(tier).send(cfg);
        rightConfigTier(tier).send(cfg);
    }

    // Phase 2 (collect): the tier emits one depth/confidence pair per crop, in dispatch order.
    for(std::size_t idx = 0; idx < merged.size(); ++idx) {
        const auto& mc = merged[idx];

        bool hasTimedOut = false;
        auto depthMsg = depthCropTier(tier).get<ImgFrame>(timeout, hasTimedOut);
        auto confMsg = confidenceCropTier(tier).get<ImgFrame>(timeout, hasTimedOut);
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
        // The correct focal for a crop of width mc.w (in full-frame px) resized to outW is
        // fxFull * outW / mc.w. Multiplying depth by fxCorrect / fxUsed restores metric depth
        // consistently across crop sizes. If the frame intrinsic is ever made crop-correct
        // upstream, fxUsed == fxCorrect and this becomes a no-op.
        const float fxUsed = depthMsg->getTransformation().getIntrinsicMatrix()[0][0];
        const float depthScale = depthFocalScale(fxFull, fxUsed, outW, mc.w);
        if(std::abs(depthScale - 1.0f) > 1e-3f) {
            depthCropMat.convertTo(depthCropMat, depthCropMat.type(), depthScale);
        }

        cv::Mat scaledDepth, scaledConf;
        if(static_cast<int>(depthCropMat.cols) != mc.w || static_cast<int>(depthCropMat.rows) != mc.h) {
            cv::resize(depthCropMat, scaledDepth, cv::Size(mc.w, mc.h), 0, 0, cv::INTER_NEAREST);
            cv::resize(confCropMat, scaledConf, cv::Size(mc.w, mc.h), 0, 0, cv::INTER_NEAREST);
        } else {
            scaledDepth = depthCropMat;
            scaledConf = confCropMat;
        }

        // Write the focused output back only over each detection box the merged crop covers (not
        // the padded/merged rectangle), so gaps between merged detections stay unfilled.
        for(const auto& det : mc.dets) {
            const Crop writeBack{mc.x, mc.y, mc.w, mc.h, det[0], det[1], det[2], det[3]};
            const CopyRegion region = computeCopyRegion(writeBack, frameWidth, frameHeight);
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
