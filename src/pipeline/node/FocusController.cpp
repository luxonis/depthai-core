#include "depthai/pipeline/node/host/FocusController.hpp"

#include <depthai/pipeline/datatype/Buffer.hpp>
#include <depthai/pipeline/datatype/ImageManipConfig.hpp>
#include <depthai/pipeline/node/NeuralDepth.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
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

void FocusController::setModels(const std::vector<DeviceModelZoo>& models) {
    if(models.empty() || static_cast<int>(models.size()) > kNumTiers) {
        throw std::invalid_argument("FocusController requires between one and three models");
    }
    tierCount_ = static_cast<int>(models.size());
    for(int tier = 0; tier < kNumTiers; ++tier) {
        const auto model = models[std::min(tier, tierCount_ - 1)];
        const auto size = NeuralDepth::getInputSize(model);
        tiers_[tier] = Tier{model, size.first, size.second};
    }
}

int FocusController::selectTier(const std::array<Tier, kNumTiers>& tiers, int tierCount, int cropW, int cropH) {
    tierCount = std::clamp(tierCount, 1, kNumTiers);
    for(int tier = 0; tier < tierCount; ++tier) {
        if(tiers[tier].w >= cropW && tiers[tier].h >= cropH) {
            return tier;
        }
    }
    return tierCount - 1;
}

int FocusController::selectTier(int cropW, int cropH) {
    return selectTier(kTiers, kNumTiers, cropW, cropH);
}

int FocusController::selectTier(const std::array<Tier, kNumTiers>& tiers, int cropW, int cropH) {
    return selectTier(tiers, kNumTiers, cropW, cropH);
}

std::vector<FocusController::MergedCrop> FocusController::orderCropsByArea(const std::vector<MergedCrop>& crops) {
    auto ordered = crops;
    std::stable_sort(ordered.begin(), ordered.end(), [](const MergedCrop& lhs, const MergedCrop& rhs) {
        return static_cast<long long>(lhs.w) * lhs.h > static_cast<long long>(rhs.w) * rhs.h;
    });
    return ordered;
}

float FocusController::estimateInferenceCostMs(int w, int h) {
    if(w <= 0 || h <= 0) {
        return 0.0f;
    }
    // Fitted to luxonis/depthai-core#1912 on-device NeuralDepth latencies (see header).
    constexpr float kBaseMs = 31.7f;
    constexpr float kMsPerPixel = 1.88e-4f;
    return kBaseMs + kMsPerPixel * static_cast<float>(w) * static_cast<float>(h);
}

int FocusController::selectTierWithinBudget(const std::array<Tier, kNumTiers>& tiers, int tierCount, int cropW, int cropH, double remainingMs) {
    tierCount = std::clamp(tierCount, 1, kNumTiers);
    // Start at the crop-appropriate tier and downgrade toward the fastest tier until one fits the
    // remaining budget. A downgraded tier is smaller than the crop, so the crop is downscaled into
    // it (with per-crop focal correction keeping depth metric) - trading resolution for time.
    const int ideal = selectTier(tiers, tierCount, cropW, cropH);
    for(int tier = ideal; tier >= 0; --tier) {
        if(static_cast<double>(estimateInferenceCostMs(tiers[tier].w, tiers[tier].h)) <= remainingMs) {
            return tier;
        }
    }
    return -1;
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

std::vector<std::array<float, 4>> FocusController::selectLargest(const std::vector<std::array<float, 4>>& normalizedBoxes) {
    if(normalizedBoxes.empty()) {
        return {};
    }
    const auto largest = std::max_element(normalizedBoxes.begin(), normalizedBoxes.end(), [](const auto& lhs, const auto& rhs) {
        return (lhs[2] - lhs[0]) * (lhs[3] - lhs[1]) < (rhs[2] - rhs[0]) * (rhs[3] - rhs[1]);
    });
    return {*largest};
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
    const auto processStart = std::chrono::steady_clock::now();
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
    if(selectionMode_ == SelectionMode::LARGEST) {
        boxes = selectLargest(boxes);
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
        std::ostringstream dbg;
        dbg << "dets=" << detections.size() << " boxes=" << boxes.size() << " crops=0 processed=0 (no regions)";
        const std::string dbgStr = dbg.str();
        auto dbgBuf = std::make_shared<Buffer>();
        dbgBuf->setData(std::vector<std::uint8_t>(dbgStr.begin(), dbgStr.end()));
        focusDebug.send(dbgBuf);
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
    auto reassemble = [&](const MergedCrop& mc, const std::shared_ptr<ImgFrame>& depthMsg, const std::shared_ptr<ImgFrame>& confMsg, int outW) {
        if(!depthMsg || !confMsg) {
            return;
        }

        cv::Mat depthCropMat = depthMsg->getFrame();
        cv::Mat confCropMat = confMsg->getFrame();
        if(depthCropMat.empty() || confCropMat.empty()) {
            return;
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
    };

    auto dispatchAndCollect = [&](const MergedCrop& mc, int selectedTier, bool reusePrevious) {
        auto cfg = std::make_shared<ImageManipConfig>();
        cfg->setReusePreviousImage(reusePrevious);
        cfg->setOutputSize(static_cast<std::uint32_t>(tiers_[selectedTier].w),
                           static_cast<std::uint32_t>(tiers_[selectedTier].h),
                           ImageManipConfig::ResizeMode::STRETCH);
        cfg->base.center = false;
        cfg->addCrop(dai::Rect(static_cast<float>(mc.x), static_cast<float>(mc.y), static_cast<float>(mc.w), static_cast<float>(mc.h)), false);
        leftConfigTier(selectedTier).send(cfg);
        rightConfigTier(selectedTier).send(cfg);
        bool hasTimedOut = false;
        auto depthMsg = depthCropTier(selectedTier).get<ImgFrame>(timeout, hasTimedOut);
        auto confMsg = confidenceCropTier(selectedTier).get<ImgFrame>(timeout, hasTimedOut);
        reassemble(mc, depthMsg, confMsg, tiers_[selectedTier].w);
    };

    // Per-crop dispatch trace filled below and formatted into the focusDebug output.
    struct CropTrace {
        int tier;
        int cropW;
        int cropH;
        int modelW;
        int modelH;
        double ms;
    };
    std::vector<CropTrace> trace;

    if(dispatchMode_ == DispatchMode::SINGLE_TIER_PER_FRAME) {
        const int selectedTier = selectTier(tiers_, tierCount_, maxW, maxH);
        // Dispatch every crop first so the selected backend remains pipelined across the frame.
        for(std::size_t idx = 0; idx < merged.size(); ++idx) {
            const auto& mc = merged[idx];
            auto cfg = std::make_shared<ImageManipConfig>();
            cfg->setReusePreviousImage(idx > 0);
            cfg->setOutputSize(static_cast<std::uint32_t>(tiers_[selectedTier].w),
                               static_cast<std::uint32_t>(tiers_[selectedTier].h),
                               ImageManipConfig::ResizeMode::STRETCH);
            cfg->base.center = false;
            cfg->addCrop(dai::Rect(static_cast<float>(mc.x), static_cast<float>(mc.y), static_cast<float>(mc.w), static_cast<float>(mc.h)), false);
            leftConfigTier(selectedTier).send(cfg);
            rightConfigTier(selectedTier).send(cfg);
        }
        for(const auto& mc : merged) {
            const auto cropStart = std::chrono::steady_clock::now();
            bool hasTimedOut = false;
            auto depthMsg = depthCropTier(selectedTier).get<ImgFrame>(timeout, hasTimedOut);
            auto confMsg = confidenceCropTier(selectedTier).get<ImgFrame>(timeout, hasTimedOut);
            reassemble(mc, depthMsg, confMsg, tiers_[selectedTier].w);
            const double cropMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cropStart).count();
            trace.push_back({selectedTier, mc.w, mc.h, tiers_[selectedTier].w, tiers_[selectedTier].h, cropMs});
        }
    } else {
        // Process crops largest-first within a per-frame time budget of one frame period. For each
        // crop, pick the smallest model that fits both the crop size and the remaining budget
        // (measured elapsed vs the #1912-derived per-model cost estimate), downgrading toward the
        // fastest model when time runs short, and stop once even the fastest model would overrun.
        // Elapsed time is measured (so model-reload cost between differently sized models is
        // accounted for); only the next crop's cost is estimated.
        const auto ordered = orderCropsByArea(merged);
        const double budgetMs = 1000.0 / std::max(0.1, static_cast<double>(targetFps_));
        std::size_t processed = 0;
        std::array<bool, kNumTiers> consumed{};
        for(const auto& mc : ordered) {
            const auto now = std::chrono::steady_clock::now();
            const double remainingMs = budgetMs - std::chrono::duration<double, std::milli>(now - processStart).count();
            int selectedTier = selectTierWithinBudget(tiers_, tierCount_, mc.w, mc.h, remainingMs);
            if(selectedTier < 0) {
                // Nothing fits the remaining budget: always process at least one crop (the fastest
                // model) so a frame is never left completely empty, then stop.
                if(processed == 0) {
                    selectedTier = 0;
                } else {
                    break;
                }
            }
            const auto cropStart = std::chrono::steady_clock::now();
            dispatchAndCollect(mc, selectedTier, consumed[selectedTier]);
            consumed[selectedTier] = true;
            const double cropMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - cropStart).count();
            trace.push_back({selectedTier, mc.w, mc.h, tiers_[selectedTier].w, tiers_[selectedTier].h, cropMs});
            ++processed;
        }
    }

    {
        const double budgetMs = 1000.0 / std::max(0.1, static_cast<double>(targetFps_));
        std::ostringstream dbg;
        dbg << "dets=" << detections.size() << " boxes=" << boxes.size() << " crops=" << merged.size() << " processed=" << trace.size()
            << " mode=" << (dispatchMode_ == DispatchMode::SINGLE_TIER_PER_FRAME ? "SINGLE_TIER" : "TIME_BUDGET")
            << " sel=" << (selectionMode_ == SelectionMode::LARGEST ? "LARGEST" : "ALL") << " fps=" << std::fixed << std::setprecision(1) << targetFps_
            << " budget_ms=" << std::setprecision(1) << budgetMs << " tiers=[";
        for(int t = 0; t < tierCount_; ++t) {
            dbg << tiers_[t].w << "x" << tiers_[t].h << (t + 1 < tierCount_ ? "," : "");
        }
        dbg << "]";
        for(std::size_t i = 0; i < trace.size(); ++i) {
            const auto& e = trace[i];
            dbg << " | crop" << i << " tier" << e.tier << " model=" << e.modelW << "x" << e.modelH << " src=" << e.cropW << "x" << e.cropH << " "
                << std::setprecision(1) << e.ms << "ms";
        }
        const std::string dbgStr = dbg.str();
        auto dbgBuf = std::make_shared<Buffer>();
        dbgBuf->setData(std::vector<std::uint8_t>(dbgStr.begin(), dbgStr.end()));
        focusDebug.send(dbgBuf);
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
