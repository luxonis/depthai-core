#include "ToFStereoFusion.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <utility>

#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"

namespace dai::gate {

namespace {

cv::Rect largestOverlapRectangle(const cv::Mat& mask) {
    std::vector<int> heights(mask.cols + 1, 0), stack;
    cv::Rect best;
    for(int y = 0; y < mask.rows; ++y) {
        for(int x = 0; x < mask.cols; ++x) heights[x] = mask.at<uint8_t>(y, x) ? heights[x] + 1 : 0;
        stack.clear();
        for(int x = 0; x <= mask.cols; ++x) {
            while(!stack.empty() && heights[stack.back()] > heights[x]) {
                const int height = heights[stack.back()];
                stack.pop_back();
                const int left = stack.empty() ? 0 : stack.back() + 1;
                const cv::Rect candidate(left, y - height + 1, x - left, height);
                if(candidate.area() > best.area()) best = candidate;
            }
            stack.push_back(x);
        }
    }
    return best;
}

constexpr float kMinFusedInverse = 1.0f / 15.0f;
constexpr float kInputQuantizationStep = 0.004764093086f;
constexpr float kDisQuantizationStep = 0.011741727591f;
constexpr float kDisInverseGain = 10.0f;
constexpr float kDisClip = 3.0f;
struct FusedDepthConfidence {
    float depthMm;
    float confidence;
};
const std::array<float, 65536>& depthInverseLut();

FusedDepthConfidence fuseDepthAndConfidence(uint16_t tofMm, uint16_t lensMm, float alpha, float confidence) {
    const auto tofValid = tofMm > 0;
    const auto lensValid = lensMm > 0;
    const auto& inverseDepth = depthInverseLut();
    const auto tofInv = inverseDepth[tofMm];
    const auto lensInv = inverseDepth[lensMm];
    const auto alphaGated = tofValid * (1.0f - lensValid * (1.0f - alpha));
    const auto fusedInv = alphaGated * tofInv + (1.0f - alphaGated) * lensInv;
    return {static_cast<float>(tofValid || lensValid) * 1000.0f / std::max(fusedInv, kMinFusedInverse), confidence};
}

const std::array<float, 65536>& depthInverseLut() {
    static const auto lut = [] {
        std::array<float, 65536> values{};
        for(size_t depthMm = 1; depthMm < values.size(); ++depthMm) {
            values[depthMm] = 1000.0f / depthMm;
        }
        return values;
    }();
    return lut;
}

uint8_t quantizeInput(float value) {
    return static_cast<uint8_t>(std::clamp(std::lround(value / kInputQuantizationStep), 0l, 255l));
}

uint8_t quantizeDisagreement(float value) {
    return static_cast<uint8_t>(std::clamp(std::lround(value / kDisQuantizationStep), 0l, 255l));
}

const std::array<uint8_t, 65536>& depthInputLut() {
    static const auto lut = [] {
        std::array<uint8_t, 65536> values{};
        for(size_t depthMm = 1; depthMm < values.size(); ++depthMm) {
            values[depthMm] = quantizeInput((1000.0f / depthMm) / 5.0f);
        }
        return values;
    }();
    return lut;
}

const std::array<uint8_t, 256>& byteInputLut() {
    static const auto lut = [] {
        std::array<uint8_t, 256> values{};
        for(size_t value = 0; value < values.size(); ++value) {
            values[value] = quantizeInput(static_cast<float>(value) / 255.0f);
        }
        return values;
    }();
    return lut;
}

struct OutputLuts {
    std::array<float, 256> alpha;
    std::array<float, 256> spec;
};

OutputLuts makeOutputLuts(const TensorInfo& logitsA, const TensorInfo& logitsSpec) {
    OutputLuts luts;
    for(size_t value = 0; value < luts.alpha.size(); ++value) {
        const auto alpha = (static_cast<float>(value) - logitsA.qpZp) * logitsA.qpScale;
        const auto spec = (static_cast<float>(value) - logitsSpec.qpZp) * logitsSpec.qpScale;
        luts.alpha[value] = 1.0f / (1.0f + std::exp(-std::clamp(alpha, -80.0f, 80.0f)));
        luts.spec[value] = 1.0f / (1.0f + std::exp(-std::clamp(spec, -80.0f, 80.0f)));
    }
    return luts;
}

uint8_t quantizeConfidence(float confidence) {
    return static_cast<uint8_t>(std::lround(std::clamp(confidence, 0.0f, 1.0f) * 255.0f));
}

std::string findTensorName(const NNData& data, const std::string& name) {
    for(const auto& candidate : data.getAllLayerNames()) {
        const auto suffix = '/' + name;
        if(candidate == name || (candidate.size() >= suffix.size() && candidate.compare(candidate.size() - suffix.size(), suffix.size(), suffix) == 0)) {
            return candidate;
        }
    }
    return {};
}

void prepareFrame(ImgFrame& frame, uint32_t width, uint32_t height, ImgFrame::Type type, const ImgFrame& metadataSource) {
    frame.setMetadata(metadataSource);
    frame.setWidth(width);
    frame.setHeight(height);
    frame.setType(type);
    frame.setStride(width * frame.getBytesPerPixel());
    frame.data->setSize(static_cast<size_t>(width) * height * frame.getBytesPerPixel());
}

}  // namespace

void ToFStereoFusion::ensureBuffers(uint32_t width, uint32_t height) {
    if(width == bufferWidth && height == bufferHeight) return;

    const auto pixels = static_cast<size_t>(width) * height;
    nnData = std::make_shared<NNData>(pixels * 7);
    nnData->data->setSize(0);
    depthFrames = std::make_shared<Pool<ImgFrame>>(4, pixels * sizeof(uint16_t));
    confidenceFrames = std::make_shared<Pool<ImgFrame>>(4, pixels);
    bufferWidth = width;
    bufferHeight = height;
}

void ToFStereoFusion::alignNeural(const ImgFrame& tofImage, const ImgFrame& depthNeural, const ImgFrame& confidenceNeural) {
    const auto width = depthNeural.getWidth();
    const auto height = depthNeural.getHeight();
    const auto tofWidth = tofImage.getWidth();
    const auto tofHeight = tofImage.getHeight();
    const cv::Size tofSize(tofWidth, tofHeight), neuralSize(width, height);
    if(alignmentInitialized
       && (tofSize != previousTofSize || neuralSize != previousNeuralSize || !tofImage.transformation.isEqualTransformation(previousTofTransformation)
           || !depthNeural.transformation.isEqualTransformation(previousNeuralTransformation))) {
        alignmentInitialized = false;
    }
    if(!alignmentInitialized) {
        const auto neuralTransform = depthNeural.transformation;
        auto tofTransform = tofImage.transformation;
        auto [transformWidth, transformHeight] = tofTransform.getSize();
        if(transformWidth != tofWidth || transformHeight != tofHeight) {
            tofTransform.addScale(static_cast<float>(tofWidth) / transformWidth, static_cast<float>(tofHeight) / transformHeight);
            tofTransform.setSize(tofWidth, tofHeight);
        }
        neuralToTofRotation = neuralTransform.getRotationMatrixTo(tofTransform);
        neuralToTofTranslation = neuralTransform.getTranslationVectorTo(tofTransform, false, LengthUnit::MILLIMETER);
        auto neuralIntrinsics = neuralTransform.getIntrinsicMatrix();
        auto tofIntrinsics = tofTransform.getIntrinsicMatrix();
        cv::Mat neuralK(3, 3, CV_32F, neuralIntrinsics[0].data());
        cv::Mat tofK(3, 3, CV_32F, tofIntrinsics[0].data());
        cv::Mat rotation(3, 3, CV_32F, neuralToTofRotation.data());
        cv::Mat translation(1, 3, CV_32F, neuralToTofTranslation.data());
        const auto neuralDistortion = neuralTransform.getDistortionCoefficients();
        cv::Mat neuralDist = neuralDistortion.empty() ? cv::Mat{} : cv::Mat(1, neuralDistortion.size(), CV_32F, const_cast<float*>(neuralDistortion.data()));
        cv::Mat tofDist;
        neuralK.convertTo(neuralK, CV_64F);
        tofK.convertTo(tofK, CV_64F);
        rotation.convertTo(rotation, CV_64F);
        translation.convertTo(translation, CV_64F);
        neuralDist.convertTo(neuralDist, CV_64F);
        cv::Mat r1, r2, p1, p2, q;
        cv::stereoRectify(
            neuralK, neuralDist, tofK, tofDist, {static_cast<int>(width), static_cast<int>(height)}, rotation, translation.t(), r1, r2, p1, p2, q);
        cv::initUndistortRectifyMap(
            neuralK, neuralDist, r1, neuralK, {static_cast<int>(width), static_cast<int>(height)}, CV_32FC1, neuralRectificationMapX, neuralRectificationMapY);
        cv::Mat newRotation = r2 * (rotation * r1.t());
        cv::Mat newTranslation = r2 * translation.t();
        newRotation.convertTo(newRotation, CV_32F);
        newTranslation.convertTo(newTranslation, CV_32F);
        newRotation.copyTo(rotation);
        newTranslation.copyTo(translation);
        std::memcpy(neuralToTofRotation.data(), rotation.data, sizeof(neuralToTofRotation));
        std::memcpy(neuralToTofTranslation.data(), translation.data, sizeof(neuralToTofTranslation));
        cv::initUndistortRectifyMap(
            neuralK, tofDist, r2.t(), tofK, {static_cast<int>(tofWidth), static_cast<int>(tofHeight)}, CV_32FC1, tofRectificationMapX, tofRectificationMapY);
        if(initialConfig->cropToOverlap) {
            cv::Mat support(neuralSize, CV_8UC1, cv::Scalar(255)), rectifiedSupport, overlap;
            cv::remap(support, rectifiedSupport, neuralRectificationMapX, neuralRectificationMapY, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
            cv::remap(rectifiedSupport, overlap, tofRectificationMapX, tofRectificationMapY, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
            overlapCrop = largestOverlapRectangle(overlap);
            if(overlapCrop.empty()) throw std::runtime_error("ToFStereoFusion: ToF and NeuralDepth images have no geometric overlap");
            croppedTransformation = tofTransform;
            croppedTransformation.addCrop(overlapCrop.x, overlapCrop.y, overlapCrop.width, overlapCrop.height);
        }
        previousTofSize = tofSize;
        previousNeuralSize = neuralSize;
        previousTofTransformation = tofImage.transformation;
        previousNeuralTransformation = depthNeural.transformation;
        alignmentInitialized = true;
    }
    cv::Mat rectifiedDepth, rectifiedConfidence;
    const cv::Mat neuralDepthImage(height, width, CV_16UC1, const_cast<uint8_t*>(depthNeural.getData().data()));
    const cv::Mat neuralConfidenceImage(height, width, CV_8UC1, const_cast<uint8_t*>(confidenceNeural.getData().data()));
    cv::remap(neuralDepthImage, rectifiedDepth, neuralRectificationMapX, neuralRectificationMapY, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    cv::remap(neuralConfidenceImage, rectifiedConfidence, neuralRectificationMapX, neuralRectificationMapY, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    cv::Mat shiftedDepth = cv::Mat::zeros(height, width, CV_16UC1);
    cv::Mat shiftedConfidence = cv::Mat::zeros(height, width, CV_8UC1);
    const auto shift = neuralToTofTranslation[0] * depthNeural.transformation.getIntrinsicMatrix()[0][0];
    for(uint32_t y = 0; y < height; ++y)
        for(int x = shift > 0 ? width - 1 : 0; x >= 0 && x < static_cast<int>(width); x += shift > 0 ? -1 : 1) {
            const auto z = rectifiedDepth.at<uint16_t>(y, x);
            if(!z) continue;
            const auto u = x + static_cast<int>(shift / z + 0.5f);
            for(const auto xx : {u, u + (shift > 0 ? 1 : -1)})
                if(xx >= 0 && xx < static_cast<int>(width)) {
                    shiftedDepth.at<uint16_t>(y, xx) = z;
                    shiftedConfidence.at<uint8_t>(y, xx) = rectifiedConfidence.at<uint8_t>(y, x);
                }
        }
    alignedNeuralDepth.resize(static_cast<size_t>(tofHeight) * tofWidth);
    alignedNeuralConfidence.resize(static_cast<size_t>(tofHeight) * tofWidth);
    cv::Mat outputDepth(tofHeight, tofWidth, CV_16UC1, alignedNeuralDepth.data());
    cv::Mat outputConfidence(tofHeight, tofWidth, CV_8UC1, alignedNeuralConfidence.data());
    cv::remap(shiftedDepth, outputDepth, tofRectificationMapX, tofRectificationMapY, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
    cv::remap(shiftedConfidence, outputConfidence, tofRectificationMapX, tofRectificationMapY, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 0);
}

ToFStereoFusion::FusionOutputs ToFStereoFusion::alphaNetwork(MessageGroup& group, bool confidenceConnected) {
    const auto preprocessStart = std::chrono::steady_clock::now();
    auto tofDepth = group.get<ImgFrame>("tofDepth");
    auto neuralDepth = group.get<ImgFrame>("neuralDepth");
    auto confidence = group.get<ImgFrame>("tofConfidence");
    auto neuralConfidence = group.get<ImgFrame>("neuralConfidence");
    if(!tofDepth || !neuralDepth || !confidence || !neuralConfidence) return {};
    if(tofDepth->getType() != ImgFrame::Type::RAW16 || neuralDepth->getType() != ImgFrame::Type::RAW16 || confidence->getType() != ImgFrame::Type::RAW8
       || neuralConfidence->getType() != ImgFrame::Type::RAW8) {
        return {};
    }

    const auto width = tofDepth->getWidth();
    const auto height = tofDepth->getHeight();
    const auto pixels = static_cast<size_t>(width) * height;
    if(confidence->getWidth() != width || confidence->getHeight() != height || neuralConfidence->getWidth() != neuralDepth->getWidth()
       || neuralConfidence->getHeight() != neuralDepth->getHeight() || tofDepth->getData().size() < pixels * sizeof(uint16_t)
       || neuralDepth->getData().size() < static_cast<size_t>(neuralDepth->getWidth()) * neuralDepth->getHeight() * sizeof(uint16_t)
       || confidence->getData().size() < pixels
       || neuralConfidence->getData().size() < static_cast<size_t>(neuralDepth->getWidth()) * neuralDepth->getHeight()) {
        return {};
    }

    alignNeural(*tofDepth, *neuralDepth, *neuralConfidence);

    const auto* tof = reinterpret_cast<const uint16_t*>(tofDepth->getData().data());
    const auto* neural = alignedNeuralDepth.data();
    const auto* tofConfidence = confidence->getData().data();
    const auto* lensConfidence = alignedNeuralConfidence.data();

    ensureBuffers(width, height);
    nnData->tensors.clear();
    nnData->data->setSize(0);
    nnData->setBufferMetadataFrom(tofDepth);
    nnData->transformation = tofDepth->transformation;
    TensorInfo info;
    info.name = "x";
    info.dataType = TensorInfo::DataType::U8F;
    info.numDimensions = 4;
    info.order = TensorInfo::StorageOrder::NHWC;
    info.dims = {1, height, width, 6};
    info.strides = {height * width * 6, width * 6, 6, 1};
    auto input = nnData->emplaceTensor(info);
    info.name = "dis";
    info.dims = {1, height, width, 1};
    info.strides = {height * width, width, 1, 1};
    auto disagreement = nnData->emplaceTensor(info);
    const auto& depthLut = depthInputLut();
    const auto& inverseDepthLut = depthInverseLut();
    const auto& byteLut = byteInputLut();
    const auto validInput = quantizeInput(1.0f);
    for(size_t i = 0; i < pixels; ++i) {
        auto* pixel = input.data() + i * 6;
        pixel[0] = depthLut[tof[i]];
        pixel[1] = depthLut[neural[i]];
        pixel[2] = byteLut[tofConfidence[i]];
        pixel[3] = tof[i] ? validInput : 0;
        pixel[4] = neural[i] ? validInput : 0;
        pixel[5] = neural[i] ? byteLut[lensConfidence[i]] : 0;
        const auto dis = tof[i] && neural[i] ? std::min(std::abs(inverseDepthLut[tof[i]] - inverseDepthLut[neural[i]]) * kDisInverseGain, kDisClip) : 0.0f;
        disagreement[i] = quantizeDisagreement(dis);
    }

    const auto inferenceStart = std::chrono::steady_clock::now();
    nnInput.send(nnData);
    auto result = nnDataInput.get<NNData>();
    if(!result) return {};
    const auto logitsAName = findTensorName(*result, "logits_a");
    const auto logitsSpecName = findTensorName(*result, "logits_spec");
    if(logitsAName.empty() || logitsSpecName.empty()) return {};
    const auto postprocessStart = std::chrono::steady_clock::now();

    const cv::Rect outputRegion = initialConfig->cropToOverlap ? overlapCrop : cv::Rect(0, 0, width, height);
    FusionOutputs outputs;
    outputs.depth = depthFrames->get();
    prepareFrame(*outputs.depth, outputRegion.width, outputRegion.height, ImgFrame::Type::RAW16, *tofDepth);
    if(initialConfig->cropToOverlap) outputs.depth->transformation = croppedTransformation;
    auto* depth = reinterpret_cast<uint16_t*>(outputs.depth->getData().data());
    uint8_t* tofNeuralConfidence = nullptr;
    if(confidenceConnected) {
        outputs.tofNeuralConfidence = confidenceFrames->get();
        prepareFrame(*outputs.tofNeuralConfidence, outputRegion.width, outputRegion.height, ImgFrame::Type::RAW8, *tofDepth);
        if(initialConfig->cropToOverlap) outputs.tofNeuralConfidence->transformation = croppedTransformation;
        tofNeuralConfidence = outputs.tofNeuralConfidence->getData().data();
    }

    const auto logitsAInfo = result->getTensorInfo(logitsAName);
    const auto logitsSpecInfo = result->getTensorInfo(logitsSpecName);
    const auto& outputData = result->getData();
    const auto directQuantizedOutput = logitsAInfo && logitsSpecInfo && logitsAInfo->dataType == TensorInfo::DataType::U8F
                                       && logitsSpecInfo->dataType == TensorInfo::DataType::U8F && outputData.size() >= logitsAInfo->offset + pixels
                                       && outputData.size() >= logitsSpecInfo->offset + pixels;
    if(directQuantizedOutput) {
        const auto luts = makeOutputLuts(*logitsAInfo, *logitsSpecInfo);
        const auto* logitsA = outputData.data() + logitsAInfo->offset;
        const auto* logitsSpec = outputData.data() + logitsSpecInfo->offset;
        for(int y = 0; y < outputRegion.height; ++y) {
            const size_t sourceRow = static_cast<size_t>(outputRegion.y + y) * width + outputRegion.x;
            const size_t outputRow = static_cast<size_t>(y) * outputRegion.width;
            for(int x = 0; x < outputRegion.width; ++x) {
                const auto source = sourceRow + x;
                const auto output = outputRow + x;
                const auto fused = fuseDepthAndConfidence(tof[source], neural[source], luts.alpha[logitsA[source]], luts.spec[logitsSpec[source]]);
                depth[output] = fused.confidence >= initialConfig->confidenceThreshold ? static_cast<uint16_t>(std::lround(fused.depthMm)) : 0;
                if(tofNeuralConfidence) tofNeuralConfidence[output] = quantizeConfidence(fused.confidence);
            }
        }
    } else {
        auto logitsA = result->getTensor<float>(logitsAName, true);
        auto logitsSpec = result->getTensor<float>(logitsSpecName, true);
        if(logitsA.size() < pixels || logitsSpec.size() < pixels) return {};
        for(int y = 0; y < outputRegion.height; ++y) {
            const size_t sourceRow = static_cast<size_t>(outputRegion.y + y) * width + outputRegion.x;
            const size_t outputRow = static_cast<size_t>(y) * outputRegion.width;
            for(int x = 0; x < outputRegion.width; ++x) {
                const auto source = sourceRow + x;
                const auto output = outputRow + x;
                const auto alpha = 1.0f / (1.0f + std::exp(-std::clamp(logitsA.data()[source], -80.0f, 80.0f)));
                const auto spec = 1.0f / (1.0f + std::exp(-std::clamp(logitsSpec.data()[source], -80.0f, 80.0f)));
                const auto fused = fuseDepthAndConfidence(tof[source], neural[source], alpha, spec);
                depth[output] = fused.confidence >= initialConfig->confidenceThreshold ? static_cast<uint16_t>(std::lround(fused.depthMm)) : 0;
                if(tofNeuralConfidence) tofNeuralConfidence[output] = quantizeConfidence(fused.confidence);
            }
        }
    }

    const auto postprocessEnd = std::chrono::steady_clock::now();
    const auto toMilliseconds = [](const auto duration) { return std::chrono::duration<double, std::milli>(duration).count(); };
    logger->debug("ToF stereo fusion timings: preprocessing {:.3f} ms, alpha network {:.3f} ms, postprocessing {:.3f} ms, total {:.3f} ms",
                  toMilliseconds(inferenceStart - preprocessStart),
                  toMilliseconds(postprocessStart - inferenceStart),
                  toMilliseconds(postprocessEnd - postprocessStart),
                  toMilliseconds(postprocessEnd - preprocessStart));

    return outputs;
}

void ToFStereoFusion::run() {
    const bool confidenceConnected = !confidence.getConnections().empty() || !confidence.getQueueConnections().empty();

    while(mainLoop()) {
        std::shared_ptr<MessageGroup> group;
        {
            auto blockEvent = inputBlockEvent();
            group = syncedInputs.get<MessageGroup>();
        }
        if(!group) continue;

        auto result = alphaNetwork(*group, confidenceConnected);
        if(!result.depth) continue;

        auto outputBlock = outputBlockEvent();
        depth.send(result.depth);
        if(result.tofNeuralConfidence) {
            confidence.send(result.tofNeuralConfidence);
        }
    }
}

}  // namespace dai::gate
