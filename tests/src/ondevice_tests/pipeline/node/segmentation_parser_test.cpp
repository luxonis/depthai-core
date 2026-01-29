#include <fp16/fp16.h>

#include <algorithm>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <vector>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/TensorInfo.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/SegmentationMask.hpp"
#include "depthai/pipeline/datatype/SegmentationParserConfig.hpp"
#include "depthai/pipeline/node/SegmentationParser.hpp"

namespace {

size_t dataTypeSize(dai::TensorInfo::DataType type) {
    switch(type) {
        case dai::TensorInfo::DataType::FP16:
            return sizeof(uint16_t);
        case dai::TensorInfo::DataType::U8F:
            return sizeof(uint8_t);
        case dai::TensorInfo::DataType::INT:
            return sizeof(int32_t);
        case dai::TensorInfo::DataType::FP32:
            return sizeof(float);
        case dai::TensorInfo::DataType::I8:
            return sizeof(int8_t);
        case dai::TensorInfo::DataType::FP64:
            return sizeof(double);
        default:
            return 0U;
    }
}

dai::TensorInfo makeNCHWTensorInfo(
    const std::string& name, dai::TensorInfo::DataType type, int channels, int height, int width, size_t elementSizeOverride = 0U) {
    dai::TensorInfo info;
    info.name = name;
    info.order = dai::TensorInfo::StorageOrder::NCHW;
    info.dataType = type;
    info.numDimensions = 4;
    info.dims = {1U, static_cast<unsigned>(channels), static_cast<unsigned>(height), static_cast<unsigned>(width)};

    const size_t elementSize = elementSizeOverride ? elementSizeOverride : dataTypeSize(type);
    const size_t strideW = elementSize;
    const size_t strideH = strideW * static_cast<size_t>(width);
    const size_t strideC = strideH * static_cast<size_t>(height);
    const size_t strideN = strideC * static_cast<size_t>(channels);

    info.strides = {static_cast<unsigned>(strideN), static_cast<unsigned>(strideC), static_cast<unsigned>(strideH), static_cast<unsigned>(strideW)};
    return info;
}

dai::TensorInfo makeNHWCTensorInfo(const std::string& name, dai::TensorInfo::DataType type, int height, int width, int channels, bool vectorizable) {
    dai::TensorInfo info;
    info.name = name;
    info.order = dai::TensorInfo::StorageOrder::NHWC;
    info.dataType = type;
    info.numDimensions = 4;
    info.dims = {1U, static_cast<unsigned>(height), static_cast<unsigned>(width), static_cast<unsigned>(channels)};

    const size_t elementSize = dataTypeSize(type);
    const size_t strideC = elementSize;
    const size_t strideW = (vectorizable ? static_cast<size_t>(channels) : static_cast<size_t>(channels + 1)) * elementSize;
    const size_t strideH = strideW * static_cast<size_t>(width);
    const size_t strideN = strideH * static_cast<size_t>(height);

    info.strides = {static_cast<unsigned>(strideN), static_cast<unsigned>(strideH), static_cast<unsigned>(strideW), static_cast<unsigned>(strideC)};
    return info;
}

size_t tensorOffset(const dai::TensorInfo& info, int n, int c, int h, int w) {
    switch(info.order) {
        case dai::TensorInfo::StorageOrder::NCHW:
            return static_cast<size_t>(n) * info.strides[0] + static_cast<size_t>(c) * info.strides[1] + static_cast<size_t>(h) * info.strides[2]
                   + static_cast<size_t>(w) * info.strides[3];
        case dai::TensorInfo::StorageOrder::NHWC:
            return static_cast<size_t>(n) * info.strides[0] + static_cast<size_t>(h) * info.strides[1] + static_cast<size_t>(w) * info.strides[2]
                   + static_cast<size_t>(c) * info.strides[3];
        default:
            return 0U;
    }
}

template <typename T, typename ValueFn>
std::shared_ptr<dai::NNData> createSyntheticNNData(const dai::TensorInfo& templateInfo, ValueFn valueFn) {
    auto nnData = std::make_shared<dai::NNData>();
    dai::TensorInfo info = templateInfo;
    auto tensorBytes = nnData->emplaceTensor(info);
    std::fill(tensorBytes.begin(), tensorBytes.end(), 0);

    const int width = info.getWidth();
    const int height = info.getHeight();
    const int channels = info.getChannels();

    nnData->transformation = dai::ImgTransformation(static_cast<size_t>(width), static_cast<size_t>(height));

    for(int h = 0; h < height; ++h) {
        for(int w = 0; w < width; ++w) {
            for(int c = 0; c < channels; ++c) {
                T value = valueFn(c, h, w);
                const size_t offset = tensorOffset(info, 0, c, h, w);
                REQUIRE(offset + sizeof(T) <= tensorBytes.size());
                std::memcpy(tensorBytes.data() + offset, &value, sizeof(T));
            }
        }
    }

    nnData->setSequenceNum(0);
    nnData->setTimestamp(std::chrono::steady_clock::now());
    return nnData;
}

template <typename ValueFn>
std::vector<uint8_t> computeExpectedMask(int width, int height, int channels, float threshold, bool backgroundClass, ValueFn valueFn, int stepSize = 1) {
    const int outWidth = width / stepSize;
    const int outHeight = height / stepSize;
    std::vector<uint8_t> mask(static_cast<size_t>(outWidth) * static_cast<size_t>(outHeight), 255);
    const int channelStart = backgroundClass ? 1 : 0;

    int outY = 0;
    for(int h = 0; h < height; h += stepSize, ++outY) {
        int outX = 0;
        for(int w = 0; w < width; w += stepSize, ++outX) {
            float bestVal = threshold;
            uint8_t bestClass = 255;
            if(backgroundClass) {
                bestVal = std::max(bestVal, valueFn(0, h, w));
            }
            for(int c = channelStart; c < channels; ++c) {
                float v = valueFn(c, h, w);
                if(v > bestVal) {
                    bestVal = v;
                    bestClass = static_cast<uint8_t>(c);
                }
            }
            mask[static_cast<size_t>(outY) * static_cast<size_t>(outWidth) + static_cast<size_t>(outX)] = bestClass;
        }
    }

    return mask;
}

void checkMaskMatches(const dai::SegmentationMask& mask, const std::vector<uint8_t>& expected, size_t width, size_t height) {
    REQUIRE(mask.getWidth() == width);
    REQUIRE(mask.getHeight() == height);
    const auto data = mask.getMaskData();
    REQUIRE(!data.empty());
    REQUIRE(data.size() == expected.size());

    for(size_t i = 0; i < expected.size(); ++i) {
        const size_t row = i / width;
        const size_t col = i % width;
        const int actual = static_cast<int>(data[i]);
        const int expectedValue = static_cast<int>(expected[i]);
        CAPTURE(i, row, col, actual, expectedValue);
        CHECK(actual == expectedValue);
    }
}

std::vector<std::shared_ptr<dai::SegmentationMask>> processSegmentationFrames(
    const dai::SegmentationParserConfig& initialConfig,
    const std::vector<std::shared_ptr<dai::NNData>>& frames,
    bool classesInOneLayer,
    bool backgroundClass,
    const std::vector<std::optional<dai::SegmentationParserConfig>>& runtimeConfigs = {},
    bool syncConfig = false) {
    dai::Pipeline pipeline;
    auto parser = pipeline.create<dai::node::SegmentationParser>();
    parser->setRunOnHost(true);
    parser->initialConfig = std::make_shared<dai::SegmentationParserConfig>(initialConfig);
    parser->properties.classesInOneLayer = classesInOneLayer;
    parser->setBackgroundClass(backgroundClass);
    if(syncConfig) {
        parser->inputConfig.setWaitForMessage(true);
    }

    auto inputQueue = parser->input.createInputQueue();
    auto configQueue = parser->inputConfig.createInputQueue();
    auto outputQueue = parser->out.createOutputQueue();

    pipeline.start();

    std::vector<std::shared_ptr<dai::SegmentationMask>> outputs;
    outputs.reserve(frames.size());

    for(size_t i = 0; i < frames.size(); ++i) {
        if(syncConfig) {
            const auto& cfg = (i < runtimeConfigs.size() && runtimeConfigs[i].has_value()) ? *runtimeConfigs[i] : initialConfig;
            configQueue->send(std::make_shared<dai::SegmentationParserConfig>(cfg));
        } else if(i < runtimeConfigs.size() && runtimeConfigs[i].has_value()) {
            configQueue->send(std::make_shared<dai::SegmentationParserConfig>(*runtimeConfigs[i]));
        }
        inputQueue->send(frames[i]);
        auto mask = outputQueue->get<dai::SegmentationMask>();
        REQUIRE(mask != nullptr);
        outputs.push_back(mask);
    }

    pipeline.stop();
    pipeline.wait();
    return outputs;
}

std::vector<float> makeArgmaxTestValues() {
    return {
        0.9f,
        0.2f,
        0.1f,  // (0,0)
        0.1f,
        0.8f,
        0.3f,  // (0,1)
        0.2f,
        0.4f,
        0.7f,  // (0,2)
        0.3f,
        0.6f,
        0.4f,  // (1,0)
        0.2f,
        0.35f,
        0.45f,  // (1,1)
        0.1f,
        0.2f,
        0.3f  // (1,2)
    };
}

}  // namespace

TEST_CASE("SegmentationParser argmax backends produce expected mask") {
    constexpr int kWidth = 3;
    constexpr int kHeight = 2;
    constexpr int kChannels = 3;

    const auto values = makeArgmaxTestValues();
    auto valueFn = [&](int c, int h, int w) { return values[static_cast<size_t>(h * kWidth + w) * static_cast<size_t>(kChannels) + static_cast<size_t>(c)]; };

    dai::SegmentationParserConfig config;
    config.setConfidenceThreshold(0.5f);
    config.setStepSize(1);

    const auto expected = computeExpectedMask(kWidth, kHeight, kChannels, 0.5f, false, valueFn, static_cast<int>(config.getStepSize()));
    const size_t maskWidth = static_cast<size_t>(kWidth) / config.getStepSize();
    const size_t maskHeight = static_cast<size_t>(kHeight) / config.getStepSize();

    SECTION("NCHW") {
        auto info = makeNCHWTensorInfo("seg", dai::TensorInfo::DataType::FP32, kChannels, kHeight, kWidth);
        auto nnData = createSyntheticNNData<float>(info, valueFn);
        auto outputs = processSegmentationFrames(config, {nnData}, false, false);
        checkMaskMatches(*outputs.front(), expected, maskWidth, maskHeight);
    }

    SECTION("NHWC vectorized") {
        auto info = makeNHWCTensorInfo("seg", dai::TensorInfo::DataType::FP32, kHeight, kWidth, kChannels, true);
        auto nnData = createSyntheticNNData<float>(info, valueFn);
        auto outputs = processSegmentationFrames(config, {nnData}, false, false);
        checkMaskMatches(*outputs.front(), expected, maskWidth, maskHeight);
    }

    SECTION("NHWC safe") {
        auto info = makeNHWCTensorInfo("seg", dai::TensorInfo::DataType::FP32, kHeight, kWidth, kChannels, false);
        auto nnData = createSyntheticNNData<float>(info, valueFn);
        auto outputs = processSegmentationFrames(config, {nnData}, false, false);
        checkMaskMatches(*outputs.front(), expected, maskWidth, maskHeight);
    }

    SECTION("FP16 argmax") {
        auto info = makeNCHWTensorInfo("seg", dai::TensorInfo::DataType::FP16, kChannels, kHeight, kWidth);
        auto fp16ValueFn = [&](int c, int h, int w) { return fp16_ieee_from_fp32_value(valueFn(c, h, w)); };
        auto nnData = createSyntheticNNData<uint16_t>(info, fp16ValueFn);
        auto roundedValueFn = [&](int c, int h, int w) { return fp16_ieee_to_fp32_value(fp16_ieee_from_fp32_value(valueFn(c, h, w))); };
        const auto expectedFp16 = computeExpectedMask(kWidth, kHeight, kChannels, 0.5f, false, roundedValueFn, static_cast<int>(config.getStepSize()));
        auto outputs = processSegmentationFrames(config, {nnData}, false, false);
        checkMaskMatches(*outputs.front(), expectedFp16, maskWidth, maskHeight);
    }

    SECTION("INT8 argmax") {
        auto info = makeNCHWTensorInfo("seg", dai::TensorInfo::DataType::I8, kChannels, kHeight, kWidth);
        info.qpScale = 1.0f;
        info.qpZp = 0;
        auto i8ValueFn = [&](int c, int h, int w) {
            float v = valueFn(c, h, w) * 100.0f;
            return static_cast<int8_t>(std::clamp(v, -128.0f, 127.0f));
        };
        auto nnData = createSyntheticNNData<int8_t>(info, i8ValueFn);
        auto expectedValueFn = [&](int c, int h, int w) { return static_cast<float>(i8ValueFn(c, h, w)); };
        const auto expectedI8 = computeExpectedMask(kWidth, kHeight, kChannels, 0.5f, false, expectedValueFn, static_cast<int>(config.getStepSize()));
        auto outputs = processSegmentationFrames(config, {nnData}, false, false);
        checkMaskMatches(*outputs.front(), expectedI8, maskWidth, maskHeight);
    }
}

TEST_CASE("SegmentationParser background class ignores channel 0") {
    constexpr int kWidth = 2;
    constexpr int kHeight = 1;
    constexpr int kChannels = 3;

    const std::vector<float> values = {
        0.95f,
        0.8f,
        0.1f,  // (0,0)
        0.2f,
        0.1f,
        0.9f  // (0,1)
    };

    auto valueFn = [&](int c, int h, int w) { return values[static_cast<size_t>(h * kWidth + w) * static_cast<size_t>(kChannels) + static_cast<size_t>(c)]; };

    dai::SegmentationParserConfig config;
    config.setConfidenceThreshold(0.3f);
    config.setStepSize(1);

    auto info = makeNCHWTensorInfo("seg", dai::TensorInfo::DataType::FP32, kChannels, kHeight, kWidth);
    auto nnData = createSyntheticNNData<float>(info, valueFn);

    const auto expected = computeExpectedMask(kWidth, kHeight, kChannels, 0.3f, true, valueFn, static_cast<int>(config.getStepSize()));
    auto outputs = processSegmentationFrames(config, {nnData}, false, true);
    const size_t maskWidth = static_cast<size_t>(kWidth) / config.getStepSize();
    const size_t maskHeight = static_cast<size_t>(kHeight) / config.getStepSize();
    checkMaskMatches(*outputs.front(), expected, maskWidth, maskHeight);
}

TEST_CASE("SegmentationParser classes-in-one-layer passes through mask") {
    constexpr int kWidth = 3;
    constexpr int kHeight = 2;

    const std::vector<uint8_t> expected = {0, 1, 2, 255, 1, 0};
    auto valueFn = [&](int, int h, int w) { return expected[static_cast<size_t>(h * kWidth + w)]; };

    dai::SegmentationParserConfig config;
    config.setStepSize(1);

    auto info = makeNCHWTensorInfo("seg", dai::TensorInfo::DataType::INT, 1, kHeight, kWidth, sizeof(uint8_t));
    auto nnData = createSyntheticNNData<uint8_t>(info, valueFn);

    auto outputs = processSegmentationFrames(config, {nnData}, true, false);
    const size_t maskWidth = static_cast<size_t>(kWidth) / config.getStepSize();
    const size_t maskHeight = static_cast<size_t>(kHeight) / config.getStepSize();
    checkMaskMatches(*outputs.front(), expected, maskWidth, maskHeight);
}

TEST_CASE("SegmentationParser runtime config update changes thresholding") {
    constexpr int kWidth = 3;
    constexpr int kHeight = 2;
    constexpr int kChannels = 3;

    const auto values = makeArgmaxTestValues();
    auto valueFn = [&](int c, int h, int w) { return values[static_cast<size_t>(h * kWidth + w) * static_cast<size_t>(kChannels) + static_cast<size_t>(c)]; };

    auto info = makeNCHWTensorInfo("seg", dai::TensorInfo::DataType::FP32, kChannels, kHeight, kWidth);
    auto nnData = createSyntheticNNData<float>(info, valueFn);

    dai::SegmentationParserConfig initialConfig;
    initialConfig.setConfidenceThreshold(0.3f);
    initialConfig.setStepSize(1);

    dai::SegmentationParserConfig updatedConfig;
    updatedConfig.setConfidenceThreshold(0.6f);
    updatedConfig.setStepSize(1);

    const auto expectedInitial = computeExpectedMask(kWidth, kHeight, kChannels, 0.3f, false, valueFn, static_cast<int>(initialConfig.getStepSize()));
    const auto expectedUpdated = computeExpectedMask(kWidth, kHeight, kChannels, 0.6f, false, valueFn, static_cast<int>(updatedConfig.getStepSize()));
    const size_t initialWidth = static_cast<size_t>(kWidth) / initialConfig.getStepSize();
    const size_t initialHeight = static_cast<size_t>(kHeight) / initialConfig.getStepSize();
    const size_t updatedWidth = static_cast<size_t>(kWidth) / updatedConfig.getStepSize();
    const size_t updatedHeight = static_cast<size_t>(kHeight) / updatedConfig.getStepSize();

    auto outputs = processSegmentationFrames(initialConfig, {nnData, nnData}, false, false, {initialConfig, updatedConfig}, true);

    REQUIRE(outputs.size() == 2);
    checkMaskMatches(*outputs[0], expectedInitial, initialWidth, initialHeight);
    checkMaskMatches(*outputs[1], expectedUpdated, updatedWidth, updatedHeight);
}
