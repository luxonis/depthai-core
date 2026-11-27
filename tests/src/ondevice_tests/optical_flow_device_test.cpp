#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "depthai/depthai.hpp"

namespace {
std::vector<uint8_t> texturedImage(unsigned int width, unsigned int height) {
    std::vector<uint8_t> noise(width * height), image(width * height);
    uint32_t state = 27;
    for(auto& value : noise) {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        value = static_cast<uint8_t>(state);
    }
    // Mild smoothing gives deterministic, textured inputs without OpenCV/assets.
    for(unsigned int y = 0; y < height; ++y) {
        for(unsigned int x = 0; x < width; ++x) {
            unsigned int sum = 4 * noise[y * width + x];
            sum += noise[y * width + (x ? x - 1 : x)];
            sum += noise[y * width + std::min(x + 1, width - 1)];
            sum += noise[(y ? y - 1 : y) * width + x];
            sum += noise[std::min(y + 1, height - 1) * width + x];
            image[y * width + x] = static_cast<uint8_t>(sum / 8);
        }
    }
    return image;
}

std::vector<uint8_t> translate(const std::vector<uint8_t>& source, unsigned int width, unsigned int height, int dx, int dy) {
    auto result = source;
    for(int y = 0; y < static_cast<int>(height); ++y) {
        for(int x = 0; x < static_cast<int>(width); ++x) {
            const auto sx = std::clamp(x - dx, 0, static_cast<int>(width) - 1);
            const auto sy = std::clamp(y - dy, 0, static_cast<int>(height) - 1);
            result[y * width + x] = source[sy * width + sx];
        }
    }
    return result;
}

float median(std::vector<float>& values) {
    REQUIRE(!values.empty());
    auto middle = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

void checkMotion(const dai::NNData& flow, unsigned int width, unsigned int height, float dx, float dy) {
    const auto forward = flow.getTensorInfo("forward");
    const auto backward = flow.getTensorInfo("backward");
    const auto confidence = flow.getTensorInfo("confidence");
    REQUIRE(forward.has_value());
    REQUIRE(backward.has_value());
    REQUIRE(confidence.has_value());
    for(const auto& tensor : {*forward, *backward}) {
        REQUIRE(tensor.dims == std::vector<unsigned int>{height, width, 2});
        REQUIRE(tensor.strides == std::vector<unsigned int>{width * 8, 8, 4});
        REQUIRE(tensor.dataType == dai::TensorInfo::DataType::FP32);
    }
    REQUIRE(confidence->dims == std::vector<unsigned int>{height, width, 1});
    REQUIRE(confidence->dataType == dai::TensorInfo::DataType::U8F);
    const auto bytes = flow.getData();
    REQUIRE(bytes.size() >= confidence->offset + static_cast<std::size_t>(width) * height);
    REQUIRE(bytes.size() >= forward->offset + static_cast<std::size_t>(width) * height * 8);
    REQUIRE(bytes.size() >= backward->offset + static_cast<std::size_t>(width) * height * 8);
    std::vector<float> forwardErrors, backwardErrors;
    unsigned int sampled = 0;
    for(unsigned int y = 48; y < height - 48; y += 2) {
        for(unsigned int x = 48; x < width - 48; x += 2) {
            ++sampled;
            const auto pixel = y * width + x;
            float f[2], b[2];
            std::memcpy(f, bytes.data() + forward->offset + pixel * 8, sizeof(f));
            std::memcpy(b, bytes.data() + backward->offset + pixel * 8, sizeof(b));
            REQUIRE(std::isfinite(f[0]));
            REQUIRE(std::isfinite(f[1]));
            REQUIRE(std::isfinite(b[0]));
            REQUIRE(std::isfinite(b[1]));
            // Raw confidence belongs to the reference grid; do not apply it to
            // backward samples. Test backward median independently on texture.
            if(bytes[confidence->offset + pixel] >= 100) forwardErrors.push_back(std::hypot(f[0] - dx, f[1] - dy));
            backwardErrors.push_back(std::hypot(b[0] + dx, b[1] + dy));
        }
    }
    REQUIRE(forwardErrors.size() > sampled / 10);
    REQUIRE(median(forwardErrors) < 1.5f);
    REQUIRE(median(backwardErrors) < 1.5f);
}
}  // namespace

TEST_CASE("RVC4 native optical flow geometry, signed motion, metadata and owning packets", "[OpticalFlow][RVC4]") {
    const std::array<std::array<unsigned int, 2>, 4> geometries = {{{128, 128}, {640, 384}, {1280, 736}, {1920, 1056}}};
    for(const auto& geometry : geometries) {
        auto device = std::make_shared<dai::Device>();
        REQUIRE(device->getPlatform() == dai::Platform::RVC4);
        const auto width = geometry[0], height = geometry[1];
        INFO("geometry=" << width << "x" << height);
        const auto base = texturedImage(width, height);
        const auto moved = translate(base, width, height, 4, -3);
        dai::Pipeline pipeline(device);
        auto flowNode = pipeline.create<dai::node::OpticalFlow>();
        auto input = flowNode->inputImage.createInputQueue(2, true);
        auto passthrough = flowNode->passthroughInputImage.createOutputQueue(2, true);
        auto output = flowNode->outputFlow.createOutputQueue(2, true);
        pipeline.start();
        const std::array<int64_t, 4> sequences = {10, 11, 14, 15};
        const std::array<int, 4> milliseconds = {1000, 1033, 1133, 1166};
        std::shared_ptr<dai::NNData> retained;
        std::vector<uint8_t> retainedBytes;
        for(std::size_t i = 0; i < sequences.size(); ++i) {
            const auto& pixels = i == 2 ? moved : base;
            auto image = std::make_shared<dai::ImgFrame>();
            image->setWidth(width).setHeight(height).setType(dai::ImgFrame::Type::GRAY8);
            image->setData(pixels);
            image->setSequenceNum(sequences[i]);
            const auto timestamp = std::chrono::steady_clock::time_point(std::chrono::milliseconds(milliseconds[i]));
            image->setTimestamp(timestamp);
            image->setTimestampDevice(timestamp);
            REQUIRE(input->trySend(image));
            bool timedOut = false;
            auto passed = passthrough->get<dai::ImgFrame>(std::chrono::seconds(30), timedOut);
            REQUIRE(!timedOut);
            REQUIRE(passed != nullptr);
            REQUIRE(passed->getSequenceNum() == sequences[i]);
            REQUIRE(passed->getTimestampDevice() == timestamp);
            REQUIRE(passed->getData().size() == pixels.size());
            REQUIRE(std::equal(pixels.begin(), pixels.end(), passed->getData().begin()));
            REQUIRE(std::equal(pixels.begin(), pixels.end(), image->getData().begin()));
            if(i == 0) {
                REQUIRE(output->tryGet<dai::MessageGroup>() == nullptr);
                continue;
            }
            auto group = output->get<dai::MessageGroup>(std::chrono::seconds(30), timedOut);
            REQUIRE(!timedOut);
            REQUIRE(group != nullptr);
            auto reference = group->get<dai::Buffer>("reference");
            auto target = group->get<dai::Buffer>("target");
            auto flow = group->get<dai::NNData>("flow");
            REQUIRE(reference != nullptr);
            REQUIRE(target != nullptr);
            REQUIRE(flow != nullptr);
            REQUIRE(group->getSequenceNum() == sequences[i]);
            REQUIRE(flow->getSequenceNum() == sequences[i]);
            REQUIRE(reference->getSequenceNum() == sequences[i - 1]);
            REQUIRE(target->getSequenceNum() == sequences[i]);
            REQUIRE(group->getTimestampDevice() == timestamp);
            REQUIRE(flow->getTimestampDevice() == timestamp);
            REQUIRE(target->getTimestampDevice() == timestamp);
            REQUIRE(reference->getTimestampDevice() == std::chrono::steady_clock::time_point(std::chrono::milliseconds(milliseconds[i - 1])));
            REQUIRE(target->getTimestamp() == timestamp);
            REQUIRE(reference->getData().empty());
            REQUIRE(target->getData().empty());
            const float sign = i == 1 ? 0.f : (i == 2 ? 1.f : -1.f);
            checkMotion(*flow, width, height, sign * 4.f, sign * -3.f);
            if(retained) {
                const auto bytes = retained->getData();
                REQUIRE(bytes.size() == retainedBytes.size());
                REQUIRE(std::equal(retainedBytes.begin(), retainedBytes.end(), bytes.begin()));
            }
            if(!retained) {
                retained = flow;
                const auto bytes = flow->getData();
                retainedBytes.assign(bytes.begin(), bytes.end());
            }
        }
        pipeline.stop();
        pipeline.wait();
    }
}
